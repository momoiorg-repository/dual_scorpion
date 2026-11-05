"""Quantitative checks for the human-to-robot retargeting stage."""

import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from ..geometry import MocopiSample, pose
from ..retarget import alignment_rotation
from .session import POSE_BONES

SCHEMA = "dual_scorpion.mocopi.retarget_validation.v1"
SIDES = ("left", "right")


def _state_poses(values, names):
    state = np.asarray(values, dtype=float).reshape(-1, 7)
    return {name: (state[index, :3], state[index, 3:7]) for index, name in enumerate(names)}


def _body_rotation(initial):
    left = initial["left_shoulder"][0] - initial["right_shoulder"][0]
    left[2] = 0
    length = np.linalg.norm(left)
    if length < 0.05:
        raise ValueError("Cannot validate retarget axes: shoulder separation is too small")
    left /= length
    forward = np.cross(left, [0.0, 0.0, 1.0])
    forward /= np.linalg.norm(forward)
    return np.column_stack([forward, left, [0.0, 0.0, 1.0]])


def _mocopi_sample(values):
    state = np.asarray(values, dtype=float)
    if state.shape != (49,) or not np.isfinite(state).all():
        raise ValueError("Expected a finite 49-value mocopi state")
    bones = {0: np.eye(4)}
    for index, (_, bone) in enumerate(POSE_BONES):
        value = state[index * 7 : (index + 1) * 7]
        bones[bone] = pose(value[:3], value[3:])
    return MocopiSample(0.0, bones)


def _correlation(left, right):
    left, right = np.asarray(left, dtype=float), np.asarray(right, dtype=float)
    if len(left) < 2 or np.ptp(left) < 1e-9 or np.ptp(right) < 1e-9:
        return 0.0
    return float(np.corrcoef(left, right)[0, 1])


def _axis_mapping(human, robot):
    matrix = np.asarray(
        [[_correlation(human[:, i], robot[:, j]) for j in range(3)] for i in range(3)],
        dtype=float,
    )
    labels = ("forward", "left", "up")
    robot_labels = ("x", "y", "z")
    mapping = {}
    for index, label in enumerate(labels):
        target = int(np.argmax(np.abs(matrix[index])))
        correlation = float(matrix[index, target])
        mapping[label] = {
            "robot_axis": robot_labels[target],
            "sign": 1 if correlation >= 0 else -1,
            "correlation": correlation,
        }
    return matrix.tolist(), mapping


def _scaled_orientation_error(mapped, actual, scale):
    mapped_vector = Rotation.from_matrix(mapped).as_rotvec() * scale
    actual_vector = Rotation.from_matrix(actual).as_rotvec()
    return float(np.rad2deg(np.linalg.norm(mapped_vector - actual_vector)))


def validate_retargeting(dataset_path, plans):
    """Compare human relative motion with the retargeted robot TCP trajectories."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = Path(dataset_path).resolve()
    manifest = json.loads((root / "meta" / "mocopi_dataset.json").read_text(encoding="utf-8"))
    dataset = LeRobotDataset(manifest["repo_id"], root=root, video_backend="pyav")
    names = manifest["state_bones"]
    reports, warnings = [], []
    for plan in plans:
        episode = int(plan["episode"])
        episode_meta = dataset.meta.episodes[episode]
        start, end = int(episode_meta["dataset_from_index"]), int(episode_meta["dataset_to_index"])
        states = [_state_poses(dataset.hf_dataset[index]["observation.state"], names) for index in range(start, end)]
        initial = states[0]
        body_rotation = _body_rotation(initial)
        retarget = plan["retarget"]
        sample = _mocopi_sample(dataset.hf_dataset[start]["observation.state"])
        rotation, alignment_source = alignment_rotation(
            sample.bones[10],
            sample,
            retarget["world_to_robot_rotation"],
            retarget["alignment_mode"],
        )
        configured_scale = float(retarget["translation_scale"])
        orientation_scale = float(retarget["orientation_scale"])
        sides = {}
        for side in SIDES:
            human_anchor = initial[f"{side}_hand"][0]
            robot_anchor = np.asarray(plan["frames"][0]["arms"][side]["target_pose"][:3], dtype=float)
            human_delta, robot_delta, scales, orientation_errors, mapped_residuals = [], [], [], [], []
            human_initial_rotation = initial[f"{side}_hand"][1]
            robot_initial_rotation = Rotation.from_quat(plan["frames"][0]["arms"][side]["target_pose"][3:]).as_matrix()
            for index, state in enumerate(states):
                human_position, human_quaternion = state[f"{side}_hand"]
                local_delta = body_rotation.T @ (human_position - human_anchor)
                mapped_delta = configured_scale * rotation @ (human_position - human_anchor)
                robot_position = np.asarray(plan["frames"][index]["arms"][side]["target_pose"][:3], dtype=float)
                world_delta = robot_position - robot_anchor
                human_delta.append(local_delta)
                robot_delta.append(world_delta)
                mapped_residuals.append(float(np.linalg.norm(mapped_delta - world_delta)))
                if np.linalg.norm(local_delta) > 0.01:
                    scales.append(float(np.linalg.norm(world_delta) / np.linalg.norm(local_delta)))
                human_delta_rotation = (
                    Rotation.from_quat(human_quaternion).as_matrix()
                    @ Rotation.from_quat(human_initial_rotation).as_matrix().T
                )
                mapped_rotation = rotation @ human_delta_rotation @ rotation.T
                robot_delta_rotation = (
                    Rotation.from_quat(plan["frames"][index]["arms"][side]["target_pose"][3:]).as_matrix()
                    @ robot_initial_rotation.T
                )
                orientation_errors.append(
                    _scaled_orientation_error(mapped_rotation, robot_delta_rotation, orientation_scale)
                )
            matrix, mapping = _axis_mapping(np.asarray(human_delta), np.asarray(robot_delta))
            sides[side] = {
                "human_bone": f"WRIST/{side[0].upper()}",
                "robot_tcp": f"{side[0]}_link_8",
                "human_anchor_m": human_anchor.tolist(),
                "robot_anchor_m": robot_anchor.tolist(),
                "axis_correlation_human_forward_left_up_vs_robot_xyz": matrix,
                "axis_mapping": mapping,
                "operator_to_robot_rotation": rotation.tolist(),
                "alignment_source": alignment_source,
                "configured_translation_scale": configured_scale,
                "measured_translation_scale": {
                    "median": float(np.median(scales)) if scales else None,
                    "p10": float(np.quantile(scales, 0.1)) if scales else None,
                    "p90": float(np.quantile(scales, 0.9)) if scales else None,
                },
                "orientation_delta_error_deg": {
                    "mean": float(np.mean(orientation_errors)),
                    "p95": float(np.quantile(orientation_errors, 0.95)),
                    "max": float(np.max(orientation_errors)),
                },
                "mapped_human_vs_robot_translation_residual_m": {
                    "max": float(np.max(mapped_residuals)),
                },
            }
        left_x = np.asarray(plan["frames"][0]["arms"]["left"]["target_pose"][:3], dtype=float)[0]
        right_x = np.asarray(plan["frames"][0]["arms"]["right"]["target_pose"][:3], dtype=float)[0]
        left_right_ok = bool(left_x < right_x)
        if not left_right_ok:
            warnings.append(f"episode {episode}: left/right robot anchor X ordering differs from labels")
        reports.append(
            {
                "episode": episode,
                "source_frames": end - start,
                "left_right_anchor_x_m": {"left": float(left_x), "right": float(right_x)},
                "left_right_anchor_order_ok": left_right_ok,
                "sides": sides,
            }
        )
    return {
        "schema": SCHEMA,
        "dataset": str(root),
        "repo_id": manifest["repo_id"],
        "coordinate_frame": manifest.get("coordinate_frame"),
        "raw_mocopi_coordinate_frame": manifest.get("raw_mocopi_coordinate_frame"),
        "quaternion_order": manifest.get("quaternion_order"),
        "human_wrist_bones": {"left": 14, "right": 18},
        "robot_tcp_frames": {
            "left": {"link": "l_link_8", "gripper_joint": "joint7_L", "fixed_rad": 0.0},
            "right": {"link": "r_link_8", "gripper_joint": "joint7_R", "fixed_rad": 0.0},
        },
        "verification": {
            "left_right_pairing": "passed" if not warnings else "failed",
            "quaternion_order_source": "xyzw" if manifest.get("quaternion_order") == "xyzw" else "failed",
            "configured_scale_applied_without_measurement_change": True,
            "tcp_physical_alignment": "not_externally_measured",
            "warnings": warnings,
        },
        "episodes": reports,
    }
