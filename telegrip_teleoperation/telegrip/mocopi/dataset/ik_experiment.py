"""Controlled IK comparison experiments and thesis-oriented metrics."""

import csv
import json
from copy import deepcopy
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from ..full_mujoco import FullMujocoSimulation
from ..geometry import world_arm_landmarks, world_hands
from ..retarget import alignment_rotation
from .fingerprint import dataset_fingerprint, file_sha256
from .replay import replay_sim
from .retarget_validation import _mocopi_sample

MODE_FOLDERS = {
    "default": "default",
    "position-only": "position_only",
    "hand-6dof": "hand_6dof",
    "hand-6dof-upper-arm": "hand_6dof_upper_arm",
}
SIDES = ("left", "right")
SCHEMA = "dual_scorpion.mocopi.ik_comparison.v1"


def _atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temp.replace(path)


def _stats(values):
    array = np.asarray([value for value in values if value is not None and np.isfinite(value)], dtype=float)
    missing = len(values) - len(array)
    if not len(array):
        return {
            "count": 0,
            "missing": missing,
            "mean": None,
            "median": None,
            "std": None,
            "rmse": None,
            "p95": None,
            "max": None,
            "min": None,
        }
    return {
        "count": len(array),
        "missing": missing,
        "mean": float(np.mean(array)),
        "median": float(np.median(array)),
        "std": float(np.std(array)),
        "rmse": float(np.sqrt(np.mean(np.square(array)))),
        "p95": float(np.quantile(array, 0.95)),
        "max": float(np.max(array)),
        "min": float(np.min(array)),
    }


def _unit(vector):
    vector = np.asarray(vector, dtype=float)
    length = np.linalg.norm(vector)
    return vector / length if np.isfinite(length) and length > 1e-8 else None


def _angle_deg(first, second):
    first, second = _unit(first), _unit(second)
    if first is None or second is None:
        return None
    return float(np.rad2deg(np.arccos(np.clip(np.dot(first, second), -1.0, 1.0))))


def _plane_normal(shoulder, elbow, hand):
    upper, forearm = elbow - shoulder, hand - elbow
    normal = _unit(np.cross(upper, forearm))
    return normal


def _swivel_deg(shoulder, elbow, hand):
    axis = _unit(hand - shoulder)
    if axis is None:
        return None
    reference = np.asarray([0.0, 0.0, 1.0]) - axis * np.dot([0.0, 0.0, 1.0], axis)
    reference = _unit(reference)
    elbow_vector = elbow - shoulder
    elbow_vector = _unit(elbow_vector - axis * np.dot(elbow_vector, axis))
    if reference is None or elbow_vector is None:
        return None
    return float(
        np.rad2deg(
            np.arctan2(np.dot(np.cross(reference, elbow_vector), axis), np.dot(reference, elbow_vector))
        )
    )


def _angle_difference_deg(first, second):
    return None if first is None or second is None else float((first - second + 180.0) % 360.0 - 180.0)


def _collision_kind(body_pairs):
    self_count = cross_count = obstacle_count = 0
    for pair, count in body_pairs.items():
        left, _, right = pair.partition("|")
        if "obstacle" in pair:
            obstacle_count += int(count)
        elif (left.startswith("l_") and right.startswith("l_")) or (
            left.startswith("r_") and right.startswith("r_")
        ):
            self_count += int(count)
        elif {left[:1], right[:1]} == {"l", "r"}:
            cross_count += int(count)
        else:
            self_count += int(count)
    return self_count, cross_count, obstacle_count


def _failure_kind(arm, failure_reason=""):
    research = arm.get("research_ik") or {}
    if research.get("failure_kind"):
        return research["failure_kind"]
    status = str(arm.get("ik_status") or failure_reason)
    if not arm.get("valid"):
        if "orientation" in status.lower():
            return "orientation_error"
        if "not reachable" in status.lower():
            return "unreachable"
        if "joint jump" in status.lower():
            return "joint_jump"
        if "joint limits" in status.lower():
            return "joint_limit"
        return "hold"
    return None


def _evaluate_plan(config, dataset_path, plan, report):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = Path(dataset_path).resolve()
    manifest = json.loads((root / "meta" / "mocopi_dataset.json").read_text(encoding="utf-8"))
    dataset = LeRobotDataset(manifest["repo_id"], root=root, video_backend="pyav")
    episode = int(plan["episode"])
    episode_meta = dataset.meta.episodes[episode]
    episode_start = int(episode_meta["dataset_from_index"])
    initial_source = int(plan["frames"][0]["source_frame"])
    initial_sample = _mocopi_sample(dataset.hf_dataset[episode_start + initial_source]["observation.state"])
    retarget = plan["retarget"]
    rotation, alignment_source = alignment_rotation(
        initial_sample.bones[10],
        initial_sample,
        retarget["world_to_robot_rotation"],
        retarget["alignment_mode"],
    )
    sim_config = {
        **config["robot"],
        "start_pose": plan.get("start_pose", config["robot"].get("retarget_start_pose", "neutral")),
    }
    simulation = FullMujocoSimulation(sim_config, gui=False, obstacles=plan.get("obstacles", []))
    initial_offsets = {}
    failure_map = {
        (int(failure["frame"]), failure["side"]): failure
        for failure in plan.get("failures", [])
        if "side" in failure and "frame" in failure
    }
    try:
        for side, shoulder_bone in (("left", 12), ("right", 16)):
            fk = simulation.arms[side].solver.fk_solver
            initial_joints = np.asarray(plan["frames"][0]["arms"][side]["joint_deg"], dtype=float)
            _, _, robot_upper = fk.arm_state(np.r_[initial_joints, 0.0])
            human_upper = rotation @ initial_sample.bones[shoulder_bone][:3, :3]
            initial_offsets[side] = robot_upper @ human_upper.T

        rows, joint_rows = [], []
        joint_series = {side: [] for side in SIDES}
        joint_times = []
        for frame_index, frame in enumerate(plan["frames"]):
            source_index = episode_start + int(frame["source_frame"])
            sample = _mocopi_sample(dataset.hf_dataset[source_index]["observation.state"])
            landmarks = world_arm_landmarks(sample.bones[10], sample)
            hands = world_hands(sample.bones[10], sample)
            frame_joints = {side: np.asarray(frame["arms"][side]["joint_deg"], dtype=float) for side in SIDES}
            for side in SIDES:
                simulation.data.qpos[simulation.qpos_indices[side][:7]] = np.deg2rad(frame_joints[side])
                simulation.data.qpos[simulation.qpos_indices[side][7]] = 0
            simulation.mujoco.mj_forward(simulation.model, simulation.data)
            self_collision, cross_collision, obstacle_collision = _collision_kind(
                simulation.collision_report().get("body_pairs", {})
            )
            joint_frame = {
                "mode": plan["ik_mode"],
                "episode": episode,
                "plan_frame": frame_index,
                "source_frame": int(frame["source_frame"]),
                "time_s": float(frame["time_s"]),
            }
            joint_times.append(float(frame["time_s"]))
            for side, shoulder_bone in (("left", 12), ("right", 16)):
                arm = frame["arms"][side]
                failure = failure_map.get((frame_index, side), {})
                failure_kind = _failure_kind(arm, failure.get("reason", ""))
                research = arm.get("research_ik") or {}
                joints = np.asarray(arm["joint_deg"], dtype=float)
                fk = simulation.arms[side].solver.fk_solver
                arm_bounds = simulation.arms[side]
                robot_shoulder, robot_elbow, robot_upper = fk.arm_state(np.r_[joints, 0.0])
                robot_tcp, robot_quaternion = fk.compute(np.r_[joints, 0.0])
                target = np.asarray(arm["target_pose"], dtype=float)
                position_error_mm = float(np.linalg.norm(target[:3] - robot_tcp) * 1000)
                orientation_error_deg = float(
                    np.rad2deg(
                        Rotation.from_quat(target[3:])
                        .inv()
                        .__mul__(Rotation.from_quat(robot_quaternion))
                        .magnitude()
                    )
                )
                human_shoulder = rotation @ landmarks[f"{side}_shoulder"][:3, 3]
                human_elbow = rotation @ landmarks[f"{side}_elbow"][:3, 3]
                human_hand = rotation @ hands[side][:3, 3]
                human_upper = human_elbow - human_shoulder
                robot_upper_vector = robot_elbow - robot_shoulder
                shoulder_elbow_error = _angle_deg(human_upper, robot_upper_vector)
                arm_plane_error = _angle_deg(
                    _plane_normal(human_shoulder, human_elbow, human_hand),
                    _plane_normal(robot_shoulder, robot_elbow, robot_tcp),
                )
                human_swivel = _swivel_deg(human_shoulder, human_elbow, human_hand)
                robot_swivel = _swivel_deg(robot_shoulder, robot_elbow, robot_tcp)
                swivel_error = _angle_difference_deg(human_swivel, robot_swivel)
                mapped_human_upper = rotation @ sample.bones[shoulder_bone][:3, :3]
                expected_upper = initial_offsets[side] @ mapped_human_upper
                upper_rotation_error = float(
                    np.rad2deg(Rotation.from_matrix(expected_upper @ robot_upper.T).magnitude())
                )
                row = {
                    "mode": plan["ik_mode"],
                    "episode": episode,
                    "plan_frame": frame_index,
                    "source_frame": int(frame["source_frame"]),
                    "time_s": float(frame["time_s"]),
                    "side": side,
                    "ik_success": bool(arm.get("valid")),
                    "failure_kind": failure_kind,
                    "hold": not bool(arm.get("valid")),
                    "fallback": bool(
                        arm.get("fallback", (arm.get("research_ik") or {}).get("fallback", False))
                    ),
                    "position_error_mm": position_error_mm,
                    "orientation_error_deg": orientation_error_deg,
                    "shoulder_elbow_angle_error_deg": shoulder_elbow_error,
                    "arm_plane_angle_error_deg": arm_plane_error,
                    "human_swivel_deg": human_swivel,
                    "robot_swivel_deg": robot_swivel,
                    "swivel_angle_error_deg": swivel_error,
                    "upper_arm_orientation_error_deg": upper_rotation_error,
                    "posture_tcp_position_delta_mm": float(
                        research.get("posture_tcp_position_delta_m") or 0.0
                    )
                    * 1000,
                    "posture_tcp_orientation_delta_deg": float(
                        research.get("posture_tcp_orientation_delta_deg") or 0.0
                    ),
                    "joint_limit_margin_deg": float(
                        min(
                            np.min(joints - arm_bounds.lower[:7]),
                            np.min(arm_bounds.upper[:7] - joints),
                        )
                    ),
                    "elbow_pose_different": bool(
                        shoulder_elbow_error is not None and shoulder_elbow_error > 10.0
                    ),
                    "self_collision_count": self_collision,
                    "cross_arm_collision_count": cross_collision,
                    "obstacle_collision_count": obstacle_collision,
                }
                rows.append(row)
                joint_frame[f"{side}_joint_deg"] = joints.tolist()
                joint_series[side].append(joints)
            joint_rows.append(joint_frame)
    finally:
        simulation.close()

    times = np.asarray(joint_times, dtype=float)
    velocities, accelerations = dict.fromkeys(SIDES), dict.fromkeys(SIDES)
    if len(times) >= 2:
        dt = np.diff(times)
        for side in SIDES:
            values = np.asarray(joint_series[side], dtype=float)
            velocity = np.zeros_like(values)
            velocity[1:] = np.divide(
                np.diff(values, axis=0),
                dt[:, None],
                out=np.zeros_like(velocity[1:]),
                where=dt[:, None] > 0,
            )
            acceleration = np.zeros_like(velocity)
            acceleration[1:] = np.divide(
                np.diff(velocity, axis=0),
                dt[:, None],
                out=np.zeros_like(acceleration[1:]),
                where=dt[:, None] > 0,
            )
            velocities[side] = velocity
            accelerations[side] = acceleration
    for joint_index in range(7):
        for row_index, row in enumerate(joint_rows):
            for side in SIDES:
                row[f"{side}_joint{joint_index}_speed_deg_s"] = (
                    float(velocities[side][row_index, joint_index]) if velocities[side] is not None else None
                )
                row[f"{side}_joint{joint_index}_acceleration_deg_s2"] = (
                    float(accelerations[side][row_index, joint_index])
                    if accelerations[side] is not None
                    else None
                )

    summaries = {}
    for side in SIDES:
        side_rows = [row for row in rows if row["side"] == side]
        success_rows = [row for row in side_rows if row["ik_success"]]
        speeds = np.abs(velocities[side]).reshape(-1).tolist() if velocities[side] is not None else []
        accelerations_side = (
            np.abs(accelerations[side]).reshape(-1).tolist() if accelerations[side] is not None else []
        )
        reversal_count = 0
        if velocities[side] is not None:
            velocity = velocities[side]
            for joint_index in range(velocity.shape[1]):
                series = velocity[:, joint_index]
                for before, after in zip(series, series[1:], strict=False):
                    if abs(before) >= 15.0 and abs(after) >= 15.0 and np.sign(before) != np.sign(after):
                        reversal_count += 1
        summaries[side] = {
            "frames": len(side_rows),
            "ik_successes": len(success_rows),
            "ik_success_rate": len(success_rows) / len(side_rows) if side_rows else None,
            "hold_count": sum(row["hold"] for row in side_rows),
            "fallback_count": sum(row["fallback"] for row in side_rows),
            "unreachable_count": sum(row["failure_kind"] == "unreachable" for row in side_rows),
            "unreachable_rate": (
                sum(row["failure_kind"] == "unreachable" for row in side_rows) / len(side_rows)
                if side_rows
                else None
            ),
            "hold_rate": (sum(row["hold"] for row in side_rows) / len(side_rows) if side_rows else None),
            "fallback_rate": (
                sum(row["fallback"] for row in side_rows) / len(side_rows) if side_rows else None
            ),
            "orientation_failure_count": sum(
                row["failure_kind"] in ("orientation_error", "posture_orientation_error") for row in side_rows
            ),
            "orientation_failure_rate": (
                sum(
                    row["failure_kind"] in ("orientation_error", "posture_orientation_error")
                    for row in side_rows
                )
                / len(side_rows)
                if side_rows
                else None
            ),
            "elbow_pose_different_frames": sum(row["elbow_pose_different"] for row in success_rows),
            "self_collision_contacts": sum(row["self_collision_count"] for row in side_rows),
            "cross_arm_collision_contacts": sum(row["cross_arm_collision_count"] for row in side_rows),
            "obstacle_collision_contacts": sum(row["obstacle_collision_count"] for row in side_rows),
            "joint_reversal_count": reversal_count,
            "stats": {
                "tcp_position_error_mm": _stats([row["position_error_mm"] for row in side_rows]),
                "tcp_orientation_error_deg": _stats([row["orientation_error_deg"] for row in side_rows]),
                "shoulder_elbow_angle_error_deg": _stats(
                    [row["shoulder_elbow_angle_error_deg"] for row in success_rows]
                ),
                "arm_plane_angle_error_deg": _stats(
                    [row["arm_plane_angle_error_deg"] for row in success_rows]
                ),
                "swivel_angle_error_deg": _stats(
                    [
                        abs(row["swivel_angle_error_deg"])
                        for row in success_rows
                        if row["swivel_angle_error_deg"] is not None
                    ]
                ),
                "upper_arm_orientation_error_deg": _stats(
                    [row["upper_arm_orientation_error_deg"] for row in success_rows]
                ),
                "joint_limit_margin_deg": _stats([row["joint_limit_margin_deg"] for row in side_rows]),
                "posture_tcp_position_delta_mm": _stats(
                    [row["posture_tcp_position_delta_mm"] for row in success_rows]
                ),
                "posture_tcp_orientation_delta_deg": _stats(
                    [row["posture_tcp_orientation_delta_deg"] for row in success_rows]
                ),
                "joint_speed_abs_deg_s": _stats(speeds),
                "joint_acceleration_abs_deg_s2": _stats(accelerations_side),
            },
        }
    left_rows = [row for row in rows if row["side"] == "left"]
    collision_totals = {
        "self_collision_contacts": sum(row["self_collision_count"] for row in left_rows),
        "cross_arm_collision_contacts": sum(row["cross_arm_collision_count"] for row in left_rows),
        "obstacle_collision_contacts": sum(row["obstacle_collision_count"] for row in left_rows),
    }
    return {
        "schema": "dual_scorpion.mocopi.ik_mode_evaluation.v1",
        "mode": plan["ik_mode"],
        "episode": episode,
        "frames": len(plan["frames"]),
        "fps": plan["fps"],
        "alignment_source": alignment_source,
        "mocopi_elbow_is_estimated": True,
        "joint_limit_source": "full_mjcf_provisional_limits_not_measured_on_hardware",
        "arm_posture": plan["arm_posture"],
        "orientation_tolerance_deg": plan["ik_orientation_tolerance_deg"],
        "report_feasibility": report.get("feasibility"),
        "sides": summaries,
        "collision_totals": collision_totals,
        "per_frame": rows,
        "joint_trajectory": joint_rows,
    }


def _write_csv(path, rows):
    path = Path(path)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _summary_rows(results):
    rows = []
    for mode in MODE_FOLDERS:
        evaluation = results.get(mode)
        if evaluation is None:
            continue
        for side in SIDES:
            side_data = evaluation["sides"][side]
            for metric, stats in side_data["stats"].items():
                row = {"mode": mode, "side": side, "metric": metric, **stats}
                rows.append(row)
            for metric in (
                "frames",
                "ik_successes",
                "ik_success_rate",
                "hold_count",
                "fallback_count",
                "unreachable_count",
                "unreachable_rate",
                "hold_rate",
                "fallback_rate",
                "orientation_failure_count",
                "orientation_failure_rate",
                "elbow_pose_different_frames",
                "self_collision_contacts",
                "cross_arm_collision_contacts",
                "obstacle_collision_contacts",
                "joint_reversal_count",
            ):
                rows.append({"mode": mode, "side": side, "metric": metric, "value": side_data[metric]})
        for metric, value in evaluation["collision_totals"].items():
            rows.append({"mode": mode, "side": "both", "metric": metric, "value": value})
    return rows


def _fairness_check(plans, results, orientation_tolerance_deg):
    """Verify that B and C differ only in the upper-arm posture objective."""
    baseline = plans.get("hand-6dof")
    posture = plans.get("hand-6dof-upper-arm")
    if baseline is None or posture is None:
        return {
            "available": False,
            "reason": "Both hand-6dof and hand-6dof-upper-arm must be run",
        }
    frames_match = len(baseline["frames"]) == len(posture["frames"]) and all(
        int(first["source_frame"]) == int(second["source_frame"])
        for first, second in zip(baseline["frames"], posture["frames"], strict=True)
    )
    target_diff = 0.0
    initial_joint_diff = 0.0
    joint_diffs = {side: [] for side in SIDES}
    if frames_match:
        for first, second in zip(baseline["frames"], posture["frames"], strict=True):
            for side in SIDES:
                target_diff = max(
                    target_diff,
                    float(
                        np.max(
                            np.abs(
                                np.asarray(first["arms"][side]["target_pose"], dtype=float)
                                - np.asarray(second["arms"][side]["target_pose"], dtype=float)
                            )
                        )
                    ),
                )
                joint_diffs[side].append(
                    np.abs(
                        np.asarray(first["arms"][side]["joint_deg"], dtype=float)
                        - np.asarray(second["arms"][side]["joint_deg"], dtype=float)
                    )
                )
        for side in SIDES:
            initial_joint_diff = max(
                initial_joint_diff,
                float(
                    np.max(
                        np.abs(
                            np.asarray(baseline["frames"][0]["arms"][side]["joint_deg"], dtype=float)
                            - np.asarray(posture["frames"][0]["arms"][side]["joint_deg"], dtype=float)
                        )
                    )
                ),
            )
    postures = {
        "hand-6dof": baseline["retarget"]["arm_posture"],
        "hand-6dof-upper-arm": posture["retarget"]["arm_posture"],
    }
    limits = {}
    for key in (
        "max_translation_m",
        "position_tolerance_m",
        "preview_max_joint_step_deg",
        "hardware_max_joint_step_deg",
        "ik_orientation_weight",
        "ik_orientation_tolerance_deg",
        "mujoco_model_sha256",
        "start_pose",
    ):
        limits[key] = {
            "hand-6dof": baseline.get(key, baseline["retarget"].get(key)),
            "hand-6dof-upper-arm": posture.get(key, posture["retarget"].get(key)),
        }
    same_limits = all(
        np.isclose(first, second, equal_nan=True)
        if isinstance(first, (int, float)) and isinstance(second, (int, float))
        else first == second
        for pair in limits.values()
        for first, second in [tuple(pair.values())]
    )
    c_rows = [row for row in results["hand-6dof-upper-arm"]["per_frame"] if row["ik_success"]]
    position_tolerance_m = float(
        baseline.get("position_tolerance_m", baseline["retarget"].get("position_tolerance_m", 0.025))
    )
    c_position_ok = not c_rows or all(
        float(row["position_error_mm"]) / 1000.0 <= position_tolerance_m + 1e-9 for row in c_rows
    )
    c_orientation_ok = not c_rows or all(
        float(row["orientation_error_deg"]) <= orientation_tolerance_deg + 1e-9 for row in c_rows
    )
    c_posture_deltas = [float(row["posture_tcp_position_delta_mm"]) for row in c_rows]
    max_posture_delta_mm = max(c_posture_deltas) if c_posture_deltas else 0.0
    all_joint_diffs = (
        np.stack([value for side in SIDES for value in joint_diffs[side]], axis=0)
        if frames_match
        else np.zeros((0, 7))
    )
    return {
        "available": True,
        "same_source_frames": bool(frames_match),
        "max_target_pose_abs_diff": target_diff,
        "same_tcp_target": bool(frames_match and target_diff <= 1e-12),
        "max_initial_joint_abs_diff_deg": initial_joint_diff,
        "same_initial_joint_pose": bool(frames_match and initial_joint_diff <= 1e-9),
        "arm_posture": postures,
        "limits_and_motion": limits,
        "same_limits_and_motion": bool(same_limits),
        "position_tolerance_m": position_tolerance_m,
        "orientation_tolerance_deg": orientation_tolerance_deg,
        "c_success_frames": len(c_rows),
        "c_failure_frames": len(results["hand-6dof-upper-arm"]["per_frame"]) - len(c_rows),
        "c_success_frames_within_tcp_tolerance": bool(c_position_ok),
        "c_success_frames_within_orientation_tolerance": bool(c_orientation_ok),
        "max_posture_tcp_position_delta_mm": max_posture_delta_mm,
        "c_posture_correction_within_0p5_mm": bool(max_posture_delta_mm <= 0.5 + 1e-9),
        "joint_trajectory_difference_deg": {
            "mean_abs": float(np.mean(all_joint_diffs)) if all_joint_diffs.size else None,
            "max_abs": float(np.max(all_joint_diffs)) if all_joint_diffs.size else None,
            "rms": float(np.sqrt(np.mean(np.square(all_joint_diffs)))) if all_joint_diffs.size else None,
            "per_side_mean_abs": {
                side: float(np.mean(np.stack(joint_diffs[side], axis=0))) if joint_diffs[side] else None
                for side in SIDES
            },
        },
    }


def run_ik_comparison(
    config,
    dataset_path,
    output,
    *,
    episode=0,
    modes=None,
    preview_max_step_deg=None,
    slowdown=1,
    orientation_tolerance_deg=None,
    obstacles=None,
):
    """Run each IK mode on the same episode and write reproducible comparison artifacts."""
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(f"Experiment output already exists: {output}")
    modes = list(MODE_FOLDERS) if modes is None else modes
    if any(mode not in MODE_FOLDERS for mode in modes):
        raise ValueError(f"Unknown IK modes: {sorted(set(modes) - set(MODE_FOLDERS))}")
    effective_orientation_tolerance = float(
        orientation_tolerance_deg
        if orientation_tolerance_deg is not None
        else config["robot"].get("ik_orientation_tolerance_deg", 15.0)
    )
    if not np.isfinite(effective_orientation_tolerance) or effective_orientation_tolerance <= 0:
        raise ValueError("orientation_tolerance_deg must be positive")
    dataset_root = Path(dataset_path).resolve()
    manifest_path = dataset_root / "meta" / "mocopi_dataset.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    output.mkdir(parents=True)
    metadata = {
        "schema": SCHEMA,
        "dataset": str(dataset_root),
        "dataset_manifest_sha256": file_sha256(manifest_path),
        "dataset_content_sha256": dataset_fingerprint(dataset_root),
        "repo_id": manifest["repo_id"],
        "episode": episode,
        "modes": modes,
        "primary_comparison": ["hand-6dof", "hand-6dof-upper-arm"],
        "preview_max_joint_step_deg": preview_max_step_deg,
        "slowdown": slowdown,
        "obstacles": str(Path(obstacles).resolve()) if obstacles else None,
        "orientation_tolerance_deg": effective_orientation_tolerance,
        "mode_conditions": {
            mode: {
                "arm_posture": (
                    config["retarget"]["arm_posture"]
                    if mode == "default"
                    else "upper-arm"
                    if mode == "hand-6dof-upper-arm"
                    else "hand-only"
                ),
                "orientation_enabled": (
                    config["retarget"].get("orientation_enabled", True)
                    if mode == "default"
                    else mode != "position-only"
                ),
                "strict_orientation_tolerance_deg": (
                    None if mode in ("default", "position-only") else effective_orientation_tolerance
                ),
            }
            for mode in modes
        },
        "retarget": config["retarget"],
        "ik": {
            "orientation_weight": config["robot"].get("ik_orientation_weight", 0.08),
            "position_tolerance_m": config["robot"]["position_tolerance_m"],
            "hardware_max_joint_step_deg": config["robot"]["max_joint_step_deg"],
        },
    }
    _atomic_json(output / "metadata.json", metadata)
    results = {}
    plans = {}
    for mode in modes:
        print(f"Running IK comparison mode: {mode}")
        mode_dir = output / MODE_FOLDERS[mode]
        mode_dir.mkdir()
        plan, report = replay_sim(
            deepcopy(config),
            dataset_root,
            episode,
            gui=False,
            realtime=False,
            show_camera=False,
            slowdown=slowdown,
            sim_model="full",
            preview_max_step_deg=preview_max_step_deg,
            obstacles=obstacles,
            ik_mode=mode,
            orientation_tolerance_deg=effective_orientation_tolerance,
        )
        plans[mode] = plan
        _atomic_json(mode_dir / "plan.json", plan)
        _atomic_json(mode_dir / "report.json", report)
        evaluation = _evaluate_plan(config, dataset_root, plan, report)
        evaluation["plan_sha256"] = file_sha256(mode_dir / "plan.json")
        evaluation["report_sha256"] = file_sha256(mode_dir / "report.json")
        _atomic_json(mode_dir / "evaluation.json", evaluation)
        _write_csv(mode_dir / "per_frame.csv", evaluation["per_frame"])
        _write_csv(mode_dir / "joint_trajectory.csv", evaluation["joint_trajectory"])
        results[mode] = evaluation

    fairness = _fairness_check(plans, results, effective_orientation_tolerance)
    metadata["fairness_check"] = fairness
    metadata["mujoco_model_sha256"] = plans[modes[0]].get("mujoco_model_sha256")
    metadata["joint_limit_source"] = "full_mjcf_provisional_limits_not_measured_on_hardware"
    _atomic_json(output / "metadata.json", metadata)

    comparison = output / "comparison"
    comparison.mkdir()
    _atomic_json(comparison / "fairness_check.json", fairness)
    summary_rows = _summary_rows(results)
    _write_csv(comparison / "summary.csv", summary_rows)
    _atomic_json(
        comparison / "summary.json",
        {
            "schema": SCHEMA,
            "dataset": str(dataset_root),
            "episode": episode,
            "primary_comparison": metadata["primary_comparison"],
            "fairness_check": fairness,
            "modes": {mode: {side: results[mode]["sides"][side] for side in SIDES} for mode in modes},
        },
    )
    per_frame = [row for mode in modes for row in results[mode]["per_frame"]]
    _write_csv(comparison / "per_frame_comparison.csv", per_frame)

    from .ik_plots import generate_comparison_plots, generate_comparison_report

    graphs = generate_comparison_plots(output, results)
    generate_comparison_report(output, metadata, results, graphs, fairness)
    return {
        "schema": SCHEMA,
        "output": str(output),
        "modes": modes,
        "episode": episode,
        "graphs": graphs,
    }
