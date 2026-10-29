"""Replay human-pose datasets using a selected robot model, with metrics."""

import json
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from ..full_mujoco import full_model_path
from ..geometry import MocopiSample, pose, world_arm_landmarks, world_hands
from ..runtime import TrackingGate
from .evaluate import _summary
from .fingerprint import dataset_fingerprint, file_sha256
from .session import POSE_BONES


def _file_hash(path):
    return file_sha256(path)


def model_fingerprint(config):
    """Identify the exact per-arm URDFs used for FK/IK, not a hard-coded revision."""
    return {side: _file_hash(path) for side, path in config["robot"]["urdf"].items()}


def _sample(state, timestamp):
    state = np.asarray(state, dtype=float)
    if state.shape != (49,) or not np.isfinite(state).all():
        raise ValueError("Expected 49 finite mocopi pose values")
    bones = {0: np.eye(4)}
    for index, (_, bone) in enumerate(POSE_BONES):
        value = state[index * 7 : (index + 1) * 7]
        bones[bone] = pose(value[:3], value[3:])
    return MocopiSample(timestamp, bones)


def _pose_vector(frame):
    return np.r_[frame[:3, 3], Rotation.from_matrix(frame[:3, :3]).as_quat()].tolist()


def replay_sim(
    config,
    dataset_path,
    episode=0,
    *,
    gui=True,
    realtime=True,
    show_camera=True,
    slowdown=1,
    sim_model="legacy",
):
    """Play an episode, returning a reviewable joint plan and simulation metrics."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    from ..__main__ import anchors

    if sim_model == "full":
        from ..full_mujoco import FullMujocoSimulation as Simulation
    elif sim_model == "legacy":
        from ..mujoco_sim import MujocoSimulation as Simulation
    else:
        raise ValueError("sim_model must be full or legacy")

    if not isinstance(slowdown, int) or not 1 <= slowdown <= 10:
        raise ValueError("slowdown must be an integer from 1 to 10")
    root = Path(dataset_path)
    manifest_path = root / "meta" / "mocopi_dataset.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema") != "dual_scorpion.mocopi.lerobot.v1" or manifest.get("gripper_recorded"):
        raise ValueError("Unsupported mocopi dataset schema")
    content_hash = dataset_fingerprint(root)
    dataset = LeRobotDataset(manifest["repo_id"], root=root, video_backend="pyav")
    if not 0 <= episode < dataset.num_episodes:
        raise ValueError(f"Episode must be between 0 and {dataset.num_episodes - 1}")
    episode_meta = dataset.meta.episodes[episode]
    start, end = int(episode_meta["dataset_from_index"]), int(episode_meta["dataset_to_index"])
    source = manifest["sessions"][manifest["episodes"][episode]["source_session"]]["source"]
    plan = {
        "schema": "dual_scorpion.mocopi.robot_plan.v1",
        "dataset_manifest_sha256": _file_hash(manifest_path),
        "dataset_content_sha256": content_hash,
        "episode": episode,
        "task": episode_meta["tasks"][0],
        "source": source,
        "fps": dataset.fps,
        "slowdown": slowdown,
        "expected_frames": (end - start) * slowdown,
        "model_kind": "full_mujoco" if sim_model == "full" else "legacy_urdf",
        "hardware_replay_allowed": sim_model == "legacy",
        "mujoco_model_sha256": _file_hash(full_model_path(config["robot"])) if sim_model == "full" else None,
        "urdf_sha256": model_fingerprint(config) if sim_model == "legacy" else None,
        "joint_names": [f"joint{i}" for i in range(7)],
        "gripper_included": False,
        "max_joint_step_deg": float(config["robot"]["max_joint_step_deg"]),
        "frames": [],
        "failures": [],
    }
    errors = {
        side: {
            "position_m": [],
            "orientation_deg": [],
            "max_joint_step_deg": [],
            "joint_limit_margin_deg": [],
        }
        for side in ("left", "right")
    }
    simulation = Simulation(config["robot"], gui=gui)
    camera_window = None
    try:
        if gui and show_camera:
            from .video_view import CameraWindow

            camera_window = CameraWindow()
        first = _sample(dataset.hf_dataset[start]["observation.state"], 0)
        head = first.bones[10]
        mappings = anchors(config, simulation, world_hands(head, first), head, first)
        gate = TrackingGate(config["tracking"])
        start_wall = time.monotonic()
        for step in range((end - start) * slowdown):
            index = start + step // slowdown
            if not simulation.is_running():
                plan["failures"].append({"frame": index - start, "reason": "viewer closed"})
                break
            sample = _sample(dataset.hf_dataset[index]["observation.state"], (index - start) / dataset.fps)
            accepted = gate.accept((sample, None), sample.timestamp)
            if accepted is None:
                plan["failures"].append({"frame": index - start, "reason": gate.reason})
                break
            head, hands = accepted
            landmarks = world_arm_landmarks(head, sample)
            targets = {side: mapping.target(hands[side]) for side, mapping in mappings.items()}
            postures = {side: mapping.posture_target(landmarks, side) for side, mapping in mappings.items()}
            previous_joints = {side: arm.last_valid_target.copy() for side, arm in simulation.arms.items()}
            simulation.update(targets, True, postures)
            frame = {"time_s": step / dataset.fps, "source_frame": index - start, "arms": {}}
            for side, arm in simulation.arms.items():
                position, quaternion = arm.solver.fk_solver.compute(arm.last_valid_target)
                actual = pose(position, quaternion)
                position_error = float(np.linalg.norm(position - targets[side][:3, 3]))
                orientation_error = float(
                    np.rad2deg(Rotation.from_matrix(targets[side][:3, :3] @ actual[:3, :3].T).magnitude())
                )
                errors[side]["position_m"].append(position_error)
                errors[side]["orientation_deg"].append(orientation_error)
                errors[side]["max_joint_step_deg"].append(
                    float(np.max(np.abs(arm.last_valid_target[:7] - previous_joints[side][:7])))
                )
                if sim_model == "legacy":
                    errors[side]["joint_limit_margin_deg"].append(
                        float(
                            np.min(
                                np.r_[
                                    arm.last_valid_target[:7] - arm.lower[:7],
                                    arm.upper[:7] - arm.last_valid_target[:7],
                                ]
                            )
                        )
                    )
                if not arm.valid:
                    plan["failures"].append({"frame": index - start, "side": side, "reason": arm.status})
                frame["arms"][side] = {
                    "joint_deg": arm.last_valid_target[:7].tolist(),
                    "target_pose": _pose_vector(targets[side]),
                    "simulated_pose": _pose_vector(actual),
                    "valid": arm.valid,
                    "workspace_clamped": bool(arm.solver.last_solve_info.get("workspace_clamped", False)),
                    "retarget_clamped": bool(
                        np.linalg.norm(
                            mappings[side].scale
                            * mappings[side].rotation
                            @ (hands[side][:3, 3] - mappings[side].human[:3, 3])
                        )
                        > mappings[side].limit
                    ),
                }
            plan["frames"].append(frame)
            if camera_window:
                image = dataset[index]["observation.images.head"].permute(1, 2, 0).numpy()
                if not camera_window.show(image):
                    plan["failures"].append({"frame": index - start, "reason": "user stopped playback"})
                    break
            if realtime:
                time.sleep(max(0, start_wall + (step + 1) / dataset.fps - time.monotonic()))
    finally:
        if camera_window:
            camera_window.close()
        simulation.close()
    if dataset_fingerprint(root) != content_hash:
        raise ValueError("Source dataset changed during replay; create a new plan")
    report = {
        "schema": "dual_scorpion.mocopi.simulation_metrics.v1",
        "frames": len(plan["frames"]),
        "requested_frames": (end - start) * slowdown,
        "source_frames": end - start,
        "slowdown": slowdown,
        "fps": dataset.fps,
        "source": source,
        "episode": episode,
        "task": plan["task"],
        "model_kind": plan["model_kind"],
        "hardware_replay_allowed": plan["hardware_replay_allowed"],
        "joint_limit_source": "unmeasured" if sim_model == "full" else "legacy_urdf",
        "mujoco_model_sha256": plan["mujoco_model_sha256"],
        "urdf_sha256": plan["urdf_sha256"],
        "ik_failures": plan["failures"],
        "tracking": {
            side: {key: _summary(values) for key, values in info.items()} for side, info in errors.items()
        },
        # The full-model preview has no reachable-workspace clamp stage, so an
        # aggregate of zero would be read as "no workspace violations". Report
        # "not evaluated" instead; reachability shows up there as IK failure or
        # position error.
        "workspace_clamp_measured": sim_model == "legacy",
        "workspace_clamped_frames": None
        if sim_model == "full"
        else {
            side: sum(frame["arms"][side]["workspace_clamped"] for frame in plan["frames"])
            for side in ("left", "right")
        },
        "retarget_clamped_frames": {
            side: sum(frame["arms"][side]["retarget_clamped"] for frame in plan["frames"])
            for side in ("left", "right")
        },
        "limits": "Kinematic FK comparison only; does not measure physical robot error or collisions",
    }
    return plan, report
