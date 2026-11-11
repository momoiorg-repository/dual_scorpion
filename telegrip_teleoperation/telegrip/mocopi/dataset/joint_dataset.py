"""Derive a distinctly labelled kinematic-joint dataset from reviewed plans."""

import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from ..ik import DualSimulation
from .fingerprint import dataset_fingerprint, file_sha256
from .real import JOINTS, plan_hash, validate_plan
from .retarget_validation import validate_retargeting

# The reference follower dataset uses right then left. Grippers are absent.
JOINT_SIDES = ("right", "left")
JOINT_NAMES = [f"{side}_{name}.pos" for side in JOINT_SIDES for name in JOINTS]
TCP_NAMES = [
    f"{side}_tcp.{component}"
    for side in JOINT_SIDES
    for component in ("x_m", "y_m", "z_m", "qx", "qy", "qz", "qw")
]
TCP_ERROR_NAMES = [
    f"{side}_{metric}" for side in JOINT_SIDES for metric in ("position_error_m", "orientation_error_deg")
]
ROBOT_STATUS_NAMES = [
    f"{side}_{metric}"
    for side in JOINT_SIDES
    for metric in ("valid", "workspace_clamped", "retarget_clamped")
]
ROBOT_WARNING_NAMES = [f"{side}_orientation_warning" for side in JOINT_SIDES]


def _joint_vector(frame):
    return np.concatenate(
        [np.asarray(frame["arms"][side]["joint_deg"], dtype=np.float32) for side in JOINT_SIDES]
    )


def _tcp_vector(frame, key):
    values = []
    for side in JOINT_SIDES:
        value = np.asarray(frame["arms"][side][key], dtype=np.float32)
        if value.shape != (7,) or not np.isfinite(value).all():
            raise ValueError(f"Invalid {key} pose for {side}")
        values.append(value)
    return np.concatenate(values)


def _tcp_error_vector(frame):
    values = []
    for side in JOINT_SIDES:
        target = np.asarray(frame["arms"][side]["target_pose"], dtype=float)
        actual = np.asarray(frame["arms"][side]["simulated_pose"], dtype=float)
        position_error = float(np.linalg.norm(target[:3] - actual[:3]))
        rotation_error = float(
            np.rad2deg(
                Rotation.from_matrix(
                    Rotation.from_quat(target[3:]).as_matrix().T @ Rotation.from_quat(actual[3:]).as_matrix()
                ).magnitude()
            )
        )
        values.extend((position_error, rotation_error))
    return np.asarray(values, dtype=np.float32)


def _status_vector(frame):
    values = []
    for side in JOINT_SIDES:
        arm = frame["arms"][side]
        values.extend(
            (
                float(arm.get("valid") is True),
                float(arm.get("workspace_clamped") is True),
                float(arm.get("retarget_clamped") is True),
            )
        )
    return np.asarray(values, dtype=np.float32)


def _source_feature(value, feature):
    shape = tuple(feature.get("shape", ()))
    array = np.asarray(value)
    if shape == (1,):
        return np.asarray([array.item()], dtype=np.dtype(feature["dtype"]))
    return array.reshape(shape).astype(np.dtype(feature["dtype"]))


def _contiguous_ranges(indices):
    ranges = []
    for index in sorted(set(indices)):
        if ranges and index == ranges[-1][1] + 1:
            ranges[-1][1] = index
        else:
            ranges.append([index, index])
    return [{"start_frame": start, "end_frame": end} for start, end in ranges]


def export_joint_dataset(
    config,
    dataset_path,
    plan_paths,
    output,
    repo_id,
    *,
    max_sim_error_m=0.025,
    max_sim_orientation_error_deg=None,
    report_paths=None,
):
    """Create a LeRobot dataset of simulated joints and recorded human video.

    The robot state at frame t is the kinematic configuration in the plan;
    action is the next planned configuration. Neither is a motor reading.
    """
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    source_root, output_root = Path(dataset_path).resolve(), Path(output).resolve()
    if not plan_paths or not repo_id.strip():
        raise ValueError("At least one plan and a nonempty repo_id are required")
    if output_root.exists():
        raise FileExistsError(f"Dataset output already exists: {output_root}")
    if output_root.is_relative_to(source_root) or source_root.is_relative_to(output_root):
        raise ValueError("Source and output datasets must be in separate directories")
    manifest_path = source_root / "meta" / "mocopi_dataset.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema") != "dual_scorpion.mocopi.lerobot.v1" or manifest.get("gripper_recorded"):
        raise ValueError("Unsupported mocopi dataset schema")
    fingerprint = dataset_fingerprint(source_root)
    manifest_hash = file_sha256(manifest_path)
    source = LeRobotDataset(manifest["repo_id"], root=source_root, video_backend="pyav")
    plans = [json.loads(Path(path).read_text(encoding="utf-8")) for path in plan_paths]
    if report_paths is not None and len(report_paths) != len(plans):
        raise ValueError("Each plan must have a corresponding report or no reports")
    model_kinds = {plan.get("model_kind", "legacy_urdf") for plan in plans}
    if len(model_kinds) != 1:
        raise ValueError("All plans must use the same simulation model")
    full_model = model_kinds == {"full_mujoco"}
    obstacle_hashes = {plan.get("obstacles_sha256") for plan in plans}
    if len(obstacle_hashes) != 1:
        raise ValueError("All plans must use the same obstacle scene")
    episodes, sources, seen = [], set(), set()
    if full_model:
        from ..full_mujoco import FullMujocoSimulation

        simulation = FullMujocoSimulation(config["robot"], gui=False, obstacles=plans[0].get("obstacles", []))
    else:
        simulation = DualSimulation(config["robot"], gui=False)
    try:
        for plan in plans:
            if (
                plan.get("dataset_content_sha256") != fingerprint
                or plan.get("dataset_manifest_sha256") != manifest_hash
            ):
                raise ValueError("Plan does not match dataset content; regenerate the plan from this dataset")
            episode = plan.get("episode")
            if (
                not isinstance(episode, int)
                or isinstance(episode, bool)
                or not 0 <= episode < source.num_episodes
            ):
                raise ValueError("Plan has an invalid source episode")
            if episode in seen:
                raise ValueError(f"Source episode {episode} appears more than once")
            seen.add(episode)
            entry = manifest["episodes"][episode]
            capture_source = manifest["sessions"][entry["source_session"]]["source"]
            if plan.get("source") != capture_source:
                raise ValueError("Plan capture source does not match the source dataset")
            sources.add(capture_source)
            metadata = source.meta.episodes[episode]
            start, end = int(metadata["dataset_from_index"]), int(metadata["dataset_to_index"])
            slowdown = plan.get("slowdown")
            if (
                not isinstance(slowdown, int)
                or isinstance(slowdown, bool)
                or not 1 <= slowdown <= 10
                or plan.get("fps") != source.fps
                or plan.get("expected_frames") != (end - start) * slowdown
            ):
                raise ValueError(f"Plan timing does not match source episode {episode}")
            validate_plan(
                plan,
                config,
                simulation,
                allow_synthetic=True,
                max_sim_error_m=max_sim_error_m,
                max_sim_orientation_error_deg=max_sim_orientation_error_deg,
                allow_preview_model=full_model,
            )
            if any(
                frame.get("source_frame") != index // slowdown for index, frame in enumerate(plan["frames"])
            ):
                raise ValueError(f"Plan source frame mapping is invalid for episode {episode}")
            episodes.append((plan, start, end))
    finally:
        simulation.close()
    if len(sources) != 1:
        raise ValueError("Refusing to mix synthetic and real human recordings in a joint dataset")
    retarget_validation = validate_retargeting(source_root, plans)

    image_shape = tuple(source.meta.features["observation.images.head"]["shape"])
    state_names = source.meta.features["observation.state"]["names"]
    capture_features = {
        name: feature
        for name, feature in source.meta.features.items()
        if name.startswith("observation.capture.")
    }
    features = {
        "observation.images.head": {
            "dtype": "video",
            "shape": image_shape,
            "names": ["channels", "height", "width"],
        },
        "observation.state": {"dtype": "float32", "shape": (len(JOINT_NAMES),), "names": JOINT_NAMES},
        "action": {"dtype": "float32", "shape": (len(JOINT_NAMES),), "names": JOINT_NAMES},
        "observation.mocopi.state": {"dtype": "float32", "shape": (len(state_names),), "names": state_names},
        "observation.mocopi.raw_state": {
            "dtype": "float32",
            "shape": (len(state_names),),
            "names": state_names,
        },
        "observation.sim.frame": {
            "dtype": "int64",
            "shape": (2,),
            "names": ["plan_frame", "human_source_frame"],
        },
        "observation.robot.tcp_pose": {
            "dtype": "float32",
            "shape": (len(TCP_NAMES),),
            "names": TCP_NAMES,
        },
        "observation.robot.target_pose": {
            "dtype": "float32",
            "shape": (len(TCP_NAMES),),
            "names": TCP_NAMES,
        },
        "observation.robot.error": {
            "dtype": "float32",
            "shape": (len(TCP_ERROR_NAMES),),
            "names": TCP_ERROR_NAMES,
        },
        "observation.robot.status": {
            "dtype": "float32",
            "shape": (len(ROBOT_STATUS_NAMES),),
            "names": ROBOT_STATUS_NAMES,
        },
        "observation.robot.collision": {
            "dtype": "float32",
            "shape": (2,),
            "names": ["contact_count", "max_penetration_m"],
        },
        "observation.robot.warning": {
            "dtype": "float32",
            "shape": (len(ROBOT_WARNING_NAMES),),
            "names": ROBOT_WARNING_NAMES,
        },
        "observation.robot.plan_time_s": {"dtype": "float64", "shape": (1,), "names": None},
    }
    features.update(capture_features)
    destination = LeRobotDataset.create(
        repo_id=repo_id,
        fps=source.fps,
        features=features,
        root=output_root,
        robot_type="dual_scorpion_kinematic",
        use_videos=True,
        vcodec="h264",
        image_writer_threads=2,
    )
    result = {
        "schema": "dual_scorpion.mocopi.kinematic_joint_dataset.v1",
        "repo_id": repo_id,
        "source_dataset_repo_id": manifest["repo_id"],
        "source_dataset_sha256": fingerprint,
        "source_dataset_manifest_sha256": manifest_hash,
        "capture_source": next(iter(sources)),
        "robot_state_source": "kinematic_simulation_not_real_encoders",
        "dataset_role": "robot_imitation_kinematic",
        "joint_unit": "degree",
        "joint_names": JOINT_NAMES,
        "state_semantics": (
            "MuJoCo FK-derived 7DoF arm configuration at this frame from mocopi retargeting and IK; "
            "not a real motor or encoder observation"
        ),
        "action_semantics": "next-frame 7DoF arm joint target from the validated simulation plan; no gripper",
        "human_action_semantics": "human dataset action remains the next measured WRIST/L and WRIST/R pose",
        "image_semantics": "human head camera at the plan source frame, not a robot camera",
        "timestamp_semantics": "LeRobot timestamp is the nominal plan grid; source capture times are preserved",
        "fps": source.fps,
        "gripper_recorded": False,
        "gripper_commanded": False,
        "gripper_fixed_zero": True,
        "gripper_feature_included": False,
        "model_kind": plans[0].get("model_kind", "legacy_urdf"),
        "start_pose": plans[0].get("start_pose", "config"),
        "home_start_pose": plans[0].get("home_start_pose"),
        "retarget_start_pose": plans[0].get("retarget_start_pose", plans[0].get("start_pose")),
        "hardware_replay_allowed": not full_model,
        "mujoco_model_sha256": plans[0].get("mujoco_model_sha256"),
        "obstacles": plans[0].get("obstacles", []),
        "obstacles_sha256": plans[0].get("obstacles_sha256"),
        "obstacle_count": plans[0].get("obstacle_count", 0),
        "urdf_sha256": plans[0]["urdf_sha256"],
        "coordinate_frame": manifest.get("coordinate_frame"),
        "raw_mocopi_coordinate_frame": manifest.get("raw_mocopi_coordinate_frame"),
        "quaternion_order": manifest.get("quaternion_order"),
        "ik_mode": plans[0].get("ik_mode", "default"),
        "arm_posture": plans[0].get("arm_posture", config["retarget"].get("arm_posture")),
        "retarget": plans[0].get("retarget", config["retarget"]),
        "ik": {
            "preview_max_joint_step_deg": plans[0].get("preview_max_joint_step_deg"),
            "hardware_max_joint_step_deg": plans[0].get("hardware_max_joint_step_deg"),
            "position_tolerance_m": float(config["robot"]["position_tolerance_m"]),
            "orientation_tolerance_deg": plans[0].get("ik_orientation_tolerance_deg"),
            "orientation_weight": plans[0].get("ik_orientation_weight"),
            "joint_limit_source": "full_mujoco_urdf_unmeasured" if full_model else "legacy_urdf",
            "gripper_joint_fixed_rad": 0.0,
        },
        "tcp": {
            "frame": "l_link_8/r_link_8 origin",
            "human_source": "mocopi WRIST/L bone 14 and WRIST/R bone 18",
            "joint7_role": "gripper; fixed at zero and excluded from dataset features",
            "physical_alignment": "model_geometry_and_initial_pose_only_not_externally_measured",
        },
        "capture_features": sorted(capture_features),
        "retarget_validation": retarget_validation,
        "episodes": [],
        "validation": {
            "exported_only_after_plan_validation": True,
            "orientation_warning_threshold_deg": 30.0,
            "orientation_warning_ranges": dict.fromkeys(("left", "right"), []),
            "max_position_error_m": dict.fromkeys(("left", "right"), 0.0),
            "max_orientation_error_deg": dict.fromkeys(("left", "right"), 0.0),
            "collision_frames": [],
            "ik_failure_frames": [],
            "status": "ok",
        },
    }
    orientation_warnings = {"left": [], "right": []}
    try:
        for plan, start, _ in episodes:
            cached_index, cached = None, None
            for index, (frame, following) in enumerate(zip(plan["frames"], plan["frames"][1:], strict=False)):
                source_index = frame["source_frame"]
                if source_index != cached_index:
                    cached = source[start + source_index]
                    cached_index = source_index
                target_pose = _tcp_vector(frame, "target_pose")
                tcp_pose = _tcp_vector(frame, "simulated_pose")
                errors = _tcp_error_vector(frame)
                orientation_warning = np.asarray([errors[1], errors[3]], dtype=np.float32) > 30.0
                for position, side in enumerate(("right", "left")):
                    result["validation"]["max_position_error_m"][side] = max(
                        result["validation"]["max_position_error_m"][side], float(errors[position * 2])
                    )
                    orientation_error = float(errors[position * 2 + 1])
                    result["validation"]["max_orientation_error_deg"][side] = max(
                        result["validation"]["max_orientation_error_deg"][side], orientation_error
                    )
                    if orientation_error > result["validation"]["orientation_warning_threshold_deg"]:
                        orientation_warnings[side].append(index)
                collision = frame.get("collision") or {"count": 0, "max_penetration_m": 0.0}
                captured = {
                    name: _source_feature(cached[name], feature) for name, feature in capture_features.items()
                }
                destination.add_frame(
                    {
                        "task": cached["task"],
                        "observation.images.head": cached["observation.images.head"].numpy(),
                        "observation.state": _joint_vector(frame),
                        "action": _joint_vector(following),
                        "observation.mocopi.state": cached["observation.state"].numpy(),
                        "observation.mocopi.raw_state": cached["observation.mocopi.raw_state"].numpy(),
                        "observation.sim.frame": np.array([index, source_index], dtype=np.int64),
                        "observation.robot.tcp_pose": tcp_pose,
                        "observation.robot.target_pose": target_pose,
                        "observation.robot.error": errors,
                        "observation.robot.status": _status_vector(frame),
                        "observation.robot.collision": np.array(
                            [collision.get("count", 0), collision.get("max_penetration_m", 0.0)],
                            dtype=np.float32,
                        ),
                        "observation.robot.warning": orientation_warning.astype(np.float32),
                        "observation.robot.plan_time_s": np.array([frame["time_s"]], dtype=np.float64),
                        **captured,
                    }
                )
            destination.save_episode()
            result["episodes"].append(
                {
                    "source_episode": plan["episode"],
                    "start_pose": plan.get("start_pose", "config"),
                    "home_start_pose": plan.get("home_start_pose"),
                    "retarget_start_pose": plan.get("retarget_start_pose", plan.get("start_pose")),
                    "obstacles_sha256": plan.get("obstacles_sha256"),
                    "plan_sha256": plan_hash(plan),
                    "plan_file": str(Path(plan_paths[len(result["episodes"])]).resolve()),
                    "report_sha256": (
                        file_sha256(report_paths[len(result["episodes"])])
                        if report_paths is not None
                        else None
                    ),
                    "slowdown": plan["slowdown"],
                    "frames": len(plan["frames"]) - 1,
                }
            )
    finally:
        destination.finalize()
    if dataset_fingerprint(source_root) != fingerprint:
        raise ValueError("Source dataset changed during export; output is incomplete")
    result["validation"]["orientation_warning_ranges"] = {
        side: _contiguous_ranges(indices) for side, indices in orientation_warnings.items()
    }
    warning_count = sum(len(indices) for indices in orientation_warnings.values())
    result["validation"]["orientation_warning_count"] = warning_count
    if warning_count:
        result["validation"]["status"] = "ok_with_orientation_warnings"
    (output_root / "meta" / "mocopi_joint_dataset.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    return result
