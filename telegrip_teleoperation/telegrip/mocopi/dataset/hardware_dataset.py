"""Replay full-CAD joint datasets on the follower after a verified joint mapping.

The full-CAD plan is a kinematic simulation, not an encoder recording.  This
module keeps that distinction: a dataset is executable only after an operator
has verified the model-joint to motor-joint signs and zero offsets on the
actual robot.  It uses the follower's existing degree calibration, per-step
clamp, start-pose check, tracking HOLD, and torque-on-disconnect behavior.
"""

from __future__ import annotations

import json
import time
from contextlib import suppress
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from ..full_mujoco import FullMujocoSimulation, full_model_path
from ..geometry import pose
from .evaluate import _summary
from .fingerprint import dataset_fingerprint, file_sha256
from .real import JOINTS, SIDES, _actions, _read_joints, arm_only_follower

JOINT_SIDES = ("right", "left")
MAPPING_SCHEMA = "dual_scorpion.mocopi.hardware_joint_mapping.v1"
REPLAY_SCHEMA = "dual_scorpion.mocopi.hardware_dataset_replay.v1"
MAPPING_VERIFICATION_SCHEMA = "dual_scorpion.mocopi.hardware_mapping_verification.v1"


def _now():
    return datetime.now(UTC).isoformat()


@dataclass
class JointFrame:
    plan_frame: int
    source_frame: int | None
    time_s: float
    joints: dict[str, np.ndarray]
    target_pose: dict[str, np.ndarray | None]
    status: np.ndarray
    collision: np.ndarray
    warning: np.ndarray
    final_action: bool = False


@dataclass
class JointEpisode:
    root: Path
    repo_id: str
    episode: int
    fps: int
    frames: list[JointFrame]
    metadata: dict
    info: dict
    dataset_sha256: str
    manifest_sha256: str
    warning_frames: list[int]


def _as_array(value, shape, label):
    array = np.asarray(value, dtype=float)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f"{label} must contain shape {shape} finite values, got {array.shape}")
    return array


def load_joint_episode(dataset_path, episode=0):
    """Load a full-CAD kinematic joint dataset and append its final next action."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = Path(dataset_path).resolve()
    metadata_path = root / "meta" / "mocopi_joint_dataset.json"
    info_path = root / "meta" / "info.json"
    if not metadata_path.is_file() or not info_path.is_file():
        raise ValueError(f"Not a mocopi joint dataset: {root}")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    info = json.loads(info_path.read_text(encoding="utf-8"))
    if metadata.get("schema") != "dual_scorpion.mocopi.kinematic_joint_dataset.v1":
        raise ValueError("Unsupported joint dataset schema")
    if metadata.get("model_kind") != "full_mujoco":
        raise ValueError("This replay path requires a full-CAD joint dataset")
    if metadata.get("robot_state_source") != "kinematic_simulation_not_real_encoders":
        raise ValueError("Dataset is not labelled as kinematic simulation")
    if metadata.get("joint_names") != [f"{side}_{name}.pos" for side in JOINT_SIDES for name in JOINTS]:
        raise ValueError("Unexpected joint order in dataset metadata")
    if metadata.get("joint_unit") not in (None, "degree", "degrees"):
        raise ValueError("Dataset joint unit must be degrees")
    if metadata.get("gripper_commanded") is not False or metadata.get("gripper_feature_included"):
        raise ValueError("Dataset contains a gripper command or gripper feature")
    if metadata.get("quaternion_order") != "xyzw":
        raise ValueError("Dataset quaternion order must be xyzw")
    if not metadata.get("mujoco_model_sha256"):
        raise ValueError("Dataset does not record the MuJoCo model hash")
    dataset = LeRobotDataset(metadata["repo_id"], root=root, video_backend="pyav")
    if not 0 <= int(episode) < dataset.num_episodes:
        raise ValueError(f"Episode must be between 0 and {dataset.num_episodes - 1}")
    episode = int(episode)
    episode_meta = dataset.meta.episodes[episode]
    start, end = int(episode_meta["dataset_from_index"]), int(episode_meta["dataset_to_index"])
    if end - start < 2:
        raise ValueError("Joint episode needs at least two rows")
    hf = dataset.hf_dataset
    state = _as_array(
        [hf[index]["observation.state"] for index in range(start, end)],
        (end - start, 14),
        "observation.state",
    )
    action = _as_array([hf[index]["action"] for index in range(start, end)], (end - start, 14), "action")
    target = _as_array(
        [hf[index]["observation.robot.target_pose"] for index in range(start, end)],
        (end - start, 14),
        "observation.robot.target_pose",
    )
    status = _as_array(
        [hf[index]["observation.robot.status"] for index in range(start, end)],
        (end - start, 6),
        "observation.robot.status",
    )
    collision = _as_array(
        [hf[index]["observation.robot.collision"] for index in range(start, end)],
        (end - start, 2),
        "observation.robot.collision",
    )
    warning = _as_array(
        [hf[index]["observation.robot.warning"] for index in range(start, end)],
        (end - start, 2),
        "observation.robot.warning",
    )
    sim_frame = np.asarray(
        [hf[index]["observation.sim.frame"] for index in range(start, end)], dtype=np.int64
    )
    plan_time = np.asarray(
        [hf[index]["observation.robot.plan_time_s"] for index in range(start, end)], dtype=float
    ).reshape(-1)
    if sim_frame.shape != (end - start, 2) or plan_time.shape != (end - start,):
        raise ValueError("Dataset frame/time columns have unexpected shapes")
    if not np.allclose(action[:-1], state[1:], atol=1e-4, rtol=0):
        raise ValueError("Dataset action is not the next frame joint state")
    valid_status = bool(np.all(status[:, [0, 3]] == 1) and np.all(status[:, [1, 2, 4, 5]] == 0))
    if not valid_status:
        raise ValueError("Dataset contains an invalid, workspace-clamped, or retarget-clamped frame")
    if np.any(collision[:, 0] > 0):
        raise ValueError("Dataset contains modeled collision frames")
    warning_frames = sorted(set(np.flatnonzero(np.any(warning > 0, axis=1)).tolist()))
    frames = []
    for index in range(end - start):
        joints = {
            side: state[index, position * 7 : (position + 1) * 7].copy()
            for position, side in enumerate(JOINT_SIDES)
        }
        target_pose = {
            side: target[index, position * 7 : (position + 1) * 7].copy()
            for position, side in enumerate(JOINT_SIDES)
        }
        frames.append(
            JointFrame(
                plan_frame=int(sim_frame[index, 0]),
                source_frame=int(sim_frame[index, 1]),
                time_s=float(plan_time[index]),
                joints=joints,
                target_pose=target_pose,
                status=status[index].copy(),
                collision=collision[index].copy(),
                warning=warning[index].copy(),
            )
        )
    final_joints = {
        side: action[-1, position * 7 : (position + 1) * 7].copy()
        for position, side in enumerate(JOINT_SIDES)
    }
    frames.append(
        JointFrame(
            plan_frame=frames[-1].plan_frame + 1,
            source_frame=None,
            time_s=frames[-1].time_s + 1.0 / float(dataset.fps),
            joints=final_joints,
            target_pose=dict.fromkeys(JOINT_SIDES),
            status=frames[-1].status.copy(),
            collision=frames[-1].collision.copy(),
            warning=frames[-1].warning.copy(),
            final_action=True,
        )
    )
    if any(not np.isfinite(value).all() for frame in frames for value in frame.joints.values()):
        raise ValueError("Dataset contains non-finite joint angles")
    if any(
        frame.target_pose[side] is None or not np.isfinite(frame.target_pose[side]).all()
        for frame in frames[:-1]
        for side in JOINT_SIDES
    ):
        raise ValueError("Dataset contains an invalid target TCP pose")
    return JointEpisode(
        root=root,
        repo_id=metadata["repo_id"],
        episode=episode,
        fps=int(dataset.fps),
        frames=frames,
        metadata=metadata,
        info=info,
        dataset_sha256=dataset_fingerprint(root, "mocopi_joint_dataset.json"),
        manifest_sha256=file_sha256(metadata_path),
        warning_frames=warning_frames,
    )


def load_mapping(path):
    path = Path(path)
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("schema") != MAPPING_SCHEMA:
        raise ValueError("Unsupported hardware joint mapping schema")
    mapping = {"path": str(path), "document": document}
    validate_mapping_document(document, require_verified=False)
    return mapping


def validate_mapping_document(document, *, require_verified=True):
    if document.get("schema") != MAPPING_SCHEMA:
        raise ValueError("Unsupported hardware joint mapping schema")
    if document.get("model_kind") != "full_mujoco":
        raise ValueError("Hardware joint mapping must target the full MuJoCo model")
    if require_verified and document.get("verified") is not True:
        raise ValueError("Hardware joint mapping is not verified; run verify-dataset-mapping first")
    sides = document.get("sides")
    if not isinstance(sides, dict) or set(sides) != set(JOINT_SIDES):
        raise ValueError("Mapping must contain right and left sides")
    for side in JOINT_SIDES:
        side_doc = sides[side]
        sign = np.asarray(side_doc.get("sign"), dtype=float)
        offset = np.asarray(side_doc.get("offset_deg"), dtype=float)
        if sign.shape != (7,) or not np.isin(sign, (-1.0, 1.0)).all():
            raise ValueError(f"{side} mapping sign must contain seven values of -1 or +1")
        if offset.shape != (7,) or not np.isfinite(offset).all():
            raise ValueError(f"{side} mapping offset must contain seven finite degrees")
        limits = side_doc.get("joint_limits_deg")
        if limits is not None:
            lower = np.asarray(limits.get("min"), dtype=float)
            upper = np.asarray(limits.get("max"), dtype=float)
            if lower.shape != (7,) or upper.shape != (7,) or not np.all(lower < upper):
                raise ValueError(f"{side} mapping joint limits are invalid")
    return document


def map_joints(document, side, model_joints):
    side_doc = document["sides"][side]
    return np.asarray(side_doc["sign"], dtype=float) * np.asarray(model_joints, dtype=float) + np.asarray(
        side_doc["offset_deg"], dtype=float
    )


def unmap_joints(document, side, motor_joints):
    side_doc = document["sides"][side]
    sign = np.asarray(side_doc["sign"], dtype=float)
    return (np.asarray(motor_joints, dtype=float) - np.asarray(side_doc["offset_deg"], dtype=float)) / sign


def calibration_sha256(robot):
    path = Path(robot.calibration_fpath)
    if not path.is_file():
        raise ValueError(f"Calibration file not found for robot {robot.id}: {path}")
    return file_sha256(path)


def validate_mapping_for_robot(document, robot):
    validate_mapping_document(document)
    if document.get("robot_id") != robot.id:
        raise ValueError(
            f"Mapping robot_id {document.get('robot_id')!r} does not match --robot-id {robot.id!r}"
        )
    current_hash = calibration_sha256(robot)
    if document.get("calibration_sha256") != current_hash:
        raise ValueError("Mapping calibration hash differs from the current robot calibration")
    return {
        "robot_id": robot.id,
        "calibration_sha256": current_hash,
        "verified_at": document.get("created_at"),
        "verification": document.get("verification"),
    }


def calibration_max_abs_deg(robot, side, joint):
    bus = getattr(robot, side + "_bus")
    calibration = bus.calibration[joint]
    resolution = bus.model_resolution_table[bus.motors[joint].model]
    max_abs = (calibration.range_max - calibration.range_min) * 180 / (resolution - 1)
    if not np.isfinite(max_abs) or max_abs <= 0:
        raise ValueError(f"Invalid calibration range for {side}_{joint}")
    return float(max_abs)


def validate_mapped_episode(episode, document, robot, *, max_source_step_deg=None):
    document = document.get("document", document)
    validate_mapping_document(document)
    source_step = 0.0
    mapped_step = 0.0
    for side in JOINT_SIDES:
        side_doc = document["sides"][side]
        limits = side_doc.get("joint_limits_deg")
        for index, frame in enumerate(episode.frames):
            mapped = map_joints(document, side, frame.joints[side])
            if not np.isfinite(mapped).all():
                raise ValueError(f"Non-finite mapped command at frame {index} ({side})")
            for joint_index, joint_name in enumerate(JOINTS):
                max_abs = calibration_max_abs_deg(robot, side, joint_name)
                if abs(float(mapped[joint_index])) > max_abs + 1e-6:
                    raise ValueError(
                        f"Mapped angle exceeds calibrated motor range at frame {index} "
                        f"({side}_{joint_name}): {mapped[joint_index]:.3f} > {max_abs:.3f}"
                    )
                if limits is not None:
                    lower = float(np.asarray(limits["min"])[joint_index])
                    upper = float(np.asarray(limits["max"])[joint_index])
                    if not lower - 1e-6 <= float(mapped[joint_index]) <= upper + 1e-6:
                        raise ValueError(
                            f"Mapped angle exceeds configured joint limit at frame {index} "
                            f"({side}_{joint_name})"
                        )
            if index:
                source_step = max(
                    source_step,
                    float(np.max(np.abs(frame.joints[side] - episode.frames[index - 1].joints[side]))),
                )
                mapped_step = max(
                    mapped_step,
                    float(
                        np.max(
                            np.abs(
                                mapped - map_joints(document, side, episode.frames[index - 1].joints[side])
                            )
                        )
                    ),
                )
    preview_step = episode.metadata.get("ik", {}).get("preview_max_joint_step_deg")
    if preview_step is None:
        preview_step = 180.0
    if max_source_step_deg is not None:
        preview_step = float(max_source_step_deg)
    if not np.isfinite(preview_step) or not 0 < preview_step <= 180:
        raise ValueError("max_source_step_deg must be in (0, 180]")
    if source_step > preview_step + 1e-6:
        raise ValueError(
            f"Source frame step {source_step:.3f} deg exceeds the recorded preview limit "
            f"{preview_step:.3f} deg"
        )
    return {
        "frames": len(episode.frames),
        "fps": episode.fps,
        "max_source_step_deg": source_step,
        "max_mapped_step_deg": mapped_step,
        "preview_step_limit_deg": preview_step,
        "warning_frames": episode.warning_frames,
    }


def select_reference_frames(frames, count=5):
    if len(frames) < 3:
        raise ValueError("At least three frames are needed for mapping verification")
    count = max(3, min(int(count), len(frames)))
    indices = sorted({round(index * (len(frames) - 1) / (count - 1)) for index in range(count)})
    return indices


def fit_joint_mapping(
    episode,
    measured,
    reference_frames,
    *,
    robot_id,
    calibration_hash,
    max_residual_deg=3.0,
    min_model_range_deg=5.0,
):
    """Fit motor = sign * model + offset and reject non-rigid/ambiguous mappings."""
    if len(measured) != len(reference_frames):
        raise ValueError("One measured joint vector is required per reference frame")
    if len(reference_frames) < 3:
        raise ValueError("At least three reference frames are required")
    if not np.isfinite(max_residual_deg) or not 0 < max_residual_deg <= 20:
        raise ValueError("max_residual_deg must be in (0, 20]")
    if not np.isfinite(min_model_range_deg) or not 0 < min_model_range_deg <= 90:
        raise ValueError("min_model_range_deg must be in (0, 90]")
    failures, per_joint = [], []
    sides = {}
    for side in JOINT_SIDES:
        signs, offsets = [], []
        model = np.asarray([episode.frames[index].joints[side] for index in reference_frames])
        motor = np.asarray([value[side] for value in measured])
        for joint_index, joint_name in enumerate(JOINTS):
            x, y = model[:, joint_index], motor[:, joint_index]
            span = float(np.ptp(x))
            if span < min_model_range_deg:
                failures.append(
                    f"{side}_{joint_name}: model range {span:.2f} deg is too small to identify the sign"
                )
                continue
            slope, intercept = np.polyfit(x, y, 1)
            residual = y - (slope * x + intercept)
            rms = float(np.sqrt(np.mean(np.square(residual))))
            maximum = float(np.max(np.abs(residual)))
            record = {
                "side": side,
                "joint": joint_name,
                "model_range_deg": span,
                "slope": float(slope),
                "intercept_deg": float(intercept),
                "rms_deg": rms,
                "max_deg": maximum,
            }
            per_joint.append(record)
            if abs(abs(float(slope)) - 1.0) > 0.1:
                failures.append(f"{side}_{joint_name}: fitted slope {slope:.3f} is not close to +/-1")
            if maximum > max_residual_deg:
                failures.append(
                    f"{side}_{joint_name}: fit residual {maximum:.2f} deg exceeds {max_residual_deg:g}"
                )
            signs.append(1 if slope >= 0 else -1)
            offsets.append(float(intercept))
        sides[side] = {"sign": signs, "offset_deg": offsets}
    report = {
        "schema": MAPPING_VERIFICATION_SCHEMA,
        "status": "failed" if failures else "ok",
        "dataset": str(episode.root),
        "dataset_sha256": episode.dataset_sha256,
        "manifest_sha256": episode.manifest_sha256,
        "episode": episode.episode,
        "reference_frames": list(reference_frames),
        "max_residual_deg": float(max_residual_deg),
        "min_model_range_deg": float(min_model_range_deg),
        "per_joint": per_joint,
        "failures": failures,
    }
    if failures:
        return None, report
    document = {
        "schema": MAPPING_SCHEMA,
        "robot_id": robot_id,
        "calibration_sha256": calibration_hash,
        "verified": True,
        "model_kind": "full_mujoco",
        "created_at": _now(),
        "verification": report,
        "sides": sides,
    }
    return document, report


def write_mapping(path, document):
    path = Path(path)
    if path.exists():
        raise FileExistsError(f"Mapping output already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")


def _write_json(path, document):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as file:
        file.write(json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def verify_dataset_mapping(
    episode,
    robot,
    output,
    *,
    reference_frames=None,
    max_residual_deg=3.0,
    min_model_range_deg=5.0,
    report=None,
):
    """Read manually posed robot angles; never enables torque or commands motion."""
    reference_frames = (
        select_reference_frames(episode.frames)
        if reference_frames is None
        else sorted({int(value) for value in reference_frames})
    )
    if len(reference_frames) < 3 or any(not 0 <= value < len(episode.frames) for value in reference_frames):
        raise ValueError("reference frames must contain at least three valid episode frame indices")
    if Path(output).exists():
        raise FileExistsError(f"Mapping output already exists: {output}")
    measured = []
    try:
        for side in SIDES:
            getattr(robot, side + "_bus").connect()
        if not robot.is_calibrated:
            raise ValueError("Motor calibration does not match the robot; recalibrate before verification")
        for side in SIDES:
            getattr(robot, side + "_bus").disable_torque(list(JOINTS))
        for index in reference_frames:
            frame = episode.frames[index]
            right = np.array2string(frame.joints["right"], precision=2, suppress_small=True)
            left = np.array2string(frame.joints["left"], precision=2, suppress_small=True)
            input(
                f"トルクを切った状態で、ロボットを frame {index} の姿勢に手で合わせてEnter\n"
                f"  right model joint_deg = {right}\n  left  model joint_deg = {left}\n"
            )
            measured.append(_read_joints(robot))
        document, verification = fit_joint_mapping(
            episode,
            measured,
            reference_frames,
            robot_id=robot.id,
            calibration_hash=calibration_sha256(robot),
            max_residual_deg=max_residual_deg,
            min_model_range_deg=min_model_range_deg,
        )
        verification["measured_motor_deg"] = {
            side: [value[side].tolist() for value in measured] for side in JOINT_SIDES
        }
        if report:
            _write_json(report, verification)
        if document is None:
            raise ValueError("Joint mapping verification failed: " + "; ".join(verification["failures"]))
        write_mapping(output, document)
        return verification
    finally:
        for side in SIDES:
            bus = getattr(robot, side + "_bus")
            if bus.is_connected:
                with suppress(Exception):
                    bus.disconnect(disable_torque=True)


def _segment_plan(previous, following, *, fps, max_step_deg, max_speed_deg_s, control_hz):
    delta = np.asarray(following) - np.asarray(previous)
    max_delta = float(np.max(np.abs(delta)))
    duration = max(
        1.0 / fps,
        max_delta / max_speed_deg_s if max_delta > 0 else 0.0,
        max_delta / (max_step_deg * control_hz) if max_delta > 0 else 0.0,
    )
    steps = max(1, int(np.ceil(duration * control_hz)), int(np.ceil(max_delta / max_step_deg)))
    return delta, duration, steps


def _target_pose_for_frame(simulation, frame, side):
    target = frame.target_pose.get(side)
    if target is not None:
        return target
    position, quaternion = simulation.arms[side].solver.fk_solver.compute(np.r_[frame.joints[side], 0.0])
    return np.r_[position, quaternion]


def execute_joint_episode(
    episode,
    document,
    config,
    robot,
    report_path,
    *,
    max_start_error_deg=2.0,
    max_tracking_error_deg=5.0,
    max_joint_speed_deg_s=15.0,
    control_hz=20.0,
    max_sim_error_m=0.025,
    max_sim_orientation_error_deg=None,
    realtime=True,
):
    """Send a verified joint trajectory with interpolation and existing safety gates."""
    if not 0 < max_start_error_deg <= 2:
        raise ValueError("max_start_error_deg must be in (0, 2]")
    if not 0 < max_tracking_error_deg <= 5:
        raise ValueError("max_tracking_error_deg must be in (0, 5]")
    if not 0 < max_joint_speed_deg_s <= 15:
        raise ValueError("max_joint_speed_deg_s must be in (0, 15]")
    if not 0 < control_hz <= 30:
        raise ValueError("control_hz must be in (0, 30]")
    if not 0 < max_sim_error_m <= 0.10:
        raise ValueError("max_sim_error_m must be in (0, 0.10]")
    mapping_path = document.get("path") if isinstance(document, dict) else None
    raw_document = document.get("document", document) if isinstance(document, dict) else document
    document = raw_document
    mapping_info = validate_mapping_for_robot(document, robot)
    step_limit = robot.config.max_relative_target
    if not robot.config.use_degrees or not isinstance(step_limit, (int, float)) or not 0 < step_limit <= 0.5:
        raise ValueError("Follower must use calibrated degrees and a per-step limit at most 0.5 deg")
    validation = validate_mapped_episode(episode, document, robot)
    model_hash = file_sha256(full_model_path(config["robot"]))
    if episode.metadata.get("mujoco_model_sha256") != model_hash:
        raise ValueError("Dataset MuJoCo model differs from the selected model; regenerate the dataset")
    simulation = FullMujocoSimulation(
        config["robot"], gui=False, obstacles=episode.metadata.get("obstacles", [])
    )
    output = Path(report_path)
    if output.exists():
        raise FileExistsError(f"Report already exists: {output}")
    report = {
        "schema": REPLAY_SCHEMA,
        "status": "starting",
        "source": "full_cad_kinematic_dataset",
        "dataset": str(episode.root),
        "dataset_sha256": episode.dataset_sha256,
        "dataset_manifest_sha256": episode.manifest_sha256,
        "repo_id": episode.repo_id,
        "episode": episode.episode,
        "robot_id": robot.id,
        "calibration_sha256": calibration_sha256(robot),
        "mapping": mapping_path,
        "mapping_verified_at": document.get("created_at"),
        "mapping_robot_check": mapping_info,
        "mujoco_model_sha256": model_hash,
        "dataset_original_hardware_replay_allowed": episode.metadata.get("hardware_replay_allowed"),
        "gripper_commanded": False,
        "simulation_only_source": True,
        "frames": [],
        "failure": None,
        "limits": {
            "max_start_error_deg": max_start_error_deg,
            "max_tracking_error_deg": max_tracking_error_deg,
            "max_joint_speed_deg_s": max_joint_speed_deg_s,
            "max_relative_target_deg": step_limit,
            "control_hz": control_hz,
            "max_sim_error_m": max_sim_error_m,
            "max_sim_orientation_error_deg": max_sim_orientation_error_deg,
            "preview_step_limit_deg": validation["preview_step_limit_deg"],
        },
        "validation": validation,
    }
    active = False
    failure = None
    current = None
    start_time = None
    next_tick = None
    control_steps = 0
    max_observed_speed = 0.0
    tcp_warning_frames = []
    tcp_orientation_warning_frames = []
    try:
        for side in SIDES:
            getattr(robot, side + "_bus").connect()
        if not robot.is_calibrated:
            raise ValueError("Motor calibration does not match the robot; recalibrate before replay")
        for side in SIDES:
            bus = getattr(robot, side + "_bus")
            if any(bus.read("Operating_Mode", name, normalize=False) != 0 for name in JOINTS):
                raise ValueError(f"{side} motors must be in position mode")
        current = _read_joints(robot)
        first = {side: map_joints(document, side, episode.frames[0].joints[side]) for side in SIDES}
        for side in SIDES:
            if np.max(np.abs(current[side] - first[side])) > max_start_error_deg:
                raise ValueError(
                    f"{side} robot start pose differs from the mapped dataset first frame by more than "
                    f"{max_start_error_deg:g} degrees"
                )
        robot.send_action(_actions(current))
        active = True
        for side in SIDES:
            getattr(robot, side + "_bus").enable_torque(list(JOINTS))
        report["status"] = "running"
        start_time = time.monotonic()
        next_tick = start_time
        previous_sent = {side: current[side].copy() for side in SIDES}
        for index in range(len(episode.frames) - 1):
            first_frame, following = episode.frames[index], episode.frames[index + 1]
            previous_motor = {side: map_joints(document, side, first_frame.joints[side]) for side in SIDES}
            target_motor = {side: map_joints(document, side, following.joints[side]) for side in SIDES}
            durations, step_counts = [], []
            for side in SIDES:
                _, duration, steps = _segment_plan(
                    previous_motor[side],
                    target_motor[side],
                    fps=episode.fps,
                    max_step_deg=step_limit,
                    max_speed_deg_s=max_joint_speed_deg_s,
                    control_hz=control_hz,
                )
                durations.append(duration)
                step_counts.append(steps)
            steps = max(step_counts)
            duration = max(durations)
            segment_duration = steps / control_hz
            for substep in range(1, steps + 1):
                alpha = substep / steps
                commanded = {
                    side: previous_motor[side] + alpha * (target_motor[side] - previous_motor[side])
                    for side in SIDES
                }
                sent = robot.send_action(_actions(commanded))
                control_steps += 1
                measured = _read_joints(robot)
                sent_motor = {
                    side: np.array([sent[f"{side}_{name}.pos"] for name in JOINTS], dtype=float)
                    for side in SIDES
                }
                for side in SIDES:
                    error = float(np.max(np.abs(commanded[side] - measured[side])))
                    if error > max_tracking_error_deg:
                        raise RuntimeError(
                            f"Robot fell behind the mapped dataset at frame {index} "
                            f"({side}, {error:.2f} deg): HOLD"
                        )
                    speed = float(np.max(np.abs(sent_motor[side] - previous_sent[side]))) * control_hz
                    max_observed_speed = max(max_observed_speed, speed)
                    previous_sent[side] = sent_motor[side].copy()
                current = measured
                if realtime:
                    next_tick += 1.0 / control_hz
                    time.sleep(max(0.0, next_tick - time.monotonic()))
            measured_model = {side: unmap_joints(document, side, measured[side]) for side in SIDES}
            sample = {
                "plan_frame": following.plan_frame,
                "source_frame": following.source_frame,
                "source_time_s": following.time_s,
                "elapsed_s": time.monotonic() - start_time,
                "control_steps": steps,
                "segment_duration_s": segment_duration,
                "requested_duration_s": 1.0 / episode.fps,
                "time_scale": segment_duration * episode.fps,
                "arms": {},
            }
            for side in SIDES:
                target_pose = _target_pose_for_frame(simulation, following, side)
                position, quaternion = simulation.arms[side].solver.fk_solver.compute(
                    np.r_[measured_model[side], 0.0]
                )
                target = pose(target_pose[:3], target_pose[3:])
                actual = pose(position, quaternion)
                tcp_position_error = float(np.linalg.norm(position - target[:3, 3]))
                tcp_orientation_error = float(
                    np.rad2deg(Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).magnitude())
                )
                if tcp_position_error > max_sim_error_m:
                    tcp_warning_frames.append(index + 1)
                if (
                    max_sim_orientation_error_deg is not None
                    and tcp_orientation_error > max_sim_orientation_error_deg
                ):
                    tcp_orientation_warning_frames.append(index + 1)
                sample["arms"][side] = {
                    "planned_model_joint_deg": following.joints[side].tolist(),
                    "mapped_motor_joint_deg": target_motor[side].tolist(),
                    "sent_motor_joint_deg": sent_motor[side].tolist(),
                    "measured_motor_joint_deg": measured[side].tolist(),
                    "measured_model_joint_deg": measured_model[side].tolist(),
                    "max_joint_error_deg": float(np.max(np.abs(target_motor[side] - measured[side]))),
                    "tcp_position_error_m": tcp_position_error,
                    "tcp_orientation_error_deg": tcp_orientation_error,
                    "target_pose_source": (
                        "dataset"
                        if following.target_pose.get(side) is not None
                        else "full_model_fk_for_final_action"
                    ),
                }
            report["frames"].append(sample)
            current = measured
        report["status"] = "complete"
    except BaseException as exc:
        failure = exc
        report["status"], report["failure"] = "aborted", str(exc) or type(exc).__name__
    finally:
        if active and robot.is_connected and current is not None:
            try:
                robot.send_action(_actions(current))
            except Exception as exc:
                report["hold_error"] = str(exc)
        for side in SIDES:
            bus = getattr(robot, side + "_bus")
            if bus.is_connected:
                try:
                    bus.disconnect(disable_torque=False)
                except Exception as exc:
                    report[f"{side}_disconnect_error"] = str(exc)
        elapsed = [row["elapsed_s"] for row in report["frames"]]
        report["source_frames"] = len(episode.frames)
        report["control_steps"] = control_steps
        report["actual_duration_s"] = elapsed[-1] if elapsed else 0.0
        report["source_duration_s"] = (
            episode.frames[-1].time_s - episode.frames[0].time_s if len(episode.frames) > 1 else 0.0
        )
        report["time_scale"] = (
            report["actual_duration_s"] / report["source_duration_s"]
            if report["source_duration_s"] > 0
            else None
        )
        report["max_observed_speed_deg_s"] = max_observed_speed
        report["tcp_warning_frames"] = sorted(set(tcp_warning_frames))
        report["tcp_orientation_warning_frames"] = sorted(set(tcp_orientation_warning_frames))
        report["tracking"] = {
            side: {
                metric: _summary([row["arms"][side][metric] for row in report["frames"]])
                for metric in (
                    "max_joint_error_deg",
                    "tcp_position_error_m",
                    "tcp_orientation_error_deg",
                )
            }
            for side in SIDES
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        if output.exists():
            output.unlink()
        _write_json(output, report)
    simulation.close()
    if failure:
        raise RuntimeError(
            f"Hardware dataset replay stopped: {failure}; report saved to {output}"
        ) from failure
    return report


def replay_joint_dataset(
    config,
    dataset_path,
    episode_index=0,
    *,
    mapping_path,
    execute=False,
    left_port=None,
    right_port=None,
    robot_id=None,
    calibration_dir=None,
    report=None,
    max_start_error_deg=2.0,
    max_tracking_error_deg=5.0,
    max_joint_speed_deg_s=15.0,
    control_hz=20.0,
    max_sim_error_m=0.025,
    max_sim_orientation_error_deg=None,
):
    episode = load_joint_episode(dataset_path, episode_index)
    mapping = load_mapping(mapping_path)
    model_hash = file_sha256(full_model_path(config["robot"]))
    if episode.metadata.get("mujoco_model_sha256") != model_hash:
        raise ValueError("Dataset MuJoCo model differs from the selected model; regenerate the dataset")
    if not robot_id:
        raise ValueError("--robot-id is required to load the follower calibration")
    robot = arm_only_follower(
        left_port or "/dev/not-used-left",
        right_port or "/dev/not-used-right",
        robot_id,
        config["robot"]["max_joint_step_deg"],
        calibration_dir,
    )
    mapping_info = validate_mapping_for_robot(mapping["document"], robot)
    validation = validate_mapped_episode(episode, mapping["document"], robot)
    if not execute:
        return {
            "schema": REPLAY_SCHEMA,
            "mode": "validation_only",
            "dataset": str(episode.root),
            "episode": episode.episode,
            "frames": len(episode.frames),
            "mapping": mapping_info,
            "gripper_commanded": False,
            "simulation_only_source": True,
            "validation": validation,
        }
    if not left_port or not right_port or not report:
        raise ValueError("--execute requires --left-port, --right-port and --report")
    before = episode.dataset_sha256
    result = execute_joint_episode(
        episode,
        mapping,
        config,
        robot,
        report,
        max_start_error_deg=max_start_error_deg,
        max_tracking_error_deg=max_tracking_error_deg,
        max_joint_speed_deg_s=max_joint_speed_deg_s,
        control_hz=control_hz,
        max_sim_error_m=max_sim_error_m,
        max_sim_orientation_error_deg=max_sim_orientation_error_deg,
    )
    if dataset_fingerprint(episode.root, "mocopi_joint_dataset.json") != before:
        raise ValueError("Source joint dataset changed during hardware replay")
    return result
