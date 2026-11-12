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
import xml.etree.ElementTree as ET
from contextlib import suppress
from dataclasses import dataclass
from datetime import UTC, datetime
from functools import lru_cache
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from ..full_mujoco import SUFFIX, FullMujocoSimulation, full_model_path
from ..geometry import pose
from .evaluate import _summary
from .fingerprint import dataset_fingerprint, file_sha256
from .real import JOINTS, SIDES, _actions, _read_joints, arm_only_follower

JOINT_SIDES = ("right", "left")
MAPPING_SCHEMA = "dual_scorpion.mocopi.hardware_joint_mapping.v1"
REPLAY_SCHEMA = "dual_scorpion.mocopi.hardware_dataset_replay.v1"
MAPPING_VERIFICATION_SCHEMA = "dual_scorpion.mocopi.hardware_mapping_verification.v1"
JOINT_VERIFICATION_SCHEMA = "dual_scorpion.mocopi.hardware_joint_verification.v1"
POSE_CONTEXT_SCHEMA = "dual_scorpion.mocopi.hardware_pose_context.v1"
METHOD_MULTI_POSE = "multi_pose_least_squares"
METHOD_REFERENCE_DIRECTION = "reference_pose_bidirectional_direction"


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
            source_frame=(frames[-1].source_frame + 1 if frames[-1].source_frame is not None else None),
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


def _joint_verification_records(document, side):
    side_doc = document["sides"][side]
    records = side_doc.get("joint_verification")
    if records is None:
        return None
    if not isinstance(records, dict):
        raise ValueError(f"{side} joint_verification must be a mapping keyed by joint name")
    if set(records) != set(JOINTS):
        missing = sorted(set(JOINTS) - set(records))
        extra = sorted(set(records) - set(JOINTS))
        raise ValueError(f"{side} joint_verification must cover every joint; missing={missing} extra={extra}")
    return records


def mapping_joint_records(document, side):
    """Return per-joint evidence, or ``None`` for a legacy whole-body-only mapping."""
    document = document.get("document", document) if isinstance(document, dict) else document
    return _joint_verification_records(document, side)


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
        records = _joint_verification_records(document, side)
        if records is not None:
            for joint_index, joint in enumerate(JOINTS):
                record = records[joint]
                if not isinstance(record, dict):
                    raise ValueError(f"{side}_{joint} verification record must be a mapping")
                if record.get("schema") not in (None, JOINT_VERIFICATION_SCHEMA):
                    raise ValueError(f"{side}_{joint} verification schema is unsupported")
                if record.get("joint") not in (None, joint):
                    raise ValueError(f"{side}_{joint} verification record names another joint")
                if record.get("side") not in (None, side):
                    raise ValueError(f"{side}_{joint} verification record names another side")
                if (
                    not isinstance(record.get("motor_id"), int)
                    or isinstance(record["motor_id"], bool)
                    or record["motor_id"] <= 0
                ):
                    raise ValueError(f"{side}_{joint} verification must contain a positive motor_id")
                if record.get("motor_model") is not None and not isinstance(record["motor_model"], str):
                    raise ValueError(f"{side}_{joint} verification motor_model must be a string")
                if record.get("sign") is not None and int(record["sign"]) not in (-1, 1):
                    raise ValueError(f"{side}_{joint} verification sign must be -1 or +1")
                if record.get("sign") is not None and int(record["sign"]) != int(sign[joint_index]):
                    raise ValueError(f"{side}_{joint} verification sign disagrees with the mapping")
                if record.get("offset_deg") is not None and (
                    not np.isfinite(float(record["offset_deg"]))
                    or abs(float(record["offset_deg"]) - float(offset[joint_index])) > 1e-6
                ):
                    raise ValueError(f"{side}_{joint} verification offset disagrees with the mapping")
                if not isinstance(record.get("verified"), bool):
                    raise ValueError(f"{side}_{joint} verification must contain a boolean verified flag")
        if require_verified:
            if records is None:
                raise ValueError(
                    "Hardware joint mapping lacks per-joint verification records; "
                    "re-run verify-dataset-mapping so every joint is checked"
                )
            for joint in JOINTS:
                if records[joint].get("verified") is not True:
                    raise ValueError(
                        f"Hardware joint mapping has an unverified joint: {side}_{joint}; "
                        "motor commands are refused"
                    )
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


def calibration_joint_limits(robot, side, joint):
    """Return the degree limits the Feetech bus applies around the calibration midpoint.

    ``MotorNormMode.DEGREES`` maps raw encoder counts to degrees about
    ``(range_min + range_max) / 2``.  The midpoint is the calibration zero, not
    the URDF/MJCF zero.  The allowed absolute angle is therefore half the
    recorded span, not the full span.
    """
    bus = getattr(robot, side + "_bus")
    calibration = bus.calibration[joint]
    resolution = int(bus.model_resolution_table[bus.motors[joint].model])
    if resolution <= 1:
        raise ValueError(f"Invalid encoder resolution for {side}_{joint}")
    scale = 360.0 / (resolution - 1)
    midpoint = 0.5 * (int(calibration.range_min) + int(calibration.range_max))
    lower = (int(calibration.range_min) - midpoint) * scale
    upper = (int(calibration.range_max) - midpoint) * scale
    if not (np.isfinite(lower) and np.isfinite(upper) and lower < upper):
        raise ValueError(f"Invalid calibration range for {side}_{joint}")
    return {
        "raw_min": int(calibration.range_min),
        "raw_max": int(calibration.range_max),
        "resolution": resolution,
        "drive_mode": int(getattr(calibration, "drive_mode", 0)),
        "homing_offset": int(getattr(calibration, "homing_offset", 0)),
        "motor_id": int(getattr(calibration, "id", bus.motors[joint].id)),
        "zero_motor_deg": 0.0,
        "min_motor_deg": float(lower),
        "max_motor_deg": float(upper),
        "span_deg": float(upper - lower),
    }


def validate_mapping_for_robot(document, robot, *, model_sha256=None):
    validate_mapping_document(document)
    if document.get("robot_id") != robot.id:
        raise ValueError(
            f"Mapping robot_id {document.get('robot_id')!r} does not match --robot-id {robot.id!r}"
        )
    current_hash = calibration_sha256(robot)
    if document.get("calibration_sha256") != current_hash:
        raise ValueError("Mapping calibration hash differs from the current robot calibration")
    if model_sha256 is not None and document.get("mujoco_model_sha256") != model_sha256:
        raise ValueError("Mapping MuJoCo model hash differs from the selected model or is missing")
    motor_ids = {}
    for side in JOINT_SIDES:
        records = mapping_joint_records(document, side)
        if records is None:
            raise ValueError("Hardware joint mapping lacks per-joint verification records")
        bus = getattr(robot, side + "_bus")
        for joint in JOINTS:
            record = records[joint]
            motor = bus.motors[joint]
            if int(record["motor_id"]) != int(motor.id):
                raise ValueError(
                    f"Mapping motor id for {side}_{joint} is {record['motor_id']}, "
                    f"but the robot motor id is {motor.id}"
                )
            if record.get("motor_model") not in (None, motor.model):
                raise ValueError(
                    f"Mapping motor model for {side}_{joint} is {record['motor_model']!r}, "
                    f"but the robot motor model is {motor.model!r}"
                )
            motor_ids[f"{side}_{joint}"] = int(motor.id)
    return {
        "robot_id": robot.id,
        "calibration_sha256": current_hash,
        "verified_at": document.get("created_at"),
        "verification": document.get("verification"),
        "motor_ids": motor_ids,
    }


def validate_mapped_episode(episode, document, robot, *, max_source_step_deg=None):
    document = document.get("document", document)
    validate_mapping_document(document)
    source_step = 0.0
    mapped_step = 0.0
    margins = {
        side: {joint: {"min_margin_deg": np.inf, "max_margin_deg": np.inf} for joint in JOINTS}
        for side in JOINT_SIDES
    }
    for side in JOINT_SIDES:
        side_doc = document["sides"][side]
        limits = side_doc.get("joint_limits_deg")
        for index, frame in enumerate(episode.frames):
            mapped = map_joints(document, side, frame.joints[side])
            if not np.isfinite(mapped).all():
                raise ValueError(f"Non-finite mapped command at frame {index} ({side})")
            for joint_index, joint_name in enumerate(JOINTS):
                calibration = calibration_joint_limits(robot, side, joint_name)
                value = float(mapped[joint_index])
                if not calibration["min_motor_deg"] - 1e-6 <= value <= calibration["max_motor_deg"] + 1e-6:
                    raise ValueError(
                        f"Mapped angle exceeds calibrated motor half-range at frame {index} "
                        f"({side}_{joint_name}): {value:.3f} not in "
                        f"[{calibration['min_motor_deg']:.3f}, {calibration['max_motor_deg']:.3f}]"
                    )
                margins[side][joint_name]["min_margin_deg"] = min(
                    margins[side][joint_name]["min_margin_deg"],
                    value - calibration["min_motor_deg"],
                )
                margins[side][joint_name]["max_margin_deg"] = min(
                    margins[side][joint_name]["max_margin_deg"],
                    calibration["max_motor_deg"] - value,
                )
                if limits is not None:
                    lower = float(np.asarray(limits["min"])[joint_index])
                    upper = float(np.asarray(limits["max"])[joint_index])
                    if not lower - 1e-6 <= value <= upper + 1e-6:
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
        "calibration_margins_deg": {
            side: {
                joint: {
                    "min_margin_deg": float(info["min_margin_deg"]),
                    "max_margin_deg": float(info["max_margin_deg"]),
                }
                for joint, info in joints.items()
            }
            for side, joints in margins.items()
        },
    }


def select_reference_frames(frames, count=5):
    if len(frames) < 3:
        raise ValueError("At least three frames are needed for mapping verification")
    count = max(3, min(int(count), len(frames)))
    indices = sorted({round(index * (len(frames) - 1) / (count - 1)) for index in range(count)})
    return indices


@lru_cache(maxsize=8)
def _model_joint_table_cached(path):
    tree = ET.parse(path)
    root = tree.getroot()
    parent_body = {}
    for body in root.iter("body"):
        for joint in body.findall("joint"):
            name = joint.get("name")
            if name:
                parent_body[name] = body.get("name")
    table = {}
    for joint in root.iter("joint"):
        name = joint.get("name")
        if not name:
            continue
        axis = joint.get("axis", "0 0 1").split()
        table[name] = {
            "axis": [float(value) for value in axis],
            "body": parent_body.get(name),
        }
    return table


def model_joint_table(config):
    """Read model joint axes without importing MuJoCo or changing the model."""
    path = str(Path(full_model_path(config["robot"])).resolve())
    return path, _model_joint_table_cached(path)


def model_joint_record(config, side, joint_index):
    path, table = model_joint_table(config)
    name = f"joint{joint_index}_{SUFFIX[side]}"
    if name not in table:
        raise ValueError(f"MuJoCo model has no joint {name}")
    return {"model_path": path, "model_joint": name, **table[name]}


def model_joint_world_axes(config, model_joints, obstacles=()):
    """Return each joint axis in the world frame at the given model pose.

    The operator can use the right-hand rule around this axis to move one joint
    in the model-positive direction during passive verification.
    """
    simulation = FullMujocoSimulation(config["robot"], gui=False, obstacles=obstacles)
    try:
        for side in JOINT_SIDES:
            simulation.data.qpos[simulation.qpos_indices[side][:7]] = np.deg2rad(model_joints[side])
            simulation.data.qpos[simulation.qpos_indices[side][7]] = 0
        simulation.mujoco.mj_forward(simulation.model, simulation.data)
        axes = {}
        for side, suffix in SUFFIX.items():
            for index in range(7):
                joint = simulation.model.joint(f"joint{index}_{suffix}")
                axes[f"{side}_joint{index}"] = (
                    simulation.data.xmat[joint.bodyid[0]].reshape(3, 3) @ joint.axis
                ).tolist()
        return axes
    finally:
        simulation.close()


def inspect_joint_mapping(episode, document, robot, config, *, current=None):
    """Build the per-joint table used by dry-run and verification reports."""
    document = document.get("document", document) if isinstance(document, dict) else document
    model_path, model_table = model_joint_table(config)
    sides = {}
    unverified = []
    for side in JOINT_SIDES:
        records = mapping_joint_records(document, side) if document else None
        bus = getattr(robot, side + "_bus")
        items = []
        for joint_index, joint in enumerate(JOINTS):
            calibration = calibration_joint_limits(robot, side, joint)
            model_name = f"joint{joint_index}_{SUFFIX[side]}"
            if model_name not in model_table:
                raise ValueError(f"MuJoCo model has no joint {model_name}")
            model_info = model_table[model_name]
            record = records[joint] if records else None
            sign = int(record["sign"]) if record and record.get("sign") is not None else None
            offset = float(record["offset_deg"]) if record and record.get("offset_deg") is not None else None
            model_at_calibration_zero = None
            calibration_model_min = None
            calibration_model_max = None
            if sign and offset is not None:
                model_at_calibration_zero = float((calibration["zero_motor_deg"] - offset) / sign)
                calibration_model_min = float((calibration["min_motor_deg"] - offset) / sign)
                calibration_model_max = float((calibration["max_motor_deg"] - offset) / sign)
            verified = bool(record and record.get("verified") is True)
            if not verified:
                unverified.append(f"{side}_{joint}")
            items.append(
                {
                    "joint": joint,
                    "dataset_index": joint_index + (0 if side == "right" else 7),
                    "motor_id": calibration["motor_id"],
                    "motor_model": bus.motors[joint].model,
                    "drive_mode": calibration["drive_mode"],
                    "homing_offset": calibration["homing_offset"],
                    "raw_range": [calibration["raw_min"], calibration["raw_max"]],
                    "model_joint": model_name,
                    "model_body": model_info.get("body"),
                    "model_axis": model_info.get("axis"),
                    "model_zero_deg": 0.0,
                    "calibration_zero_motor_deg": calibration["zero_motor_deg"],
                    "calibration_min_motor_deg": calibration["min_motor_deg"],
                    "calibration_max_motor_deg": calibration["max_motor_deg"],
                    "calibration_span_deg": calibration["span_deg"],
                    "model_at_calibration_zero_deg": model_at_calibration_zero,
                    "calibration_model_min_deg": calibration_model_min,
                    "calibration_model_max_deg": calibration_model_max,
                    "sign": sign,
                    "offset_deg": offset,
                    "verified": verified,
                    "verification_method": record.get("method") if record else None,
                    "verification": record,
                }
            )
        sides[side] = items
    pairing = []
    for joint_index, joint in enumerate(JOINTS):
        right_axis = np.asarray(sides["right"][joint_index]["model_axis"], dtype=float)
        left_axis = np.asarray(sides["left"][joint_index]["model_axis"], dtype=float)
        dot = float(np.dot(right_axis, left_axis))
        pairing.append(
            {
                "joint": joint,
                "right_model_joint": sides["right"][joint_index]["model_joint"],
                "left_model_joint": sides["left"][joint_index]["model_joint"],
                "right_motor_id": sides["right"][joint_index]["motor_id"],
                "left_motor_id": sides["left"][joint_index]["motor_id"],
                "axis_dot": dot,
                "axis_relation": "same" if dot > 0.99 else "opposite" if dot < -0.99 else "mixed",
            }
        )
    current_table = None
    if current is not None:
        current_table = {
            side: {
                "motor_deg": current[side].tolist(),
                "model_deg": unmap_joints(document, side, current[side]).tolist() if document else None,
            }
            for side in JOINT_SIDES
        }
    return {
        "schema": "dual_scorpion.mocopi.hardware_joint_inspection.v1",
        "model_path": model_path,
        "sides": sides,
        "left_right_pairing": pairing,
        "unverified_joints": unverified,
        "all_joints_verified": not unverified and document is not None,
        "robot_current_pose": current_table,
        "scope": (
            "Joint order, model axes, motor ids, and calibration ranges are read from the "
            "model and calibration file. Signs and offsets are only confirmed by the recorded "
            "per-joint verification; model collision limits are the unmeasured full-CAD preview."
        ),
    }


def build_pose_context(episode, document, robot, config, *, current=None):
    """Keep model zero, calibration zero, current pose, and dataset start distinct."""
    document = document.get("document", document) if isinstance(document, dict) else document
    inspection = inspect_joint_mapping(episode, document, robot, config, current=current)
    first = episode.frames[0]
    context = {
        "schema": POSE_CONTEXT_SCHEMA,
        "model_zero_deg": {side: [0.0] * 7 for side in JOINT_SIDES},
        "calibration_zero": {
            side: {
                "motor_deg": [0.0] * 7,
                "model_deg": unmap_joints(document, side, np.zeros(7)).tolist(),
            }
            for side in JOINT_SIDES
        },
        "dataset_initial_pose": {
            side: {
                "model_deg": first.joints[side].tolist(),
                "mapped_motor_deg": map_joints(document, side, first.joints[side]).tolist(),
            }
            for side in JOINT_SIDES
        },
        "robot_current_pose": inspection["robot_current_pose"],
        "start_error_deg": None,
        "inspection": inspection,
        "note": (
            "Model zero, calibration zero, current pose, and dataset initial pose are "
            "separate quantities; none is inferred from another."
        ),
    }
    if current is not None:
        errors = {}
        for side in JOINT_SIDES:
            target = map_joints(document, side, first.joints[side])
            errors[side] = {
                joint: float(abs(current[side][index] - target[index])) for index, joint in enumerate(JOINTS)
            }
        context["start_error_deg"] = {
            "max": max(value for side in JOINT_SIDES for value in errors[side].values()),
            "per_joint": errors,
        }
    return context


def _model_evidence(config, side, joint_index):
    if config is None:
        return None, None
    record = model_joint_record(config, side, joint_index)
    return record["model_joint"], record["axis"]


def fit_joint_verification(
    model_angles,
    motor_angles,
    *,
    side,
    joint,
    motor_id,
    motor_model=None,
    method=METHOD_MULTI_POSE,
    model_axis=None,
    calibration=None,
    max_residual_deg=3.0,
    min_model_range_deg=5.0,
    max_slope_error=0.1,
    evidence=None,
):
    """Fit one joint from measured model/motor pairs and report its own status."""
    if not np.isfinite(max_residual_deg) or not 0 < max_residual_deg <= 20:
        raise ValueError("max_residual_deg must be in (0, 20]")
    if not np.isfinite(min_model_range_deg) or not 0 < min_model_range_deg <= 90:
        raise ValueError("min_model_range_deg must be in (0, 90]")
    if not np.isfinite(max_slope_error) or not 0 <= max_slope_error <= 0.5:
        raise ValueError("max_slope_error must be in [0, 0.5]")
    x = np.asarray(model_angles, dtype=float).reshape(-1)
    y = np.asarray(motor_angles, dtype=float).reshape(-1)
    failures = []
    if x.shape != y.shape:
        raise ValueError(f"{side}_{joint} model and motor samples must have the same length")
    if len(x) < 2 or not np.isfinite(x).all() or not np.isfinite(y).all():
        raise ValueError(f"{side}_{joint} needs at least two finite model/motor samples")
    span = float(np.ptp(x))
    if span < min_model_range_deg:
        failures.append(f"{side}_{joint}: model range {span:.2f} deg is too small to identify the sign")
    slope, intercept = (1.0, 0.0)
    if span > 0:
        slope, intercept = np.polyfit(x, y, 1)
    sign = 1 if slope >= 0 else -1
    offset = float(np.mean(y - sign * x))
    residual = y - (sign * x + offset)
    rms = float(np.sqrt(np.mean(np.square(residual))))
    maximum = float(np.max(np.abs(residual)))
    if abs(abs(float(slope)) - 1.0) > max_slope_error:
        failures.append(
            f"{side}_{joint}: fitted slope {slope:.3f} is not close to +/-1 (tolerance {max_slope_error:g})"
        )
    if maximum > max_residual_deg:
        failures.append(
            f"{side}_{joint}: residual after fixing sign to {sign:+d} is {maximum:.2f} deg "
            f"(limit {max_residual_deg:g})"
        )
    record = {
        "schema": JOINT_VERIFICATION_SCHEMA,
        "side": side,
        "joint": joint,
        "motor_id": int(motor_id),
        "motor_model": motor_model,
        "method": method,
        "verified": not failures,
        "model_axis": model_axis,
        "calibration": calibration,
        "model_angles_deg": x.tolist(),
        "motor_angles_deg": y.tolist(),
        "model_range_deg": span,
        "slope": float(slope),
        "sign": sign,
        "offset_deg": offset,
        "rms_deg": rms,
        "max_residual_deg": maximum,
        "max_slope_error": float(max_slope_error),
        "scale_basis": "direct_drive_1_to_1" if method == METHOD_REFERENCE_DIRECTION else "fitted",
        "scale_verified": method != METHOD_REFERENCE_DIRECTION,
        "evidence": evidence or {},
        "failures": failures,
    }
    return record


def fit_joint_from_reference(
    model_reference_deg,
    motor_reference_deg,
    motor_positive_deg,
    motor_negative_deg,
    *,
    side,
    joint,
    motor_id,
    motor_model=None,
    model_axis=None,
    calibration=None,
    min_test_deg=5.0,
    other_joint_violations=None,
    evidence=None,
):
    """Verify one joint from a reference pose and two opposite-direction nudges.

    The encoder direction identifies the sign.  The reference pose supplies the
    zero offset.  The scale is the direct-drive 1:1 relationship and is marked
    as not independently measured.
    """
    values = np.asarray(
        [model_reference_deg, motor_reference_deg, motor_positive_deg, motor_negative_deg],
        dtype=float,
    )
    if not np.isfinite(values).all():
        raise ValueError(f"{side}_{joint} direction test readings must be finite")
    positive_delta = float(motor_positive_deg - motor_reference_deg)
    negative_delta = float(motor_negative_deg - motor_reference_deg)
    failures = []
    if not np.isfinite(min_test_deg) or not 0 < min_test_deg <= 90:
        raise ValueError("min_test_deg must be in (0, 90]")
    if abs(positive_delta) < min_test_deg:
        failures.append(
            f"{side}_{joint}: positive-direction test moved only {positive_delta:+.2f} deg "
            f"(need {min_test_deg:g})"
        )
    if abs(negative_delta) < min_test_deg:
        failures.append(
            f"{side}_{joint}: negative-direction test moved only {negative_delta:+.2f} deg "
            f"(need {min_test_deg:g})"
        )
    if positive_delta * negative_delta >= 0:
        failures.append(f"{side}_{joint}: the two direction tests did not reverse the encoder")
    if other_joint_violations:
        failures.append(
            f"{side}_{joint}: other joints moved during the direction test: "
            + ", ".join(sorted(other_joint_violations))
        )
    sign = 1 if positive_delta > 0 else -1
    offset = float(motor_reference_deg - sign * model_reference_deg)
    record = {
        "schema": JOINT_VERIFICATION_SCHEMA,
        "side": side,
        "joint": joint,
        "motor_id": int(motor_id),
        "motor_model": motor_model,
        "method": METHOD_REFERENCE_DIRECTION,
        "verified": not failures,
        "model_axis": model_axis,
        "calibration": calibration,
        "model_angles_deg": [float(model_reference_deg)],
        "motor_angles_deg": [float(motor_reference_deg)],
        "model_range_deg": None,
        "slope": float(sign),
        "sign": sign,
        "offset_deg": offset,
        "rms_deg": 0.0,
        "max_residual_deg": 0.0,
        "positive_delta_deg": positive_delta,
        "negative_delta_deg": negative_delta,
        "min_test_deg": float(min_test_deg),
        "other_joint_violations": other_joint_violations or {},
        "scale_basis": "direct_drive_1_to_1",
        "scale_verified": False,
        "evidence": evidence or {},
        "failures": failures,
    }
    return record


def _build_mapping_document(
    episode,
    records_by_side,
    *,
    robot_id,
    calibration_hash,
    config=None,
    method,
    verification=None,
):
    failures = []
    sides = {}
    for side in JOINT_SIDES:
        signs, offsets = [], []
        records = records_by_side[side]
        for joint in JOINTS:
            record = records[joint]
            signs.append(int(record["sign"]))
            offsets.append(float(record["offset_deg"]))
            failures.extend(record.get("failures", []))
            if record.get("verified") is not True:
                failures.append(f"{side}_{joint}: joint verification is not complete")
        sides[side] = {
            "sign": signs,
            "offset_deg": offsets,
            "joint_verification": records,
        }
    document = {
        "schema": MAPPING_SCHEMA,
        "robot_id": robot_id,
        "calibration_sha256": calibration_hash,
        "verified": not failures,
        "model_kind": "full_mujoco",
        "created_at": _now(),
        "verification": verification or {},
        "sides": sides,
    }
    if config is not None:
        model_path, _ = model_joint_table(config)
        document["mujoco_model_sha256"] = file_sha256(model_path)
    else:
        document["mujoco_model_sha256"] = episode.metadata.get("mujoco_model_sha256")
    document.update(
        {
            "dataset": str(episode.root),
            "dataset_sha256": episode.dataset_sha256,
            "dataset_manifest_sha256": episode.manifest_sha256,
            "episode": episode.episode,
            "verification_method": method,
            "failures": failures,
        }
    )
    return document


def fit_joint_mapping(
    episode,
    measured,
    reference_frames,
    *,
    robot_id,
    calibration_hash,
    robot=None,
    config=None,
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
    records_by_side = {}
    per_joint = []
    for side in JOINT_SIDES:
        model = np.asarray([episode.frames[index].joints[side] for index in reference_frames])
        motor = np.asarray([value[side] for value in measured])
        records = {}
        for joint_index, joint_name in enumerate(JOINTS):
            motor_id = (
                joint_index + 1 if robot is None else getattr(robot, side + "_bus").motors[joint_name].id
            )
            motor_model = None if robot is None else getattr(robot, side + "_bus").motors[joint_name].model
            calibration = None if robot is None else calibration_joint_limits(robot, side, joint_name)
            model_name, model_axis = _model_evidence(config, side, joint_index)
            record = fit_joint_verification(
                model[:, joint_index],
                motor[:, joint_index],
                side=side,
                joint=joint_name,
                motor_id=motor_id,
                motor_model=motor_model,
                method=METHOD_MULTI_POSE,
                model_axis=model_axis,
                calibration=calibration,
                max_residual_deg=max_residual_deg,
                min_model_range_deg=min_model_range_deg,
            )
            if config is not None and model_name is None:
                raise ValueError(f"No model joint for {side}_{joint_name}")
            record["model_joint"] = model_name
            records[joint_name] = record
            per_joint.append(
                {
                    "side": side,
                    "joint": joint_name,
                    "model_range_deg": record["model_range_deg"],
                    "slope": record["slope"],
                    "intercept_deg": record["offset_deg"],
                    "corrected_max_residual_deg": record["max_residual_deg"],
                    "verified": record["verified"],
                    "failures": record["failures"],
                }
            )
        records_by_side[side] = records
    report = {
        "schema": MAPPING_VERIFICATION_SCHEMA,
        "status": "failed"
        if any(not record["verified"] for records in records_by_side.values() for record in records.values())
        else "ok",
        "dataset": str(episode.root),
        "dataset_sha256": episode.dataset_sha256,
        "manifest_sha256": episode.manifest_sha256,
        "episode": episode.episode,
        "reference_frames": list(reference_frames),
        "max_residual_deg": float(max_residual_deg),
        "min_model_range_deg": float(min_model_range_deg),
        "method": METHOD_MULTI_POSE,
        "per_joint": per_joint,
        "failures": [failure for item in per_joint for failure in item["failures"]],
    }
    document = _build_mapping_document(
        episode,
        records_by_side,
        robot_id=robot_id,
        calibration_hash=calibration_hash,
        config=config,
        method=METHOD_MULTI_POSE,
        verification=report,
    )
    if not document["verified"]:
        return None, report
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
    config=None,
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
            robot=robot,
            config=config,
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


def verify_dataset_mapping_jointwise(
    episode,
    robot,
    output,
    *,
    config,
    reference_frame=None,
    sign_test_deg=10.0,
    max_other_joint_delta_deg=5.0,
    report=None,
):
    """Verify one joint at a time from a single reference pose.

    The operator moves the robot to one explicitly selected dataset frame, then
    nudges one joint in the model-positive and model-negative directions.  This
    never enables torque.  The whole-body home pose is not assumed to match any
    model pose, and frame 0 (the model-neutral start) is not selected implicitly.
    """
    if reference_frame is None:
        raise ValueError(
            "joint-wise verification requires an explicit --reference-frame; "
            "choose a dataset frame that is safe and reachable on the real robot"
        )
    if not 0 <= int(reference_frame) < len(episode.frames):
        raise ValueError("reference_frame must be a valid episode frame index")
    if not np.isfinite(sign_test_deg) or not 2 <= sign_test_deg <= 45:
        raise ValueError("sign_test_deg must be in [2, 45]")
    if not np.isfinite(max_other_joint_delta_deg) or not 0 <= max_other_joint_delta_deg <= 20:
        raise ValueError("max_other_joint_delta_deg must be in [0, 20]")
    if Path(output).exists():
        raise FileExistsError(f"Mapping output already exists: {output}")
    reference_frame = int(reference_frame)
    frame = episode.frames[reference_frame]
    if np.any(frame.warning > 0) or float(frame.collision[0]) > 0:
        print(
            f"WARNING: reference frame {reference_frame} has recorded warning={frame.warning.tolist()} "
            f"collision={frame.collision.tolist()}; choose another frame if a cleaner pose is available"
        )
    records_by_side = {side: {} for side in JOINT_SIDES}
    started = False
    try:
        for side in SIDES:
            getattr(robot, side + "_bus").connect()
        started = True
        if not robot.is_calibrated:
            raise ValueError("Motor calibration does not match the robot; recalibrate before verification")
        for side in SIDES:
            getattr(robot, side + "_bus").disable_torque(list(JOINTS))
        direct_drive = input(
            "14関節すべてがサーボホーン直結（減速機・ベルトなし、1:1）であることを確認しましたか？ [y/N]: "
        )
        if direct_drive.strip().lower() not in ("y", "yes"):
            raise ValueError(
                "Joint-wise verification assumes direct-drive 1:1 joints; use --mode poses "
                "or confirm the mechanical coupling before recording a mapping"
            )
        right = np.array2string(frame.joints["right"], precision=2, suppress_small=True)
        left = np.array2string(frame.joints["left"], precision=2, suppress_small=True)
        input(
            "トルクを切った状態で、腕を支え、ロボットを参照フレーム "
            f"{reference_frame} の姿勢に手で合わせてEnter\n"
            f"  right model joint_deg = {right}\n  left  model joint_deg = {left}\n"
            "この1姿勢だけを全身で合わせます。以降は関節ごとに1関節だけ動かします。\n"
        )
        reference_motor = _read_joints(robot)
        current_reference = {side: reference_motor[side].copy() for side in JOINT_SIDES}
        world_axes = model_joint_world_axes(config, frame.joints, episode.metadata.get("obstacles", []))
        for side in JOINT_SIDES:
            bus = getattr(robot, side + "_bus")
            for joint_index, joint in enumerate(JOINTS):
                model_name, model_axis = _model_evidence(config, side, joint_index)
                calibration = calibration_joint_limits(robot, side, joint)
                model_reference = float(frame.joints[side][joint_index])
                world_axis = np.asarray(world_axes[f"{side}_{joint}"], dtype=float)
                print(
                    f"\n[{side}_{joint}] motor id={calibration['motor_id']} "
                    f"model_joint={model_name} axis={np.round(model_axis, 4).tolist()} "
                    f"axis_world={np.round(world_axis, 4).tolist()} "
                    f"model_ref={model_reference:+.2f} deg "
                    f"calibrated=[{calibration['min_motor_deg']:+.2f}, "
                    f"{calibration['max_motor_deg']:+.2f}] deg"
                )
                input(
                    f"{side}_{joint} だけをモデル正方向へ {sign_test_deg:g} deg 以上動かしてEnter "
                    "(他の関節は動かさない; 腕を支える)\n"
                )
                motor_positive = _read_joints(robot)
                input(
                    f"{side}_{joint} だけをモデル負方向へ {sign_test_deg:g} deg 以上動かしてEnter "
                    "(他の関節は動かさない; 腕を支える)\n"
                )
                motor_negative = _read_joints(robot)
                other_joint_violations = {}
                for other_side in JOINT_SIDES:
                    for other_index, other_joint in enumerate(JOINTS):
                        if other_side == side and other_index == joint_index:
                            continue
                        deviation = max(
                            abs(
                                motor_positive[other_side][other_index]
                                - current_reference[other_side][other_index]
                            ),
                            abs(
                                motor_negative[other_side][other_index]
                                - current_reference[other_side][other_index]
                            ),
                        )
                        if deviation > max_other_joint_delta_deg:
                            other_joint_violations[f"{other_side}_{other_joint}"] = float(deviation)
                record = fit_joint_from_reference(
                    model_reference,
                    reference_motor[side][joint_index],
                    motor_positive[side][joint_index],
                    motor_negative[side][joint_index],
                    side=side,
                    joint=joint,
                    motor_id=calibration["motor_id"],
                    motor_model=bus.motors[joint].model,
                    model_axis=model_axis,
                    calibration=calibration,
                    min_test_deg=sign_test_deg,
                    other_joint_violations=other_joint_violations,
                    evidence={
                        "reference_frame": reference_frame,
                        "reference_model_deg": model_reference,
                        "reference_motor_deg": float(reference_motor[side][joint_index]),
                        "model_positive_motor_deg": float(motor_positive[side][joint_index]),
                        "model_negative_motor_deg": float(motor_negative[side][joint_index]),
                        "model_positive_axis_world": world_axis.tolist(),
                        "operator_confirmed_reference_pose": True,
                        "operator_confirmed_direct_drive_1_to_1": True,
                    },
                )
                record["model_joint"] = model_name
                records_by_side[side][joint] = record
                input(
                    f"{side}_{joint} を参照姿勢へ戻してEnter (次の関節の試験前に、他の関節も参照姿勢に戻す)\n"
                )
                returned = _read_joints(robot)
                return_error = abs(returned[side][joint_index] - reference_motor[side][joint_index])
                if return_error > max_other_joint_delta_deg:
                    record["failures"].append(
                        f"{side}_{joint}: did not return to the reference pose "
                        f"({return_error:.2f} deg > {max_other_joint_delta_deg:g})"
                    )
                    record["verified"] = False
                current_reference[side][joint_index] = returned[side][joint_index]
                print(
                    f"  -> sign={record['sign']:+d} offset={record['offset_deg']:+.2f} deg "
                    f"verified={record['verified']}"
                )
                if record["failures"]:
                    print("  failures: " + "; ".join(record["failures"]))
        verification = {
            "schema": MAPPING_VERIFICATION_SCHEMA,
            "status": "ok"
            if all(record["verified"] for records in records_by_side.values() for record in records.values())
            else "failed",
            "dataset": str(episode.root),
            "dataset_sha256": episode.dataset_sha256,
            "manifest_sha256": episode.manifest_sha256,
            "episode": episode.episode,
            "reference_frame": reference_frame,
            "reference_model_joint_deg": {side: frame.joints[side].tolist() for side in JOINT_SIDES},
            "reference_motor_deg": {side: reference_motor[side].tolist() for side in JOINT_SIDES},
            "sign_test_deg": float(sign_test_deg),
            "method": METHOD_REFERENCE_DIRECTION,
            "per_joint": [
                {
                    "side": side,
                    "joint": joint,
                    "motor_id": record["motor_id"],
                    "sign": record["sign"],
                    "offset_deg": record["offset_deg"],
                    "positive_delta_deg": record.get("positive_delta_deg"),
                    "negative_delta_deg": record.get("negative_delta_deg"),
                    "verified": record["verified"],
                    "failures": record["failures"],
                }
                for side in JOINT_SIDES
                for joint, record in records_by_side[side].items()
            ],
            "failures": [
                failure
                for records in records_by_side.values()
                for record in records.values()
                for failure in record["failures"]
            ],
        }
        if report:
            _write_json(report, verification)
        document = _build_mapping_document(
            episode,
            records_by_side,
            robot_id=robot.id,
            calibration_hash=calibration_sha256(robot),
            config=config,
            method=METHOD_REFERENCE_DIRECTION,
            verification=verification,
        )
        if not document["verified"]:
            raise ValueError("Joint-wise mapping verification failed: " + "; ".join(verification["failures"]))
        write_mapping(output, document)
        return verification
    finally:
        if started:
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


def _collision_at_model_joints(simulation, joints):
    """Evaluate the model at arbitrary joints without moving the viewer state."""
    saved = simulation.data.qpos.copy()
    try:
        for side in JOINT_SIDES:
            simulation.data.qpos[simulation.qpos_indices[side][:7]] = np.deg2rad(joints[side])
            simulation.data.qpos[simulation.qpos_indices[side][7]] = 0
        simulation.mujoco.mj_forward(simulation.model, simulation.data)
        return simulation.collision_report()
    finally:
        simulation.data.qpos[:] = saved
        simulation.mujoco.mj_forward(simulation.model, simulation.data)


def evaluate_initial_transition(
    current,
    episode,
    document,
    robot,
    *,
    max_step_deg,
    max_speed_deg_s,
    control_hz,
    simulation=None,
):
    """Evaluate a joint-space move from the current pose to the dataset start.

    This only evaluates.  Execution requires the explicit ``allow_initial_move``
    flag on the replay command.  The modeled path is checked for calibration
    limits and full-model self/floor collisions; external obstacles remain
    unverified.
    """
    document = document.get("document", document) if isinstance(document, dict) else document
    first_model = {side: episode.frames[0].joints[side] for side in JOINT_SIDES}
    current_model = {side: unmap_joints(document, side, current[side]) for side in JOINT_SIDES}
    scale = max(float(max_step_deg), 1e-9)
    delta = max(
        float(np.max(np.abs(current[side] - map_joints(document, side, first_model[side]))))
        for side in JOINT_SIDES
    )
    duration = max(
        delta / float(max_speed_deg_s) if delta > 0 else 0.0,
        delta / (scale * float(control_hz)) if delta > 0 else 0.0,
    )
    steps = max(1, int(np.ceil(duration * control_hz)), int(np.ceil(delta / scale)))
    limit_violations = []
    for side in JOINT_SIDES:
        for index, joint in enumerate(JOINTS):
            limits = calibration_joint_limits(robot, side, joint)
            for label, value in (
                ("current", current[side][index]),
                ("dataset_start", map_joints(document, side, first_model[side])[index]),
            ):
                if not limits["min_motor_deg"] - 1e-6 <= float(value) <= limits["max_motor_deg"] + 1e-6:
                    limit_violations.append(
                        {
                            "side": side,
                            "joint": joint,
                            "pose": label,
                            "value_deg": float(value),
                            "min_deg": limits["min_motor_deg"],
                            "max_deg": limits["max_motor_deg"],
                        }
                    )
    collision_frames = []
    max_penetration_m = 0.0
    body_pairs = {}
    for step in range(steps + 1):
        alpha = step / steps
        model_joints = {
            side: current_model[side] + alpha * (first_model[side] - current_model[side])
            for side in JOINT_SIDES
        }
        collision = _collision_at_model_joints(simulation, model_joints) if simulation is not None else None
        if collision is not None and collision["count"]:
            collision_frames.append(step)
            max_penetration_m = max(max_penetration_m, collision["max_penetration_m"])
            for pair, count in collision["body_pairs"].items():
                body_pairs[pair] = body_pairs.get(pair, 0) + count
    return {
        "required": delta > 0,
        "safe": not limit_violations and not collision_frames,
        "delta_deg": delta,
        "steps": steps,
        "estimated_duration_s": steps / float(control_hz),
        "max_step_deg": float(max_step_deg),
        "max_speed_deg_s": float(max_speed_deg_s),
        "control_hz": float(control_hz),
        "limit_violations": limit_violations,
        "collision_checked": simulation is not None,
        "collision_frames": collision_frames,
        "max_penetration_m": max_penetration_m,
        "body_pairs": body_pairs,
        "scope": (
            "Joint-space linear transition from the measured current pose to the mapped "
            "dataset first frame. Modeled floor/self collisions are checked; external "
            "obstacles and physical joint hard stops are not verified."
        ),
    }


def execute_joint_episode(
    episode,
    document,
    config,
    robot,
    report_path,
    *,
    max_start_error_deg=2.0,
    max_tracking_error_deg=5.0,
    max_joint_speed_deg_s=5.0,
    control_hz=20.0,
    max_sim_error_m=0.025,
    max_sim_orientation_error_deg=None,
    allow_initial_move=False,
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
    model_hash = file_sha256(full_model_path(config["robot"]))
    if episode.metadata.get("mujoco_model_sha256") != model_hash:
        raise ValueError("Dataset MuJoCo model differs from the selected model; regenerate the dataset")
    mapping_info = validate_mapping_for_robot(document, robot, model_sha256=model_hash)
    step_limit = robot.config.max_relative_target
    if not robot.config.use_degrees or not isinstance(step_limit, (int, float)) or not 0 < step_limit <= 0.5:
        raise ValueError("Follower must use calibrated degrees and a per-step limit at most 0.5 deg")
    validation = validate_mapped_episode(episode, document, robot)
    inspection = inspect_joint_mapping(episode, document, robot, config)
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
        "joint_inspection": inspection,
        "pose_context": None,
        "pose_context_after_initial_transition": None,
        "initial_transition": None,
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
            "allow_initial_move": bool(allow_initial_move),
        },
        "validation": validation,
    }
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
        start_errors = {
            side: {
                joint: float(abs(current[side][index] - first[side][index]))
                for index, joint in enumerate(JOINTS)
            }
            for side in SIDES
        }
        start_max = max(value for side in SIDES for value in start_errors[side].values())
        report["pose_context"] = build_pose_context(episode, document, robot, config, current=current)
        report["start_pose"] = {
            "max_error_deg": start_max,
            "per_joint_deg": start_errors,
            "tolerance_deg": max_start_error_deg,
        }
        current_model = {side: unmap_joints(document, side, current[side]) for side in SIDES}
        start_collision = _collision_at_model_joints(simulation, current_model)
        report["start_pose"]["modeled_collision"] = start_collision
        if start_collision["count"]:
            raise ValueError(
                "Robot current pose has a modeled self/floor collision "
                f"(count={start_collision['count']}, "
                f"max_penetration_m={start_collision['max_penetration_m']:.4f}); "
                "move the robot away before replay"
            )
        transition = {
            "required": start_max > max_start_error_deg,
            "applied": False,
            "reason": None if start_max <= max_start_error_deg else "start pose differs",
        }
        report["initial_transition"] = transition
        if start_max > max_start_error_deg and not allow_initial_move:
            worst = sorted(
                ((value, side, joint) for side in SIDES for joint, value in start_errors[side].items()),
                reverse=True,
            )[:5]
            details = ", ".join(f"{side}_{joint}={value:.2f}" for value, side, joint in worst)
            raise ValueError(
                f"Robot start pose differs from the mapped dataset first frame by up to "
                f"{start_max:.2f} deg (limit {max_start_error_deg:g}; {details}). "
                "Move the robot manually under support or review the dry-run and use "
                "--allow-initial-move explicitly."
            )
        if start_max > max_start_error_deg:
            transition = evaluate_initial_transition(
                current,
                episode,
                document,
                robot,
                max_step_deg=step_limit,
                max_speed_deg_s=max_joint_speed_deg_s,
                control_hz=control_hz,
                simulation=simulation,
            )
            transition["applied"] = False
            report["initial_transition"] = transition
            if not transition["safe"]:
                raise ValueError(
                    "Initial transition to the dataset start is not safe: "
                    f"limit_violations={len(transition['limit_violations'])}, "
                    f"modeled_collision_frames={transition['collision_frames'][:10]}; "
                    "move the robot manually under support"
                )
        initial_sent = robot.send_action(_actions(current))
        for side in SIDES:
            getattr(robot, side + "_bus").enable_torque(list(JOINTS))
        report["status"] = "running"
        start_time = time.monotonic()
        next_tick = start_time
        previous_sent = {side: current[side].copy() for side in SIDES}
        if transition["required"]:
            steps = transition["steps"]
            transition_start = {side: current[side].copy() for side in SIDES}
            for substep in range(1, steps + 1):
                alpha = substep / steps
                transition_target = {
                    side: transition_start[side] + alpha * (first[side] - transition_start[side])
                    for side in SIDES
                }
                sent = robot.send_action(_actions(transition_target))
                control_steps += 1
                measured = _read_joints(robot)
                sent_motor = {
                    side: np.array([sent[f"{side}_{name}.pos"] for name in JOINTS], dtype=float)
                    for side in SIDES
                }
                for side in SIDES:
                    error = float(np.max(np.abs(transition_target[side] - measured[side])))
                    if error > max_tracking_error_deg:
                        raise RuntimeError(
                            f"Robot fell behind the initial transition ({side}, {error:.2f} deg): HOLD"
                        )
                    speed = float(np.max(np.abs(sent_motor[side] - previous_sent[side]))) * control_hz
                    max_observed_speed = max(max_observed_speed, speed)
                    previous_sent[side] = sent_motor[side].copy()
                current = measured
                initial_sent = sent
                if realtime:
                    next_tick += 1.0 / control_hz
                    time.sleep(max(0.0, next_tick - time.monotonic()))
            final_errors = {
                side: {
                    joint: float(abs(current[side][index] - first[side][index]))
                    for index, joint in enumerate(JOINTS)
                }
                for side in SIDES
            }
            final_max = max(value for side in SIDES for value in final_errors[side].values())
            transition.update(
                {
                    "applied": True,
                    "final_start_error_deg": final_max,
                    "final_start_error_per_joint_deg": final_errors,
                    "control_steps": steps,
                    "elapsed_s": time.monotonic() - start_time,
                }
            )
            if final_max > max_start_error_deg:
                raise RuntimeError(
                    f"Initial transition did not reach the dataset start pose "
                    f"({final_max:.2f} deg > {max_start_error_deg:g} deg): HOLD"
                )
        report["initial_transition"] = transition
        report["pose_context_after_initial_transition"] = (
            build_pose_context(episode, document, robot, config, current=current)
            if transition.get("applied")
            else None
        )
        initial_frame = episode.frames[0]
        initial_sent_motor = {
            side: np.array([initial_sent[f"{side}_{name}.pos"] for name in JOINTS], dtype=float)
            for side in SIDES
        }
        initial_measured_model = {side: unmap_joints(document, side, current[side]) for side in SIDES}
        initial_sample = {
            "plan_frame": initial_frame.plan_frame,
            "source_frame": initial_frame.source_frame,
            "source_time_s": initial_frame.time_s,
            "elapsed_s": time.monotonic() - start_time,
            "control_steps": control_steps,
            "segment_duration_s": 0.0,
            "requested_duration_s": 1.0 / episode.fps,
            "time_scale": 0.0,
            "arms": {},
        }
        for side in SIDES:
            target_pose = _target_pose_for_frame(simulation, initial_frame, side)
            position, quaternion = simulation.arms[side].solver.fk_solver.compute(
                np.r_[initial_measured_model[side], 0.0]
            )
            target = pose(target_pose[:3], target_pose[3:])
            actual = pose(position, quaternion)
            tcp_position_error = float(np.linalg.norm(position - target[:3, 3]))
            tcp_orientation_error = float(
                np.rad2deg(Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).magnitude())
            )
            if tcp_position_error > max_sim_error_m:
                tcp_warning_frames.append(0)
            if (
                max_sim_orientation_error_deg is not None
                and tcp_orientation_error > max_sim_orientation_error_deg
            ):
                tcp_orientation_warning_frames.append(0)
            initial_sample["arms"][side] = {
                "planned_model_joint_deg": initial_frame.joints[side].tolist(),
                "mapped_motor_joint_deg": map_joints(document, side, initial_frame.joints[side]).tolist(),
                "sent_motor_joint_deg": initial_sent_motor[side].tolist(),
                "measured_motor_joint_deg": current[side].tolist(),
                "measured_model_joint_deg": initial_measured_model[side].tolist(),
                "max_joint_error_deg": float(
                    np.max(np.abs(map_joints(document, side, initial_frame.joints[side]) - current[side]))
                ),
                "tcp_position_error_m": tcp_position_error,
                "tcp_orientation_error_deg": tcp_orientation_error,
                "target_pose_source": "dataset",
            }
        report["frames"].append(initial_sample)
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
        if robot.is_connected and current is not None:
            try:
                robot.send_action(_actions(current))
                report["hold_sent"] = True
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
        transition_elapsed = (
            (report.get("initial_transition") or {}).get("elapsed_s", 0.0)
            if (report.get("initial_transition") or {}).get("applied")
            else 0.0
        )
        report["transition_elapsed_s"] = transition_elapsed
        report["dataset_elapsed_s"] = (
            max(0.0, report["actual_duration_s"] - transition_elapsed) if elapsed else 0.0
        )
        report["time_scale"] = (
            report["dataset_elapsed_s"] / report["source_duration_s"]
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
    max_joint_speed_deg_s=5.0,
    control_hz=20.0,
    max_sim_error_m=0.025,
    max_sim_orientation_error_deg=None,
    check_start=False,
    allow_initial_move=False,
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
    mapping_info = validate_mapping_for_robot(mapping["document"], robot, model_sha256=model_hash)
    validation = validate_mapped_episode(episode, mapping["document"], robot)
    inspection = inspect_joint_mapping(episode, mapping["document"], robot, config)
    if not execute:
        result = {
            "schema": REPLAY_SCHEMA,
            "mode": "validation_only",
            "dataset": str(episode.root),
            "episode": episode.episode,
            "frames": len(episode.frames),
            "mapping": mapping_info,
            "joint_inspection": inspection,
            "pose_context": build_pose_context(episode, mapping["document"], robot, config),
            "initial_transition": None,
            "gripper_commanded": False,
            "simulation_only_source": True,
            "validation": validation,
            "unverified": {
                "joints": inspection["unverified_joints"],
                "model_joint_limits": "full-CAD MJCF limits are unmeasured; preview only",
                "external_obstacles": "not modeled unless --obstacles was used when the dataset was built",
                "physical_tcp_accuracy": "not measured; FK from motor encoders only",
                "mapping_scale": (
                    "multi-pose mappings fit the slope; joint-wise mappings use the "
                    "direct-drive 1:1 scale and mark scale_verified=false"
                ),
            },
        }
        if not check_start:
            return result
        if not left_port or not right_port:
            raise ValueError("--check-start requires --left-port and --right-port")
        step_limit = robot.config.max_relative_target
        if (
            not robot.config.use_degrees
            or not isinstance(step_limit, (int, float))
            or not 0 < step_limit <= 0.5
        ):
            raise ValueError(
                "Follower must use calibrated degrees and a scalar per-step limit at most 0.5 deg"
            )
        current = None
        simulation = None
        try:
            for side in SIDES:
                getattr(robot, side + "_bus").connect()
            if not robot.is_calibrated:
                raise ValueError("Motor calibration does not match the robot; recalibrate before replay")
            current = _read_joints(robot)
            result["pose_context"] = build_pose_context(
                episode, mapping["document"], robot, config, current=current
            )
            simulation = FullMujocoSimulation(
                config["robot"], gui=False, obstacles=episode.metadata.get("obstacles", [])
            )
            transition = evaluate_initial_transition(
                current,
                episode,
                mapping["document"],
                robot,
                max_step_deg=step_limit,
                max_speed_deg_s=max_joint_speed_deg_s,
                control_hz=control_hz,
                simulation=simulation,
            )
            result["initial_transition"] = transition
            result["start_pose"] = {
                "max_error_deg": result["pose_context"]["start_error_deg"]["max"],
                "per_joint_deg": result["pose_context"]["start_error_deg"]["per_joint"],
                "tolerance_deg": max_start_error_deg,
                "within_tolerance": result["pose_context"]["start_error_deg"]["max"] <= max_start_error_deg,
            }
            return result
        finally:
            if simulation is not None:
                simulation.close()
            for side in SIDES:
                bus = getattr(robot, side + "_bus")
                if bus.is_connected:
                    with suppress(Exception):
                        bus.disconnect(disable_torque=True)
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
        allow_initial_move=allow_initial_move,
    )
    if dataset_fingerprint(episode.root, "mocopi_joint_dataset.json") != before:
        raise ValueError("Source joint dataset changed during hardware replay")
    return result
