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

from ..agent.scene import SceneGeometry
from ..full_mujoco import SUFFIX, FullMujocoSimulation, full_model_path
from ..geometry import pose
from .evaluate import _summary
from .fingerprint import dataset_fingerprint, file_sha256
from .real import GRIPPER, JOINTS, SIDES, _actions, _read_joints, arm_and_gripper_follower, arm_only_follower
from .replay_geometry import ReplayGeometry, measured_scene_ready

JOINT_SIDES = ("right", "left")
MAPPING_SCHEMA = "dual_scorpion.mocopi.hardware_joint_mapping.v1"
REPLAY_SCHEMA = "dual_scorpion.mocopi.hardware_dataset_replay.v1"
MAPPING_VERIFICATION_SCHEMA = "dual_scorpion.mocopi.hardware_mapping_verification.v1"
JOINT_VERIFICATION_SCHEMA = "dual_scorpion.mocopi.hardware_joint_verification.v1"
POSE_CONTEXT_SCHEMA = "dual_scorpion.mocopi.hardware_pose_context.v1"
FT_DATASET_SCHEMA = "dual_scorpion.mocopi.ft_motor_dataset.v1"
METHOD_MULTI_POSE = "multi_pose_least_squares"
METHOD_REFERENCE_DIRECTION = "reference_pose_bidirectional_direction"
METHOD_PER_JOINT_REFERENCE = "per_joint_reference_angle_and_direction"
METHOD_PER_JOINT_UNIDIRECTIONAL = "per_joint_reference_unidirectional"
METHOD_PER_JOINT_MULTI_ANGLE = "per_joint_multi_angle_fit"
METHOD_CALIBRATION_MIDPOINT = "lerobot_calibration_midpoint"
METHOD_UNMEASURED = "unmeasured"
METHOD_DIRECTION_ONLY = "direction_only_unmeasured_offset"
CALIBRATION_ASSUMED = "calibration_assumed"
PHYSICALLY_VERIFIED = "physically_verified"
MEASURED_MAPPING_METHODS = {
    METHOD_MULTI_POSE,
    METHOD_REFERENCE_DIRECTION,
    METHOD_PER_JOINT_REFERENCE,
    METHOD_PER_JOINT_UNIDIRECTIONAL,
    METHOD_PER_JOINT_MULTI_ANGLE,
}

# Assembly-based sign assumptions for drive_mode == 0.
DEFAULT_FULL_CAD_SIGNS_DRIVE_MODE_0 = {
    "right": (-1, 1, -1, 1, 1, -1, -1),
    "left": (1, -1, 1, -1, -1, 1, -1),
}


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


def _record_method(record):
    method = record.get("method")
    verification_method = record.get("verification_method")
    if method is not None and verification_method is not None and method != verification_method:
        raise ValueError(
            f"Verification record has conflicting method={method!r} and "
            f"verification_method={verification_method!r}"
        )
    return verification_method if verification_method is not None else method


def _record_evidence(record):
    evidence = record.get("evidence")
    verification_evidence = record.get("verification_evidence")
    if evidence is not None and verification_evidence is not None and evidence != verification_evidence:
        raise ValueError("Verification record has conflicting evidence and verification_evidence")
    return verification_evidence if verification_evidence is not None else evidence


def _assumed_joint(record):
    return bool(record and _record_method(record) == METHOD_CALIBRATION_MIDPOINT)


def _physically_verified_joint(record):
    if not record or record.get("verification_status") == CALIBRATION_ASSUMED:
        return False
    method = _record_method(record)
    if (
        method not in MEASURED_MAPPING_METHODS
        or record.get("verified") is not True
        or record.get("sign_verified") is False
        or record.get("offset_verified") is False
        or record.get("failures")
    ):
        return False
    model, motor = record.get("model_angles_deg"), record.get("motor_angles_deg")
    if (
        not isinstance(model, (tuple, list))
        or not isinstance(motor, (tuple, list))
        or not model
        or len(model) != len(motor)
        or not all(_is_finite_number(value) for value in (*model, *motor))
    ):
        return False
    if len(model) > 1:
        return max(model) > min(model)
    return any(
        _is_finite_number(delta) and delta != 0
        for delta in (record.get("positive_delta_deg"), record.get("negative_delta_deg"))
    )


def mapping_verification_status(document):
    records = [
        (mapping_joint_records(document, side) or {}).get(joint) for side in JOINT_SIDES for joint in JOINTS
    ]
    if document.get("verified") is True and all(map(_physically_verified_joint, records)):
        return PHYSICALLY_VERIFIED
    if any(map(_assumed_joint, records)) and all(
        _assumed_joint(record) or _physically_verified_joint(record) for record in records
    ):
        return CALIBRATION_ASSUMED
    return "partial"


def _mark_assumption(record):
    return {
        **record,
        "verification_status": CALIBRATION_ASSUMED,
        "verified": False,
        "sign_verified": False,
        "offset_verified": False,
        "scale_verified": False,
    }


def _is_valid_sign_value(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and value in (-1, 1, -1.0, 1.0)


def _is_finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and bool(np.isfinite(float(value)))


def validate_mapping_document(document, *, require_verified=True, allow_calibration_assumed=False):
    if document.get("schema") != MAPPING_SCHEMA:
        raise ValueError("Unsupported hardware joint mapping schema")
    if document.get("model_kind") != "full_mujoco":
        raise ValueError("Hardware joint mapping must target the full MuJoCo model")
    doc_verified = document.get("verified")
    if not isinstance(doc_verified, bool):
        raise ValueError("Hardware joint mapping must contain a boolean verified flag")
    if require_verified and not doc_verified and not allow_calibration_assumed:
        raise ValueError("Hardware joint mapping is not verified; run verify-dataset-mapping first")
    sides = document.get("sides")
    if not isinstance(sides, dict) or set(sides) != set(JOINT_SIDES):
        raise ValueError("Mapping must contain right and left sides")
    strict_all = bool(require_verified or doc_verified)
    for side in JOINT_SIDES:
        side_doc = sides[side]
        if not isinstance(side_doc, dict):
            raise ValueError(f"{side} mapping must be a mapping")
        raw_signs = side_doc.get("sign")
        raw_offsets = side_doc.get("offset_deg")
        if not isinstance(raw_signs, (list, tuple, np.ndarray)) or len(raw_signs) != 7:
            raise ValueError(f"{side} mapping sign must contain seven entries")
        if not isinstance(raw_offsets, (list, tuple, np.ndarray)) or len(raw_offsets) != 7:
            raise ValueError(f"{side} mapping offset must contain seven entries")
        for index, (sign_val, offset_val) in enumerate(zip(raw_signs, raw_offsets, strict=True)):
            if sign_val is not None and not _is_valid_sign_value(sign_val):
                raise ValueError(f"{side}_{JOINTS[index]} mapping sign must be -1, +1, or null")
            if offset_val is not None and not _is_finite_number(offset_val):
                raise ValueError(f"{side}_{JOINTS[index]} mapping offset must be finite degrees or null")
            if strict_all and (sign_val is None or offset_val is None):
                raise ValueError(
                    f"{side}_{JOINTS[index]} mapping is not verified (sign or offset is missing)"
                )
        limits = side_doc.get("joint_limits_deg")
        if limits is not None:
            if not isinstance(limits, dict):
                raise ValueError(f"{side} mapping joint limits must be a mapping")
            lower = np.asarray(limits.get("min"), dtype=float)
            upper = np.asarray(limits.get("max"), dtype=float)
            if (
                lower.shape != (7,)
                or upper.shape != (7,)
                or not np.isfinite(lower).all()
                or not np.isfinite(upper).all()
                or not np.all(lower < upper)
            ):
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
                expected_model_joint = f"joint{joint_index}_{SUFFIX[side]}"
                if record.get("model_joint") not in (None, expected_model_joint):
                    raise ValueError(
                        f"{side}_{joint} verification model_joint is {record.get('model_joint')!r}, "
                        f"expected {expected_model_joint!r}"
                    )
                if (
                    not isinstance(record.get("motor_id"), int)
                    or isinstance(record["motor_id"], bool)
                    or record["motor_id"] <= 0
                ):
                    raise ValueError(f"{side}_{joint} verification must contain a positive motor_id")
                if record.get("motor_model") is not None and not isinstance(record["motor_model"], str):
                    raise ValueError(f"{side}_{joint} verification motor_model must be a string")
                _record_method(record)
                _record_evidence(record)
                rec_sign = record.get("sign")
                rec_offset = record.get("offset_deg")
                map_sign = raw_signs[joint_index]
                map_offset = raw_offsets[joint_index]
                if rec_sign is not None and not _is_valid_sign_value(rec_sign):
                    raise ValueError(f"{side}_{joint} verification sign must be -1 or +1")
                if rec_sign is not None and map_sign is not None and int(rec_sign) != int(map_sign):
                    raise ValueError(f"{side}_{joint} verification sign disagrees with the mapping")
                if (rec_sign is None) != (map_sign is None) and record.get("verified") is True:
                    raise ValueError(f"{side}_{joint} verification sign disagrees with the mapping")
                if rec_offset is not None and not _is_finite_number(rec_offset):
                    raise ValueError(f"{side}_{joint} verification offset must be finite")
                if (
                    rec_offset is not None
                    and map_offset is not None
                    and abs(float(rec_offset) - float(map_offset)) > 1e-6
                ):
                    raise ValueError(f"{side}_{joint} verification offset disagrees with the mapping")
                if (rec_offset is None) != (map_offset is None) and record.get("verified") is True:
                    raise ValueError(f"{side}_{joint} verification offset disagrees with the mapping")
                if not isinstance(record.get("verified"), bool):
                    raise ValueError(f"{side}_{joint} verification must contain a boolean verified flag")
                if record["verified"] is True:
                    if map_sign is None or map_offset is None:
                        raise ValueError(f"{side}_{joint} is marked verified but lacks sign or offset")
                    if record.get("failures"):
                        raise ValueError(
                            f"{side}_{joint} is marked verified while failures remain: {record['failures']}"
                        )
        if strict_all:
            if records is None:
                raise ValueError(
                    "Hardware joint mapping lacks per-joint verification records; "
                    "re-run verify-dataset-mapping so every joint is checked"
                )
            for joint in JOINTS:
                record = records[joint]
                if _assumed_joint(record) and (allow_calibration_assumed or not require_verified):
                    if not record.get("failures"):
                        continue
                elif _physically_verified_joint(record):
                    continue
                raise ValueError(
                    f"Hardware joint mapping has an unverified joint: {side}_{joint}; "
                    "mapping is not verified for motion"
                )
    if (
        require_verified
        and not doc_verified
        and (not allow_calibration_assumed or mapping_verification_status(document) != CALIBRATION_ASSUMED)
    ):
        raise ValueError("Hardware joint mapping is not verified; run verify-dataset-mapping first")
    return document


def _side_mapping_vectors(document, side):
    side_doc = document["sides"][side]
    raw_signs = side_doc["sign"]
    raw_offsets = side_doc["offset_deg"]
    if any(not _is_valid_sign_value(val) for val in raw_signs):
        raise ValueError(f"{side} mapping contains unverified or invalid sign")
    if any(not _is_finite_number(val) for val in raw_offsets):
        raise ValueError(f"{side} mapping contains unverified or non-finite offset_deg")
    return np.asarray(raw_signs, dtype=float), np.asarray(raw_offsets, dtype=float)


def map_joints(document, side, model_joints):
    sign, offset = _side_mapping_vectors(document, side)
    return sign * np.asarray(model_joints, dtype=float) + offset


def unmap_joints(document, side, motor_joints):
    sign, offset = _side_mapping_vectors(document, side)
    return (np.asarray(motor_joints, dtype=float) - offset) / sign


def calibration_sha256(robot):
    path = Path(robot.calibration_fpath)
    if not path.is_file():
        raise ValueError(f"Calibration file not found for robot {robot.id}: {path}")
    return file_sha256(path)


def _calibration_signature(calibration):
    return {
        name: [
            calibration[name].id,
            calibration[name].homing_offset,
            calibration[name].range_min,
            calibration[name].range_max,
        ]
        for name in JOINTS
        if name in calibration
    }


def _connect_checked_buses(robot, results, *, sides=SIDES):
    """Identify each bus by its live register signature before any write."""
    buses = {side: getattr(robot, side + "_bus") for side in SIDES}
    devices = {side: str(Path(buses[side].port).resolve()) for side in sides}
    if len(set(devices.values())) != len(devices):
        raise ValueError("Left and right ports resolve to the same serial device")
    expected = {side: _calibration_signature(bus.calibration) for side, bus in buses.items()}
    for side in sides:
        bus = buses[side]
        results[side] = {"port": str(bus.port), "resolved_device": devices[side]}
        bus.connect()
        live = _calibration_signature(bus.read_calibration())
        matches = [label for label, signature in expected.items() if signature == live]
        results[side].update(
            calibration_registers=live,
            register_order=["motor_id", "homing_offset", "range_min", "range_max"],
            matching_sides=matches,
            calibration_matches_out_of_7={
                label: sum(name in live and live[name] == signature[name] for name in JOINTS)
                for label, signature in expected.items()
            },
        )
    for side in sides:
        matches = results[side]["matching_sides"]
        if matches != [side]:
            if len(matches) == 1:
                raise ValueError(
                    f"{side} port matches the {matches[0]} arm calibration; check left/right wiring"
                )
            raise ValueError(
                f"{side} port has no unique matching calibration signature; verify the USB wiring and "
                "read the saved calibration before deciding whether to recalibrate"
            )


def _read_bus_state(robot, results):
    for side in SIDES:
        bus = getattr(robot, side + "_bus")
        results[side].update(
            operating_mode=bus.sync_read("Operating_Mode", normalize=False),
            torque_enable=bus.sync_read("Torque_Enable", normalize=False),
        )
    return all(
        row["operating_mode"].get(joint) == 0 and row["torque_enable"].get(joint) in (0, 1)
        for row in results.values()
        for joint in JOINTS
    )


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
        "midpoint_raw": float(midpoint),
        "resolution": resolution,
        "drive_mode": int(getattr(calibration, "drive_mode", 0)),
        "homing_offset": int(getattr(calibration, "homing_offset", 0)),
        "motor_id": int(getattr(calibration, "id", bus.motors[joint].id)),
        "zero_motor_deg": 0.0,
        "min_motor_deg": float(lower),
        "max_motor_deg": float(upper),
        "span_deg": float(upper - lower),
    }


def motor_pose_limit_violations(motor_joints_by_side, document, robot, *, pose_label="pose"):
    """Return calibrated motor half-range and configured joint limit violations."""
    document = document.get("document", document) if isinstance(document, dict) else document
    violations = []
    for side in JOINT_SIDES:
        limits_cfg = document["sides"][side].get("joint_limits_deg") if document else None
        values = np.asarray(motor_joints_by_side[side], dtype=float)
        for index, joint in enumerate(JOINTS):
            cal = calibration_joint_limits(robot, side, joint)
            value = float(values[index])
            if not np.isfinite(value) or not (
                cal["min_motor_deg"] - 1e-6 <= value <= cal["max_motor_deg"] + 1e-6
            ):
                violations.append(
                    {
                        "side": side,
                        "joint": joint,
                        "pose": pose_label,
                        "limit_source": "calibration_half_range",
                        "value_deg": value,
                        "min_deg": cal["min_motor_deg"],
                        "max_deg": cal["max_motor_deg"],
                    }
                )
            if limits_cfg is not None:
                lower = float(np.asarray(limits_cfg["min"])[index])
                upper = float(np.asarray(limits_cfg["max"])[index])
                if not np.isfinite(value) or not (lower - 1e-6 <= value <= upper + 1e-6):
                    violations.append(
                        {
                            "side": side,
                            "joint": joint,
                            "pose": pose_label,
                            "limit_source": "configured_joint_limits_deg",
                            "value_deg": value,
                            "min_deg": lower,
                            "max_deg": upper,
                        }
                    )
    return violations


def validate_mapping_for_robot(document, robot, *, model_sha256=None, allow_calibration_assumed=False):
    validate_mapping_document(document, allow_calibration_assumed=allow_calibration_assumed)
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
        for joint_index, joint in enumerate(JOINTS):
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
            expected_model_joint = f"joint{joint_index}_{SUFFIX[side]}"
            if record.get("model_joint") not in (None, expected_model_joint):
                raise ValueError(
                    f"Mapping model_joint for {side}_{joint} is {record.get('model_joint')!r}, "
                    f"expected {expected_model_joint!r}"
                )
            motor_ids[f"{side}_{joint}"] = int(motor.id)
    verification_status = mapping_verification_status(document)
    return {
        "robot_id": robot.id,
        "calibration_sha256": current_hash,
        "verification_status": verification_status,
        "verified_at": document.get("created_at") if verification_status == PHYSICALLY_VERIFIED else None,
        "verification": document.get("verification"),
        "motor_ids": motor_ids,
    }


def validate_mapped_episode(episode, document, robot, *, max_source_step_deg=None):
    document = document.get("document", document)
    validate_mapping_document(document, allow_calibration_assumed=True)
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


def _describe_axis(axis):
    vector = np.asarray(axis, dtype=float)
    index = int(np.argmax(np.abs(vector)))
    names = ("X", "Y", "Z")
    sign = "+" if vector[index] >= 0 else "-"
    return f"{sign}{names[index]}"


def _cad_zero_geometry_summary(model_joint, axis, parent_link, child_link, child_to_next_m):
    axis_vec = np.asarray(axis, dtype=float)
    axis_norm = float(np.linalg.norm(axis_vec))
    axis_unit = axis_vec / axis_norm if axis_norm > 0 else np.array([0.0, 0.0, 1.0])
    v_out = np.asarray(child_to_next_m, dtype=float) if child_to_next_m is not None else None
    out_norm = float(np.linalg.norm(v_out)) if v_out is not None else 0.0
    if v_out is not None and out_norm > 1e-6:
        out_unit = v_out / out_norm
        tangent = np.cross(axis_unit, out_unit)
        tangent_norm = float(np.linalg.norm(tangent))
        if tangent_norm > 0.25:
            motion_desc = f"positive rotation swings {child_link} toward {_describe_axis(tangent)}"
        else:
            motion_desc = f"axial roll/yaw around {_describe_axis(axis_unit)} (right-hand rule)"
        out_desc = f"child link extends primarily toward {_describe_axis(out_unit)} at model zero"
    else:
        motion_desc = f"positive rotation follows right-hand rule around {_describe_axis(axis_unit)}"
        out_desc = "terminal link at model zero"
    joint_notes = {
        "joint0": "Shoulder pitch: at q_model=0 upper arm hangs down (-Z); q_model=-90 points forward (+Y).",
        "joint1": "Shoulder yaw/roll: at q_model=0 downstream bracket faces forward (+Y), axis parallel to shoulder.",
        "joint2": "Shoulder roll/abduction: at q_model=0 upper arm aligns straight down (-Z) with link_2.",
        "joint3": "Elbow pitch: at q_model=0 forearm is bent ~90 deg forward (+Y) from upper arm (-Z); q_model=+90 is straight down.",
        "joint4": "Forearm roll: at q_model=0 wrist pitch axis is parallel to elbow pitch axis (bracket square/un-twisted).",
        "joint5": "Wrist pitch: at q_model=0 wrist extends straight inline (+Y) with forearm.",
        "joint6": "Wrist roll: at q_model=0 gripper mount sits flat/horizontal relative to wrist bracket.",
    }
    base_name = model_joint.split("_", 1)[0]
    return {
        "parent_link": parent_link,
        "child_link": child_link,
        "local_axis_dominant": _describe_axis(axis_unit),
        "positive_rotation_effect": motion_desc,
        "zero_link_direction": out_desc,
        "cad_reference_note": joint_notes.get(base_name, ""),
        "note": "CAD geometry defines q_model=0 only; real motor sign and offset_deg require physical verification.",
    }


@lru_cache(maxsize=8)
def _model_joint_table_cached(path):
    tree = ET.parse(path)
    root = tree.getroot()
    joint_info = {}
    children_map = {}

    def _walk(body_elem, parent_name):
        body_name = body_elem.get("name")
        pos = [float(v) for v in body_elem.get("pos", "0 0 0").split()]
        if parent_name is not None and body_name is not None:
            children_map.setdefault(parent_name, []).append((body_name, pos))
        for joint in body_elem.findall("joint"):
            jname = joint.get("name")
            if jname:
                axis = [float(v) for v in joint.get("axis", "0 0 1").split()]
                joint_info[jname] = {
                    "axis": axis,
                    "body": body_name,
                    "parent_link": parent_name,
                    "child_link": body_name,
                    "child_origin_m": pos,
                }
        for child_body in body_elem.findall("body"):
            _walk(child_body, body_name)

    worldbody = root.find("worldbody")
    if worldbody is not None:
        for top_body in worldbody.findall("body"):
            _walk(top_body, "world")

    table = {}
    for joint in root.iter("joint"):
        name = joint.get("name")
        if not name:
            continue
        info = dict(
            joint_info.get(
                name,
                {
                    "axis": [float(v) for v in joint.get("axis", "0 0 1").split()],
                    "body": None,
                    "parent_link": None,
                    "child_link": None,
                    "child_origin_m": None,
                },
            )
        )
        child_link = info.get("child_link")
        next_children = children_map.get(child_link, []) if child_link else []
        child_to_next_m = next_children[0][1] if next_children else None
        info["child_to_next_m"] = child_to_next_m
        info["cad_zero_geometry"] = _cad_zero_geometry_summary(
            name,
            info["axis"],
            info.get("parent_link"),
            child_link,
            child_to_next_m,
        )
        table[name] = info
    return table


def model_joint_table(config):
    """Read model joint axes and parent/child geometry without importing MuJoCo."""
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
    """Build the per-joint table used by inspection, dry-run, and verification reports."""
    document = document.get("document", document) if isinstance(document, dict) else document
    model_path, model_table = model_joint_table(config)
    sides = {}
    verified_joints = []
    unverified = []
    for side in JOINT_SIDES:
        records = mapping_joint_records(document, side) if document else None
        side_doc = document["sides"][side] if document else None
        limits_cfg = side_doc.get("joint_limits_deg") if side_doc else None
        bus = getattr(robot, side + "_bus")
        items = []
        for joint_index, joint in enumerate(JOINTS):
            calibration = calibration_joint_limits(robot, side, joint)
            model_name = f"joint{joint_index}_{SUFFIX[side]}"
            if model_name not in model_table:
                raise ValueError(f"MuJoCo model has no joint {model_name}")
            model_info = model_table[model_name]
            record = records[joint] if records else None
            if _assumed_joint(record):
                record = _mark_assumption(record)
            raw_sign = (
                record["sign"]
                if record and record.get("sign") is not None
                else (side_doc["sign"][joint_index] if side_doc else None)
            )
            raw_offset = (
                record["offset_deg"]
                if record and record.get("offset_deg") is not None
                else (side_doc["offset_deg"][joint_index] if side_doc else None)
            )
            sign = int(raw_sign) if _is_valid_sign_value(raw_sign) else None
            offset = float(raw_offset) if _is_finite_number(raw_offset) else None
            model_at_calibration_zero = None
            calibration_model_min = None
            calibration_model_max = None
            if sign in (-1, 1) and offset is not None:
                model_at_calibration_zero = float((calibration["zero_motor_deg"] - offset) / sign)
                calibration_model_min = float((calibration["min_motor_deg"] - offset) / sign)
                calibration_model_max = float((calibration["max_motor_deg"] - offset) / sign)
            verified = _physically_verified_joint(record)
            sign_verified = bool(
                record.get("sign_verified")
                if record and "sign_verified" in record
                else (verified and sign in (-1, 1))
            )
            offset_verified = bool(
                record.get("offset_verified")
                if record and "offset_verified" in record
                else (verified and offset is not None)
            )
            scale_verified = bool(record.get("scale_verified")) if record else False
            if verified:
                verified_joints.append(f"{side}_{joint}")
            else:
                unverified.append(f"{side}_{joint}")
            status = (
                PHYSICALLY_VERIFIED
                if verified
                else CALIBRATION_ASSUMED
                if _assumed_joint(record)
                else "partial_sign_only"
                if sign_verified and not offset_verified
                else "failed"
                if record and record.get("failures")
                else "unverified"
            )
            configured_limits = (
                [float(limits_cfg["min"][joint_index]), float(limits_cfg["max"][joint_index])]
                if limits_cfg is not None
                else None
            )
            motor_dir = "same_as_model" if sign == 1 else "opposite_to_model" if sign == -1 else "unverified"
            items.append(
                {
                    "joint": joint,
                    "joint_name": f"{side}_{joint}",
                    "dataset_index": joint_index + (0 if side == "right" else 7),
                    "motor_id": calibration["motor_id"],
                    "motor_model": bus.motors[joint].model,
                    "drive_mode": calibration["drive_mode"],
                    "motor_direction": "normal (drive_mode=0)"
                    if calibration["drive_mode"] == 0
                    else "inverted (drive_mode=1)",
                    "model_to_motor_direction": motor_dir,
                    "homing_offset": calibration["homing_offset"],
                    "raw_range": [calibration["raw_min"], calibration["raw_max"]],
                    "calibration_midpoint_raw": calibration["midpoint_raw"],
                    "lerobot_normalization": (
                        "degrees = (-1 if drive_mode else 1) * (raw - midpoint) * 360 / (resolution - 1)"
                    ),
                    "model_joint": model_name,
                    "parent_link": model_info.get("parent_link"),
                    "child_link": model_info.get("child_link"),
                    "model_body": model_info.get("body"),
                    "model_axis": model_info.get("axis"),
                    "model_zero_deg": 0.0,
                    "cad_zero_geometry": model_info.get("cad_zero_geometry"),
                    "calibration_zero_motor_deg": calibration["zero_motor_deg"],
                    "calibration_min_motor_deg": calibration["min_motor_deg"],
                    "calibration_max_motor_deg": calibration["max_motor_deg"],
                    "calibration_span_deg": calibration["span_deg"],
                    "configured_joint_limits_deg": configured_limits,
                    "model_at_calibration_zero_deg": model_at_calibration_zero,
                    "calibration_model_min_deg": calibration_model_min,
                    "calibration_model_max_deg": calibration_model_max,
                    "sign": sign,
                    "offset_deg": offset,
                    "sign_verified": sign_verified,
                    "offset_verified": offset_verified,
                    "scale_verified": scale_verified,
                    "verified": verified,
                    "verification_status": status,
                    "verification_method": _record_method(record) if record else None,
                    "verification_evidence": _record_evidence(record) if record else None,
                    "failures": list(record.get("failures", [])) if record else [],
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
        current_table = {}
        for side in JOINT_SIDES:
            model_vals = None
            if document is not None:
                model_vals = []
                for idx in range(7):
                    item = sides[side][idx]
                    if item["sign"] in (-1, 1) and item["offset_deg"] is not None:
                        model_vals.append(float((current[side][idx] - item["offset_deg"]) / item["sign"]))
                    else:
                        model_vals.append(None)
            current_table[side] = {
                "motor_deg": current[side].tolist(),
                "model_deg": model_vals,
            }
    return {
        "schema": "dual_scorpion.mocopi.hardware_joint_inspection.v1",
        "model_path": model_path,
        "sides": sides,
        "left_right_pairing": pairing,
        "verified_joints": verified_joints,
        "unverified_joints": unverified,
        "all_joints_verified": not unverified and document is not None and document.get("verified") is True,
        "robot_current_pose": current_table,
        "scope": (
            "Joint order, model axes, parent/child links, motor ids, and calibration ranges are read "
            "from the model and calibration file. Signs and offsets are only confirmed by recorded "
            "per-joint verification; unmeasured joints stay null/unverified."
        ),
    }


def build_pose_context(episode, document, robot, config, *, current=None):
    """Keep model zero, calibration zero, current pose, and dataset start distinct."""
    document = document.get("document", document) if isinstance(document, dict) else document
    inspection = inspect_joint_mapping(episode, document, robot, config, current=current)
    first = episode.frames[0]
    calibration_zero = {}
    dataset_initial_pose = {}
    for side in JOINT_SIDES:
        cal_model = []
        mapped_start = []
        for index in range(7):
            item = inspection["sides"][side][index]
            sign = item["sign"]
            offset = item["offset_deg"]
            if sign in (-1, 1) and offset is not None:
                cal_model.append(float((0.0 - offset) / sign))
                mapped_start.append(float(sign * first.joints[side][index] + offset))
            else:
                cal_model.append(None)
                mapped_start.append(None)
        calibration_zero[side] = {"motor_deg": [0.0] * 7, "model_deg": cal_model}
        dataset_initial_pose[side] = {
            "model_deg": first.joints[side].tolist(),
            "mapped_motor_deg": mapped_start,
        }
    context = {
        "schema": POSE_CONTEXT_SCHEMA,
        "model_zero_deg": {side: [0.0] * 7 for side in JOINT_SIDES},
        "calibration_zero": calibration_zero,
        "dataset_initial_pose": dataset_initial_pose,
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
        mapped_errors = []
        for side in JOINT_SIDES:
            errors[side] = {}
            for index, joint in enumerate(JOINTS):
                target_val = dataset_initial_pose[side]["mapped_motor_deg"][index]
                if target_val is not None:
                    err = float(abs(current[side][index] - target_val))
                    errors[side][joint] = err
                    mapped_errors.append(err)
                else:
                    errors[side][joint] = None
        context["start_error_deg"] = {
            "max": max(mapped_errors) if mapped_errors else None,
            "per_joint": errors,
        }
    return context


def inspect_dataset_mapping(
    episode,
    robot,
    config,
    *,
    mapping_path=None,
    document=None,
    check_start=False,
):
    """Run read-only joint inspection, optionally reading current angles without altering torque."""
    if document is None and mapping_path is not None:
        document = load_mapping(mapping_path)["document"]
    elif isinstance(document, dict) and "document" in document:
        document = document["document"]
    if document is not None:
        validate_mapping_document(document, require_verified=False)

    current = None
    ports = {}
    if check_start:
        try:
            _connect_checked_buses(robot, ports)
            current = _read_joints(robot)
        finally:
            for side in SIDES:
                bus = getattr(robot, side + "_bus")
                if bus.is_connected:
                    with suppress(Exception):
                        bus.disconnect(disable_torque=False)

    inspection = inspect_joint_mapping(episode, document, robot, config, current=current)
    return {
        "schema": "dual_scorpion.mocopi.hardware_joint_inspection.v1",
        "dataset": str(episode.root),
        "episode": episode.episode,
        "mapping": str(mapping_path) if mapping_path else None,
        "mapping_verified": inspection["all_joints_verified"],
        "usb_ports": ports,
        "inspection": inspection,
        "pose_context": (
            build_pose_context(episode, document, robot, config, current=current)
            if document is not None
            else None
        ),
        "robot_current_motor_deg": (
            {side: current[side].tolist() for side in JOINT_SIDES} if current is not None else None
        ),
        "scope": (
            "Read-only inspection. Constructing the follower loads the existing calibration file; "
            "no motor command, enable_torque, or disable_torque call is made."
        ),
    }


def _model_evidence(config, side, joint_index):
    if config is None:
        return None, None, None, None
    record = model_joint_record(config, side, joint_index)
    return record["model_joint"], record["axis"], record.get("parent_link"), record.get("child_link")


def _parse_model_direction(value):
    if isinstance(value, (int, float)) and not isinstance(value, bool) and value in (1, -1, 1.0, -1.0):
        return int(value)
    text = str(value).strip().lower()
    if text in ("+", "+1", "1", "pos", "positive"):
        return 1
    if text in ("-", "-1", "neg", "negative"):
        return -1
    raise ValueError(f"Expected model direction '+'/'-1' or '-'/+1, got {value!r}")


def _direction_summary(
    sign,
    positive_delta,
    negative_delta,
    failures,
    *,
    direction_mode,
    extra=None,
):
    payload = {
        "sign": sign,
        "motor_direction": "same_as_model"
        if sign == 1
        else "opposite_to_model"
        if sign == -1
        else "unverified",
        "direction_mode": direction_mode,
        "positive_delta_deg": positive_delta,
        "negative_delta_deg": negative_delta,
        "sign_verified": not failures and sign in (-1, 1),
        "failures": failures,
    }
    if extra:
        payload.update(extra)
    return payload


def verify_joint_direction(
    motor_baseline_deg,
    motor_positive_deg,
    motor_negative_deg,
    *,
    side,
    joint,
    min_test_deg=10.0,
    other_joint_violations=None,
):
    """Verify motor sign independently from opposite-direction motion around the joint axis."""
    values = np.asarray([motor_baseline_deg, motor_positive_deg, motor_negative_deg], dtype=float)
    if not np.isfinite(values).all():
        raise ValueError(f"{side}_{joint} direction test readings must be finite")
    if not np.isfinite(min_test_deg) or not 0 < min_test_deg <= 90:
        raise ValueError("min_test_deg must be in (0, 90]")
    positive_delta = float(motor_positive_deg - motor_baseline_deg)
    negative_delta = float(motor_negative_deg - motor_baseline_deg)
    failures = []
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
    if positive_delta > 0 and negative_delta < 0:
        sign = 1
    elif positive_delta < 0 and negative_delta > 0:
        sign = -1
    else:
        sign = None
    return _direction_summary(
        sign,
        positive_delta,
        negative_delta,
        failures,
        direction_mode="bidirectional",
    )


def verify_joint_direction_unidirectional(
    motor_start_deg,
    motor_moved_deg,
    *,
    expected_model_direction,
    side,
    joint,
    min_test_deg=10.0,
    other_joint_violations=None,
):
    """Verify motor sign from a single unobstructed movement away from a desk/stop."""
    values = np.asarray([motor_start_deg, motor_moved_deg], dtype=float)
    if not np.isfinite(values).all():
        raise ValueError(f"{side}_{joint} unidirectional test readings must be finite")
    if not np.isfinite(min_test_deg) or not 0 < min_test_deg <= 90:
        raise ValueError("min_test_deg must be in (0, 90]")
    dir_int = _parse_model_direction(expected_model_direction)
    observed_delta = float(motor_moved_deg - motor_start_deg)
    failures = []
    if abs(observed_delta) < min_test_deg:
        failures.append(
            f"{side}_{joint}: unidirectional direction test moved only {observed_delta:+.2f} deg "
            f"(need {min_test_deg:g})"
        )
    if other_joint_violations:
        failures.append(
            f"{side}_{joint}: other joints moved during the direction test: "
            + ", ".join(sorted(other_joint_violations))
        )
    sign = (1 if observed_delta * dir_int > 0 else -1) if abs(observed_delta) >= 1e-6 else None
    return _direction_summary(
        sign,
        observed_delta if dir_int == 1 else None,
        observed_delta if dir_int == -1 else None,
        failures,
        direction_mode="unidirectional",
        extra={
            "expected_model_direction": dir_int,
            "observed_delta_deg": observed_delta,
            "start_motor_deg": float(motor_start_deg),
            "moved_motor_deg": float(motor_moved_deg),
        },
    )


def build_unmeasured_joint_record(
    *,
    side,
    joint,
    motor_id,
    motor_model=None,
    model_joint=None,
    parent_link=None,
    child_link=None,
    model_axis=None,
    calibration=None,
    sign=None,
    sign_verified=False,
    positive_delta_deg=None,
    negative_delta_deg=None,
    reason="reference model angle is unmeasured",
    evidence=None,
    failures=None,
):
    """Create a structured unverified record without guessing unknown sign or offset."""
    method = METHOD_DIRECTION_ONLY if sign_verified else METHOD_UNMEASURED
    ev = dict(evidence or {})
    ev.setdefault("reason", reason)
    record_failures = list(failures or [])
    if not record_failures:
        record_failures.append(f"{side}_{joint}: {reason}")
    return {
        "schema": JOINT_VERIFICATION_SCHEMA,
        "side": side,
        "joint": joint,
        "motor_id": int(motor_id),
        "motor_model": motor_model,
        "model_joint": model_joint,
        "parent_link": parent_link,
        "child_link": child_link,
        "method": method,
        "verification_method": method,
        "verified": False,
        "sign_verified": bool(sign_verified),
        "offset_verified": False,
        "model_axis": model_axis,
        "calibration": calibration,
        "model_angles_deg": [],
        "motor_angles_deg": [],
        "model_range_deg": None,
        "slope": float(sign) if sign in (-1, 1) else None,
        "sign": int(sign) if sign in (-1, 1) else None,
        "motor_direction": "same_as_model"
        if sign == 1
        else "opposite_to_model"
        if sign == -1
        else "unverified",
        "offset_deg": None,
        "rms_deg": None,
        "max_residual_deg": None,
        "positive_delta_deg": positive_delta_deg,
        "negative_delta_deg": negative_delta_deg,
        "scale_basis": "unverified",
        "scale_verified": False,
        "evidence": ev,
        "verification_evidence": ev,
        "failures": record_failures,
    }


def fit_joint_verification(
    model_angles,
    motor_angles,
    *,
    side,
    joint,
    motor_id,
    motor_model=None,
    method=METHOD_MULTI_POSE,
    model_joint=None,
    parent_link=None,
    child_link=None,
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
    ev = dict(evidence or {})
    record = {
        "schema": JOINT_VERIFICATION_SCHEMA,
        "side": side,
        "joint": joint,
        "motor_id": int(motor_id),
        "motor_model": motor_model,
        "model_joint": model_joint,
        "parent_link": parent_link,
        "child_link": child_link,
        "method": method,
        "verification_method": method,
        "verified": not failures,
        "sign_verified": not failures,
        "offset_verified": not failures,
        "model_axis": model_axis,
        "calibration": calibration,
        "model_angles_deg": x.tolist(),
        "motor_angles_deg": y.tolist(),
        "model_range_deg": span,
        "slope": float(slope),
        "sign": sign,
        "motor_direction": "same_as_model" if sign == 1 else "opposite_to_model",
        "offset_deg": offset,
        "rms_deg": rms,
        "max_residual_deg": maximum,
        "max_slope_error": float(max_slope_error),
        "scale_basis": "direct_drive_1_to_1" if method == METHOD_REFERENCE_DIRECTION else "fitted",
        "scale_verified": method != METHOD_REFERENCE_DIRECTION,
        "evidence": ev,
        "verification_evidence": ev,
        "failures": failures,
    }
    return record


def fit_joint_from_reference(
    model_reference_deg,
    motor_reference_deg,
    motor_positive_deg=None,
    motor_negative_deg=None,
    *,
    side,
    joint,
    motor_id,
    motor_model=None,
    model_joint=None,
    parent_link=None,
    child_link=None,
    model_axis=None,
    calibration=None,
    min_test_deg=5.0,
    motor_baseline_deg=None,
    other_joint_violations=None,
    direct_drive_confirmed=True,
    measurement_basis=None,
    method=METHOD_REFERENCE_DIRECTION,
    evidence=None,
    direction_result=None,
):
    """Verify one joint from a grounded reference angle and bidirectional or unidirectional motion."""
    if not np.isfinite([float(model_reference_deg), float(motor_reference_deg)]).all():
        raise ValueError(f"{side}_{joint} reference readings must be finite")
    if direction_result is None:
        if motor_positive_deg is None or motor_negative_deg is None:
            raise ValueError(f"{side}_{joint} requires bidirectional readings or direction_result")
        baseline = motor_reference_deg if motor_baseline_deg is None else motor_baseline_deg
        if not np.isfinite([float(baseline), float(motor_positive_deg), float(motor_negative_deg)]).all():
            raise ValueError(f"{side}_{joint} direction test readings must be finite")
        direction = verify_joint_direction(
            baseline,
            motor_positive_deg,
            motor_negative_deg,
            side=side,
            joint=joint,
            min_test_deg=min_test_deg,
            other_joint_violations=other_joint_violations,
        )
    else:
        direction = direction_result
    failures = list(direction["failures"])
    if not direct_drive_confirmed:
        failures.append(f"{side}_{joint}: direct-drive 1:1 coupling was not confirmed")
    if measurement_basis is not None and not str(measurement_basis).strip():
        failures.append(f"{side}_{joint}: measurement_basis is empty")
    sign = direction["sign"] if direction["sign"] in (-1, 1) else None
    offset = float(motor_reference_deg - sign * model_reference_deg) if sign in (-1, 1) else None
    ev = dict(evidence or {})
    if measurement_basis is not None:
        ev.setdefault("measurement_basis", str(measurement_basis).strip())
    ev.setdefault("direction_mode", direction.get("direction_mode", "bidirectional"))
    ev.setdefault("reference_model_deg", float(model_reference_deg))
    ev.setdefault("reference_motor_deg", float(motor_reference_deg))
    if direction_result is None:
        baseline = motor_reference_deg if motor_baseline_deg is None else motor_baseline_deg
        ev.setdefault("direction_baseline_motor_deg", float(baseline))
        ev.setdefault("model_positive_motor_deg", float(motor_positive_deg))
        ev.setdefault("model_negative_motor_deg", float(motor_negative_deg))
    else:
        ev.setdefault("direction_test", direction)
    verified = bool(not failures and direction["sign_verified"] and sign in (-1, 1) and offset is not None)
    return {
        "schema": JOINT_VERIFICATION_SCHEMA,
        "side": side,
        "joint": joint,
        "motor_id": int(motor_id),
        "motor_model": motor_model,
        "model_joint": model_joint,
        "parent_link": parent_link,
        "child_link": child_link,
        "method": method,
        "verification_method": method,
        "verified": verified,
        "sign_verified": bool(direction["sign_verified"] and sign in (-1, 1)),
        "offset_verified": verified,
        "model_axis": model_axis,
        "calibration": calibration,
        "model_angles_deg": [float(model_reference_deg)],
        "motor_angles_deg": [float(motor_reference_deg)],
        "model_range_deg": None,
        "slope": float(sign) if sign in (-1, 1) else None,
        "sign": int(sign) if sign in (-1, 1) else None,
        "motor_direction": direction["motor_direction"],
        "offset_deg": offset,
        "rms_deg": 0.0 if offset is not None else None,
        "max_residual_deg": 0.0 if offset is not None else None,
        "positive_delta_deg": direction["positive_delta_deg"],
        "negative_delta_deg": direction["negative_delta_deg"],
        "min_test_deg": float(min_test_deg),
        "other_joint_violations": other_joint_violations or {},
        "scale_basis": "direct_drive_1_to_1",
        "scale_verified": False,
        "evidence": ev,
        "verification_evidence": ev,
        "failures": failures,
    }


def fit_joint_from_unidirectional_reference(
    model_reference_deg,
    motor_reference_deg,
    motor_start_deg,
    motor_moved_deg,
    *,
    expected_model_direction,
    side,
    joint,
    motor_id,
    motor_model=None,
    model_joint=None,
    parent_link=None,
    child_link=None,
    model_axis=None,
    calibration=None,
    min_test_deg=10.0,
    other_joint_violations=None,
    direct_drive_confirmed=True,
    measurement_basis=None,
    evidence=None,
):
    """Verify one joint using a single unobstructed movement direction and one reference angle."""
    direction = verify_joint_direction_unidirectional(
        motor_start_deg,
        motor_moved_deg,
        expected_model_direction=expected_model_direction,
        side=side,
        joint=joint,
        min_test_deg=min_test_deg,
        other_joint_violations=other_joint_violations,
    )
    return fit_joint_from_reference(
        model_reference_deg,
        motor_reference_deg,
        side=side,
        joint=joint,
        motor_id=motor_id,
        motor_model=motor_model,
        model_joint=model_joint,
        parent_link=parent_link,
        child_link=child_link,
        model_axis=model_axis,
        calibration=calibration,
        min_test_deg=min_test_deg,
        other_joint_violations=other_joint_violations,
        direct_drive_confirmed=direct_drive_confirmed,
        measurement_basis=measurement_basis,
        method=METHOD_PER_JOINT_UNIDIRECTIONAL,
        evidence=evidence,
        direction_result=direction,
    )


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
    verified_joints = []
    unverified_joints = []
    sides = {}
    for side in JOINT_SIDES:
        signs, offsets = [], []
        records = records_by_side[side]
        for joint in JOINTS:
            record = records[joint]
            rec_sign = record.get("sign")
            rec_offset = record.get("offset_deg")
            signs.append(int(rec_sign) if _is_valid_sign_value(rec_sign) else None)
            offsets.append(float(rec_offset) if _is_finite_number(rec_offset) else None)
            failures.extend(record.get("failures", []))
            if _physically_verified_joint(record):
                verified_joints.append(f"{side}_{joint}")
            else:
                unverified_joints.append(f"{side}_{joint}")
                if not _assumed_joint(record) and not record.get("failures"):
                    failures.append(f"{side}_{joint}: joint verification is not complete")
        sides[side] = {
            "sign": signs,
            "offset_deg": offsets,
            "joint_verification": records,
        }
    all_verified = not failures and not unverified_joints
    document = {
        "schema": MAPPING_SCHEMA,
        "robot_id": robot_id,
        "calibration_sha256": calibration_hash,
        "verified": all_verified,
        "verified_joints": verified_joints,
        "unverified_joints": unverified_joints,
        "model_kind": "full_mujoco",
        "created_at": _now(),
        "verification": verification or {},
        "sides": sides,
    }
    if config is not None:
        model_path, _ = model_joint_table(config)
        document["mujoco_model_sha256"] = file_sha256(model_path)
    elif episode is not None:
        document["mujoco_model_sha256"] = episode.metadata.get("mujoco_model_sha256")
    else:
        document["mujoco_model_sha256"] = None
    document.update(
        {
            "dataset": str(episode.root) if episode is not None else None,
            "dataset_sha256": episode.dataset_sha256 if episode is not None else None,
            "dataset_manifest_sha256": episode.manifest_sha256 if episode is not None else None,
            "episode": episode.episode if episode is not None else None,
            "verification_method": method,
            "failures": failures,
        }
    )
    document["verification_status"] = mapping_verification_status(document)
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
            model_name, model_axis, parent_link, child_link = _model_evidence(config, side, joint_index)
            record = fit_joint_verification(
                model[:, joint_index],
                motor[:, joint_index],
                side=side,
                joint=joint_name,
                motor_id=motor_id,
                motor_model=motor_model,
                method=METHOD_MULTI_POSE,
                model_joint=model_name,
                parent_link=parent_link,
                child_link=child_link,
                model_axis=model_axis,
                calibration=calibration,
                max_residual_deg=max_residual_deg,
                min_model_range_deg=min_model_range_deg,
            )
            if config is not None and model_name is None:
                raise ValueError(f"No model joint for {side}_{joint_name}")
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


def write_mapping(path, document, *, overwrite=False):
    path = Path(path)
    if path.exists() and not overwrite:
        raise FileExistsError(f"Mapping output already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")


def _write_json(path, document):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as file:
        file.write(json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def _confirm_torque_disable(sides):
    sides_label = ", ".join(side.upper() for side in sides)
    response = input(
        f"注意: {sides_label} アームを手で支えて落下しないことを確認した上で、"
        "選択した関節のトルクを解除しますか？ [y/N]: "
    )
    if response.strip().lower() not in ("y", "yes"):
        raise ValueError("Torque disable was not confirmed; aborting without changing motor torque")


def _resolve_arm_joint_selectors(joints=None, arm="both"):
    if arm not in ("both", "right", "left"):
        raise ValueError("arm must be 'both', 'right', or 'left'")
    target_sides = JOINT_SIDES if arm == "both" else (arm,)
    if joints is None:
        return [(side, joint) for side in target_sides for joint in JOINTS]
    raw_items = []
    for item in [joints] if isinstance(joints, str) else list(joints):
        for token in str(item).split(","):
            token = token.strip()
            if token:
                raw_items.append(token)
    if not raw_items:
        raise ValueError("No joints selected")
    selected = []
    seen = set()
    for token in raw_items:
        lower = token.lower()
        side_prefix = None
        joint_name = lower
        for side in JOINT_SIDES:
            if lower.startswith(f"{side}_"):
                side_prefix = side
                joint_name = lower[len(side) + 1 :]
                break
        if joint_name not in JOINTS:
            raise ValueError(f"Unknown arm joint {token!r}; valid joints are {', '.join(JOINTS)}")
        sides = (side_prefix,) if side_prefix else target_sides
        for side in sides:
            if side not in target_sides:
                raise ValueError(f"Joint {token!r} conflicts with --arm {arm}")
            key = (side, joint_name)
            if key not in seen:
                seen.add(key)
                selected.append(key)
    return selected


def _read_joints_for_sides(robot, sides):
    if set(sides) == set(SIDES):
        return _read_joints(robot)
    result = {}
    for side in sides:
        bus = getattr(robot, side + "_bus")
        if hasattr(bus, "sync_read"):
            obs = bus.sync_read("Present_Position")
            result[side] = np.array([obs[name] for name in JOINTS], dtype=float)
        else:
            full = _read_joints(robot)
            result[side] = full[side]
        if not np.isfinite(result[side]).all():
            raise ValueError(f"Invalid {side} encoder readings")
    return result


def _load_existing_mapping(existing_mapping, robot, config):
    if existing_mapping is None:
        return None
    if isinstance(existing_mapping, (str, Path)):
        doc = load_mapping(existing_mapping)["document"]
    elif isinstance(existing_mapping, dict):
        doc = existing_mapping.get("document", existing_mapping)
        validate_mapping_document(doc, require_verified=False)
    else:
        raise ValueError("existing_mapping must be a path or mapping dict")
    if doc.get("robot_id") != robot.id:
        raise ValueError(
            f"Existing mapping robot_id {doc.get('robot_id')!r} does not match --robot-id {robot.id!r}"
        )
    if doc.get("calibration_sha256") != calibration_sha256(robot):
        raise ValueError("Existing mapping calibration hash differs from current robot calibration")
    if config is not None:
        model_path, _ = model_joint_table(config)
        if doc.get("mujoco_model_sha256") != file_sha256(model_path):
            raise ValueError("Existing mapping MuJoCo model hash differs from selected model")
    return doc


def _evaluate_single_joint_measurement(
    entry,
    *,
    side,
    joint,
    joint_index,
    robot,
    config,
    sign_test_deg=10.0,
    max_residual_deg=3.0,
    min_model_range_deg=5.0,
    max_slope_error=0.1,
):
    bus = getattr(robot, side + "_bus")
    calibration = calibration_joint_limits(robot, side, joint)
    model_name, model_axis, parent_link, child_link = _model_evidence(config, side, joint_index)
    motor_id = calibration["motor_id"]
    motor_model = bus.motors[joint].model

    if not isinstance(entry, dict) or entry.get("unmeasured") is True:
        reason = (
            entry.get("reason")
            if isinstance(entry, dict) and entry.get("reason")
            else "joint measurement not provided"
        )
        return build_unmeasured_joint_record(
            side=side,
            joint=joint,
            motor_id=motor_id,
            motor_model=motor_model,
            model_joint=model_name,
            parent_link=parent_link,
            child_link=child_link,
            model_axis=model_axis,
            calibration=calibration,
            reason=reason,
        )

    dir_info = entry.get("direction_test")
    dir_result = None
    if dir_info is not None:
        if not isinstance(dir_info, dict):
            raise ValueError(f"{side}_{joint} direction_test must be a mapping")
        if dir_info.get("mode") == "unidirectional" or (
            "start_motor_deg" in dir_info and "moved_motor_deg" in dir_info
        ):
            dir_result = verify_joint_direction_unidirectional(
                dir_info["start_motor_deg"],
                dir_info["moved_motor_deg"],
                expected_model_direction=dir_info["expected_model_direction"],
                side=side,
                joint=joint,
                min_test_deg=sign_test_deg,
                other_joint_violations=dir_info.get("other_joint_violations"),
            )
        else:
            dir_result = verify_joint_direction(
                dir_info["baseline_motor_deg"],
                dir_info["positive_motor_deg"],
                dir_info["negative_motor_deg"],
                side=side,
                joint=joint,
                min_test_deg=sign_test_deg,
                other_joint_violations=dir_info.get("other_joint_violations"),
            )

    if "model_angles_deg" in entry and "motor_angles_deg" in entry:
        model_angles = [float(v) for v in entry["model_angles_deg"]]
        motor_angles = [float(v) for v in entry["motor_angles_deg"]]
    elif "model_reference_deg" in entry and "motor_reference_deg" in entry:
        model_angles = [float(entry["model_reference_deg"])]
        motor_angles = [float(entry["motor_reference_deg"])]
    else:
        model_angles = []
        motor_angles = []

    basis = str(entry.get("measurement_basis", "")).strip()
    if not model_angles:
        reason = entry.get("reason") or "reference model angle is unmeasured"
        return build_unmeasured_joint_record(
            side=side,
            joint=joint,
            motor_id=motor_id,
            motor_model=motor_model,
            model_joint=model_name,
            parent_link=parent_link,
            child_link=child_link,
            model_axis=model_axis,
            calibration=calibration,
            sign=dir_result["sign"] if dir_result else None,
            sign_verified=bool(dir_result and dir_result["sign_verified"]),
            positive_delta_deg=dir_result["positive_delta_deg"] if dir_result else None,
            negative_delta_deg=dir_result["negative_delta_deg"] if dir_result else None,
            reason=reason,
            evidence={"direction_test": dir_info} if dir_info else {},
            failures=list(dir_result["failures"]) if dir_result and dir_result["failures"] else None,
        )

    if len(model_angles) >= 2:
        ev = {"measurement_basis": basis} if basis else {}
        if dir_info is not None:
            ev["direction_test"] = dir_info
        record = fit_joint_verification(
            model_angles,
            motor_angles,
            side=side,
            joint=joint,
            motor_id=motor_id,
            motor_model=motor_model,
            method=METHOD_PER_JOINT_MULTI_ANGLE,
            model_joint=model_name,
            parent_link=parent_link,
            child_link=child_link,
            model_axis=model_axis,
            calibration=calibration,
            max_residual_deg=max_residual_deg,
            min_model_range_deg=min_model_range_deg,
            max_slope_error=max_slope_error,
            evidence=ev,
        )
        if not basis:
            record["failures"].append(f"{side}_{joint}: measurement_basis is required")
            record["verified"] = False
            record["offset_verified"] = False
        if dir_result is not None:
            record["positive_delta_deg"] = dir_result["positive_delta_deg"]
            record["negative_delta_deg"] = dir_result["negative_delta_deg"]
            if dir_result["failures"]:
                record["failures"].extend(dir_result["failures"])
                record["verified"] = False
            elif dir_result["sign"] != record["sign"]:
                record["failures"].append(
                    f"{side}_{joint}: direction test sign {dir_result['sign']:+d} disagrees with "
                    f"multi-angle fitted sign {record['sign']:+d}"
                )
                record["verified"] = False
                record["sign_verified"] = False
        return record

    # Single reference point: requires independent bidirectional or unidirectional direction_test.
    if dir_info is None:
        return build_unmeasured_joint_record(
            side=side,
            joint=joint,
            motor_id=motor_id,
            motor_model=motor_model,
            model_joint=model_name,
            parent_link=parent_link,
            child_link=child_link,
            model_axis=model_axis,
            calibration=calibration,
            reason="single reference angle requires an independent bidirectional or unidirectional direction_test",
            evidence={"model_reference_deg": model_angles[0], "motor_reference_deg": motor_angles[0]},
        )

    direct_drive_confirmed = entry.get("direct_drive_confirmed") is True
    single_method = (
        METHOD_PER_JOINT_UNIDIRECTIONAL
        if dir_result.get("direction_mode") == "unidirectional"
        else METHOD_PER_JOINT_REFERENCE
    )
    return fit_joint_from_reference(
        model_angles[0],
        motor_angles[0],
        side=side,
        joint=joint,
        motor_id=motor_id,
        motor_model=motor_model,
        model_joint=model_name,
        parent_link=parent_link,
        child_link=child_link,
        model_axis=model_axis,
        calibration=calibration,
        min_test_deg=sign_test_deg,
        other_joint_violations=dir_info.get("other_joint_violations"),
        direct_drive_confirmed=direct_drive_confirmed,
        measurement_basis=basis,
        method=single_method,
        evidence={
            "measurement_basis": basis,
            "direction_mode": dir_result.get("direction_mode"),
            "operator_confirmed_direct_drive_1_to_1": direct_drive_confirmed,
            "whole_body_pose_required": False,
        },
        direction_result=dir_result,
    )


def verify_joint_mapping_from_measurements(
    measurements,
    robot,
    *,
    config,
    episode=None,
    existing_mapping=None,
    sign_test_deg=10.0,
    max_residual_deg=3.0,
    min_model_range_deg=5.0,
    max_slope_error=0.1,
):
    """Verify per-joint mapping from grounded single-joint measurements without whole-body posing."""
    if isinstance(measurements, (str, Path)):
        measurements_doc = yaml.safe_load(Path(measurements).read_text(encoding="utf-8"))
    elif isinstance(measurements, dict):
        measurements_doc = measurements
    else:
        raise ValueError("measurements must be a dict or path to YAML/JSON")
    if not isinstance(measurements_doc, dict):
        raise ValueError("measurements document must be a mapping")

    joints_map = {}
    if isinstance(measurements_doc.get("joints"), dict):
        joints_map.update(measurements_doc["joints"])
    if isinstance(measurements_doc.get("sides"), dict):
        for side in JOINT_SIDES:
            side_entries = measurements_doc["sides"].get(side, {})
            if isinstance(side_entries, dict):
                for joint, val in side_entries.items():
                    joints_map[f"{side}_{joint}"] = val

    existing = _load_existing_mapping(existing_mapping, robot, config)
    records_by_side = {side: {} for side in JOINT_SIDES}
    for side in JOINT_SIDES:
        bus = getattr(robot, side + "_bus")
        existing_records = (mapping_joint_records(existing, side) or {}) if existing is not None else {}
        for joint_index, joint in enumerate(JOINTS):
            key = f"{side}_{joint}"
            if key in joints_map:
                record = _evaluate_single_joint_measurement(
                    joints_map[key],
                    side=side,
                    joint=joint,
                    joint_index=joint_index,
                    robot=robot,
                    config=config,
                    sign_test_deg=sign_test_deg,
                    max_residual_deg=max_residual_deg,
                    min_model_range_deg=min_model_range_deg,
                    max_slope_error=max_slope_error,
                )
            elif joint in existing_records:
                record = dict(existing_records[joint])
                if _assumed_joint(record):
                    record = _mark_assumption(record)
            else:
                model_name, model_axis, parent_link, child_link = _model_evidence(config, side, joint_index)
                cal = calibration_joint_limits(robot, side, joint)
                record = build_unmeasured_joint_record(
                    side=side,
                    joint=joint,
                    motor_id=cal["motor_id"],
                    motor_model=bus.motors[joint].model,
                    model_joint=model_name,
                    parent_link=parent_link,
                    child_link=child_link,
                    model_axis=model_axis,
                    calibration=cal,
                    reason="not measured in verification session",
                )
            records_by_side[side][joint] = record

    per_joint = [
        {
            "side": side,
            "joint": joint,
            "motor_id": record["motor_id"],
            "model_joint": record.get("model_joint"),
            "sign": record.get("sign"),
            "offset_deg": record.get("offset_deg"),
            "sign_verified": record.get("sign_verified"),
            "offset_verified": record.get("offset_verified"),
            "scale_verified": record.get("scale_verified"),
            "verification_method": _record_method(record),
            "verified": record["verified"],
            "failures": record["failures"],
        }
        for side in JOINT_SIDES
        for joint, record in records_by_side[side].items()
    ]
    all_failures = [failure for item in per_joint for failure in item["failures"]]
    verification = {
        "schema": MAPPING_VERIFICATION_SCHEMA,
        "status": "ok" if not all_failures and all(item["verified"] for item in per_joint) else "failed",
        "dataset": str(episode.root) if episode is not None else None,
        "dataset_sha256": episode.dataset_sha256 if episode is not None else None,
        "manifest_sha256": episode.manifest_sha256 if episode is not None else None,
        "episode": episode.episode if episode is not None else None,
        "whole_body_pose_required": False,
        "sign_test_deg": float(sign_test_deg),
        "method": METHOD_PER_JOINT_REFERENCE,
        "per_joint": per_joint,
        "failures": all_failures,
    }
    document = _build_mapping_document(
        episode,
        records_by_side,
        robot_id=robot.id,
        calibration_hash=calibration_sha256(robot),
        config=config,
        method=METHOD_PER_JOINT_REFERENCE,
        verification=verification,
    )
    if existing is not None:
        for side in JOINT_SIDES:
            for key in ("joint_limits_deg", "joint_limits_evidence"):
                if key in existing["sides"][side]:
                    document["sides"][side][key] = existing["sides"][side][key]
    verification["verification_status"] = document["verification_status"]
    verification["unverified_joints"] = document["unverified_joints"]
    return document, verification


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
    """Read manually posed robot angles; confirms arm support before disabling torque."""
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
        _connect_checked_buses(robot, {})
        _confirm_torque_disable(JOINT_SIDES)
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
                    bus.disconnect(disable_torque=False)


def build_calibration_mapping(
    robot,
    *,
    config,
    episode=None,
    output=None,
    report=None,
    overwrite=False,
):
    """Build an assumed preview mapping from LeRobot zero and assembly sign conventions."""
    if output is not None and Path(output).exists() and not overwrite:
        raise FileExistsError(f"Mapping output already exists: {output}")
    records_by_side = {side: {} for side in JOINT_SIDES}
    per_joint = []
    for side in JOINT_SIDES:
        bus = getattr(robot, side + "_bus")
        for joint_index, joint in enumerate(JOINTS):
            calibration = calibration_joint_limits(robot, side, joint)
            motor_id = int(calibration["motor_id"])
            motor_model = bus.motors[joint].model
            model_name, model_axis, parent_link, child_link = _model_evidence(config, side, joint_index)
            drive_sign = -1 if int(calibration["drive_mode"]) else 1
            base_sign = int(DEFAULT_FULL_CAD_SIGNS_DRIVE_MODE_0[side][joint_index])
            sign = base_sign * drive_sign
            offset_deg = 0.0
            failures = []
            if episode is not None:
                model_vals = np.array(
                    [frame.joints[side][joint_index] for frame in episode.frames], dtype=float
                )
                mapped_vals = sign * model_vals + offset_deg
                min_mapped = float(np.min(mapped_vals))
                max_mapped = float(np.max(mapped_vals))
                if (
                    min_mapped < calibration["min_motor_deg"] - 1e-6
                    or max_mapped > calibration["max_motor_deg"] + 1e-6
                ):
                    failures.append(
                        f"{side}_{joint}: mapped dataset range [{min_mapped:+.2f}, {max_mapped:+.2f}] deg "
                        f"exceeds calibrated motor limits "
                        f"[{calibration['min_motor_deg']:+.2f}, {calibration['max_motor_deg']:+.2f}] deg"
                    )
            evidence = {
                "calibration_midpoint_raw": calibration["midpoint_raw"],
                "drive_mode": calibration["drive_mode"],
                "cad_assembly_default_sign": base_sign,
                "manual_cad_pose_required": False,
                "whole_body_pose_required": False,
            }
            record = {
                "schema": JOINT_VERIFICATION_SCHEMA,
                "side": side,
                "joint": joint,
                "motor_id": motor_id,
                "motor_model": motor_model,
                "model_joint": model_name,
                "parent_link": parent_link,
                "child_link": child_link,
                "method": METHOD_CALIBRATION_MIDPOINT,
                "verification_method": METHOD_CALIBRATION_MIDPOINT,
                "verification_status": CALIBRATION_ASSUMED,
                "verified": False,
                "sign_verified": False,
                "offset_verified": False,
                "model_axis": model_axis,
                "calibration": calibration,
                "model_angles_deg": [],
                "motor_angles_deg": [],
                "model_range_deg": None,
                "slope": float(sign),
                "sign": int(sign),
                "motor_direction": "same_as_model" if sign == 1 else "opposite_to_model",
                "offset_deg": offset_deg,
                "rms_deg": None,
                "max_residual_deg": None,
                "positive_delta_deg": None,
                "negative_delta_deg": None,
                "scale_basis": "direct_drive_1_to_1",
                "scale_verified": False,
                "evidence": evidence,
                "verification_evidence": dict(evidence),
                "failures": failures,
            }
            records_by_side[side][joint] = record
            per_joint.append(
                {
                    "side": side,
                    "joint": joint,
                    "motor_id": motor_id,
                    "model_joint": model_name,
                    "sign": int(sign),
                    "offset_deg": offset_deg,
                    "sign_verified": False,
                    "offset_verified": False,
                    "scale_verified": False,
                    "verification_method": METHOD_CALIBRATION_MIDPOINT,
                    "verification_status": CALIBRATION_ASSUMED,
                    "verified": False,
                    "failures": failures,
                }
            )
    all_failures = [failure for item in per_joint for failure in item["failures"]]
    verification = {
        "schema": MAPPING_VERIFICATION_SCHEMA,
        "status": "failed" if all_failures else CALIBRATION_ASSUMED,
        "dataset": str(episode.root) if episode is not None else None,
        "dataset_sha256": episode.dataset_sha256 if episode is not None else None,
        "manifest_sha256": episode.manifest_sha256 if episode is not None else None,
        "episode": episode.episode if episode is not None else None,
        "whole_body_pose_required": False,
        "manual_cad_pose_required": False,
        "method": METHOD_CALIBRATION_MIDPOINT,
        "per_joint": per_joint,
        "failures": all_failures,
    }
    document = _build_mapping_document(
        episode,
        records_by_side,
        robot_id=robot.id,
        calibration_hash=calibration_sha256(robot),
        config=config,
        method=METHOD_CALIBRATION_MIDPOINT,
        verification=verification,
    )
    if report is not None:
        _write_json(report, verification)
    if all_failures:
        raise ValueError("Calibration mapping validation failed: " + "; ".join(all_failures))
    if output is not None:
        write_mapping(output, document, overwrite=overwrite)
    return document, verification


def verify_dataset_mapping_jointwise(
    episode,
    robot,
    output,
    *,
    config,
    joints=None,
    arm="both",
    existing_mapping=None,
    measurements=None,
    sign_test_deg=10.0,
    max_other_joint_delta_deg=5.0,
    max_residual_deg=3.0,
    min_model_range_deg=5.0,
    allow_partial=False,
    report=None,
):
    """Verify each joint independently without whole-body dataset pose matching.

    Each joint is verified one at a time in its own parent-child joint frame:
    - ``sign`` is checked by opposite-direction manual motion around the CAD axis.
    - ``offset_deg`` is computed from one or more operator-measured known model
      angles for that joint (or left ``unverified`` if unmeasured).
    - Whole-body posing of all 14 joints to a dataset frame is never required.
    """
    if not np.isfinite(sign_test_deg) or not 2 <= sign_test_deg <= 45:
        raise ValueError("sign_test_deg must be in [2, 45]")
    if not np.isfinite(max_other_joint_delta_deg) or not 0 <= max_other_joint_delta_deg <= 20:
        raise ValueError("max_other_joint_delta_deg must be in [0, 20]")
    if Path(output).exists():
        raise FileExistsError(f"Mapping output already exists: {output}")

    if measurements is not None:
        document, verification = verify_joint_mapping_from_measurements(
            measurements,
            robot,
            config=config,
            episode=episode,
            existing_mapping=existing_mapping,
            sign_test_deg=sign_test_deg,
            max_residual_deg=max_residual_deg,
            min_model_range_deg=min_model_range_deg,
        )
        if report:
            _write_json(report, verification)
        if not document["verified"] and not allow_partial:
            raise ValueError("Joint-wise mapping verification failed: " + "; ".join(verification["failures"]))
        write_mapping(output, document)
        return verification

    selected = _resolve_arm_joint_selectors(joints=joints, arm=arm)
    active_sides = tuple(side for side in JOINT_SIDES if any(s == side for s, _ in selected))
    # Validate any existing partial mapping up front before touching motor connections or torque.
    _load_existing_mapping(existing_mapping, robot, config)
    measured_entries = {}
    _, model_table = model_joint_table(config)

    try:
        _connect_checked_buses(robot, {}, sides=active_sides)
        _confirm_torque_disable(active_sides)
        for side in active_sides:
            getattr(robot, side + "_bus").disable_torque(list(JOINTS))

        direct_drive_input = input(
            "選択した関節がサーボホーン直結（減速機・ベルトなし、1:1）であることを確認しましたか？\n"
            "(1点基準でoffsetを求める場合に必要です; 2点以上の角度で検証する場合は 'n' でも可) [y/N]: "
        )
        direct_drive_confirmed = direct_drive_input.strip().lower() in ("y", "yes")

        for side, joint in selected:
            joint_index = JOINTS.index(joint)
            calibration = calibration_joint_limits(robot, side, joint)
            model_name = f"joint{joint_index}_{SUFFIX[side]}"
            model_info = model_table[model_name]
            cad_geom = model_info["cad_zero_geometry"]
            print(
                f"\n[{side}_{joint}] motor_id={calibration['motor_id']} "
                f"model_joint={model_name} ({model_info.get('parent_link')} -> {model_info.get('child_link')}) "
                f"axis={np.round(model_info['axis'], 4).tolist()}\n"
                f"  calibration: drive_mode={calibration['drive_mode']} homing_offset={calibration['homing_offset']} "
                f"limits=[{calibration['min_motor_deg']:+.2f}, {calibration['max_motor_deg']:+.2f}] deg\n"
                f"  CAD zero: {cad_geom['cad_reference_note']} ({cad_geom['positive_rotation_effect']})\n"
                "  全関節をMID(中央)に揃える必要はありません。机に置いた現在位置から動かせる側へ1回動かすだけで検証できます。"
            )
            mode_prompt = (
                input(
                    f"{side}_{joint} の方向テスト方法を選択してEnter\n"
                    f"  [Enter/'both': 正負両方向 | '+': CAD正方向のみ | '-': CAD負方向のみ | 'skip': 未測定スキップ]: "
                )
                .strip()
                .lower()
            )
            if mode_prompt in ("s", "skip", "unmeasured"):
                measured_entries[f"{side}_{joint}"] = {
                    "unmeasured": True,
                    "reason": "operator skipped joint",
                }
                continue

            if mode_prompt in ("+", "+1", "pos", "positive", "-", "-1", "neg", "negative"):
                expected_dir = _parse_model_direction(mode_prompt)
                start_all = _read_joints_for_sides(robot, active_sides)
                start_motor = float(start_all[side][joint_index])
                dir_desc = (
                    f"CAD正方向（{cad_geom['positive_rotation_effect']}）"
                    if expected_dir == 1
                    else f"CAD負方向（{cad_geom['positive_rotation_effect']} の反対側）"
                )
                input(
                    f"{side}_{joint}（現在 {start_motor:+.2f} deg）だけを {dir_desc} へ "
                    f"{sign_test_deg:g} deg 以上（机・ストッパに当たらない側へ）動かしてEnter（腕を支える）: "
                )
                moved_all = _read_joints_for_sides(robot, active_sides)
                other_joint_violations = {}
                for other_side in active_sides:
                    for other_index, other_joint in enumerate(JOINTS):
                        if other_side == side and other_index == joint_index:
                            continue
                        deviation = abs(
                            moved_all[other_side][other_index] - start_all[other_side][other_index]
                        )
                        if deviation > max_other_joint_delta_deg:
                            other_joint_violations[f"{other_side}_{other_joint}"] = float(deviation)
                direction_test = {
                    "mode": "unidirectional",
                    "start_motor_deg": start_motor,
                    "moved_motor_deg": float(moved_all[side][joint_index]),
                    "expected_model_direction": expected_dir,
                    "other_joint_violations": other_joint_violations,
                }
            else:
                baseline_all = _read_joints_for_sides(robot, active_sides)
                baseline_motor = float(baseline_all[side][joint_index])
                input(
                    f"{side}_{joint} だけをモデル正方向（{cad_geom['positive_rotation_effect']}）へ "
                    f"{sign_test_deg:g} deg 以上動かしてEnter（腕を支える）: "
                )
                positive_all = _read_joints_for_sides(robot, active_sides)
                input(
                    f"{side}_{joint} だけをモデル負方向（基準位置を反対側へ {sign_test_deg:g} deg 以上超える位置）"
                    "へ動かしてEnter（腕を支える）: "
                )
                negative_all = _read_joints_for_sides(robot, active_sides)

                other_joint_violations = {}
                for other_side in active_sides:
                    for other_index, other_joint in enumerate(JOINTS):
                        if other_side == side and other_index == joint_index:
                            continue
                        deviation = max(
                            abs(
                                positive_all[other_side][other_index] - baseline_all[other_side][other_index]
                            ),
                            abs(
                                negative_all[other_side][other_index] - baseline_all[other_side][other_index]
                            ),
                        )
                        if deviation > max_other_joint_delta_deg:
                            other_joint_violations[f"{other_side}_{other_joint}"] = float(deviation)

                direction_test = {
                    "baseline_motor_deg": baseline_motor,
                    "positive_motor_deg": float(positive_all[side][joint_index]),
                    "negative_motor_deg": float(negative_all[side][joint_index]),
                    "other_joint_violations": other_joint_violations,
                }

            angle_raw = input(
                f"{side}_{joint} の既知のモデル角度(deg)を入力してください "
                "(例: 0 / 90 / 2点以上なら '0, 90' / 未測定なら空Enterまたは 'unmeasured'): "
            ).strip()
            if not angle_raw or angle_raw.lower() in ("u", "unmeasured", "skip", "s"):
                measured_entries[f"{side}_{joint}"] = {
                    "direction_test": direction_test,
                    "reason": "reference model angle is unmeasured",
                }
                continue

            parsed_angles = [float(part.strip()) for part in angle_raw.split(",") if part.strip()]
            if not parsed_angles or not np.isfinite(parsed_angles).all():
                raise ValueError(f"{side}_{joint}: invalid model angle input {angle_raw!r}")

            basis = input(
                f"{side}_{joint} の測定根拠を入力してください "
                "(例: CAD基準合わせ / 90度スコヤ / 水平器 / 分度器 / 治具): "
            ).strip()

            motor_refs = []
            for model_angle in parsed_angles:
                input(
                    f"{side}_{joint} を測定したモデル角 {model_angle:+.2f} deg の姿勢に合わせてEnter "
                    "(他関節の全身姿勢合わせは不要): "
                )
                sample = _read_joints_for_sides(robot, active_sides)
                motor_refs.append(float(sample[side][joint_index]))

            if len(parsed_angles) == 1:
                measured_entries[f"{side}_{joint}"] = {
                    "direction_test": direction_test,
                    "direct_drive_confirmed": direct_drive_confirmed,
                    "measurement_basis": basis,
                    "model_reference_deg": parsed_angles[0],
                    "motor_reference_deg": motor_refs[0],
                }
            else:
                measured_entries[f"{side}_{joint}"] = {
                    "direction_test": direction_test,
                    "measurement_basis": basis,
                    "model_angles_deg": parsed_angles,
                    "motor_angles_deg": motor_refs,
                }

        document, verification = verify_joint_mapping_from_measurements(
            {"joints": measured_entries},
            robot,
            config=config,
            episode=episode,
            existing_mapping=existing_mapping,
            sign_test_deg=sign_test_deg,
            max_residual_deg=max_residual_deg,
            min_model_range_deg=min_model_range_deg,
        )
        if report:
            _write_json(report, verification)
        if not document["verified"] and not allow_partial:
            raise ValueError("Joint-wise mapping verification failed: " + "; ".join(verification["failures"]))
        write_mapping(output, document)
        return verification
    finally:
        for side in active_sides:
            bus = getattr(robot, side + "_bus")
            if bus.is_connected:
                with suppress(Exception):
                    bus.disconnect(disable_torque=False)


def _segment_plan(previous, following, *, fps, max_step_deg, max_speed_deg_s, control_hz):
    delta = np.asarray(following) - np.asarray(previous)
    max_delta = float(np.max(np.abs(delta)))
    duration = max(
        1.0 / fps,
        max_delta / max_speed_deg_s if max_delta > 0 else 0.0,
        max_delta / (max_step_deg * control_hz) if max_delta > 0 else 0.0,
    )
    steps = max(1, int(np.ceil(duration * control_hz)), int(np.ceil(max_delta / max_step_deg)))
    return steps


def _episode_segments(episode, document, *, max_step_deg, max_speed_deg_s, control_hz):
    mapped = [
        {side: map_joints(document, side, frame.joints[side]) for side in SIDES} for frame in episode.frames
    ]
    return [
        (
            previous,
            target,
            max(
                _segment_plan(
                    previous[side],
                    target[side],
                    fps=episode.fps,
                    max_step_deg=max_step_deg,
                    max_speed_deg_s=max_speed_deg_s,
                    control_hz=control_hz,
                )
                for side in SIDES
            ),
        )
        for previous, target in zip(mapped[:-1], mapped[1:], strict=True)
    ]


def _interpolate(start, target, alpha):
    return {side: start[side] + alpha * (target[side] - start[side]) for side in SIDES}


def _model_pose(document, motor_joints):
    return {side: unmap_joints(document, side, motor_joints[side]) for side in SIDES}


def _max_joint_delta(first, second):
    return max(float(np.max(np.abs(first[side] - second[side]))) for side in SIDES)


MAX_FEEDBACK_AGE_S = 0.5


def _read_replay_feedback(robot, scene=None):
    started = time.monotonic()
    current = _read_joints(robot)
    if scene is not None and scene.replay_gripper_poses is not None:
        for side, reference in scene.replay_gripper_poses.items():
            bus = getattr(robot, side + "_bus")
            count = bus.read("Present_Position", GRIPPER, normalize=False)
            if (
                not _is_finite_number(count)
                or not 0 <= count <= 4095
                or abs(count - reference.encoder_count) > 1
            ):
                raise RuntimeError(f"{side} gripper differs from its measured fixed pose: HOLD")
    if time.monotonic() - started > MAX_FEEDBACK_AGE_S:
        raise RuntimeError("Encoder feedback took more than 0.5 s: HOLD")
    return current, started


def _verify_initial_hold(robot, current):
    for side in SIDES:
        bus = getattr(robot, side + "_bus")
        goals = bus.sync_read("Goal_Position", list(JOINTS))
        for index, joint in enumerate(JOINTS):
            cal = calibration_joint_limits(robot, side, joint)
            encoder_tick_deg = 360 / (cal["resolution"] - 1)
            if not _is_finite_number(goals.get(joint)) or (
                abs(float(goals[joint]) - current[side][index]) > encoder_tick_deg + 1e-6
            ):
                raise RuntimeError(f"{side}_{joint} initial hold goal readback differs from the encoder")


def _validate_replay_limits(start_error, tracking_error, speed, hz, position_error, orientation_error):
    for name, value, upper in (
        ("max_start_error_deg", start_error, 2),
        ("max_tracking_error_deg", tracking_error, 5),
        ("max_joint_speed_deg_s", speed, 15),
        ("control_hz", hz, 30),
        ("max_sim_error_m", position_error, 0.10),
    ):
        if not 0 < value <= upper:
            raise ValueError(f"{name} must be in (0, {upper:g}]")
    if orientation_error is not None and not 0 < orientation_error <= 180:
        raise ValueError("max_sim_orientation_error_deg must be in (0, 180]")


def _start_pose_report(context, transition, document, robot, tolerance):
    current = context["robot_current_pose"]
    target = context["dataset_initial_pose"]
    counts = {}
    for side in SIDES:
        counts[side] = {}
        for index, joint in enumerate(JOINTS):
            cal = calibration_joint_limits(robot, side, joint)
            direction = -1 if cal["drive_mode"] else 1
            factor = direction * (cal["resolution"] - 1) / 360
            midpoint = cal["midpoint_raw"]
            counts[side][joint] = {
                "present_position_count": round(current[side]["motor_deg"][index] * factor + midpoint),
                "expected_goal_position_count": int(
                    direction * target[side]["mapped_motor_deg"][index] * (cal["resolution"] - 1) / 360
                    + midpoint
                ),
            }
    errors = context["start_error_deg"]
    return {
        "max_error_deg": errors["max"],
        "per_joint_deg": errors["per_joint"],
        "tolerance_deg": tolerance,
        "within_tolerance": errors["max"] <= tolerance,
        "modeled_collision": transition["samples"][0]["collision"],
        "limit_violations": motor_pose_limit_violations(
            {side: current[side]["motor_deg"] for side in SIDES}, document, robot, pose_label="current"
        ),
        "encoder_counts": counts,
    }


def _motion_prerequisites(document, robot, scene):
    blockers = []
    if mapping_verification_status(document) != PHYSICALLY_VERIFIED:
        blockers.append("Physically measured sign and zero offset are required for all 14 joints")
    for side in SIDES:
        limits = document["sides"][side].get("joint_limits_deg")
        evidence = document["sides"][side].get("joint_limits_evidence")
        if limits is None or not isinstance(evidence, str) or not evidence.strip():
            blockers.append(
                f"{side}: measured physical joint/wrist/cable limits and their evidence are required"
            )
            continue
        lower_limits = np.asarray(limits["min"], dtype=float)
        upper_limits = np.asarray(limits["max"], dtype=float)
        for joint, lower, upper in zip(JOINTS, lower_limits, upper_limits, strict=True):
            cal = calibration_joint_limits(robot, side, joint)
            if lower < cal["min_motor_deg"] or upper > cal["max_motor_deg"]:
                blockers.append(f"{side}_{joint}: physical replay limits must fit inside the calibration")
    if not measured_scene_ready(scene):
        blockers.append("A measured scene with a table box and 0.45 m support stick is required")
    if scene is None or scene.replay_gripper_poses is None:
        blockers.append("Measured fixed gripper poses are required for the Full-CAD link clearance check")
    return blockers


def _check_episode_geometry(episode, segments, document, geometry):
    violations = []
    min_table_clearance = None
    samples_checked = 0

    def check(frame, step, model):
        nonlocal samples_checked, min_table_clearance
        result = geometry.check_pose(model)
        samples_checked += 1
        clearance = result["table_clearance_m"]
        if clearance is not None:
            min_table_clearance = (
                clearance if min_table_clearance is None else min(min_table_clearance, clearance)
            )
        if result["collision"]["count"] or result["clearance_violations"] or result["model_limit_violations"]:
            violations.append({"frame": frame, "substep": step, **result})

    check(0, 0, episode.frames[0].joints)
    for frame, (previous, target, steps) in enumerate(segments, 1):
        for step in range(1, steps + 1):
            check(frame, step, _model_pose(document, _interpolate(previous, target, step / steps)))
    return {
        "samples_checked": samples_checked,
        "modeled_safe": not violations,
        "minimum_table_clearance_m": min_table_clearance,
        "violations": violations,
    }


def _target_pose_for_frame(simulation, frame, side):
    target = frame.target_pose.get(side)
    if target is not None:
        return target
    position, quaternion = simulation.arms[side].solver.fk_solver.compute(np.r_[frame.joints[side], 0.0])
    return np.r_[position, quaternion]


def _normalize_initial_move_strategy(strategy):
    value = str(strategy or "auto").strip().lower().replace("-", "_")
    if value not in ("auto", "linear", "lift_first"):
        raise ValueError(f"Unsupported initial_move_strategy {strategy!r}; use auto, linear, or lift-first")
    return value


def _build_transition_stages(current, mapped_first, *, strategy):
    if strategy == "linear":
        return [("linear", current, mapped_first)]
    # Moving distal pitch first can raise or lower the hands; FK evaluates the result.
    pitch_indices = (5, 3)
    lifted_mid = {}
    for side in JOINT_SIDES:
        mid = np.asarray(current[side], dtype=float).copy()
        for idx in pitch_indices:
            mid[idx] = float(mapped_first[side][idx])
        lifted_mid[side] = mid
    stage1_delta = max(float(np.max(np.abs(lifted_mid[s] - current[s]))) for s in JOINT_SIDES)
    stage2_delta = max(float(np.max(np.abs(mapped_first[s] - lifted_mid[s]))) for s in JOINT_SIDES)
    if stage1_delta <= 1e-6 or stage2_delta <= 1e-6:
        return [("linear", current, mapped_first)]
    return [
        ("align_joint3_and_joint5", current, lifted_mid),
        ("align_remaining_joints", lifted_mid, mapped_first),
    ]


def _evaluate_transition_stages(
    stage_specs,
    *,
    current,
    mapped_first,
    document,
    robot,
    max_step_deg,
    max_speed_deg_s,
    control_hz,
    geometry,
    chosen_strategy,
):
    scale = max(float(max_step_deg), 1e-9)
    limit_violations = motor_pose_limit_violations(
        current, document, robot, pose_label="current"
    ) + motor_pose_limit_violations(mapped_first, document, robot, pose_label="dataset_start")

    stages = []
    collision_frames = []
    clearance_violation_steps = []
    samples = []
    max_penetration_m = 0.0
    body_pairs = {}
    total_steps = 0
    max_command_step_deg = 0.0
    max_command_speed_deg_s = 0.0
    total_direct_delta = max(float(np.max(np.abs(current[s] - mapped_first[s]))) for s in JOINT_SIDES)

    for stage_name, stage_start, stage_target in stage_specs:
        stage_delta = max(float(np.max(np.abs(stage_start[s] - stage_target[s]))) for s in JOINT_SIDES)
        duration = max(
            stage_delta / float(max_speed_deg_s) if stage_delta > 0 else 0.0,
            stage_delta / (scale * float(control_hz)) if stage_delta > 0 else 0.0,
        )
        steps = max(1, int(np.ceil(duration * control_hz)), int(np.ceil(stage_delta / scale)))
        step_size = float(stage_delta / steps) if stage_delta > 0 else 0.0
        speed_val = float(step_size * float(control_hz))
        max_command_step_deg = max(max_command_step_deg, step_size)
        max_command_speed_deg_s = max(max_command_speed_deg_s, speed_val)

        start_step_idx = 0 if not stages else 1
        if geometry is not None:
            for step in range(start_step_idx, steps + 1):
                command = _interpolate(stage_start, stage_target, step / steps)
                model = _model_pose(document, command)
                check = geometry.check_pose(model, include_heights=True)
                step_number = total_steps + step
                collision = check["collision"]
                if collision["count"]:
                    collision_frames.append(step_number)
                    max_penetration_m = max(max_penetration_m, collision["max_penetration_m"])
                    for pair, count in collision["body_pairs"].items():
                        body_pairs[pair] = body_pairs.get(pair, 0) + count
                if check["clearance_violations"]:
                    clearance_violation_steps.append(step_number)
                limit_violations.extend(
                    {"step": step_number, **item} for item in check["model_limit_violations"]
                )
                samples.append(
                    {
                        "step": step_number,
                        "stage": stage_name,
                        "target_motor_deg": {side: command[side].tolist() for side in SIDES},
                        "full_cad_deg": {side: model[side].tolist() for side in SIDES},
                        **check,
                    }
                )

        stages.append(
            {
                "name": stage_name,
                "start_motor_deg": {s: np.asarray(stage_start[s], dtype=float).tolist() for s in JOINT_SIDES},
                "target_motor_deg": {
                    s: np.asarray(stage_target[s], dtype=float).tolist() for s in JOINT_SIDES
                },
                "delta_deg": stage_delta,
                "steps": steps,
                "max_command_step_deg": step_size,
                "max_command_speed_deg_s": speed_val,
            }
        )
        total_steps += steps

    speed_and_step_within_limits = (
        max_command_step_deg <= float(max_step_deg) + 1e-6
        and max_command_speed_deg_s <= float(max_speed_deg_s) + 1e-6
    )
    modeled_safe = (
        geometry is not None
        and not limit_violations
        and not collision_frames
        and not clearance_violation_steps
        and speed_and_step_within_limits
    )
    scene = geometry.scene if geometry is not None else None
    blockers = _motion_prerequisites(document, robot, scene)
    return {
        "required": total_direct_delta > 0,
        "strategy": chosen_strategy,
        "stages": stages,
        "safe": modeled_safe and not blockers,
        "modeled_safe": modeled_safe,
        "blockers": blockers,
        "delta_deg": total_direct_delta,
        "steps": total_steps,
        "estimated_duration_s": total_steps / float(control_hz),
        "max_step_deg": float(max_step_deg),
        "max_speed_deg_s": float(max_speed_deg_s),
        "max_command_step_deg": max_command_step_deg,
        "max_command_speed_deg_s": max_command_speed_deg_s,
        "speed_and_step_within_limits": speed_and_step_within_limits,
        "control_hz": float(control_hz),
        "limit_violations": limit_violations,
        "collision_checked": geometry is not None,
        "collision_frames": collision_frames,
        "clearance_violation_steps": clearance_violation_steps,
        "samples": samples,
        "max_penetration_m": max_penetration_m,
        "body_pairs": body_pairs,
        "scope": (
            f"Joint-space {chosen_strategy} transition from the measured current pose to the mapped "
            "dataset first frame. FK, arm mesh heights, floor/self/inter-arm contacts and joint "
            "limits are sampled at each setpoint. Table clearance requires a measured scene."
        ),
    }


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
    geometry=None,
    strategy="auto",
):
    """Evaluate a joint-space move from the current pose to the dataset start.

    ``lift-first`` aligns joint3 and joint5 first. ``auto`` checks the straight path
    before trying the staged path.
    """
    normalized_strategy = _normalize_initial_move_strategy(strategy)
    document = document.get("document", document) if isinstance(document, dict) else document
    if geometry is None and simulation is not None:
        geometry = ReplayGeometry(simulation)
    first_model = {side: episode.frames[0].joints[side] for side in JOINT_SIDES}
    mapped_first = {side: map_joints(document, side, first_model[side]) for side in JOINT_SIDES}

    if normalized_strategy == "lift_first":
        specs = _build_transition_stages(current, mapped_first, strategy="lift_first")
        return _evaluate_transition_stages(
            specs,
            current=current,
            mapped_first=mapped_first,
            document=document,
            robot=robot,
            max_step_deg=max_step_deg,
            max_speed_deg_s=max_speed_deg_s,
            control_hz=control_hz,
            geometry=geometry,
            chosen_strategy="lift_first" if len(specs) > 1 else "linear",
        )

    linear_eval = _evaluate_transition_stages(
        _build_transition_stages(current, mapped_first, strategy="linear"),
        current=current,
        mapped_first=mapped_first,
        document=document,
        robot=robot,
        max_step_deg=max_step_deg,
        max_speed_deg_s=max_speed_deg_s,
        control_hz=control_hz,
        geometry=geometry,
        chosen_strategy="linear",
    )
    if normalized_strategy == "auto" and not linear_eval["modeled_safe"] and linear_eval["required"]:
        lift_specs = _build_transition_stages(current, mapped_first, strategy="lift_first")
        if len(lift_specs) > 1:
            lift_eval = _evaluate_transition_stages(
                lift_specs,
                current=current,
                mapped_first=mapped_first,
                document=document,
                robot=robot,
                max_step_deg=max_step_deg,
                max_speed_deg_s=max_speed_deg_s,
                control_hz=control_hz,
                geometry=geometry,
                chosen_strategy="lift_first",
            )
            if lift_eval["modeled_safe"]:
                lift_eval["fallback_from_linear_collisions"] = linear_eval["collision_frames"][:10]
                return lift_eval
    return linear_eval


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
    initial_move_strategy="auto",
    scene=None,
    realtime=True,
):
    """Send a verified joint trajectory with interpolation and existing safety gates."""
    _validate_replay_limits(
        max_start_error_deg,
        max_tracking_error_deg,
        max_joint_speed_deg_s,
        control_hz,
        max_sim_error_m,
        max_sim_orientation_error_deg,
    )
    mapping_path = document.get("path") if isinstance(document, dict) else None
    raw_document = document.get("document", document) if isinstance(document, dict) else document
    document = raw_document
    model_hash = file_sha256(full_model_path(config["robot"]))
    if episode.metadata.get("mujoco_model_sha256") != model_hash:
        raise ValueError("Dataset MuJoCo model differs from the selected model; regenerate the dataset")
    mapping_info = validate_mapping_for_robot(
        document, robot, model_sha256=model_hash, allow_calibration_assumed=True
    )
    step_limit = robot.config.max_relative_target
    if not robot.config.use_degrees or not isinstance(step_limit, (int, float)) or not 0 < step_limit <= 0.5:
        raise ValueError("Follower must use calibrated degrees and a per-step limit at most 0.5 deg")
    validation = validate_mapped_episode(episode, document, robot)
    inspection = inspect_joint_mapping(episode, document, robot, config)
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
        "mapping_verified_at": mapping_info["verified_at"],
        "mapping_robot_check": mapping_info,
        "mujoco_model_sha256": model_hash,
        "dataset_original_hardware_replay_allowed": episode.metadata.get("hardware_replay_allowed"),
        "gripper_commanded": False,
        "simulation_only_source": True,
        "joint_inspection": inspection,
        "scene": scene.to_dict() if scene is not None else None,
        "usb_ports": {},
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
            "max_feedback_age_s": MAX_FEEDBACK_AGE_S,
        },
        "validation": validation,
    }
    failure = None
    current = None
    start_time = None
    last_command_at = None
    geometry = None
    writes_started = False
    control_steps = 0
    max_observed_speed = 0.0
    tcp_warning_frames = []
    tcp_orientation_warning_frames = []

    def check_feedback_pose(joints):
        if motor_pose_limit_violations(joints, document, robot):
            raise RuntimeError("Encoder feedback exceeds calibrated or physical joint limits: HOLD")
        observed = geometry.check_pose(_model_pose(document, joints))
        if (
            observed["collision"]["count"]
            or observed["model_limit_violations"]
            or observed["clearance_violations"]
        ):
            raise RuntimeError("Encoder feedback violates modeled limits or clearance: HOLD")

    def send_checked_target(target, label):
        nonlocal control_steps, last_command_at, current, sample_at, max_observed_speed, previous_sent
        if realtime:
            time.sleep(max(0.0, last_command_at + 1.0 / control_hz - time.monotonic()))
        if time.monotonic() - sample_at > MAX_FEEDBACK_AGE_S:
            current, sample_at = _read_replay_feedback(robot, scene)
            check_feedback_pose(current)
        if time.monotonic() - sample_at > MAX_FEEDBACK_AGE_S:
            raise RuntimeError(f"Encoder feedback is stale at {label}: HOLD")
        if _max_joint_delta(target, current) > step_limit + 1e-6:
            raise RuntimeError(
                f"Robot fell behind at {label}; target exceeds the encoder-relative step: HOLD"
            )
        last_command_at = time.monotonic()
        sent = robot.send_action(_actions(target))
        control_steps += 1
        sent_motor = {
            side: np.array([sent[f"{side}_{name}.pos"] for name in JOINTS], dtype=float) for side in SIDES
        }
        current, sample_at = _read_replay_feedback(robot, scene)
        speed = _max_joint_delta(sent_motor, previous_sent) * control_hz
        max_observed_speed = max(max_observed_speed, speed)
        previous_sent = sent_motor
        if not np.isfinite(speed) or _max_joint_delta(sent_motor, target) > 1e-6:
            raise RuntimeError(f"Follower changed the validated setpoint at {label}: HOLD")
        if _max_joint_delta(current, target) > max_tracking_error_deg:
            raise RuntimeError(
                f"Robot fell behind at {label}; encoder tracking error exceeds tolerance: HOLD"
            )
        check_feedback_pose(current)
        return sent_motor

    try:
        blockers = _motion_prerequisites(document, robot, scene)
        if blockers:
            raise ValueError("Replay prerequisites: " + ", ".join(blockers))
        geometry = ReplayGeometry.build(config["robot"], episode.metadata.get("obstacles", []), scene)
        simulation = geometry.simulation
        _connect_checked_buses(robot, report["usb_ports"])
        if not _read_bus_state(robot, report["usb_ports"]):
            raise ValueError("Both arms must report position mode and valid torque readback")
        current, sample_at = _read_replay_feedback(robot, scene)
        report["pose_context"] = build_pose_context(episode, document, robot, config, current=current)
        transition = evaluate_initial_transition(
            current,
            episode,
            document,
            robot,
            max_step_deg=step_limit,
            max_speed_deg_s=max_joint_speed_deg_s,
            control_hz=control_hz,
            geometry=geometry,
            strategy=initial_move_strategy,
        )
        transition["applied"] = False
        report["initial_transition"] = transition
        report["start_pose"] = _start_pose_report(
            report["pose_context"], transition, document, robot, max_start_error_deg
        )
        if not transition["safe"]:
            raise ValueError(
                "Initial transition to the dataset start is not safe: "
                f"limit_violations={len(transition['limit_violations'])}, "
                f"modeled_collision_frames={transition['collision_frames'][:10]}, "
                f"clearance_violation_steps={transition['clearance_violation_steps'][:10]}"
            )
        if not report["start_pose"]["within_tolerance"] and not allow_initial_move:
            start_max = report["start_pose"]["max_error_deg"]
            raise ValueError(
                f"Robot start pose differs from the dataset start by {start_max:.2f} deg "
                f"(limit {max_start_error_deg:g}); review the read-only report before using --allow-initial-move"
            )
        segments = _episode_segments(
            episode,
            document,
            max_step_deg=step_limit,
            max_speed_deg_s=max_joint_speed_deg_s,
            control_hz=control_hz,
        )
        report["trajectory_geometry"] = _check_episode_geometry(episode, segments, document, geometry)
        if not report["trajectory_geometry"]["modeled_safe"]:
            raise ValueError(
                "Interpolated dataset trajectory violates modeled collision, clearance, or joint limits"
            )
        live, sample_at = _read_replay_feedback(robot, scene)
        if _max_joint_delta(live, current) > 1e-6:
            raise RuntimeError(
                "Start encoders changed during preflight; repeat --check-start from the new pose"
            )
        writes_started = True
        initial_sent = robot.send_action(_actions(current))
        initial_sent_motor = {
            side: np.array([initial_sent[f"{side}_{name}.pos"] for name in JOINTS], dtype=float)
            for side in SIDES
        }
        if _max_joint_delta(initial_sent_motor, current) > 1e-6:
            raise RuntimeError("Follower changed the initial hold target: HOLD")
        _verify_initial_hold(robot, current)
        report["initial_hold_verified"] = True
        live, sample_at = _read_replay_feedback(robot, scene)
        if _max_joint_delta(live, current) > step_limit:
            raise RuntimeError("Unexpected movement before enabling torque: HOLD")
        for side in SIDES:
            disabled = [joint for joint in JOINTS if not report["usb_ports"][side]["torque_enable"][joint]]
            if disabled:
                getattr(robot, side + "_bus").enable_torque(disabled)
        live, sample_at = _read_replay_feedback(robot, scene)
        if _max_joint_delta(live, current) > step_limit:
            raise RuntimeError("Unexpected movement while establishing initial hold: HOLD")
        current = live
        check_feedback_pose(current)
        report["status"] = "running"
        start_time = time.monotonic()
        last_command_at = start_time
        previous_sent = initial_sent_motor
        if transition["required"]:
            total_steps = transition["steps"]
            for sample in transition["samples"][1:]:
                target = {side: np.asarray(sample["target_motor_deg"][side]) for side in SIDES}
                initial_sent_motor = send_checked_target(target, f"initial transition step {sample['step']}")
            first = {side: map_joints(document, side, episode.frames[0].joints[side]) for side in SIDES}
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
                    "control_steps": total_steps,
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
            position, quaternion = geometry.tcp_pose(side)
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
        for index, (previous_motor, target_motor, steps) in enumerate(segments):
            following = episode.frames[index + 1]
            segment_duration = steps / control_hz
            for substep in range(1, steps + 1):
                commanded = _interpolate(previous_motor, target_motor, substep / steps)
                sent_motor = send_checked_target(commanded, f"dataset frame {index + 1}, substep {substep}")
            measured = current
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
                position, quaternion = geometry.tcp_pose(side)
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
        report["status"] = "complete"
    except BaseException as exc:
        failure = exc
        report["status"], report["failure"] = "aborted", str(exc) or type(exc).__name__
    finally:
        if writes_started and robot.is_connected:
            try:
                hold, _ = _read_replay_feedback(robot)
                if motor_pose_limit_violations(hold, None, robot):
                    raise RuntimeError("No calibrated encoder pose for HOLD")
                robot.send_action(_actions(hold))
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
        if geometry is not None:
            with suppress(Exception):
                geometry.simulation.close()
        _write_json(output, report)
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
    mapping_path=None,
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
    initial_move_strategy="auto",
    scene_geometry=None,
):
    _validate_replay_limits(
        max_start_error_deg,
        max_tracking_error_deg,
        max_joint_speed_deg_s,
        control_hz,
        max_sim_error_m,
        max_sim_orientation_error_deg,
    )
    scene = SceneGeometry.from_file(scene_geometry) if scene_geometry is not None else None
    if report is not None and Path(report).exists():
        raise FileExistsError(f"Report already exists: {report}")
    episode = load_joint_episode(dataset_path, episode_index)
    model_hash = file_sha256(full_model_path(config["robot"]))
    if episode.metadata.get("mujoco_model_sha256") != model_hash:
        raise ValueError("Dataset MuJoCo model differs from the selected model; regenerate the dataset")
    if not robot_id:
        raise ValueError("--robot-id is required to load the follower calibration")
    measured_scene = scene if measured_scene_ready(scene) else None
    follower = (
        arm_and_gripper_follower
        if measured_scene is not None and measured_scene.replay_gripper_poses is not None
        else arm_only_follower
    )
    robot = follower(
        left_port or "/dev/not-used-left",
        right_port or "/dev/not-used-right",
        robot_id,
        config["robot"]["max_joint_step_deg"],
        calibration_dir,
    )
    if mapping_path is not None:
        mapping = load_mapping(mapping_path)
    else:
        cal_doc, _ = build_calibration_mapping(robot, config=config, episode=episode)
        mapping = {"path": None, "document": cal_doc}
    document = mapping["document"]
    mapping_info = validate_mapping_for_robot(
        document, robot, model_sha256=model_hash, allow_calibration_assumed=True
    )
    validation = validate_mapped_episode(episode, mapping["document"], robot)
    inspection = inspect_joint_mapping(episode, mapping["document"], robot, config)
    if not execute:
        result = {
            "schema": REPLAY_SCHEMA,
            "mode": "validation_only",
            "checked_at": _now(),
            "dataset": str(episode.root),
            "dataset_sha256": episode.dataset_sha256,
            "dataset_manifest_sha256": episode.manifest_sha256,
            "mujoco_model_sha256": model_hash,
            "mapping_sha256": file_sha256(mapping_path) if mapping_path is not None else None,
            "episode": episode.episode,
            "frames": len(episode.frames),
            "mapping": mapping_info,
            "scene": scene.to_dict() if scene is not None else None,
            "usb_ports": {},
            "joint_inspection": inspection,
            "pose_context": build_pose_context(episode, document, robot, config),
            "initial_transition": None,
            "gripper_commanded": False,
            "simulation_only_source": True,
            "validation": validation,
            "unverified": {
                "joints": inspection["unverified_joints"],
                "model_joint_limits": "Full CAD preview bounds; physical limits require measured evidence",
                "external_obstacles": "Physical clearance requires a measured scene",
                "physical_tcp_accuracy": "Estimated using Full-CAD FK and encoder readings",
            },
        }
        blockers = _motion_prerequisites(document, robot, scene)
        result["preflight"] = {"ready": False, "ready_with_initial_move": False, "blockers": blockers}
        geometry = None
        try:
            if not check_start:
                blockers.append("Current pose and trajectory geometry require --check-start")
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
            _connect_checked_buses(robot, result["usb_ports"])
            if not _read_bus_state(robot, result["usb_ports"]):
                blockers.append("Both arms must report position mode and valid torque readback")
            current, _ = _read_replay_feedback(robot, measured_scene)
            result["pose_context"] = build_pose_context(episode, document, robot, config, current=current)
            geometry = ReplayGeometry.build(config["robot"], episode.metadata.get("obstacles", []), scene)
            transition = evaluate_initial_transition(
                current,
                episode,
                document,
                robot,
                max_step_deg=step_limit,
                max_speed_deg_s=max_joint_speed_deg_s,
                control_hz=control_hz,
                geometry=geometry,
                strategy=initial_move_strategy,
            )
            result["initial_transition"] = transition
            start = _start_pose_report(
                result["pose_context"], transition, document, robot, max_start_error_deg
            )
            result["start_pose"] = start
            if not transition["modeled_safe"]:
                blockers.append(
                    "Initial transition violates modeled collision, clearance, speed or joint bounds"
                )
            segments = _episode_segments(
                episode,
                document,
                max_step_deg=step_limit,
                max_speed_deg_s=max_joint_speed_deg_s,
                control_hz=control_hz,
            )
            trajectory = _check_episode_geometry(episode, segments, document, geometry)
            result["trajectory_geometry"] = trajectory
            if not trajectory["modeled_safe"]:
                blockers.append("Interpolated dataset violates modeled collision, clearance or joint bounds")
            live, _ = _read_replay_feedback(robot, measured_scene)
            if _max_joint_delta(current, live) > 1e-6:
                blockers.append(
                    "Start encoders changed during preflight; repeat --check-start from the new pose"
                )
            result["preflight"]["ready_with_initial_move"] = not blockers
            if not start["within_tolerance"]:
                blockers.append(
                    f"Initial pose differs from the dataset start by {start['max_error_deg']:.2f} deg"
                )
            result["preflight"]["ready"] = not blockers
            return result
        except (ValueError, OSError, RuntimeError) as exc:
            blockers.append(str(exc))
            raise
        finally:
            if geometry is not None:
                geometry.simulation.close()
            for side in SIDES:
                bus = getattr(robot, side + "_bus")
                if bus.is_connected:
                    with suppress(Exception):
                        bus.disconnect(disable_torque=False)
            if report is not None:
                _write_json(report, result)
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
        initial_move_strategy=initial_move_strategy,
        scene=scene,
    )
    if dataset_fingerprint(episode.root, "mocopi_joint_dataset.json") != before:
        raise ValueError("Source joint dataset changed during hardware replay")
    return result


def _ft_feature_names(*, include_gripper):
    if not include_gripper:
        return [f"{side}_{name}.pos" for side in JOINT_SIDES for name in JOINTS]
    names = []
    for side in JOINT_SIDES:
        names.extend(f"{side}_{name}.pos" for name in JOINTS)
        names.append(f"{side}_gripper.pos")
    return names


def _map_ft_motor_vector(document, model_vector_14, *, include_gripper, gripper_value_deg):
    vec = np.asarray(model_vector_14, dtype=float)
    if vec.shape != (14,) or not np.isfinite(vec).all():
        raise ValueError(f"Expected 14 finite model joint angles, got shape {vec.shape}")
    right_motor = map_joints(document, "right", vec[:7])
    left_motor = map_joints(document, "left", vec[7:])
    if not include_gripper:
        return np.concatenate([right_motor, left_motor]).astype(np.float32)
    grip = np.array([float(gripper_value_deg)], dtype=float)
    return np.concatenate([right_motor, grip, left_motor, grip]).astype(np.float32)


def export_ft_dataset(
    dataset_path,
    mapping_path,
    output,
    repo_id,
    *,
    config=None,
    robot_id=None,
    calibration_dir=None,
    replay_report=None,
    allow_without_replay_report=False,
    include_gripper=True,
    gripper_value_deg=0.0,
    image_key="observation.images.head",
    episode_index=None,
):
    """Apply a verified hardware mapping to a full-CAD joint dataset for policy fine-tuning.

    Converts ``observation.state`` and ``action`` from uncalibrated Full-CAD model
    degrees (``q_model``) into the follower's calibrated motor degree space
    (``q_motor = sign * q_model + offset_deg``), optionally padding constant
    gripper entries (16-dim follower format) and aliasing the camera key without
    mutating the source dataset or claiming physical gripper capture.
    """
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    source_root = Path(dataset_path).resolve()
    output_root = Path(output).resolve()
    if output_root.exists():
        raise FileExistsError(f"FT dataset output already exists: {output_root}")
    if output_root.is_relative_to(source_root) or source_root.is_relative_to(output_root):
        raise ValueError("Source and FT output datasets must be in separate directories")
    if not str(repo_id).strip():
        raise ValueError("repo_id must be nonempty")
    if not str(image_key).strip():
        raise ValueError("image_key must be nonempty")
    if not np.isfinite(float(gripper_value_deg)):
        raise ValueError("gripper_value_deg must be finite")

    meta_path = source_root / "meta" / "mocopi_joint_dataset.json"
    info_path = source_root / "meta" / "info.json"
    if not meta_path.is_file() or not info_path.is_file():
        raise ValueError(f"Not a mocopi joint dataset: {source_root}")
    source_meta = json.loads(meta_path.read_text(encoding="utf-8"))
    source_fp = dataset_fingerprint(source_root, "mocopi_joint_dataset.json")
    model_hash = (
        file_sha256(full_model_path(config["robot"])) if config else source_meta.get("mujoco_model_sha256")
    )
    if source_meta.get("mujoco_model_sha256") != model_hash:
        raise ValueError("Dataset MuJoCo model differs from selected config model")

    step_limit = float(config["robot"]["max_joint_step_deg"]) if config else 0.5
    if mapping_path is not None:
        mapping = load_mapping(mapping_path)
        document = mapping["document"]
        effective_robot_id = robot_id or document.get("robot_id")
        if not effective_robot_id:
            raise ValueError("robot_id is required either in the mapping YAML or via --robot-id")
        robot = arm_only_follower(
            "/dev/not-used-left",
            "/dev/not-used-right",
            effective_robot_id,
            step_limit,
            calibration_dir,
        )
    else:
        effective_robot_id = robot_id
        if not effective_robot_id:
            raise ValueError("--robot-id is required when --mapping is omitted")
        robot = arm_only_follower(
            "/dev/not-used-left",
            "/dev/not-used-right",
            effective_robot_id,
            step_limit,
            calibration_dir,
        )
        document, _ = build_calibration_mapping(robot, config=config)
    mapping_info = validate_mapping_for_robot(
        document,
        robot,
        model_sha256=model_hash,
        allow_calibration_assumed=allow_without_replay_report and replay_report is None,
    )

    replay_report_info = None
    if replay_report is not None:
        report_path = Path(replay_report).resolve()
        rep = json.loads(report_path.read_text(encoding="utf-8"))
        if rep.get("schema") != REPLAY_SCHEMA or rep.get("status") != "complete":
            raise ValueError(
                f"Hardware replay report {report_path} must have schema={REPLAY_SCHEMA!r} and status='complete'"
            )
        if rep.get("gripper_commanded") is not False:
            raise ValueError("Hardware replay report shows unexpected gripper commands")
        if rep.get("dataset_sha256") != source_fp:
            raise ValueError("Hardware replay report dataset_sha256 does not match source dataset")
        if rep.get("mujoco_model_sha256") != model_hash:
            raise ValueError("Hardware replay report mujoco_model_sha256 does not match model")
        if rep.get("calibration_sha256") != mapping_info["calibration_sha256"]:
            raise ValueError("Hardware replay report calibration_sha256 does not match robot calibration")
        if rep.get("robot_id") != effective_robot_id:
            raise ValueError("Hardware replay report robot_id does not match selected robot")
        replay_report_info = {
            "path": str(report_path),
            "sha256": file_sha256(report_path),
            "status": rep["status"],
            "frames": len(rep.get("frames", [])),
        }
    elif not allow_without_replay_report:
        raise ValueError(
            "FT dataset export requires a completed hardware replay verification report "
            "(--replay-report), or pass --allow-without-replay-report explicitly"
        )

    source = LeRobotDataset(source_meta["repo_id"], root=source_root, video_backend="pyav")
    if episode_index is not None:
        if not 0 <= int(episode_index) < source.num_episodes:
            raise ValueError(f"Episode index {episode_index} out of range (0..{source.num_episodes - 1})")
        episode_indices = [int(episode_index)]
    else:
        episode_indices = list(range(source.num_episodes))

    validated_episodes = []
    for ep_idx in episode_indices:
        ep = load_joint_episode(source_root, ep_idx)
        validation = validate_mapped_episode(ep, document, robot)
        validated_episodes.append((ep_idx, validation))

    src_cam_key = "observation.images.head"
    if src_cam_key not in source.meta.features:
        raise ValueError(f"Source dataset missing expected camera feature {src_cam_key!r}")
    cam_meta = source.meta.features[src_cam_key]
    ft_names = _ft_feature_names(include_gripper=include_gripper)
    features = {
        image_key: {
            "dtype": "video",
            "shape": tuple(cam_meta["shape"]),
            "names": list(cam_meta.get("names") or ["channels", "height", "width"]),
        },
        "observation.state": {"dtype": "float32", "shape": (len(ft_names),), "names": ft_names},
        "action": {"dtype": "float32", "shape": (len(ft_names),), "names": ft_names},
    }
    destination = LeRobotDataset.create(
        repo_id=repo_id,
        fps=int(source.fps),
        features=features,
        root=output_root,
        robot_type="dual_scorpion_follower",
        use_videos=True,
        vcodec="h264",
        image_writer_threads=2,
    )
    exported_episodes = []
    try:
        for ep_idx, validation in validated_episodes:
            ep_meta = source.meta.episodes[ep_idx]
            start = int(ep_meta["dataset_from_index"])
            end = int(ep_meta["dataset_to_index"])
            for row_idx in range(start, end):
                row = source[row_idx]
                mapped_state = _map_ft_motor_vector(
                    document,
                    row["observation.state"].numpy(),
                    include_gripper=include_gripper,
                    gripper_value_deg=gripper_value_deg,
                )
                mapped_action = _map_ft_motor_vector(
                    document,
                    row["action"].numpy(),
                    include_gripper=include_gripper,
                    gripper_value_deg=gripper_value_deg,
                )
                destination.add_frame(
                    {
                        "task": row["task"],
                        image_key: row[src_cam_key].numpy(),
                        "observation.state": mapped_state,
                        "action": mapped_action,
                    }
                )
            destination.save_episode()
            exported_episodes.append(
                {
                    "episode_index": ep_idx,
                    "frames": end - start,
                    "calibration_margins_deg": validation["calibration_margins_deg"],
                }
            )
    finally:
        destination.finalize()

    if dataset_fingerprint(source_root, "mocopi_joint_dataset.json") != source_fp:
        raise ValueError("Source dataset changed during FT export")

    manifest = {
        "schema": FT_DATASET_SCHEMA,
        "repo_id": repo_id,
        "source_dataset": str(source_root),
        "source_dataset_repo_id": source_meta["repo_id"],
        "source_dataset_sha256": source_fp,
        "robot_id": effective_robot_id,
        "calibration_sha256": mapping_info["calibration_sha256"],
        "mujoco_model_sha256": model_hash,
        "mapping_path": str(Path(mapping_path).resolve()) if mapping_path is not None else None,
        "mapping_verification_method": document.get("verification_method"),
        "mapping_verification_status": mapping_info["verification_status"],
        "hardware_replay_verified": replay_report_info is not None,
        "hardware_replay_report": replay_report_info,
        "fps": int(source.fps),
        "robot_type": "dual_scorpion_follower",
        "state_action_names": ft_names,
        "state_action_dim": len(ft_names),
        "joint_unit": "degree",
        "include_gripper": bool(include_gripper),
        "gripper_synthesized_constant": bool(include_gripper),
        "gripper_value_deg": float(gripper_value_deg) if include_gripper else None,
        "camera_mapping": {"source_key": src_cam_key, "exported_key": image_key},
        "state_semantics": (
            "Full-CAD model joints mapped via q_motor = sign * q_model + offset_deg into "
            "the follower calibrated degree space; synthesized constant gripper if include_gripper=True."
        ),
        "action_semantics": (
            "Next-frame Full-CAD model target mapped via q_motor = sign * q_model + offset_deg "
            "into follower calibrated motor degrees."
        ),
        "episodes": exported_episodes,
    }
    (output_root / "meta" / "mocopi_ft_dataset.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    return manifest
