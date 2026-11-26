"""Zero-CAD guided Follower mapping session and YAML generation for Dual Scorpion."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, Field

from .hardware import scorpion_follower
from .scorpion_view import SCORPION_MODEL_DIR, supports_verified_view_mapping
from .utils.config import is_valid_robot_name, save_robot_record

logger = logging.getLogger(__name__)

mapping_active = False
_active_robot = None


class MappingStartRequest(BaseModel):
    follower_left_port: str
    follower_right_port: str
    follower_config: str


class JointCaptureInput(BaseModel):
    model_reference_deg: float
    motor_reference_deg: float
    start_motor_deg: float
    moved_motor_deg: float
    expected_model_direction: Literal[-1, 1]


class GripperEndpointsInput(BaseModel):
    closed: float
    open: float


class MappingSaveRequest(BaseModel):
    robot_name: str | None = None
    follower_left_port: str
    follower_right_port: str
    follower_config: str
    joints: dict[str, JointCaptureInput] = Field(default_factory=dict)
    view_gripper_deg: dict[str, GripperEndpointsInput] | None = None
    filename: str | None = None
    allow_partial: bool = True


def _disconnect_robot(robot) -> None:
    if robot is None:
        return
    try:
        robot.disconnect()
    except Exception as exc:
        logger.warning("Error disconnecting mapping follower: %s", exc)


def start_mapping_session(request: MappingStartRequest) -> dict:
    """Connect the Dual Scorpion follower and disable torque so joints move freely by hand."""
    global mapping_active, _active_robot
    from . import record as _record, replay as _replay, rollout as _rollout, teleoperate as _teleop
    from .calibrate import calibration_manager

    with _teleop.follower_lock:
        if calibration_manager.get_status().calibration_active:
            return {"success": False, "message": "Calibration is currently active."}
        if _teleop.teleoperation_busy():
            return {"success": False, "message": "Teleoperation is currently active. Stop it first."}
        if _record.recording_active:
            return {"success": False, "message": "Recording is currently active. Stop it first."}
        if _rollout.inference_active:
            return {"success": False, "message": "Inference is currently active. Stop it first."}
        if _replay.replay_active:
            return {"success": False, "message": "Replay is currently active. Stop it first."}
        if mapping_active:
            return {"success": False, "message": "Mapping session is already active."}
        mapping_active = True

    robot = None
    try:
        from lerobot.robots import make_robot_from_config

        cfg = scorpion_follower(
            request.follower_left_port,
            request.follower_right_port,
            request.follower_config,
        )
        robot = make_robot_from_config(cfg)
        robot.connect(calibrate=False)
        for side in ("left_bus", "right_bus"):
            bus = getattr(robot, side, None)
            if bus is not None:
                bus.disable_torque()
        with _teleop.follower_lock:
            _active_robot = robot
        return {"success": True, "message": "Follower connected with torque disabled."}
    except Exception as exc:
        _disconnect_robot(robot)
        with _teleop.follower_lock:
            _active_robot = None
            mapping_active = False
        return {"success": False, "message": str(exc)}


def read_mapping_positions() -> dict:
    """Read live motor positions from the connected Follower mapping session."""
    from . import teleoperate as _teleop

    with _teleop.follower_lock:
        robot = _active_robot
        active = mapping_active
    if not active or robot is None:
        return {"success": False, "active": False, "positions": {}, "message": "Mapping session is not active."}
    try:
        obs = robot.get_observation()
        positions = {
            key: float(val)
            for key, val in obs.items()
            if key.endswith(".pos") and isinstance(val, (int, float))
        }
        return {"success": True, "active": True, "positions": positions}
    except Exception as exc:
        return {"success": False, "active": True, "positions": {}, "message": str(exc)}


def stop_mapping_session() -> dict:
    """Disconnect the Follower mapping session and release ports."""
    global mapping_active, _active_robot
    from . import teleoperate as _teleop

    with _teleop.follower_lock:
        robot = _active_robot
        _active_robot = None
        mapping_active = False
    _disconnect_robot(robot)
    return {"success": True, "message": "Mapping session stopped."}


def _to_visual_mapping(document: dict) -> dict | None:
    sides = document.get("sides")
    if not isinstance(sides, dict):
        return None
    gripper_doc = document.get("view_gripper_deg") or {}
    out: dict = {}
    for side in ("right", "left"):
        arm = sides.get(side, {})
        signs = [s if s in (-1, 1) else (1 if side == "left" else -1) for s in arm.get("sign", [1] * 7)]
        offsets = [
            float(o) if isinstance(o, (int, float)) else 0.0
            for o in arm.get("offset_deg", [0.0] * 7)
        ]
        limits = gripper_doc.get(side, {})
        out[side] = {
            "sign": signs,
            "offset_deg": offsets,
            "gripper_closed_deg": float(limits.get("closed", 0.0)),
            "gripper_open_deg": float(limits.get("open", -45.0)),
        }
    return out


def save_mapping_from_wizard(request: MappingSaveRequest) -> dict:
    """Build and write a hardware mapping YAML from visual 3D alignment captures."""
    if not supports_verified_view_mapping():
        return {"success": False, "message": "Mapping verification tools are not available on this branch."}
    if not request.joints:
        return {"success": False, "message": "Capture at least one joint before saving."}

    from lerobot.robots import make_robot_from_config
    from telegrip.mocopi.dataset.hardware_dataset import (
        verify_joint_mapping_from_measurements,
        write_mapping,
    )

    cfg = scorpion_follower(
        request.follower_left_port,
        request.follower_right_port,
        request.follower_config,
    )
    robot = make_robot_from_config(cfg)

    measurements_joints = {
        joint_key: {
            "model_reference_deg": entry.model_reference_deg,
            "motor_reference_deg": entry.motor_reference_deg,
            "direct_drive_confirmed": True,
            "measurement_basis": "lelab_3d_visual_match",
            "direction_test": {
                "mode": "unidirectional",
                "start_motor_deg": entry.start_motor_deg,
                "moved_motor_deg": entry.moved_motor_deg,
                "expected_model_direction": entry.expected_model_direction,
            },
        }
        for joint_key, entry in request.joints.items()
    }
    model_cfg = {"robot": {"mujoco_full_model": str(SCORPION_MODEL_DIR / "mujoco/scorpion.xml")}}
    document, verification = verify_joint_mapping_from_measurements(
        {"joints": measurements_joints},
        robot,
        config=model_cfg,
        sign_test_deg=10.0,
    )
    if not document["verified"] and not request.allow_partial:
        return {
            "success": False,
            "message": "; ".join(verification.get("failures") or ["Mapping verification incomplete."]),
            "verification": verification,
        }

    if request.view_gripper_deg:
        document["view_gripper_deg"] = {
            side: {"closed": float(ep.closed), "open": float(ep.open)}
            for side, ep in request.view_gripper_deg.items()
            if side in ("right", "left")
        }

    raw_name = (request.filename or f"{robot.id}-mapping.yaml").strip()
    if Path(raw_name).name != raw_name or Path(raw_name).suffix not in {".yaml", ".yml"}:
        return {"success": False, "message": "Invalid mapping filename."}

    out_path = SCORPION_MODEL_DIR.parent / "config" / "mocopi" / "hardware" / raw_name
    write_mapping(out_path, document, overwrite=True)

    if request.robot_name and is_valid_robot_name(request.robot_name):
        patch: dict = {"view_mapping": raw_name}
        vis = _to_visual_mapping(document)
        if vis is not None:
            patch["visual_mapping"] = vis
        save_robot_record(request.robot_name, patch, allow_create=False)

    return {
        "success": True,
        "filename": raw_name,
        "verified": bool(document.get("verified")),
        "verification_status": document.get("verification_status"),
        "unverified_joints": document.get("unverified_joints", []),
        "failures": verification.get("failures", []),
    }
