"""Configuration for the real-robot agent harness.

The harness never hard-codes a camera device, serial port, or robot ID. Every
value comes from an explicit YAML file supplied by the operator.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from ...config import GRIPPER_CLOSED_ANGLE, GRIPPER_OPEN_ANGLE


@dataclass(frozen=True)
class CameraSpec:
    camera_id: str
    device: str | int
    width: int
    height: int
    fps: int
    fourcc: str | None = None
    coordinate_frame: str | None = None
    extrinsics_path: str | None = None


@dataclass(frozen=True)
class RobotSpec:
    robot_id: str
    left_port: str
    right_port: str
    calibration_dir: Path | None = None
    max_step_deg: float = 0.5


@dataclass(frozen=True)
class GripperSpec:
    open_position: float = GRIPPER_OPEN_ANGLE
    closed_position: float = GRIPPER_CLOSED_ANGLE
    min_position: float = 0.0
    max_position: float = 100.0
    position_tolerance: float = 2.0
    motion_observation_threshold: float = 0.1
    max_step: float = 2.0
    settle_s: float = 0.02
    poll_interval_s: float = 0.05
    motion_timeout_s: float = 1.0
    grasp_timeout_s: float = 2.0
    load_threshold: float | None = None
    current_threshold: float | None = None


@dataclass(frozen=True)
class HarnessConfig:
    robot_config_path: Path
    cameras: tuple[CameraSpec, ...]
    operation_grants_file: Path | None
    max_feedback_age_s: float
    max_plan_frames: int
    execution_report_dir: Path
    robot: RobotSpec | None = None
    gripper: GripperSpec = GripperSpec()
    scene_geometry_path: Path | None = None

    @classmethod
    def from_file(cls, path: str | Path) -> HarnessConfig:
        config_path = Path(path).resolve()
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        if not isinstance(raw, dict):
            raise ValueError("agent harness config must be a YAML object")
        cameras_raw = raw.get("cameras", [])
        if not isinstance(cameras_raw, list):
            raise ValueError("agent.cameras must be a list")
        cameras = tuple(_camera_from_dict(item, config_path.parent) for item in cameras_raw)
        robot_raw = raw.get("robot")
        robot_spec = None if robot_raw is None else _robot_from_dict(robot_raw, config_path.parent)
        gripper_raw = raw.get("gripper", {})
        gripper_spec = _gripper_from_dict(gripper_raw)
        scene_value = raw.get("scene_geometry")
        scene_path = None
        if scene_value is not None:
            scene_path = Path(scene_value)
            if not scene_path.is_absolute():
                scene_path = config_path.parent / scene_path
            scene_path = scene_path.resolve()
        robot_config_value = raw.get("robot_config")
        if robot_config_value is None:
            raise ValueError("agent.robot_config is required")
        robot_config_path = Path(robot_config_value)
        if not robot_config_path.is_absolute():
            robot_config_path = config_path.parent / robot_config_path
        robot_config = robot_config_path.resolve()
        grants_file = raw.get("operation_grants_file")
        max_age = float(raw.get("max_feedback_age_s", 0.5))
        max_frames = int(raw.get("max_plan_frames", 240))
        report_dir = Path(raw.get("execution_report_dir", "outputs/agent/executions"))
        if not report_dir.is_absolute():
            report_dir = config_path.parent / report_dir
        if max_age <= 0 or max_frames <= 1:
            raise ValueError("max_feedback_age_s and max_plan_frames must be positive")
        return cls(
            robot_config_path=robot_config,
            cameras=cameras,
            operation_grants_file=Path(grants_file).expanduser().resolve() if grants_file else None,
            max_feedback_age_s=max_age,
            max_plan_frames=max_frames,
            execution_report_dir=report_dir,
            robot=robot_spec,
            gripper=gripper_spec,
            scene_geometry_path=scene_path,
        )


def _camera_from_dict(value: dict[str, Any], base_dir: Path) -> CameraSpec:
    if not isinstance(value, dict):
        raise ValueError("each camera entry must be an object")
    camera_id = value.get("id")
    device = value.get("device")
    if not isinstance(camera_id, str) or not camera_id.strip():
        raise ValueError("camera.id must be a non-empty string")
    if device is None:
        raise ValueError(f"camera {camera_id!r} requires a device")
    width = int(value.get("width", 640))
    height = int(value.get("height", 480))
    fps = int(value.get("fps", 30))
    if width <= 0 or height <= 0 or fps <= 0:
        raise ValueError(f"camera {camera_id!r} dimensions and fps must be positive")
    extrinsics = value.get("extrinsics")
    if extrinsics is not None:
        extrinsics_path = Path(extrinsics)
        if not extrinsics_path.is_absolute():
            extrinsics_path = base_dir / extrinsics_path
    else:
        extrinsics_path = None
    return CameraSpec(
        camera_id=camera_id,
        device=device,
        width=width,
        height=height,
        fps=fps,
        fourcc=value.get("fourcc"),
        coordinate_frame=value.get("coordinate_frame"),
        extrinsics_path=str(extrinsics_path) if extrinsics_path else None,
    )


def _robot_from_dict(value: dict[str, Any], base_dir: Path) -> RobotSpec:
    if not isinstance(value, dict):
        raise ValueError("agent.robot must be an object")
    robot_id = value.get("id")
    left_port = value.get("left_port")
    right_port = value.get("right_port")
    if not isinstance(robot_id, str) or not robot_id.strip():
        raise ValueError("agent.robot.id must be a non-empty string")
    if not isinstance(left_port, str) or not left_port.strip():
        raise ValueError("agent.robot.left_port must be a device path")
    if not isinstance(right_port, str) or not right_port.strip():
        raise ValueError("agent.robot.right_port must be a device path")
    calibration_dir_value = value.get("calibration_dir")
    calibration_dir = None
    if calibration_dir_value is not None:
        calibration_dir = Path(calibration_dir_value).expanduser()
        if not calibration_dir.is_absolute():
            calibration_dir = base_dir / calibration_dir
    max_step = float(value.get("max_step_deg", 0.5))
    if not 0 < max_step <= 15:
        raise ValueError("agent.robot.max_step_deg must be in (0, 15]")
    return RobotSpec(
        robot_id=robot_id,
        left_port=left_port,
        right_port=right_port,
        calibration_dir=calibration_dir.resolve() if calibration_dir else None,
        max_step_deg=max_step,
    )


def _gripper_from_dict(value: Any) -> GripperSpec:
    if value is None:
        return GripperSpec()
    if not isinstance(value, dict):
        raise ValueError("agent.gripper must be an object")
    spec = GripperSpec(
        open_position=float(value.get("open_position", 0.0)),
        closed_position=float(value.get("closed_position", 45.0)),
        min_position=float(value.get("min_position", 0.0)),
        max_position=float(value.get("max_position", 100.0)),
        position_tolerance=float(value.get("position_tolerance", 2.0)),
        motion_observation_threshold=float(value.get("motion_observation_threshold", 0.1)),
        max_step=float(value.get("max_step", 2.0)),
        settle_s=float(value.get("settle_s", 0.02)),
        poll_interval_s=float(value.get("poll_interval_s", 0.05)),
        motion_timeout_s=float(value.get("motion_timeout_s", 1.0)),
        grasp_timeout_s=float(value.get("grasp_timeout_s", 2.0)),
        load_threshold=value.get("load_threshold"),
        current_threshold=value.get("current_threshold"),
    )
    positive = (
        spec.position_tolerance,
        spec.motion_observation_threshold,
        spec.max_step,
        spec.settle_s,
        spec.poll_interval_s,
        spec.motion_timeout_s,
        spec.grasp_timeout_s,
    )
    if any(not np.isfinite(item) or item <= 0 for item in positive):
        raise ValueError(
            "gripper tolerance, observation threshold, step, settle, poll, and timeout values must be positive"
        )
    if not spec.min_position < spec.max_position:
        raise ValueError("gripper min_position must be less than max_position")
    if not spec.min_position <= spec.open_position <= spec.max_position:
        raise ValueError("gripper open_position is outside the normalized range")
    if not spec.min_position <= spec.closed_position <= spec.max_position:
        raise ValueError("gripper closed_position is outside the normalized range")
    if spec.open_position == spec.closed_position:
        raise ValueError("gripper open and closed positions must differ")
    for name in ("load_threshold", "current_threshold"):
        threshold = getattr(spec, name)
        if threshold is not None and (not np.isfinite(threshold) or threshold <= 0):
            raise ValueError(f"gripper {name} must be positive when provided")
    return spec
