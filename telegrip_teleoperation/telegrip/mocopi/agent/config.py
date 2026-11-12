"""Configuration for the real-robot agent harness.

The harness never hard-codes a camera device, serial port, or robot ID. Every
value comes from an explicit YAML file supplied by the operator.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


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
class HarnessConfig:
    robot_config_path: Path
    cameras: tuple[CameraSpec, ...]
    authorization_token_file: Path | None
    max_feedback_age_s: float
    max_plan_frames: int
    execution_report_dir: Path

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
        robot_config_value = raw.get("robot_config")
        if robot_config_value is None:
            raise ValueError("agent.robot_config is required")
        robot_config_path = Path(robot_config_value)
        if not robot_config_path.is_absolute():
            robot_config_path = config_path.parent / robot_config_path
        robot_config = robot_config_path.resolve()
        token_file = raw.get("authorization_token_file")
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
            authorization_token_file=Path(token_file).resolve() if token_file else None,
            max_feedback_age_s=max_age,
            max_plan_frames=max_frames,
            execution_report_dir=report_dir,
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
