"""Typed, provenance-aware WorldState for the real Dual Scorpion robot."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


class StateSource:
    MEASURED = "measured"
    ESTIMATED = "estimated"
    LLM_SEMANTIC = "llm_semantic"
    SIMULATED = "simulated"
    UNKNOWN = "unknown"
    ALL = {MEASURED, ESTIMATED, LLM_SEMANTIC, SIMULATED, UNKNOWN}


@dataclass(frozen=True)
class ObjectState:
    id: str
    label: str
    source: str
    position_m: tuple[float, float, float] | None
    orientation_xyzw: tuple[float, float, float, float] | None
    confidence: float | None
    timestamp_monotonic_ns: int
    coordinate_validation: dict[str, Any] | None
    properties: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> ObjectState:
        _require_object(value, "object")
        object_id = _required_string(value, "id", "object.id")
        label = _required_string(value, "label", f"object {object_id!r} label")
        source = value.get("source", StateSource.UNKNOWN)
        if source not in StateSource.ALL:
            raise ValueError(f"object {object_id!r} has invalid source {source!r}")
        position = _optional_vector(value.get("position_m"), 3, f"object {object_id!r} position_m")
        orientation = _optional_vector(
            value.get("orientation_xyzw"), 4, f"object {object_id!r} orientation_xyzw"
        )
        if orientation is not None and abs(np.linalg.norm(orientation) - 1.0) > 1e-5:
            raise ValueError(f"object {object_id!r} orientation must be a unit quaternion")
        confidence = value.get("confidence")
        if confidence is not None and (not np.isfinite(confidence) or not 0 <= confidence <= 1):
            raise ValueError(f"object {object_id!r} confidence must be in [0, 1]")
        timestamp = int(value.get("timestamp_monotonic_ns", 0))
        validation = value.get("coordinate_validation")
        if validation is not None and not isinstance(validation, dict):
            raise ValueError(f"object {object_id!r} coordinate_validation must be an object")
        if position is not None and not validation:
            raise ValueError(
                f"object {object_id!r} has a 3D position without coordinate_validation; "
                "semantic image estimates must not be treated as metric coordinates"
            )
        return cls(
            id=object_id,
            label=label,
            source=source,
            position_m=None if position is None else tuple(float(item) for item in position),
            orientation_xyzw=None if orientation is None else tuple(float(item) for item in orientation),
            confidence=None if confidence is None else float(confidence),
            timestamp_monotonic_ns=timestamp,
            coordinate_validation=dict(validation or {}),
            properties=dict(value.get("properties", {})),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "label": self.label,
            "source": self.source,
            "position_m": None if self.position_m is None else list(self.position_m),
            "orientation_xyzw": None if self.orientation_xyzw is None else list(self.orientation_xyzw),
            "confidence": self.confidence,
            "timestamp_monotonic_ns": self.timestamp_monotonic_ns,
            "coordinate_validation": None
            if self.coordinate_validation is None
            else dict(self.coordinate_validation),
            "properties": dict(self.properties),
        }


@dataclass(frozen=True)
class ArmState:
    side: str
    source: str
    joint_deg: tuple[float, ...]
    tcp_position_m: tuple[float, float, float] | None
    tcp_orientation_xyzw: tuple[float, float, float, float] | None
    connected: bool
    calibrated: bool
    timestamp_monotonic_ns: int
    command_error_deg: float | None = None
    gripper_state: str = "not_integrated"

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> ArmState:
        _require_object(value, "arm")
        side = _required_string(value, "side", "arm.side")
        if side not in {"left", "right"}:
            raise ValueError("arm.side must be left or right")
        source = value.get("source", StateSource.MEASURED)
        if source not in StateSource.ALL:
            raise ValueError(f"arm {side!r} has invalid source {source!r}")
        joints = _optional_vector(value.get("joint_deg"), 7, f"arm {side!r} joint_deg")
        if joints is None:
            raise ValueError(f"arm {side!r} joint_deg is required")
        position = _optional_vector(value.get("tcp_position_m"), 3, f"arm {side!r} tcp_position_m")
        orientation = _optional_vector(
            value.get("tcp_orientation_xyzw"), 4, f"arm {side!r} tcp_orientation_xyzw"
        )
        if orientation is not None and abs(np.linalg.norm(orientation) - 1.0) > 1e-5:
            raise ValueError(f"arm {side!r} TCP orientation must be a unit quaternion")
        command_error = value.get("command_error_deg")
        if command_error is not None and not np.isfinite(command_error):
            raise ValueError(f"arm {side!r} command_error_deg must be finite")
        return cls(
            side=side,
            source=source,
            joint_deg=tuple(float(item) for item in joints),
            tcp_position_m=None if position is None else tuple(float(item) for item in position),
            tcp_orientation_xyzw=None
            if orientation is None
            else tuple(float(item) for item in orientation),
            connected=bool(value.get("connected", False)),
            calibrated=bool(value.get("calibrated", False)),
            timestamp_monotonic_ns=int(value.get("timestamp_monotonic_ns", 0)),
            command_error_deg=None if command_error is None else float(command_error),
            gripper_state=str(value.get("gripper_state", "not_integrated")),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "side": self.side,
            "source": self.source,
            "joint_deg": list(self.joint_deg),
            "tcp_position_m": None if self.tcp_position_m is None else list(self.tcp_position_m),
            "tcp_orientation_xyzw": None
            if self.tcp_orientation_xyzw is None
            else list(self.tcp_orientation_xyzw),
            "connected": self.connected,
            "calibrated": self.calibrated,
            "timestamp_monotonic_ns": self.timestamp_monotonic_ns,
            "command_error_deg": self.command_error_deg,
            "gripper_state": self.gripper_state,
        }


@dataclass(frozen=True)
class CameraState:
    camera_id: str
    connected: bool
    last_frame_monotonic_ns: int | None
    width: int
    height: int
    coordinate_frame: str | None
    last_error: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "camera_id": self.camera_id,
            "connected": self.connected,
            "last_frame_monotonic_ns": self.last_frame_monotonic_ns,
            "width": self.width,
            "height": self.height,
            "coordinate_frame": self.coordinate_frame,
            "last_error": self.last_error,
        }


@dataclass(frozen=True)
class WorldState:
    updated_monotonic_ns: int
    arms: dict[str, ArmState]
    cameras: dict[str, CameraState]
    objects: dict[str, ObjectState]
    task_progress: dict[str, Any]
    metadata: dict[str, Any]

    def __post_init__(self) -> None:
        if set(self.arms) != {"left", "right"}:
            raise ValueError("WorldState must contain exactly left and right arm states")

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": "dual_scorpion.agent.world_state.v1",
            "updated_monotonic_ns": self.updated_monotonic_ns,
            "arms": {side: arm.to_dict() for side, arm in self.arms.items()},
            "cameras": {camera_id: state.to_dict() for camera_id, state in self.cameras.items()},
            "objects": {object_id: state.to_dict() for object_id, state in self.objects.items()},
            "task_progress": dict(self.task_progress),
            "metadata": dict(self.metadata),
        }


def _require_object(value: Any, label: str) -> None:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an object")


def _required_string(value: dict[str, Any], key: str, label: str) -> str:
    result = value.get(key)
    if not isinstance(result, str) or not result.strip():
        raise ValueError(f"{label} must be a non-empty string")
    return result


def _optional_vector(value: Any, size: int, label: str) -> np.ndarray | None:
    if value is None:
        return None
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{label} must contain {size} finite numbers")
    return result
