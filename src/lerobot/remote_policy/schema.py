"""Versioned physical observation and action contracts."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import asdict, dataclass, field
from numbers import Real
from typing import Any
from uuid import uuid4

import numpy as np

PROTOCOL_VERSION = "1.0"
ABSOLUTE = "absolute_joint_position"
RELATIVE = "relative_joint_position"
DELTA = "delta_joint_position"
JOINT_REPRESENTATIONS = (ABSOLUTE, RELATIVE, DELTA)
MAX_CAMERAS = 32


class PolicyError(ValueError):
    """An invalid or incompatible policy request, response, or configuration."""


class StaleResponseError(PolicyError):
    """The result exceeded its local age budget."""


def require_text(value: str, label: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise PolicyError(f"{label} must be a non-empty string")


def names(values: tuple[str, ...] | list[str], label: str, *, allow_empty: bool = False) -> tuple[str, ...]:
    if not isinstance(values, (tuple, list)) or (not values and not allow_empty):
        raise PolicyError(f"{label} must be an array of names")
    if any(not isinstance(value, str) or not value.strip() for value in values):
        raise PolicyError(f"{label} must contain non-empty names")
    if len(set(values)) != len(values):
        raise PolicyError(f"{label} contains duplicate names")
    return tuple(values)


def check_units(units: dict[str, str], features: tuple[str, ...], label: str) -> None:
    if not isinstance(units, dict) or set(units) != set(features):
        raise PolicyError(f"{label} must name a unit for each feature")
    for unit in units.values():
        require_text(unit, label)


def positive(value: float, label: str) -> None:
    if not isinstance(value, Real) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        raise PolicyError(f"{label} must be positive and finite")


def positive_int(value: int, label: str) -> None:
    if type(value) is not int or value <= 0:
        raise PolicyError(f"{label} must be a positive integer")


def finite_array(array: np.ndarray, ndim: int, label: str) -> np.ndarray:
    array = np.asarray(array)
    if array.ndim != ndim or not array.size or array.dtype.kind not in "fiu":
        raise PolicyError(f"{label} must be a non-empty {ndim}-dimensional numeric array")
    with np.errstate(over="ignore", invalid="ignore"):
        result = np.array(array, dtype=np.float32, order="C", copy=True)
    if not np.isfinite(result).all():
        raise PolicyError(f"{label} contains non-finite or out-of-range values")
    result.setflags(write=False)
    return result


def _timestamp(value: int) -> None:
    if type(value) is not int or value < 0:
        raise PolicyError("timestamps must be non-negative integer nanoseconds")


@dataclass(frozen=True)
class RobotObservation:
    """One RGB/state sample. All timestamps use the declared clock domain."""

    robot_id: str
    robot_type: str
    timestamp_ns: int
    state: np.ndarray
    state_names: tuple[str, ...]
    state_units: dict[str, str]
    images: dict[str, np.ndarray]
    image_timestamps_ns: dict[str, int]
    task: str | None = None
    clock: str = "monotonic"
    observation_id: str = field(default_factory=lambda: uuid4().hex)
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for label in ("robot_id", "robot_type", "observation_id"):
            require_text(getattr(self, label), label)
        _timestamp(self.timestamp_ns)
        if self.clock not in ("monotonic", "dataset"):
            raise PolicyError("unknown observation clock domain")
        if self.task is not None:
            require_text(self.task, "task")
        if not isinstance(self.metadata, dict):
            raise PolicyError("observation metadata must be an object")
        state_names = names(self.state_names, "state_names")
        check_units(self.state_units, state_names, "state_units")
        state = finite_array(self.state, 1, "state")
        if state.size != len(state_names):
            raise PolicyError("state length does not match state_names")
        if not isinstance(self.images, dict) or len(self.images) > MAX_CAMERAS:
            raise PolicyError(f"images must be a mapping of up to {MAX_CAMERAS} cameras")
        if (
            not isinstance(self.image_timestamps_ns, dict)
            or self.images.keys() != self.image_timestamps_ns.keys()
        ):
            raise PolicyError("image_timestamps_ns must contain exactly the camera names")
        for name, image in self.images.items():
            require_text(name, "camera name")
            _timestamp(self.image_timestamps_ns[name])
            if (
                not isinstance(image, np.ndarray)
                or image.dtype != np.uint8
                or image.ndim != 3
                or image.shape[-1] != 3
                or not image.size
            ):
                raise PolicyError(f"{name}: RGB images must be non-empty uint8 HWC arrays")
        object.__setattr__(self, "state_names", state_names)
        object.__setattr__(self, "state", state)


@dataclass(frozen=True)
class PolicyCapabilities:
    policy_id: str
    robot_types: tuple[str, ...]
    required_images: tuple[str, ...]
    state_features: tuple[str, ...]
    state_units: dict[str, str]
    action_features: tuple[str, ...]
    action_units: dict[str, str]
    action_horizon: int
    action_representation: str
    control_fps: float
    absolute_features: tuple[str, ...] = ()
    requires_task: bool = False
    protocol_version: str = PROTOCOL_VERSION

    def __post_init__(self) -> None:
        for label in ("policy_id", "action_representation", "protocol_version"):
            require_text(getattr(self, label), label)
        for label in (
            "robot_types",
            "required_images",
            "state_features",
            "action_features",
            "absolute_features",
        ):
            values = names(
                getattr(self, label), label, allow_empty=label in {"required_images", "absolute_features"}
            )
            object.__setattr__(self, label, values)
        _check_absolute_features(self.action_features, self.absolute_features, self.action_representation)
        check_units(self.state_units, self.state_features, "state_units")
        check_units(self.action_units, self.action_features, "action_units")
        positive_int(self.action_horizon, "action_horizon")
        positive(self.control_fps, "control_fps")
        if type(self.requires_task) is not bool:
            raise PolicyError("requires_task must be a boolean")

    @property
    def action_dim(self) -> int:
        return len(self.action_features)

    def to_dict(self) -> dict[str, Any]:
        return {**asdict(self), "action_dim": self.action_dim}

    @classmethod
    def from_dict(cls, document: dict[str, Any]) -> PolicyCapabilities:
        if not isinstance(document, dict):
            raise PolicyError("policy capabilities must be an object")
        data = dict(document)
        declared_dim = data.pop("action_dim", None)
        try:
            result = cls(**data)
        except TypeError as exc:
            raise PolicyError("invalid policy capability fields") from exc
        if "action_dim" in document and (type(declared_dim) is not int or declared_dim != result.action_dim):
            raise PolicyError("action_dim does not match action_features")
        return result

    @property
    def fingerprint(self) -> str:
        canonical = json.dumps(self.to_dict(), sort_keys=True, allow_nan=False, separators=(",", ":"))
        return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


@dataclass(frozen=True)
class JointLimit:
    """Position, rate, and per-waypoint limits, expressed in the feature's unit."""

    minimum: float
    maximum: float
    max_speed: float
    max_step: float

    def __post_init__(self) -> None:
        if (
            any(
                not isinstance(value, Real) or isinstance(value, bool) or not math.isfinite(value)
                for value in (self.minimum, self.maximum)
            )
            or self.minimum >= self.maximum
        ):
            raise PolicyError("joint limits require finite, increasing position bounds")
        positive(self.max_speed, "max_speed")
        positive(self.max_step, "max_step")


@dataclass(frozen=True)
class RobotCapabilities:
    robot_type: str
    state_features: tuple[str, ...]
    state_units: dict[str, str]
    images: tuple[str, ...]
    action_features: tuple[str, ...]
    action_units: dict[str, str]
    max_action_horizon: int
    max_control_fps: float
    action_representations: tuple[str, ...] = JOINT_REPRESENTATIONS

    def __post_init__(self) -> None:
        require_text(self.robot_type, "robot_type")
        for label in ("state_features", "images", "action_features", "action_representations"):
            values = names(getattr(self, label), label, allow_empty=label == "images")
            object.__setattr__(self, label, values)
        check_units(self.state_units, self.state_features, "state_units")
        check_units(self.action_units, self.action_features, "action_units")
        positive_int(self.max_action_horizon, "max_action_horizon")
        positive(self.max_control_fps, "max_control_fps")


@dataclass(frozen=True)
class InferenceRequest:
    request_id: str
    capability_fingerprint: str
    observation: RobotObservation

    def __post_init__(self) -> None:
        require_text(self.request_id, "request_id")
        if (
            not isinstance(self.capability_fingerprint, str)
            or len(self.capability_fingerprint) != 64
            or any(c not in "0123456789abcdef" for c in self.capability_fingerprint)
        ):
            raise PolicyError("invalid capability fingerprint")


@dataclass(frozen=True)
class PolicyOutput:
    request_id: str
    observation_id: str
    policy_id: str
    actions: np.ndarray
    action_names: tuple[str, ...]
    action_representation: str
    action_units: dict[str, str]
    fps: float
    absolute_features: tuple[str, ...] = ()
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for label in ("request_id", "observation_id", "policy_id", "action_representation"):
            require_text(getattr(self, label), label)
        if not isinstance(self.metadata, dict):
            raise PolicyError("action metadata must be an object")
        action_names = names(self.action_names, "action_names")
        absolute_features = names(self.absolute_features, "absolute_features", allow_empty=True)
        _check_absolute_features(action_names, absolute_features, self.action_representation)
        check_units(self.action_units, action_names, "action_units")
        positive(self.fps, "fps")
        actions = finite_array(self.actions, 2, "action chunk")
        if actions.shape[1] != len(action_names):
            raise PolicyError("action width does not match action_names")
        object.__setattr__(self, "action_names", action_names)
        object.__setattr__(self, "absolute_features", absolute_features)
        object.__setattr__(self, "actions", actions)


def _check_absolute_features(
    action_names: tuple[str, ...], absolute_features: tuple[str, ...], representation: str
) -> None:
    if set(absolute_features) - set(action_names):
        raise PolicyError("absolute_features must reference action names")
    if absolute_features and representation == ABSOLUTE:
        raise PolicyError("absolute_features is only used with relative or delta actions")
