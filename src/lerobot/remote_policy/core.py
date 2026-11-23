"""Negotiation, inference, and physical joint chunk validation."""

from __future__ import annotations

import contextlib
import time
from collections.abc import Callable
from dataclasses import dataclass
from threading import Lock
from typing import Any, Protocol
from uuid import uuid4

import numpy as np

from .schema import (
    DELTA,
    JOINT_REPRESENTATIONS,
    PROTOCOL_VERSION,
    RELATIVE,
    InferenceRequest,
    JointLimit,
    PolicyCapabilities,
    PolicyError,
    PolicyOutput,
    RobotCapabilities,
    RobotObservation,
    StaleResponseError,
    positive,
)


@dataclass(frozen=True)
class ActionValidation:
    """Dry-run validation result from the robot's local safety adapter."""

    frames: int
    checks: dict[str, Any]
    blockers: tuple[str, ...] = ()


class RobotAdapter(Protocol):
    def get_capabilities(self) -> RobotCapabilities: ...
    def get_observation(self, task: str | None = None) -> RobotObservation: ...
    def validate_action(self, output: PolicyOutput, observation: RobotObservation) -> ActionValidation: ...
    def execute_action(self, output: PolicyOutput, observation: RobotObservation) -> None: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...


class PolicyAdapter(Protocol):
    def load(self, checkpoint: str | None) -> None: ...
    def get_capabilities(self) -> PolicyCapabilities: ...
    def preprocess(self, observation: RobotObservation) -> Any: ...
    def predict(self, inputs: Any) -> Any: ...
    def postprocess(self, prediction: Any) -> np.ndarray: ...


class TransportAdapter(Protocol):
    def get_capabilities(self, *, timeout: float) -> PolicyCapabilities: ...
    def infer(self, request: InferenceRequest, *, timeout: float) -> PolicyOutput: ...
    def close(self) -> None: ...


def _require_representation(representation: str) -> None:
    if representation not in JOINT_REPRESENTATIONS:
        raise PolicyError(f"Unsupported action representation: {representation}")


def _check_policy(policy: PolicyCapabilities) -> None:
    if policy.protocol_version != PROTOCOL_VERSION:
        raise PolicyError(f"Unsupported policy protocol version: {policy.protocol_version}")
    _require_representation(policy.action_representation)


def _check_features(required: dict[str, str], available: dict[str, str], label: str) -> None:
    missing = required.keys() - available.keys()
    if missing:
        raise PolicyError(f"missing {label} features: {sorted(missing)}")
    incompatible = [name for name, unit in required.items() if available[name] != unit]
    if incompatible:
        raise PolicyError(f"{label} units mismatch: {sorted(incompatible)}")


def _check_images(required: tuple[str, ...], available: Any) -> None:
    missing = set(required) - set(available)
    if missing:
        raise PolicyError(f"missing required cameras: {sorted(missing)}")


def negotiate(robot: RobotCapabilities, policy: PolicyCapabilities) -> None:
    """Reject incompatible contracts before inference or connection to hardware."""
    _check_policy(policy)
    if robot.robot_type not in policy.robot_types:
        raise PolicyError(f"policy {policy.policy_id!r} does not support robot type {robot.robot_type!r}")
    _check_images(policy.required_images, robot.images)
    _check_features(policy.state_units, robot.state_units, "state")
    _check_features(robot.action_units, policy.action_units, "action")
    if robot.action_units.keys() != policy.action_units.keys():
        raise PolicyError("policy action names must exactly cover the robot's action features")
    if policy.action_representation not in robot.action_representations:
        raise PolicyError(f"Unsupported action representation for this robot: {policy.action_representation}")
    if policy.action_horizon > robot.max_action_horizon:
        raise PolicyError("policy action horizon exceeds robot limit")
    if policy.control_fps > robot.max_control_fps:
        raise PolicyError("policy control fps exceeds robot limit")


def check_observation(observation: RobotObservation, policy: PolicyCapabilities) -> None:
    if observation.robot_type not in policy.robot_types:
        raise PolicyError(
            f"policy {policy.policy_id!r} does not support robot type {observation.robot_type!r}"
        )
    _check_features(policy.state_units, observation.state_units, "state")
    _check_images(policy.required_images, observation.images)
    if policy.requires_task and observation.task is None:
        raise PolicyError("policy requires a task instruction")


def check_feedback_age(
    observation: RobotObservation, max_age_s: float, *, cameras: tuple[str, ...] = ()
) -> None:
    if observation.clock != "monotonic":
        raise StaleResponseError("live feedback requires the local monotonic clock")
    now_ns = time.monotonic_ns()
    timestamps = (observation.timestamp_ns, *(observation.image_timestamps_ns[key] for key in cameras))
    if min(timestamps) < now_ns - max_age_s * 1e9 or max(timestamps) > now_ns:
        raise StaleResponseError("robot feedback or camera frame is stale or has an invalid timestamp")


def _check_response(output: PolicyOutput, request: InferenceRequest, policy: PolicyCapabilities) -> None:
    if output.request_id != request.request_id:
        raise PolicyError("response request ID mismatch")
    if output.observation_id != request.observation.observation_id:
        raise PolicyError("response observation ID mismatch")
    if output.policy_id != policy.policy_id:
        raise PolicyError("response policy ID mismatch")
    if output.actions.shape != (policy.action_horizon, policy.action_dim):
        raise PolicyError("action chunk shape does not match negotiated horizon and width")
    if (
        output.action_units != policy.action_units
        or output.action_representation != policy.action_representation
        or output.fps != policy.control_fps
        or set(output.absolute_features) != set(policy.absolute_features)
    ):
        raise PolicyError("response action metadata does not match negotiated capabilities")


def resolve_joint_chunk(
    output: PolicyOutput,
    observation: RobotObservation,
    robot: RobotCapabilities,
    limits: dict[str, JointLimit],
) -> np.ndarray:
    """Convert to robot order and physical absolute positions, without clipping."""
    _require_representation(output.action_representation)
    if output.action_representation not in robot.action_representations:
        raise PolicyError(f"Unsupported action representation for this robot: {output.action_representation}")
    if output.observation_id != observation.observation_id or observation.robot_type != robot.robot_type:
        raise PolicyError("action chunk does not match the reference observation and robot type")
    if output.actions.shape[0] > robot.max_action_horizon or output.fps > robot.max_control_fps:
        raise PolicyError("action chunk exceeds the robot horizon or fps limit")
    if output.action_units != robot.action_units:
        raise PolicyError("action names or units do not match the robot")
    _check_features(robot.action_units, observation.state_units, "reference state")
    if limits.keys() != robot.action_units.keys():
        raise PolicyError("position and velocity limits are required for every action feature")

    order = robot.action_features
    reference = observation.state[[observation.state_names.index(name) for name in order]]
    positions = output.actions[:, [output.action_names.index(name) for name in order]].astype(np.float64)
    relative = [index for index, name in enumerate(order) if name not in output.absolute_features]
    if output.action_representation == RELATIVE:
        positions[:, relative] += reference[relative]
    elif output.action_representation == DELTA:
        positions[:, relative] = reference[relative] + np.cumsum(positions[:, relative], axis=0)
    trajectory = np.vstack((reference, positions))
    steps = np.abs(np.diff(trajectory, axis=0))
    for index, name in enumerate(order):
        limit = limits[name]
        if np.any(trajectory[:, index] < limit.minimum) or np.any(trajectory[:, index] > limit.maximum):
            raise PolicyError(f"{name}: position is outside the allowed range")
        if np.any(steps[:, index] > limit.max_step):
            raise PolicyError(f"{name}: action discontinuity exceeds the per-waypoint limit")
        if np.any(steps[:, index] > limit.max_speed / output.fps):
            raise PolicyError(f"{name}: action exceeds the velocity limit")
    return positions


class InferenceBackend:
    """Use one loaded adapter per worker. A processor may cache request state."""

    def __init__(self, policy: PolicyAdapter):
        self._policy = policy
        self._capabilities = policy.get_capabilities()
        _check_policy(self._capabilities)
        self._lock = Lock()

    def get_capabilities(self) -> PolicyCapabilities:
        return self._capabilities

    def infer(
        self, request: InferenceRequest, *, is_active: Callable[[], bool] | None = None
    ) -> PolicyOutput:
        caps = self._capabilities
        if request.capability_fingerprint != caps.fingerprint:
            raise PolicyError("backend capabilities changed or do not match the request")
        check_observation(request.observation, caps)
        with self._lock:
            if is_active is not None and not is_active():
                raise TimeoutError("inference request expired before prediction")
            inputs = self._policy.preprocess(request.observation)
            actions = self._policy.postprocess(self._policy.predict(inputs))
            output = PolicyOutput(
                request_id=request.request_id,
                observation_id=request.observation.observation_id,
                policy_id=caps.policy_id,
                actions=actions,
                action_names=caps.action_features,
                action_representation=caps.action_representation,
                action_units=caps.action_units,
                fps=caps.control_fps,
                absolute_features=caps.absolute_features,
            )
        _check_response(output, request, caps)
        return output


class LocalTransport:
    """Direct calls for local GPU inference and offline file exchange."""

    def __init__(self, backend: InferenceBackend):
        self.backend = backend

    def get_capabilities(self, *, timeout: float) -> PolicyCapabilities:
        return self.backend.get_capabilities()

    def infer(self, request: InferenceRequest, *, timeout: float) -> PolicyOutput:
        deadline = time.monotonic() + timeout
        output = self.backend.infer(request, is_active=lambda: time.monotonic() < deadline)
        if time.monotonic() >= deadline:
            raise TimeoutError("local inference exceeded its deadline")
        return output

    def close(self) -> None:
        pass


class PolicyClient:
    def __init__(
        self,
        transport: TransportAdapter,
        *,
        policy_id: str | None = None,
        timeout_s: float = 30.0,
        max_response_age_s: float = 30.0,
    ):
        positive(timeout_s, "timeout_s")
        positive(max_response_age_s, "max_response_age_s")
        self.transport = transport
        self.policy_id = policy_id
        self.timeout_s = timeout_s
        self.max_response_age_s = max_response_age_s

    def capabilities(self, robot: RobotCapabilities | None = None) -> PolicyCapabilities:
        policy = self.transport.get_capabilities(timeout=self.timeout_s)
        _check_policy(policy)
        if self.policy_id is not None and self.policy_id != policy.policy_id:
            raise PolicyError(f"expected policy {self.policy_id!r}, received {policy.policy_id!r}")
        if robot is not None:
            negotiate(robot, policy)
        return policy

    def predict(
        self, observation: RobotObservation, *, robot: RobotCapabilities | None = None
    ) -> PolicyOutput:
        return self._predict(observation, self.capabilities(robot))

    def run(
        self, robot: RobotAdapter, *, task: str | None = None, dry_run: bool = True
    ) -> tuple[PolicyOutput, ActionValidation]:
        """Capture and validate one live chunk using a single negotiation."""
        if type(dry_run) is not bool:
            raise PolicyError("dry_run must be a boolean")
        policy = self.capabilities(robot.get_capabilities())
        observation = robot.get_observation(task)
        output = self._predict(observation, policy, live=True)
        validation = robot.validate_action(output, observation)
        if not dry_run:
            if validation.blockers:
                raise PolicyError("execution blocked: " + "; ".join(validation.blockers))
            check_feedback_age(observation, self.max_response_age_s, cameras=policy.required_images)
            try:
                robot.execute_action(output, observation)
            except BaseException:
                with contextlib.suppress(Exception):
                    robot.stop()
                raise
        return output, validation

    def _predict(
        self, observation: RobotObservation, policy: PolicyCapabilities, *, live: bool = False
    ) -> PolicyOutput:
        check_observation(observation, policy)
        if live:
            check_feedback_age(observation, self.max_response_age_s, cameras=policy.required_images)
        request = InferenceRequest(uuid4().hex, policy.fingerprint, observation)
        started = time.monotonic()
        output = self.transport.infer(request, timeout=min(self.timeout_s, self.max_response_age_s))
        elapsed = time.monotonic() - started
        if elapsed > self.max_response_age_s:
            raise StaleResponseError("policy response exceeded the local age budget")
        if elapsed > self.timeout_s:
            raise TimeoutError("policy response exceeded the inference deadline")
        if live:
            check_feedback_age(observation, self.max_response_age_s, cameras=policy.required_images)
        _check_response(output, request, policy)
        return output
