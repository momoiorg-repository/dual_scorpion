"""Deterministic adapters for exercising the full stack without hardware or ML."""

import time

import numpy as np

from ..core import ActionValidation, check_feedback_age, resolve_joint_chunk
from ..schema import (
    ABSOLUTE,
    JointLimit,
    PolicyCapabilities,
    PolicyError,
    PolicyOutput,
    RobotCapabilities,
    RobotObservation,
)
from .common import state_vector


class MockRobot:
    def __init__(
        self,
        *,
        robot_type: str = "mock_arm",
        robot_id: str = "mock",
        state_features: tuple[str, ...] = ("joint", "gripper"),
        state_units: dict[str, str] | None = None,
        action_features: tuple[str, ...] | None = None,
        images: tuple[str, ...] = ("front",),
        max_action_horizon: int = 100,
        max_control_fps: float = 30,
    ):
        state_units = (
            {name: "deg" if name != "gripper" else "lerobot_0_100" for name in state_features}
            if state_units is None
            else state_units
        )
        action_features = tuple(state_features) if action_features is None else tuple(action_features)
        self._capabilities = RobotCapabilities(
            robot_type,
            state_features,
            state_units,
            images,
            action_features,
            {name: state_units[name] for name in action_features},
            max_action_horizon,
            max_control_fps,
        )
        self._robot_id = robot_id
        self._state = np.array(
            [50 if state_units[name] == "lerobot_0_100" else 0 for name in state_features], dtype=np.float32
        )
        self._limits = {
            name: JointLimit(0, 100, 60, 3) if unit == "lerobot_0_100" else JointLimit(-180, 180, 30, 1)
            for name, unit in self._capabilities.action_units.items()
        }
        self._live: RobotObservation | None = None
        self.executed_chunks: list[np.ndarray] = []
        self.stopped = False

    def get_capabilities(self) -> RobotCapabilities:
        return self._capabilities

    def get_observation(self, task: str | None = None) -> RobotObservation:
        caps = self._capabilities
        timestamp = time.monotonic_ns()
        images = {
            name: np.full((8 + index, 12, 3), 25 * (index % 10), dtype=np.uint8)
            for index, name in enumerate(caps.images)
        }
        self._live = RobotObservation(
            self._robot_id,
            caps.robot_type,
            timestamp,
            self._state,
            caps.state_features,
            caps.state_units,
            images,
            dict.fromkeys(images, timestamp),
            task=task,
            metadata={"source": "mock"},
        )
        return self._live

    def validate_action(self, output: PolicyOutput, observation: RobotObservation) -> ActionValidation:
        positions = resolve_joint_chunk(output, observation, self._capabilities, self._limits)
        return ActionValidation(len(positions), {"joint_and_gripper_limits": "passed", "mock": True})

    def execute_action(self, output: PolicyOutput, observation: RobotObservation) -> None:
        if self.stopped or observation is not self._live:
            raise PolicyError("mock execution requires current, unconsumed feedback and a clear stop latch")
        check_feedback_age(observation, 1.0)
        positions = resolve_joint_chunk(output, observation, self._capabilities, self._limits)
        self.executed_chunks.append(positions)
        for index, action in enumerate(self._capabilities.action_features):
            self._state[self._capabilities.state_features.index(action)] = positions[-1, index]
        self._live = None

    def stop(self) -> None:
        self.stopped = True

    def close(self) -> None:
        self._live = None

    def doctor(self) -> dict:
        return {"robot_id": self._robot_id, "mock": True, "hardware_connected": False}


class MockPolicy:
    """Return a hold-position chunk in any of the supported representations."""

    def __init__(self, capabilities: PolicyCapabilities):
        self._capabilities = capabilities
        for name, unit in capabilities.action_units.items():
            if capabilities.state_units.get(name) != unit:
                raise PolicyError("mock policy needs matching observation features for each action")

    def load(self, checkpoint: str | None) -> None:
        if checkpoint is not None:
            raise PolicyError("mock policy does not load a checkpoint")

    def get_capabilities(self) -> PolicyCapabilities:
        return self._capabilities

    def preprocess(self, observation: RobotObservation) -> np.ndarray:
        return state_vector(observation, self._capabilities.action_features)

    def predict(self, inputs: np.ndarray) -> np.ndarray:
        caps = self._capabilities
        if caps.action_representation != ABSOLUTE:
            inputs = np.where([name in caps.absolute_features for name in caps.action_features], inputs, 0)
        return np.repeat(inputs[None], caps.action_horizon, axis=0)

    def postprocess(self, prediction: np.ndarray) -> np.ndarray:
        return prediction
