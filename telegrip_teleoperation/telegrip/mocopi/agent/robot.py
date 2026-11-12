"""Real-follower adapter for state reads and safe trajectory execution."""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

SIDES = ("left", "right")
JOINTS = tuple(f"joint{index}" for index in range(7))


@dataclass(frozen=True)
class RobotReading:
    connected: bool
    calibrated: bool
    joints_deg: dict[str, tuple[float, ...]]
    timestamp_monotonic_ns: int
    errors: dict[str, str]


class RealRobotAdapter:
    """Wrap the existing arm-only Dual Scorpion follower.

    The adapter deliberately keeps the proven follower construction and replay
    execution from ``dataset.real``. It adds no new motor protocol, does not
    calibrate, and does not send gripper commands.
    """

    def __init__(
        self,
        *,
        left_port: str,
        right_port: str,
        robot_id: str,
        calibration_dir: str | Path | None = None,
        max_step_deg: float = 0.5,
        follower: Any | None = None,
    ):
        self.left_port = left_port
        self.right_port = right_port
        self.robot_id = robot_id
        self.calibration_dir = calibration_dir
        self.max_step_deg = float(max_step_deg)
        self.follower = follower
        self._constructed = follower is not None

    def construct(self) -> None:
        if self._constructed:
            return
        from ..dataset.real import arm_only_follower

        self.follower = arm_only_follower(
            self.left_port,
            self.right_port,
            self.robot_id,
            self.max_step_deg,
            self.calibration_dir,
        )
        self._constructed = True

    def connect(self) -> None:
        self.construct()
        for side in SIDES:
            bus = getattr(self.follower, side + "_bus")
            if not bus.is_connected:
                bus.connect()

    def disconnect(self, *, disable_torque: bool = False) -> None:
        if self.follower is None:
            return
        for side in SIDES:
            bus = getattr(self.follower, side + "_bus", None)
            if bus is not None and bus.is_connected:
                bus.disconnect(disable_torque)

    @property
    def is_connected(self) -> bool:
        if self.follower is None:
            return False
        return all(getattr(self.follower, side + "_bus").is_connected for side in SIDES)

    @property
    def is_calibrated(self) -> bool:
        if self.follower is None:
            return False
        return bool(self.follower.is_calibrated)

    def status(self) -> dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "connected": self.is_connected,
            "calibrated": self.is_calibrated,
            "left_port": self.left_port,
            "right_port": self.right_port,
            "gripper_control": "not_integrated",
            "motor_command_scope": "joint0-joint6 only",
        }

    def read(self) -> RobotReading:
        if self.follower is None or not self.is_connected:
            raise RuntimeError("real robot is not connected")
        timestamp = time.monotonic_ns()
        observation = self.follower.get_observation()
        joints: dict[str, tuple[float, ...]] = {}
        errors: dict[str, str] = {}
        for side in SIDES:
            values = []
            for joint in JOINTS:
                key = f"{side}_{joint}.pos"
                if key not in observation:
                    errors[side] = f"missing {key}"
                    break
                value = float(observation[key])
                if not np.isfinite(value):
                    errors[side] = f"non-finite {key}"
                    break
                values.append(value)
            if side in errors:
                continue
            joints[side] = tuple(values)
        if errors or set(joints) != set(SIDES):
            raise RuntimeError(f"invalid follower observation: {errors}")
        return RobotReading(
            connected=self.is_connected,
            calibrated=self.is_calibrated,
            joints_deg=joints,
            timestamp_monotonic_ns=timestamp,
            errors={},
        )

    def execute_plan(
        self,
        plan: dict[str, Any],
        robot_config: dict[str, Any],
        simulation: Any,
        report_path: str | Path,
        *,
        stop_event: Any | None = None,
    ) -> dict[str, Any]:
        from ..dataset.real import execute_plan

        self.construct()
        return execute_plan(
            plan,
            robot_config,
            simulation,
            self.follower,
            Path(report_path),
            max_start_error_deg=2.0,
            max_tracking_error_deg=5.0,
            allow_synthetic=False,
            realtime=True,
            stop_event=stop_event,
        )
