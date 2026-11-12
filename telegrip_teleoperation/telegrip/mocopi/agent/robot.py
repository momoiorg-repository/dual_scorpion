"""Real-follower adapter for state reads and safe trajectory execution."""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import draccus
import numpy as np

from lerobot.motors import MotorCalibration

SIDES = ("left", "right")
JOINTS = tuple(f"joint{index}" for index in range(7))
EXPECTED_MOTOR_IDS = {joint: index + 1 for index, joint in enumerate(JOINTS)}


@dataclass(frozen=True)
class RobotReading:
    connected: bool
    calibrated: bool
    joints_deg: dict[str, tuple[float, ...]]
    timestamp_monotonic_ns: int
    errors: dict[str, str]


def load_calibration_document(path: str | Path) -> dict[str, MotorCalibration]:
    """Load a LeRobot MotorCalibration mapping using the standard JSON decoder."""
    calibration_path = Path(path)
    with calibration_path.open(encoding="utf-8") as file, draccus.config_type("json"):
        document = draccus.load(dict[str, MotorCalibration], file)
    if not isinstance(document, dict):
        raise ValueError("calibration JSON root must be a MotorCalibration mapping")
    return document


def validate_calibration_document(
    document: dict[str, MotorCalibration],
    *,
    robot_id: str,
    path: str | Path,
) -> dict[str, Any]:
    """Validate IDs, joint correspondence, direction flags, and offsets.

    This does not infer a correct direction or offset from geometry. It verifies
    that the existing LeRobot calibration is complete, internally consistent,
    and attached to the selected robot ID.
    """
    errors: list[str] = []
    calibration_path = Path(path)
    if calibration_path.stem != robot_id:
        errors.append(f"calibration file ID {calibration_path.stem!r} does not match robot ID {robot_id!r}")
    details: dict[str, Any] = {}
    for side in SIDES:
        for joint in JOINTS:
            key = f"{side}_{joint}"
            calibration = document.get(key)
            if calibration is None:
                errors.append(f"missing calibration for {key}")
                continue
            if isinstance(calibration, dict):
                calibration = MotorCalibration(**calibration)
            expected_id = EXPECTED_MOTOR_IDS[joint]
            if calibration.id != expected_id:
                errors.append(f"{key} motor ID {calibration.id} does not match expected {expected_id}")
            if calibration.drive_mode not in (0, 1):
                errors.append(f"{key} drive_mode must be 0 or 1")
            if not isinstance(calibration.homing_offset, int):
                errors.append(f"{key} homing_offset must be an integer")
            if not isinstance(calibration.range_min, int) or not isinstance(calibration.range_max, int):
                errors.append(f"{key} calibrated ranges must be integers")
            elif not 0 <= calibration.range_min < calibration.range_max <= 4095:
                errors.append(f"{key} calibrated range is invalid")
            details[key] = {
                "id": calibration.id,
                "drive_mode": calibration.drive_mode,
                "homing_offset": calibration.homing_offset,
                "range_min": calibration.range_min,
                "range_max": calibration.range_max,
            }
    expected_arm_keys = {f"{side}_{joint}" for side in SIDES for joint in JOINTS}
    unexpected = sorted(set(document) - expected_arm_keys - {f"{side}_gripper" for side in SIDES})
    if unexpected:
        errors.append(f"unexpected calibration entries: {unexpected}")
    return {
        "status": "valid" if not errors else "invalid",
        "path": str(calibration_path),
        "robot_id": robot_id,
        "arm_joint_count": len(expected_arm_keys.intersection(document)),
        "gripper_entries": sorted(
            key for key in document if key.endswith("_gripper")
        ),
        "joints": details,
        "errors": errors,
    }


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

    def validate_calibration(self) -> dict[str, Any]:
        try:
            self.construct()
            if self.follower is None:
                raise RuntimeError("follower construction failed")
            calibration_path = Path(self.follower.calibration_fpath)
            if not calibration_path.is_file():
                raise FileNotFoundError(f"calibration file not found: {calibration_path}")
            document = load_calibration_document(calibration_path)
            report = validate_calibration_document(
                document,
                robot_id=self.robot_id,
                path=calibration_path,
            )
            errors = list(report["errors"])
            for side in SIDES:
                bus = getattr(self.follower, side + "_bus")
                if set(bus.calibration) != set(JOINTS):
                    errors.append(f"{side} bus must contain exactly joint0-joint6")
                    continue
                for joint in JOINTS:
                    key = f"{side}_{joint}"
                    expected = document.get(key)
                    actual = bus.calibration[joint]
                    if expected is None or actual != expected:
                        errors.append(f"{side} bus calibration differs from file for {joint}")
            report["errors"] = errors
            report["status"] = "valid" if not errors else "invalid"
            report["gripper_commands_integrated"] = False
            return report
        except Exception as exc:
            return {
                "status": "invalid",
                "path": None,
                "robot_id": self.robot_id,
                "arm_joint_count": 0,
                "gripper_entries": [],
                "joints": {},
                "errors": [str(exc)],
                "gripper_commands_integrated": False,
            }

    def configuration_status(self) -> dict[str, Any]:
        return {
            "configured": True,
            "robot_id": self.robot_id,
            "connected": self.is_connected,
            "calibrated": self.is_calibrated,
            "left_port": self.left_port,
            "right_port": self.right_port,
            "calibration": self.validate_calibration(),
            "gripper_control": "not_integrated",
            "motor_command_scope": "joint0-joint6 only",
            "auto_home_on_startup": False,
            "initial_state_source": "encoder_reading_only",
        }

    def connect(self) -> None:
        calibration = self.validate_calibration()
        if calibration["status"] != "valid":
            raise RuntimeError(f"real robot calibration is invalid: {calibration['errors']}")
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
