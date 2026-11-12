"""Real-follower adapter for arms, grippers, state reads, and safe execution."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import draccus
import numpy as np

from lerobot.motors import MotorCalibration, MotorNormMode

from .config import GripperSpec

SIDES = ("left", "right")
JOINTS = tuple(f"joint{index}" for index in range(7))
GRIPPER = "gripper"
ALL_MOTORS = (*JOINTS, GRIPPER)
EXPECTED_MOTOR_IDS = {joint: index + 1 for index, joint in enumerate(JOINTS)} | {GRIPPER: 8}


@dataclass(frozen=True)
class RobotReading:
    connected: bool
    calibrated: bool
    joints_deg: dict[str, tuple[float, ...]]
    timestamp_monotonic_ns: int
    errors: dict[str, str]
    gripper_positions: dict[str, float] | None = None


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
    """Validate arm and gripper calibration without changing any value."""
    errors: list[str] = []
    calibration_path = Path(path)
    if calibration_path.stem != robot_id:
        errors.append(f"calibration file ID {calibration_path.stem!r} does not match robot ID {robot_id!r}")
    details: dict[str, Any] = {}
    for side in SIDES:
        for motor in ALL_MOTORS:
            key = f"{side}_{motor}"
            calibration = document.get(key)
            if calibration is None:
                errors.append(f"missing calibration for {key}")
                continue
            if isinstance(calibration, dict):
                calibration = MotorCalibration(**calibration)
            expected_id = EXPECTED_MOTOR_IDS[motor]
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
    expected_keys = {f"{side}_{motor}" for side in SIDES for motor in ALL_MOTORS}
    unexpected = sorted(set(document) - expected_keys)
    if unexpected:
        errors.append(f"unexpected calibration entries: {unexpected}")
    return {
        "status": "valid" if not errors else "invalid",
        "path": str(calibration_path),
        "robot_id": robot_id,
        "arm_joint_count": 14,
        "gripper_entries": [f"{side}_gripper" for side in SIDES],
        "joints": details,
        "errors": errors,
    }


class RealRobotAdapter:
    """Wrap the existing Dual Scorpion follower and its calibrated grippers."""

    def __init__(
        self,
        *,
        left_port: str,
        right_port: str,
        robot_id: str,
        calibration_dir: str | Path | None = None,
        max_step_deg: float = 0.5,
        gripper: GripperSpec | None = None,
        follower: Any | None = None,
    ):
        self.left_port = left_port
        self.right_port = right_port
        self.robot_id = robot_id
        self.calibration_dir = calibration_dir
        self.max_step_deg = float(max_step_deg)
        self.gripper = gripper or GripperSpec()
        self.follower = follower
        self._constructed = follower is not None
        self._bus_lock = threading.RLock()

    def construct(self) -> None:
        if self._constructed:
            return
        from ..dataset.real import arm_and_gripper_follower

        self.follower = arm_and_gripper_follower(
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
                if set(bus.calibration) != set(ALL_MOTORS):
                    errors.append(f"{side} bus must contain exactly joint0-joint6 and gripper")
                    continue
                for motor in ALL_MOTORS:
                    key = f"{side}_{motor}"
                    expected = document.get(key)
                    actual = bus.calibration[motor]
                    if expected is None or actual != expected:
                        errors.append(f"{side} bus calibration differs from file for {motor}")
                if bus.motors[GRIPPER].norm_mode is not MotorNormMode.RANGE_0_100:
                    errors.append(f"{side} gripper must use LeRobot RANGE_0_100 normalization")
            report["errors"] = errors
            report["status"] = "valid" if not errors else "invalid"
            report["gripper_commands_integrated"] = True
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
                "gripper_commands_integrated": True,
            }

    def configuration_status(self) -> dict[str, Any]:
        calibration = self.validate_calibration()
        return {
            "configured": True,
            "robot_id": self.robot_id,
            "connected": self.is_connected,
            "calibrated": self.is_calibrated,
            "left_port": self.left_port,
            "right_port": self.right_port,
            "calibration": calibration,
            "gripper_control": "integrated_software_only_physical_execution_not_verified",
            "motor_command_scope": "joint0-joint6 and gripper",
            "auto_home_on_startup": False,
            "initial_state_source": "encoder_reading_only",
        }

    def connect(self) -> None:
        calibration = self.validate_calibration()
        if calibration["status"] != "valid":
            raise RuntimeError(f"real robot calibration is invalid: {calibration['errors']}")
        self.construct()
        with self._bus_lock:
            for side in SIDES:
                bus = getattr(self.follower, side + "_bus")
                if not bus.is_connected:
                    bus.connect()

    def disconnect(self, *, disable_torque: bool = False) -> None:
        if self.follower is None:
            return
        with self._bus_lock:
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
        if not self.is_connected:
            return all(
                set(getattr(self.follower, side + "_bus").calibration) == set(ALL_MOTORS)
                for side in SIDES
            )
        return bool(self.follower.is_calibrated)

    def status(self) -> dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "connected": self.is_connected,
            "calibrated": self.is_calibrated,
            "left_port": self.left_port,
            "right_port": self.right_port,
            "gripper_control": "integrated_software_only_physical_execution_not_verified",
            "motor_command_scope": "joint0-joint6 and gripper",
        }

    def read(self) -> RobotReading:
        if self.follower is None or not self.is_connected:
            raise RuntimeError("real robot is not connected")
        timestamp = time.monotonic_ns()
        with self._bus_lock:
            observation = self.follower.get_observation()
            joints: dict[str, tuple[float, ...]] = {}
            grippers: dict[str, float] = {}
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
                gripper_key = f"{side}_{GRIPPER}.pos"
                if gripper_key in observation:
                    gripper_value = float(observation[gripper_key])
                    if np.isfinite(gripper_value):
                        grippers[side] = gripper_value
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
                gripper_positions=grippers,
            )

    def read_gripper_state(self, side: str) -> dict[str, Any]:
        _validate_side(side)
        if self.follower is None or not self.is_connected:
            raise RuntimeError("real robot is not connected")
        bus = getattr(self.follower, side + "_bus")
        timestamp = time.monotonic_ns()
        with self._bus_lock:
            position = float(bus.sync_read("Present_Position", [GRIPPER], normalize=True)[GRIPPER])
            raw_position = float(bus.sync_read("Present_Position", [GRIPPER], normalize=False)[GRIPPER])
            load = self._optional_raw_read(bus, "Present_Load")
            current = self._optional_raw_read(bus, "Present_Current")
        if not np.isfinite(position):
            raise RuntimeError(f"{side} gripper position is non-finite")
        calibration = bus.calibration[GRIPPER]
        state = _gripper_position_state(position, self.gripper)
        return {
            "side": side,
            "timestamp_monotonic_ns": timestamp,
            "position": position,
            "raw_position": raw_position,
            "state": state,
            "load": load,
            "current": current,
            "calibration": {
                "id": calibration.id,
                "drive_mode": calibration.drive_mode,
                "homing_offset": calibration.homing_offset,
                "range_min": calibration.range_min,
                "range_max": calibration.range_max,
                "normalization": "range_0_100",
            },
            "open_position": self.gripper.open_position,
            "closed_position": self.gripper.closed_position,
            "grasp_verified": False,
            "grasp_state": "unverified",
            "verification_basis": [],
        }

    def set_gripper_position(
        self,
        side: str,
        position: float,
        *,
        stop_event: threading.Event | None = None,
    ) -> dict[str, Any]:
        return self._move_gripper(
            side,
            float(position),
            operation="set_position",
            stop_event=stop_event,
        )

    def open_gripper(self, side: str, *, stop_event: threading.Event | None = None) -> dict[str, Any]:
        return self._move_gripper(
            side,
            self.gripper.open_position,
            operation="open",
            stop_event=stop_event,
        )

    def close_gripper(self, side: str, *, stop_event: threading.Event | None = None) -> dict[str, Any]:
        return self._move_gripper(
            side,
            self.gripper.closed_position,
            operation="close",
            stop_event=stop_event,
        )

    def release_gripper(self, side: str, *, stop_event: threading.Event | None = None) -> dict[str, Any]:
        return self.open_gripper(side, stop_event=stop_event)

    def hold_gripper(self, side: str, *, stop_event: threading.Event | None = None) -> dict[str, Any]:
        state = self.read_gripper_state(side)
        return self._move_gripper(
            side,
            state["position"],
            operation="hold",
            stop_event=stop_event,
            max_step=self.gripper.max_position - self.gripper.min_position,
        )

    def grasp(self, side: str, *, stop_event: threading.Event | None = None) -> dict[str, Any]:
        result = self._move_gripper(
            side,
            self.gripper.closed_position,
            operation="grasp",
            stop_event=stop_event,
            monitor_stall=True,
        )
        thresholds_met = (
            self.gripper.load_threshold is not None
            and self.gripper.current_threshold is not None
            and result.get("load") is not None
            and result.get("current") is not None
            and result["load"] >= self.gripper.load_threshold
            and result["current"] >= self.gripper.current_threshold
        )
        position_stalled = not result["position_reached"] and result["position_error"] > self.gripper.position_tolerance
        if thresholds_met and position_stalled:
            result["grasp_verified"] = True
            result["grasp_state"] = "threshold_verified"
            result["verification_basis"] = [
                "configured_present_load_threshold",
                "configured_present_current_threshold",
                "encoder_position_stall",
            ]
        else:
            result["grasp_verified"] = False
            result["grasp_state"] = "unverified"
            result["verification_basis"] = ["encoder_position_only"]
        return result

    def execute_plan(
        self,
        plan: dict[str, Any],
        robot_config: dict[str, Any],
        simulation: Any,
        report_path: str | Path,
        *,
        stop_event: threading.Event | None = None,
    ) -> dict[str, Any]:
        from ..dataset.real import execute_plan

        self.construct()
        with self._bus_lock:
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

    def _move_gripper(
        self,
        side: str,
        target: float,
        *,
        operation: str,
        stop_event: threading.Event | None = None,
        max_step: float | None = None,
        monitor_stall: bool = False,
    ) -> dict[str, Any]:
        _validate_side(side)
        if not self.gripper.min_position <= target <= self.gripper.max_position:
            raise ValueError(
                f"{side} gripper target {target} is outside "
                f"[{self.gripper.min_position}, {self.gripper.max_position}]"
            )
        if self.follower is None or not self.is_connected:
            raise RuntimeError("real robot is not connected")
        bus = getattr(self.follower, side + "_bus")
        step = self.gripper.max_step if max_step is None else float(max_step)
        if step <= 0:
            raise ValueError("gripper step must be positive")
        started = time.monotonic_ns()
        commands: list[float] = []
        stopped = False
        with self._bus_lock:
            current = float(bus.sync_read("Present_Position", [GRIPPER], normalize=True)[GRIPPER])
            # Overwrite any retained Goal_Position before torque is enabled. This
            # prevents a stale goal from yanking the gripper when torque starts.
            bus.sync_write("Goal_Position", {GRIPPER: current}, normalize=True)
            bus.enable_torque([GRIPPER])
            hold_measured = float(bus.sync_read("Present_Position", [GRIPPER], normalize=True)[GRIPPER])
            hold_step = abs(hold_measured - current)
            initial_hold = {
                "commanded_position": current,
                "measured_position": hold_measured,
                "position_step": hold_step,
                "within_tolerance": hold_step <= self.gripper.position_tolerance,
            }
            if hold_step > self.gripper.position_tolerance:
                raise RuntimeError(
                    f"{side} gripper moved while enabling the initial position hold: {initial_hold}"
                )
            direction = 1.0 if target >= current else -1.0
            while abs(target - current) > self.gripper.position_tolerance:
                if stop_event is not None and stop_event.is_set():
                    stopped = True
                    break
                if (time.monotonic_ns() - started) / 1e9 > self.gripper.grasp_timeout_s:
                    stopped = True
                    break
                next_position = float(
                    np.clip(
                        current + direction * min(step, abs(target - current)),
                        self.gripper.min_position,
                        self.gripper.max_position,
                    )
                )
                bus.sync_write("Goal_Position", {GRIPPER: next_position}, normalize=True)
                commands.append(next_position)
                time.sleep(self.gripper.settle_s)
                current = float(bus.sync_read("Present_Position", [GRIPPER], normalize=True)[GRIPPER])
                if monitor_stall:
                    load = self._optional_raw_read(bus, "Present_Load")
                    current_raw = self._optional_raw_read(bus, "Present_Current")
                    if (
                        self.gripper.load_threshold is not None
                        and self.gripper.current_threshold is not None
                        and load is not None
                        and current_raw is not None
                        and load >= self.gripper.load_threshold
                        and current_raw >= self.gripper.current_threshold
                    ):
                        break
            if stopped or monitor_stall:
                held = float(bus.sync_read("Present_Position", [GRIPPER], normalize=True)[GRIPPER])
                bus.sync_write("Goal_Position", {GRIPPER: held}, normalize=True)
            final = float(bus.sync_read("Present_Position", [GRIPPER], normalize=True)[GRIPPER])
            raw_final = float(bus.sync_read("Present_Position", [GRIPPER], normalize=False)[GRIPPER])
            load = self._optional_raw_read(bus, "Present_Load")
            current_reading = self._optional_raw_read(bus, "Present_Current")
        position_error = abs(final - target)
        return {
            "side": side,
            "operation": operation,
            "initial_hold": initial_hold,
            "status": "complete" if not stopped else "stopped",
            "target_position": target,
            "final_position": final,
            "raw_final_position": raw_final,
            "position_error": position_error,
            "position_reached": position_error <= self.gripper.position_tolerance,
            "stopped": stopped,
            "commands": commands,
            "load": load,
            "current": current_reading,
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "elapsed_s": (time.monotonic_ns() - started) / 1e9,
            "grasp_verified": False,
            "grasp_state": "unverified",
            "verification_basis": ["encoder_position_only"],
        }

    @staticmethod
    def _optional_raw_read(bus: Any, register: str) -> float | None:
        try:
            value = bus.sync_read(register, [GRIPPER], normalize=False)[GRIPPER]
            result = float(value)
            return result if np.isfinite(result) else None
        except Exception:
            return None


def _validate_side(side: str) -> None:
    if side not in SIDES:
        raise ValueError("side must be left or right")


def _gripper_position_state(position: float, spec: GripperSpec) -> str:
    if abs(position - spec.open_position) <= spec.position_tolerance:
        return "open"
    if abs(position - spec.closed_position) <= spec.position_tolerance:
        return "closed"
    return "intermediate"
