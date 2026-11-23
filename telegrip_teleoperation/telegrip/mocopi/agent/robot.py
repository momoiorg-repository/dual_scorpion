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
        if thresholds_met and position_stalled and result["motion_observed"]:
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

    def stop_gripper(self, side: str) -> dict[str, Any]:
        _validate_side(side)
        if self.follower is None or not self.is_connected:
            return {"side": side, "status": "not_connected", "torque_disabled": False}
        with self._bus_lock:
            report = self._safe_stop_gripper(getattr(self.follower, side + "_bus"))
        return {"side": side, **report}

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
        motion_commands: list[float] = []
        position_samples: list[dict[str, Any]] = []
        communication_errors: list[str] = []
        initial_position: float | None = None
        final: float | None = None
        raw_final: float | None = None
        load: float | None = None
        current_reading: float | None = None
        initial_hold: dict[str, Any] = {}
        safe_stop: dict[str, Any] | None = None
        diagnostics: dict[str, Any] = {}
        motion_onset_s: float | None = None
        stopped = False
        stop_reason = "pending"
        status = "pending"
        error: str | None = None

        def required_read(register: str, *, normalize: bool = False) -> float:
            return float(bus.sync_read(register, [GRIPPER], normalize=normalize)[GRIPPER])

        def optional_read(register: str) -> float | None:
            try:
                return self._optional_raw_read(bus, register)
            except Exception as exc:
                communication_errors.append(f"{register}: {exc}")
                return None

        try:
            with self._bus_lock:
                current = required_read("Present_Position", normalize=True)
                initial_position = current
                diagnostics = {
                    "operating_mode": required_read("Operating_Mode"),
                    "initial_torque_enable": required_read("Torque_Enable"),
                    "initial_goal_position": required_read("Goal_Position", normalize=True),
                    "initial_voltage": optional_read("Present_Voltage"),
                    "initial_temperature": optional_read("Present_Temperature"),
                    "initial_motor_status": optional_read("Status"),
                }
                if diagnostics["operating_mode"] != 0:
                    status = "invalid_operating_mode"
                    stop_reason = "invalid_operating_mode"
                    safe_stop = self._safe_stop_gripper(bus)
                    final = required_read("Present_Position", normalize=True)
                    return self._gripper_result(
                        side=side,
                        operation=operation,
                        status=status,
                        stop_reason=stop_reason,
                        target=target,
                        initial_position=initial_position,
                        final=final,
                        raw_final=raw_final,
                        commands=commands,
                        motion_commands=motion_commands,
                        position_samples=position_samples,
                        diagnostics=diagnostics,
                        communication_errors=communication_errors,
                        safe_stop=safe_stop,
                        motion_onset_s=motion_onset_s,
                        started=started,
                        load=load,
                        current=current_reading,
                        error=f"Operating_Mode={diagnostics['operating_mode']}; expected position mode 0",
                    )

                # Overwrite a retained goal before enabling torque. This prevents
                # a stale Goal_Position from moving the gripper when torque starts.
                bus.sync_write("Goal_Position", {GRIPPER: current}, normalize=True)
                commands.append(current)
                bus.enable_torque([GRIPPER])
                hold_measured = required_read("Present_Position", normalize=True)
                hold_goal_readback = required_read("Goal_Position", normalize=True)
                hold_torque = required_read("Torque_Enable")
                hold_step = abs(hold_measured - current)
                initial_hold = {
                    "commanded_position": current,
                    "measured_position": hold_measured,
                    "goal_readback": hold_goal_readback,
                    "torque_enable": hold_torque,
                    "position_step": hold_step,
                    "within_tolerance": hold_step <= self.gripper.position_tolerance,
                    "goal_readback_match": abs(hold_goal_readback - current) <= 0.1,
                    "torque_enabled": hold_torque == 1,
                }
                diagnostics.update(
                    {
                        "initial_hold": initial_hold,
                        "voltage": optional_read("Present_Voltage"),
                        "temperature": optional_read("Present_Temperature"),
                        "motor_status": optional_read("Status"),
                    }
                )
                if not initial_hold["within_tolerance"]:
                    status = "initial_hold_motion"
                    stop_reason = "initial_hold_motion"
                    safe_stop = self._safe_stop_gripper(bus)
                    final = required_read("Present_Position", normalize=True)
                    return self._gripper_result(
                        side=side,
                        operation=operation,
                        status=status,
                        stop_reason=stop_reason,
                        target=target,
                        initial_position=initial_position,
                        final=final,
                        raw_final=raw_final,
                        commands=commands,
                        motion_commands=motion_commands,
                        position_samples=position_samples,
                        diagnostics=diagnostics,
                        communication_errors=communication_errors,
                        safe_stop=safe_stop,
                        motion_onset_s=motion_onset_s,
                        started=started,
                        load=load,
                        current=current_reading,
                        error="gripper moved while enabling the initial position hold",
                    )
                if not initial_hold["torque_enabled"] or not initial_hold["goal_readback_match"]:
                    status = "initial_hold_invalid"
                    stop_reason = "initial_hold_invalid"
                    safe_stop = self._safe_stop_gripper(bus)
                    final = required_read("Present_Position", normalize=True)
                    return self._gripper_result(
                        side=side,
                        operation=operation,
                        status=status,
                        stop_reason=stop_reason,
                        target=target,
                        initial_position=initial_position,
                        final=final,
                        raw_final=raw_final,
                        commands=commands,
                        motion_commands=motion_commands,
                        position_samples=position_samples,
                        diagnostics=diagnostics,
                        communication_errors=communication_errors,
                        safe_stop=safe_stop,
                        motion_onset_s=motion_onset_s,
                        started=started,
                        load=load,
                        current=current_reading,
                        error=f"invalid initial hold state: {initial_hold}",
                    )

                direction = 1.0 if target >= current else -1.0
                motion_required = abs(target - initial_position) > 1e-9
                if not motion_required:
                    status = "complete" if operation == "hold" else "no_motion_required"
                    stop_reason = "no_motion_required"
                    final = current
                    return self._gripper_result(
                        side=side,
                        operation=operation,
                        status=status,
                        stop_reason=stop_reason,
                        target=target,
                        initial_position=initial_position,
                        final=final,
                        raw_final=raw_final,
                        commands=commands,
                        motion_commands=motion_commands,
                        position_samples=position_samples,
                        diagnostics=diagnostics,
                        communication_errors=communication_errors,
                        safe_stop=safe_stop,
                        motion_onset_s=motion_onset_s,
                        started=started,
                        load=load,
                        current=current_reading,
                    )

                if stop_event is not None and stop_event.is_set():
                    stopped = True
                    stop_reason = "operator_stop"
                    status = "stopped"
                    safe_stop = self._safe_stop_gripper(bus)
                    final = required_read("Present_Position", normalize=True)
                    return self._gripper_result(
                        side=side,
                        operation=operation,
                        status=status,
                        stop_reason=stop_reason,
                        target=target,
                        initial_position=initial_position,
                        final=final,
                        raw_final=raw_final,
                        commands=commands,
                        motion_commands=motion_commands,
                        position_samples=position_samples,
                        diagnostics=diagnostics,
                        communication_errors=communication_errors,
                        safe_stop=safe_stop,
                        motion_onset_s=motion_onset_s,
                        started=started,
                        load=load,
                        current=current_reading,
                    )

                # Command even when the requested change is smaller than
                # position_tolerance. The tolerance only decides whether the
                # final position is acceptable.
                next_position = float(
                    np.clip(
                        current + direction * min(step, abs(target - current)),
                        self.gripper.min_position,
                        self.gripper.max_position,
                    )
                )
                bus.sync_write("Goal_Position", {GRIPPER: next_position}, normalize=True)
                commands.append(next_position)
                motion_commands.append(next_position)
                goal_readback = required_read("Goal_Position", normalize=True)
                torque_readback = required_read("Torque_Enable")
                diagnostics.update(
                    {
                        "commanded_goal_position": next_position,
                        "goal_readback": goal_readback,
                        "goal_readback_error": abs(goal_readback - next_position),
                        "torque_enable": torque_readback,
                    }
                )
                if abs(goal_readback - next_position) > 0.1 or torque_readback != 1:
                    status = "goal_write_invalid"
                    stop_reason = "goal_write_invalid"
                    safe_stop = self._safe_stop_gripper(bus)
                    final = required_read("Present_Position", normalize=True)
                    return self._gripper_result(
                        side=side,
                        operation=operation,
                        status=status,
                        stop_reason=stop_reason,
                        target=target,
                        initial_position=initial_position,
                        final=final,
                        raw_final=raw_final,
                        commands=commands,
                        motion_commands=motion_commands,
                        position_samples=position_samples,
                        diagnostics=diagnostics,
                        communication_errors=communication_errors,
                        safe_stop=safe_stop,
                        motion_onset_s=motion_onset_s,
                        started=started,
                        load=load,
                        current=current_reading,
                        error=f"goal={goal_readback}, torque={torque_readback}",
                    )

                time.sleep(self.gripper.settle_s)
                timeout_ns = started + int(self.gripper.motion_timeout_s * 1e9)
                while time.monotonic_ns() < timeout_ns:
                    if stop_event is not None and stop_event.is_set():
                        stopped = True
                        stop_reason = "operator_stop"
                        break
                    current = required_read("Present_Position", normalize=True)
                    sample_elapsed = (time.monotonic_ns() - started) / 1e9
                    load = self._optional_raw_read(bus, "Present_Load")
                    current_reading = self._optional_raw_read(bus, "Present_Current")
                    moving = optional_read("Moving")
                    sample = {
                        "elapsed_s": sample_elapsed,
                        "present_position": current,
                        "moving": moving,
                        "load": load,
                        "current": current_reading,
                        "voltage": optional_read("Present_Voltage"),
                        "motor_status": optional_read("Status"),
                    }
                    position_samples.append(sample)
                    if motion_onset_s is None and abs(current - initial_position) >= self.gripper.motion_observation_threshold:
                        motion_onset_s = sample_elapsed
                    if (
                        monitor_stall
                        and self.gripper.load_threshold is not None
                        and self.gripper.current_threshold is not None
                        and load is not None
                        and current_reading is not None
                        and load >= self.gripper.load_threshold
                        and current_reading >= self.gripper.current_threshold
                    ):
                        stop_reason = "grasp_threshold_reached"
                        break
                    if abs(target - current) <= self.gripper.position_tolerance and motion_onset_s is not None:
                        break
                    # For a multi-step requested movement, issue the next bounded
                    # step only after the previous command moved the encoder.
                    if motion_onset_s is not None and abs(target - current) > self.gripper.position_tolerance:
                        next_position = float(
                            np.clip(
                                current + direction * min(step, abs(target - current)),
                                self.gripper.min_position,
                                self.gripper.max_position,
                            )
                        )
                        if not motion_commands or abs(next_position - motion_commands[-1]) > 1e-9:
                            bus.sync_write("Goal_Position", {GRIPPER: next_position}, normalize=True)
                            commands.append(next_position)
                            motion_commands.append(next_position)
                            diagnostics["goal_readback"] = required_read("Goal_Position", normalize=True)
                    time.sleep(self.gripper.poll_interval_s)

                final = required_read("Present_Position", normalize=True)
                raw_final = required_read("Present_Position", normalize=False)
                load = self._optional_raw_read(bus, "Present_Load")
                current_reading = self._optional_raw_read(bus, "Present_Current")
                position_error = abs(final - target)
                measured_movement = abs(final - initial_position)
                motion_observed = measured_movement >= self.gripper.motion_observation_threshold
                position_reached = position_error <= self.gripper.position_tolerance
                diagnostics.update(
                    {
                        "final_torque_enable": required_read("Torque_Enable"),
                        "final_goal_position": required_read("Goal_Position", normalize=True),
                        "final_voltage": optional_read("Present_Voltage"),
                        "final_temperature": optional_read("Present_Temperature"),
                        "final_motor_status": optional_read("Status"),
                        "motion_onset_s": motion_onset_s,
                    }
                )
                if stopped:
                    status = "stopped"
                    safe_stop = self._safe_stop_gripper(bus)
                elif not motion_commands:
                    status = "command_not_sent"
                    stop_reason = "command_not_sent"
                    safe_stop = self._safe_stop_gripper(bus)
                elif not motion_observed:
                    status = "motion_not_observed"
                    stop_reason = "motion_not_observed"
                    safe_stop = self._safe_stop_gripper(bus)
                elif not position_reached:
                    status = "position_not_reached"
                    stop_reason = "position_not_reached"
                else:
                    status = "complete"
                    if stop_reason == "pending":
                        stop_reason = "target_reached"
        except BaseException as exc:
            error = str(exc)
            communication_errors.append(error)
            status = "communication_error"
            stop_reason = "communication_error"
            try:
                safe_stop = self._safe_stop_gripper(bus)
            except Exception as stop_exc:
                safe_stop = {"status": "failed", "error": str(stop_exc)}
            try:
                final = required_read("Present_Position", normalize=True)
            except Exception as read_exc:
                communication_errors.append(f"final Present_Position: {read_exc}")

        return self._gripper_result(
            side=side,
            operation=operation,
            status=status,
            stop_reason=stop_reason,
            target=target,
            initial_position=initial_position,
            final=final,
            raw_final=raw_final,
            commands=commands,
            motion_commands=motion_commands,
            position_samples=position_samples,
            diagnostics=diagnostics,
            communication_errors=communication_errors,
            safe_stop=safe_stop,
            motion_onset_s=motion_onset_s,
            started=started,
            load=load,
            current=current_reading,
            error=error,
        )

    def _safe_stop_gripper(self, bus: Any) -> dict[str, Any]:
        """Hold the measured gripper position, then disable torque.

        Disabling torque after the hold prevents a retained Goal_Position from
        producing delayed motion after a failed diagnostic or explicit stop.
        """
        started = time.monotonic_ns()
        try:
            with self._bus_lock:
                present = float(bus.sync_read("Present_Position", [GRIPPER], normalize=True)[GRIPPER])
                bus.sync_write("Goal_Position", {GRIPPER: present}, normalize=True)
                goal_readback = float(bus.sync_read("Goal_Position", [GRIPPER], normalize=True)[GRIPPER])
                bus.disable_torque([GRIPPER])
                torque_readback = float(bus.sync_read("Torque_Enable", [GRIPPER], normalize=False)[GRIPPER])
                held = float(bus.sync_read("Present_Position", [GRIPPER], normalize=True)[GRIPPER])
            goal_match = abs(goal_readback - present) <= 0.1
            return {
                "status": (
                    "complete"
                    if torque_readback == 0 and goal_match
                    else "hold_not_verified"
                    if not goal_match
                    else "torque_not_disabled"
                ),
                "hold_commanded_position": present,
                "hold_goal_readback": goal_readback,
                "hold_goal_match": goal_match,
                "hold_measured_position": held,
                "hold_movement": abs(held - present),
                "torque_enable": torque_readback,
                "torque_disabled": torque_readback == 0,
                "elapsed_s": (time.monotonic_ns() - started) / 1e9,
            }
        except Exception as exc:
            return {
                "status": "failed",
                "error": str(exc),
                "elapsed_s": (time.monotonic_ns() - started) / 1e9,
            }

    def _gripper_result(
        self,
        *,
        side: str,
        operation: str,
        status: str,
        stop_reason: str,
        target: float,
        initial_position: float | None,
        final: float | None,
        raw_final: float | None,
        commands: list[float],
        motion_commands: list[float],
        position_samples: list[dict[str, Any]],
        diagnostics: dict[str, Any],
        communication_errors: list[str],
        safe_stop: dict[str, Any] | None,
        motion_onset_s: float | None,
        started: int,
        load: float | None,
        current: float | None,
        error: str | None = None,
    ) -> dict[str, Any]:
        if initial_position is None or final is None:
            measured_movement = None
            position_error = None
            motion_observed = False
            position_reached = False
        else:
            measured_movement = abs(final - initial_position)
            position_error = abs(final - target)
            motion_observed = measured_movement >= self.gripper.motion_observation_threshold
            position_reached = position_error <= self.gripper.position_tolerance
        return {
            "side": side,
            "operation": operation,
            "status": status,
            "stop_reason": stop_reason,
            "target_position": target,
            "initial_position": initial_position,
            "final_position": final,
            "raw_final_position": raw_final,
            "measured_movement": measured_movement,
            "position_error": position_error,
            "position_reached": position_reached,
            "motion_required": abs(target - (initial_position if initial_position is not None else target)) > 1e-9,
            "motion_observation_threshold": self.gripper.motion_observation_threshold,
            "motion_observed": motion_observed,
            "motion_onset_s": motion_onset_s,
            "stopped": status == "stopped",
            "commands": commands,
            "motion_commands": motion_commands,
            "command_count": len(commands),
            "motion_command_count": len(motion_commands),
            "command_sent": bool(commands),
            "motion_command_sent": bool(motion_commands),
            "position_samples": position_samples,
            "sample_count": len(position_samples),
            "poll_interval_s": self.gripper.poll_interval_s,
            "motion_timeout_s": self.gripper.motion_timeout_s,
            "diagnostics": diagnostics,
            "communication_errors": communication_errors,
            "safe_stop": safe_stop,
            "load": load,
            "current": current,
            "error": error,
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
