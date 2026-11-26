#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
# Modifications Copyright 2025 S.Satoya
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from collections.abc import Sequence
from contextlib import ExitStack

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.partial_calibration import (
    calibration_for_prefixed_bus,
    recalibrate_prefixed_bus_joints,
    resolve_prefixed_joint_selectors,
)

from ..teleoperator import Teleoperator
from .config_dual_scorpion_leader import DualScorpionLeaderConfig

logger = logging.getLogger(__name__)


def _arm_motors(norm_mode_body: MotorNormMode) -> dict[str, Motor]:
    return {
        "joint0": Motor(1, "sts3215", norm_mode_body),
        "joint1": Motor(2, "sts3215", norm_mode_body),
        "joint2": Motor(3, "sts3215", norm_mode_body),
        "joint3": Motor(4, "sts3215", norm_mode_body),
        "joint4": Motor(5, "sts3215", norm_mode_body),
        "joint5": Motor(6, "sts3215", norm_mode_body),
        "joint6": Motor(7, "sts3215", norm_mode_body),
        "gripper": Motor(8, "sts3215", MotorNormMode.RANGE_0_100),
    }


def _calibration_for_arm(
    calibration: dict[str, MotorCalibration],
    arm: str,
) -> dict[str, MotorCalibration]:
    return calibration_for_prefixed_bus(calibration, arm)


class DualScorpionLeader(Teleoperator):
    """Dual Scorpion bimanual leader arm."""

    config_class = DualScorpionLeaderConfig
    name = "dual_scorpion_leader"

    def __init__(self, config: DualScorpionLeaderConfig):
        super().__init__(config)
        self.config = config
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100

        self.right_bus = FeetechMotorsBus(
            port=self.config.right_arm_port,
            motors=_arm_motors(norm_mode_body),
            calibration=_calibration_for_arm(self.calibration, "right"),
        )
        self.left_bus = FeetechMotorsBus(
            port=self.config.left_arm_port,
            motors=_arm_motors(norm_mode_body),
            calibration=_calibration_for_arm(self.calibration, "left"),
        )

    @property
    def action_features(self) -> dict[str, type]:
        features = {}
        features.update({f"right_{motor}.pos": float for motor in self.right_bus.motors})
        features.update({f"left_{motor}.pos": float for motor in self.left_bus.motors})
        return features

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.right_bus.is_connected and self.left_bus.is_connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        with ExitStack() as cleanup:
            for bus in (self.right_bus, self.left_bus):
                bus.connect()
                cleanup.callback(bus.disconnect, False)
            if (calibrate or self.config.require_calibrated) and not self.is_calibrated:
                if self.config.require_calibrated:
                    raise ValueError(f"{self}: saved calibration does not match both leader buses.")
                logger.info("Motor calibration differs from the saved calibration or the file is missing")
                self.calibrate()
            self.configure()
            cleanup.pop_all()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.right_bus.is_calibrated and self.left_bus.is_calibrated

    def calibrate(self) -> None:
        """
        Run calibration for the Dual Scorpion Leader Arm.
        キャリブレーションを実行する
        """
        if self.calibration:
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
                self.right_bus.write_calibration(_calibration_for_arm(self.calibration, "right"))
                self.left_bus.write_calibration(_calibration_for_arm(self.calibration, "left"))
                return

        logger.info(f"\nRunning calibration for {self}")
        self.right_bus.disable_torque()
        self.left_bus.disable_torque()

        for motor in self.right_bus.motors:
            self.right_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
        for motor in self.left_bus.motors:
            self.left_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        self.calibration = {}

        input(f"Move RIGHT {self} to the middle of its range of motion and press ENTER....")
        right_homing_offsets = self.right_bus.set_half_turn_homings()
        print(
            "Move all joints sequentially through their entire ranges "
            "of motion.\nRecording positions. Press ENTER to stop..."
        )
        right_range_mins, right_range_maxes = self.right_bus.record_ranges_of_motion()
        for motor, m in self.right_bus.motors.items():
            self.calibration[f"right_{motor}"] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=right_homing_offsets[motor],
                range_min=right_range_mins[motor],
                range_max=right_range_maxes[motor],
            )

        input(f"Move LEFT {self} to the middle of its range of motion and press ENTER....")
        homing_offsets_left = self.left_bus.set_half_turn_homings()
        print(
            "Move all joints sequentially through their entire ranges "
            "of motion.\nRecording positions. Press ENTER to stop..."
        )
        range_mins_left, range_maxes_left = self.left_bus.record_ranges_of_motion()
        for motor, m in self.left_bus.motors.items():
            self.calibration[f"left_{motor}"] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets_left[motor],
                range_min=range_mins_left[motor],
                range_max=range_maxes_left[motor],
            )

        print("Saving calibration...")

        self.right_bus.write_calibration(_calibration_for_arm(self.calibration, "right"))
        self.left_bus.write_calibration(_calibration_for_arm(self.calibration, "left"))

        self._save_calibration()
        print(f"Calibration saved to {self.calibration_fpath}")

    def calibrate_joints(self, joints: Sequence[str]) -> None:
        """
        Recalibrate only selected joints while preserving the rest of the calibration file.
        選択したジョイントのみ再キャリブレーションする
        """
        if not self.calibration:
            raise RuntimeError(
                "Partial calibration requires an existing calibration file. "
                "Run full calibration first, or pass the same --teleop.id used before."
            )

        logger.info(f"\nRunning partial calibration for {self} for joints: {', '.join(joints)}")
        selected = resolve_prefixed_joint_selectors(joints, motor_names=self.right_bus.motors)

        self.calibration = recalibrate_prefixed_bus_joints(
            calibration=self.calibration,
            prefix="right",
            bus=self.right_bus,
            motors=selected["right"],
            device_label=str(self),
            position_mode_value=OperatingMode.POSITION.value,
        )
        self.calibration = recalibrate_prefixed_bus_joints(
            calibration=self.calibration,
            prefix="left",
            bus=self.left_bus,
            motors=selected["left"],
            device_label=str(self),
            position_mode_value=OperatingMode.POSITION.value,
        )

        self._save_calibration()
        print(f"Partial calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        self.right_bus.disable_torque()
        self.left_bus.disable_torque()

        self.right_bus.configure_motors()
        self.left_bus.configure_motors()

        for motor in self.right_bus.motors:
            self.right_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
        for motor in self.left_bus.motors:
            self.left_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

    def setup_motors(self, arm: str | None = None) -> None:
        """
        Reflash IDs/baud for motors. Optionally limit to one arm.

        Args:
            arm: 'right', 'left', or 'both' (default).
        """
        arm = arm or "both"
        if arm not in ("right", "left", "both"):
            raise ValueError("arm must be one of: 'right', 'left', 'both'")

        if arm in ("right", "both"):
            for motor in reversed(self.right_bus.motors):
                input(
                    f"Connect the controller board to the '{motor}' motor only (RIGHT arm) and press enter."
                )
                self.right_bus.setup_motor(motor)
                print(f"RIGHT '{motor}' motor id set to {self.right_bus.motors[motor].id}")

        if arm in ("left", "both"):
            for motor in reversed(self.left_bus.motors):
                input(f"Connect the controller board to the '{motor}' motor only (LEFT arm) and press enter.")
                self.left_bus.setup_motor(motor)
                print(f"LEFT '{motor}' motor id set to {self.left_bus.motors[motor].id}")

    @check_if_not_connected
    def get_action(self) -> dict[str, float]:
        start = time.perf_counter()
        action_right = self.right_bus.sync_read("Present_Position")
        action_left = self.left_bus.sync_read("Present_Position")

        action = {f"right_{motor}.pos": val for motor, val in action_right.items()}
        action.update({f"left_{motor}.pos": val for motor, val in action_left.items()})

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError

    def disconnect(self) -> None:
        connected_buses = [bus for bus in (self.left_bus, self.right_bus) if bus.is_connected]
        if not connected_buses:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        with ExitStack() as cleanup:
            for bus in connected_buses:
                cleanup.callback(bus.disconnect)

        logger.info(f"{self} disconnected.")
