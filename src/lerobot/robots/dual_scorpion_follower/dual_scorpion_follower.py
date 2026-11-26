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
import math
import time
from collections.abc import Sequence
from contextlib import ExitStack
from functools import cached_property

from lerobot.cameras import make_cameras_from_configs
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.partial_calibration import (
    calibration_for_prefixed_bus,
    recalibrate_prefixed_bus_joints,
    resolve_prefixed_joint_selectors,
)

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_dual_scorpion_follower import DualScorpionFollowerConfig

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


class DualScorpionFollower(Robot):
    """Dual Scorpion bimanual follower arm."""

    config_class = DualScorpionFollowerConfig
    name = "dual_scorpion_follower"

    def __init__(self, config: DualScorpionFollowerConfig):
        super().__init__(config)
        self.config = config
        if not self.calibration and config.calibration_dir is None:
            legacy_calibration_fpath = (
                HF_LEROBOT_CALIBRATION / ROBOTS / "so101_dual_follower" / f"{self.id}.json"
            )
            if legacy_calibration_fpath.is_file():
                self._load_calibration(legacy_calibration_fpath)

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
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        ft = {}
        ft.update({f"right_{motor}.pos": float for motor in self.right_bus.motors})
        ft.update({f"left_{motor}.pos": float for motor in self.left_bus.motors})
        return ft

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return (
            self.right_bus.is_connected
            and self.left_bus.is_connected
            and all(cam.is_connected for cam in self.cameras.values())
        )

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        """
        We assume that at connection time, arm is in a rest position,
        and torque can be safely disabled to run calibration.
        接続時にはアームが休止位置にあると想定しており、
        トルクを安全に無効にしてキャリブレーションを実行できます。
        """
        configured = False
        with ExitStack() as cleanup:
            for bus in (self.right_bus, self.left_bus):
                bus.connect()
                cleanup.callback(
                    lambda bus=bus: bus.disconnect(configured and self.config.disable_torque_on_disconnect)
                )
            if (calibrate or self.config.require_calibrated) and not self.is_calibrated:
                if self.config.require_calibrated:
                    raise ValueError(f"{self}: saved calibration does not match both follower buses.")
                logger.info("Motor calibration differs from the saved calibration or the file is missing")
                self.calibrate()
            for cam in self.cameras.values():
                cam.connect()
                cleanup.callback(cam.disconnect)
            configured = True
            self.configure()
            cleanup.pop_all()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.right_bus.is_calibrated and self.left_bus.is_calibrated

    def calibrate(self) -> None:
        """
        Run the calibration for both arms.
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

        logger.info(f"\nRunning calibration of {self}")
        self.right_bus.disable_torque()
        self.left_bus.disable_torque()

        for motor in self.right_bus.motors:
            self.right_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
        for motor in self.left_bus.motors:
            self.left_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        self.calibration = {}

        # Right arm calibration / 右腕のキャリブレーション
        input(f"Move RIGHT {self} to the middle of its range of motion and press ENTER....")
        right_homing_offsets = self.right_bus.set_half_turn_homings()
        print(
            "Move all joints sequentially through their entire ranges "
            "and press ENTER when done with each joint."
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

        # Left arm calibration/ 左腕のキャリブレーション
        input(f"Move LEFT {self} to the middle of its range of motion and press ENTER....")
        left_homing_offsets = self.left_bus.set_half_turn_homings()
        print(
            "Move all joints sequentially through their entire ranges "
            "and press ENTER when done with each joint."
        )
        left_range_mins, left_range_maxes = self.left_bus.record_ranges_of_motion()
        for motor, m in self.left_bus.motors.items():
            self.calibration[f"left_{motor}"] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=left_homing_offsets[motor],
                range_min=left_range_mins[motor],
                range_max=left_range_maxes[motor],
            )

        self.right_bus.write_calibration(_calibration_for_arm(self.calibration, "right"))
        self.left_bus.write_calibration(_calibration_for_arm(self.calibration, "left"))

        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    def calibrate_joints(self, joints: Sequence[str]) -> None:
        """
        Recalibrate only selected joints while preserving the rest of the calibration file.
        選択したジョイントのみ再キャリブレーションする
        """
        if not self.calibration:
            raise RuntimeError(
                "Partial calibration requires an existing calibration file. "
                "Run full calibration first, or pass the same --robot.id used before."
            )

        logger.info(f"\nRunning partial calibration of {self} for joints: {', '.join(joints)}")
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
        print("Partial calibration saved to", self.calibration_fpath)

    def configure(self) -> None:
        """
        Apply the motor settings for both arms.
        モーターの設定を適用する
        """
        if not self.config.require_calibrated:
            with self.right_bus.torque_disabled(), self.left_bus.torque_disabled():
                self._configure_motors()
            return

        buses = (self.right_bus, self.left_bus)
        for bus in buses:
            bus.disable_torque()
        self._configure_motors()
        for bus in buses:
            present = bus.sync_read("Present_Position", normalize=False)
            if present.keys() != bus.motors.keys() or any(
                not bus.calibration[motor].range_min <= position <= bus.calibration[motor].range_max
                for motor, position in present.items()
            ):
                raise RuntimeError("Cannot hold the follower at a position outside its saved calibration.")
            bus.sync_write("Goal_Position", present, normalize=False)
            if bus.sync_read("Goal_Position", normalize=False) != present:
                raise RuntimeError("Follower startup hold could not be confirmed.")
        for bus in buses:
            bus.enable_torque()

    def _configure_motors(self) -> None:
        for bus in (self.right_bus, self.left_bus):
            bus.configure_motors()
            bus.set_operating_mode("position")
            for motor in bus.motors:
                bus.write("P_Coefficient", motor, 16)
                bus.write("I_Coefficient", motor, 0)
                bus.write("D_Coefficient", motor, 32)
                if motor == "gripper":
                    bus.write("Max_Torque_Limit", motor, 500)
                    bus.write("Protection_Current", motor, 250)
                    bus.write("Overload_Torque", motor, 25)

    def setup_motors(self, arm: str | None = None) -> None:
        """
        Reflash IDs/baud for motors. Optionally limit to one arm to allow single-bus setups.

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
    def get_joint_positions(self) -> dict[str, float]:
        """Read both arms without acquiring camera frames."""
        # Read arm position
        start = time.perf_counter()
        right_obs_dict = self.right_bus.sync_read("Present_Position")
        left_obs_dict = self.left_bus.sync_read("Present_Position")

        # Add proper prefixes to match the expected feature names
        right_obs_dict = {f"right_{motor}.pos": val for motor, val in right_obs_dict.items()}
        left_obs_dict = {f"left_{motor}.pos": val for motor, val in left_obs_dict.items()}

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        return {**right_obs_dict, **left_obs_dict}

    def get_observation(self) -> RobotObservation:
        obs_dict: RobotObservation = self.get_joint_positions()

        # capture camera images
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.read_latest()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        """Command arm to move to a target joint configuration.

        The relative action magnitude may be clipped depending on the configuration parameter
        `max_relative_target`. In this case, the action sent differs from original action.
        Thus, this function always returns the action actually sent.

        Raises:
            RobotDeviceNotConnectedError: if robot is not connected.

        Returns:
            the action sent to the motors, potentially clipped.
        """
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}
        right_goal_pos = {
            key.removeprefix("right_"): val for key, val in goal_pos.items() if key.startswith("right_")
        }
        left_goal_pos = {
            key.removeprefix("left_"): val for key, val in goal_pos.items() if key.startswith("left_")
        }

        # Cap goal position when too far away from present position.
        # /!\ Slower fps expected due to reading from the follower.
        # 現在の位置から離れすぎている場合は、目標位置をキャップします。
        # /!\ フォロワーからの読み取りにより、fps が遅くなることが予想されます。
        if self.config.max_relative_target is not None:
            started = time.monotonic()
            right_present_pos = self.right_bus.sync_read("Present_Position")
            left_present_pos = self.left_bus.sync_read("Present_Position")

            goal_present_pos = {
                f"right_{motor}": (goal, right_present_pos[motor]) for motor, goal in right_goal_pos.items()
            }
            goal_present_pos.update(
                {f"left_{motor}": (goal, left_present_pos[motor]) for motor, goal in left_goal_pos.items()}
            )

            if self.config.require_calibrated:
                if time.monotonic() - started > 0.25:
                    raise RuntimeError("Follower position feedback is stale.")
                for bus, goals in ((self.right_bus, right_goal_pos), (self.left_bus, left_goal_pos)):
                    for motor, value in goals.items():
                        calibration = bus.calibration[motor]
                        mode = bus.motors[motor].norm_mode
                        if mode is MotorNormMode.DEGREES:
                            upper = (calibration.range_max - calibration.range_min) * 180 / (
                                bus.resolution(bus.motors[motor].model) - 1
                            )
                            lower = -upper
                        else:
                            lower, upper = (0, 100) if mode is MotorNormMode.RANGE_0_100 else (-100, 100)
                        if not math.isfinite(value) or not lower - 1e-5 <= value <= upper + 1e-5:
                            raise ValueError(f"Follower target for {motor} is outside its calibrated range.")
            safe_goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)
            right_goal_pos = {
                key.removeprefix("right_"): val
                for key, val in safe_goal_pos.items()
                if key.startswith("right_")
            }
            left_goal_pos = {
                key.removeprefix("left_"): val
                for key, val in safe_goal_pos.items()
                if key.startswith("left_")
            }

        self.right_bus.sync_write("Goal_Position", right_goal_pos)
        self.left_bus.sync_write("Goal_Position", left_goal_pos)
        return {
            **{f"right_{motor}.pos": val for motor, val in right_goal_pos.items()},
            **{f"left_{motor}.pos": val for motor, val in left_goal_pos.items()},
        }

    def disconnect(self) -> None:
        with ExitStack() as cleanup:
            for cam in self.cameras.values():
                if cam.is_connected:
                    cleanup.callback(cam.disconnect)
            connected_buses = [bus for bus in (self.left_bus, self.right_bus) if bus.is_connected]
            if not connected_buses and not any(cam.is_connected for cam in self.cameras.values()):
                raise DeviceNotConnectedError(f"{self} is not connected.")
            for bus in connected_buses:
                cleanup.callback(bus.disconnect, self.config.disable_torque_on_disconnect)

        logger.info(f"{self} disconnected.")
