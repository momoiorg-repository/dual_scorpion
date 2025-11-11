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
from functools import cached_property
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_dual_scorpion_follower import DualScorpionFollowerConfig

logger = logging.getLogger(__name__)

class DualScorpionFollower(Robot):
    """
    SO-101 Dual Follower Arm
    """
    config_class = DualScorpionFollowerConfig
    name = "so101_dual_follower"

    def __init__(self, config: DualScorpionFollowerConfig):
        """
        Initialize the SO-101 Dual Follower Arm with the provided configuration.
        SO-101 双腕フォロワーアームを初期化
        """
        super().__init__(config)
        self.config = config
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100

        # Separate calibration data for right and left arms / キャリブレーションデータを右腕と左腕用に分離
        # "right_"で始まるモーターのキャリブレーションデータのみ抽出
        right_calibration = {
            motor.replace("right_", ""): calib 
            for motor, calib in self.calibration.items() 
            if motor.startswith("right_")
        }
        # "left_"で始まるモーターのキャリブレーションデータのみ抽出
        left_calibration = {  
            motor.replace("left_", ""): calib 
            for motor, calib in self.calibration.items() 
            if motor.startswith("left_")
        }

        # Initialize right follower motors / 右腕のモーターバスの初期化
        self.right_bus = FeetechMotorsBus(
            port=self.config.right_arm_port,
            motors={
                "joint0": Motor(1, "sts3215", norm_mode_body),
                "joint1": Motor(2, "sts3215", norm_mode_body),
                "joint2": Motor(3, "sts3215", norm_mode_body),
                "joint3": Motor(4, "sts3215", norm_mode_body),
                "joint4": Motor(5, "sts3215", norm_mode_body),
                "joint5": Motor(6, "sts3215", norm_mode_body),
                "joint6": Motor(7, "sts3215", norm_mode_body),
                "gripper": Motor(8, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=right_calibration,
        )
        # Initialize left follower motors / 左腕のモーターバスの初期化
        self.left_bus = FeetechMotorsBus(
            port=self.config.left_arm_port,
            motors={
                "joint0": Motor(1, "sts3215", norm_mode_body),
                "joint1": Motor(2, "sts3215", norm_mode_body),
                "joint2": Motor(3, "sts3215", norm_mode_body),
                "joint3": Motor(4, "sts3215", norm_mode_body),
                "joint4": Motor(5, "sts3215", norm_mode_body),
                "joint5": Motor(6, "sts3215", norm_mode_body),
                "joint6": Motor(7, "sts3215", norm_mode_body),
                "gripper": Motor(8, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=left_calibration,
        )
        # Initialize cameras / カメラの初期化
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
        """
        Check if the devices is connected.
        デバイスが接続されているかを確認する
        """
        return self.right_bus.is_connected and \
            self.left_bus.is_connected and \
            all(cam.is_connected for cam in self.cameras.values())
    
    def connect(self, calibrate: bool = True) -> None:
        """
        We assume that at connection time, arm is in a rest position,
        and torque can be safely disabled to run calibration.
        接続時にはアームが休止位置にあると想定しており、
        トルクを安全に無効にしてキャリブレーションを実行できます。
        """
        if self.is_connected:  # すでに接続されている場合はエラーを投げる
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.right_bus.connect()  # 右腕のモーターバスを接続
        self.left_bus.connect()  # 左腕のモーターバスを接続

        if not self.is_calibrated and calibrate:  # キャリブレーションが必要な場合は実行
            self.calibrate()

        for cam in self.cameras.values():  # カメラを接続
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        """
        Check if the arms is calibrated.
        アームがキャリブレーションされているかどうかを確認する
        """
        return self.right_bus.is_calibrated and self.left_bus.is_calibrated
    
    def calibrate(self) -> None:
        """
        Run the calibration for both arms.
        キャリブレーションを実行する
        """
        logger.info(f"\nRunning calibration of {self}")
        self.right_bus.disable_torque()  # 右腕のトルクを無効にする
        self.left_bus.disable_torque()  # 左腕のトルクを無効にする

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

        # error:
        # self.right_bus.write_calibration(self.calibration)
        # self.left_bus.write_calibration(self.calibration)

        # キャリブレーションを保存する前に、右腕と左腕のキャリブレーションデータを分ける必要がある
        # Extract calibration data for the right arm / 右腕用のキャリブレーションデータを抽出
        right_calibration = {
            motor.replace("right_", ""): calib
            for motor, calib in self.calibration.items()
            if motor.startswith("right_")
        }
        # Extract calibration data for the left arm / 左腕用のキャリブレーションデータを抽出
        left_calibration = {
            motor.replace("left_", ""): calib
            for motor, calib in self.calibration.items()
            if motor.startswith("left_")
        }
        self.right_bus.write_calibration(right_calibration)
        self.left_bus.write_calibration(left_calibration)

        self._save_calibration()  # キャリブレーションを保存
        print("Calibration saved to", self.calibration_fpath)

    def configure(self) -> None:
        """
        Apply the motor settings for both arms.
        モーターの設定を適用する
        """
        with self.right_bus.torque_disabled(), self.left_bus.torque_disabled():
            self.right_bus.configure_motors()
            self.left_bus.configure_motors()
            for motor in self.right_bus.motors:
                self.right_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
                self.right_bus.write("P_Coefficient", motor, 16)
                # Set I_Coefficient and D_Coefficient to default value 0 and 32
                self.right_bus.write("I_Coefficient", motor, 0)
                self.right_bus.write("D_Coefficient", motor, 32)
            for motor in self.left_bus.motors:
                self.left_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
                self.left_bus.write("P_Coefficient", motor, 16)
                # Set I_Coefficient and D_Coefficient to default value 0 and 32
                self.left_bus.write("I_Coefficient", motor, 0)
                self.left_bus.write("D_Coefficient", motor, 32)

    def setup_motors(self) -> None:
        for motor in reversed(self.right_bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.right_bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.right_bus.motors[motor].id}")
        for motor in reversed(self.left_bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.left_bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.left_bus.motors[motor].id}")

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        
        # Read arm position
        start = time.perf_counter()
        right_obs_dict = self.right_bus.sync_read("Present_Position")
        left_obs_dict = self.left_bus.sync_read("Present_Position")

        # Add proper prefixes to match the expected feature names
        right_obs_dict = {f"right_{motor}.pos": val for motor, val in right_obs_dict.items()}
        left_obs_dict = {f"left_{motor}.pos": val for motor, val in left_obs_dict.items()}

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Combine both arm observations
        obs_dict = {**right_obs_dict, **left_obs_dict}

        # capture camera images
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict
    
    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Command arm to move to a target joint configuration.

        The relative action magnitude may be clipped depending on the configuration parameter
        `max_relative_target`. In this case, the action sent differs from original action.
        Thus, this function always returns the action actually sent.

        Raises:
            RobotDeviceNotConnectedError: if robot is not connected.

        Returns:
            the action sent to the motors, potentially clipped.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        # Cap goal position when too far away from present position.
        # /!\ Slower fps expected due to reading from the follower.
        # 現在の位置から離れすぎている場合は、目標位置をキャップします。
        # /!\ フォロワーからの読み取りにより、fps が遅くなることが予想されます。
        if self.config.max_relative_target is not None:
            right_present_pos = self.right_bus.sync_read("Present_Position")
            left_present_pos = self.left_bus.sync_read("Present_Position")

            goal_present_pos = {
                f"right_{key}": (g_pos, right_present_pos[key]) 
                for key, g_pos in goal_pos.items() 
                if key.startswith("right_")
            }
            goal_present_pos.update({
                f"left_{key}": (g_pos, left_present_pos[key]) 
                for key, g_pos in goal_pos.items() 
                if key.startswith("left_")
            })

            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        # Send goal position to the arms
        # 右腕用の目標位置を抽出
        right_goal_pos = {
            key.replace("right_", ""): val 
            for key, val in goal_pos.items() 
            if key.startswith("right_")
        }
        # 左腕用の目標位置を抽出
        left_goal_pos = {
            key.replace("left_", ""): val 
            for key, val in goal_pos.items() 
            if key.startswith("left_")
        }

        self.right_bus.sync_write("Goal_Position", right_goal_pos)
        self.left_bus.sync_write("Goal_Position", left_goal_pos)
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}
    
    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.right_bus.disconnect(self.config.disable_torque_on_disconnect)
        self.left_bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")
    
