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

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)

from ..teleoperator import Teleoperator
from .config_dual_scropion_leader import DualScropionLeaderConfig

logger = logging.getLogger(__name__)

class DualScropionLeader(Teleoperator):
    """
    Dual Scropion Leader Arm
    """
    config_class = DualScropionLeaderConfig    
    name = "dual_scropion_leader"

    def __init__(self, config: DualScropionLeaderConfig):
        """
        Initialize the Dual Scropion Leader Arm
        Dual Scropion 双腕リーダーアームを初期化
        """
        super().__init__(config)
        self.config = config
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100

        # Separate calibration data for right and left arms / キャリブレーションデータを右腕と左腕用に分離
        right_calibration = {
            motor.replace("right_", ""): calib 
            for motor, calib in self.calibration.items() 
            if motor.startswith("right_")
        }
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

    @property
    def action_features(self) -> dict[str, type]:
        """
        Returns the action features for the Dual Scropion Leader Arm.
        アクション特徴を返す
        """
        features = {}
        # Right arm motors / 右腕のモーター
        features.update({f"right_{motor}.pos": float for motor in self.right_bus.motors})
        #  Left arm motors / 左腕のモーター
        features.update({f"left_{motor}.pos": float for motor in self.left_bus.motors})
        return features
    
    @property
    def feedback_features(self) -> dict[str, type]:
        return {}
    
    @property
    def is_connected(self) -> bool:
        """
        Check if the devices is connected.
        デバイスが接続されているかを確認する
        """
        return self.right_bus.is_connected and self.left_bus.is_connected
    
    def connect(self, calibrate: bool = True) -> None:
        """
        Connect the devices.
        デバイスを接続する
        """
        if self.is_connected:  # 既に接続されている場合はエラーを投げる
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.right_bus.connect()  # 右腕のモーターバスを接続
        self.left_bus.connect()  # 左腕のモーターバスを接続

        if not self.is_calibrated and calibrate:  # キャリブレーションが必要な場合は実行
            self.calibrate()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        """
        Check if the devices is calibrated.
        キャリブレーションが完了しているかを確認する
        """
        return self.right_bus.is_calibrated and self.left_bus.is_calibrated
    
    def calibrate(self) -> None:
        """
        Run calibration for the Dual Scropion Leader Arm.
        キャリブレーションを実行する
        """
        logger.info(f"\nRunning calibration for {self}")
        self.right_bus.disable_torque()  # 右腕のトルクを無効化
        self.left_bus.disable_torque()  # 左腕のトルクを無効化

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
            "of motion.\nRecording positions. Press ENTER to stop..."
        )
        right_range_mins, right_range_maxes = self.right_bus.record_ranges_of_motion()
        for motor, m in self.right_bus.motors.items():  # motor = "shoulder_pan", "shoulder_lift", ... | m = Motor instance
            self.calibration[f"right_{motor}"] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=right_homing_offsets[motor],
                range_min=right_range_mins[motor],
                range_max=right_range_maxes[motor],
            )

        # Left arm calibration / 左腕のキャリブレーション
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

        # error:
        # self.right_bus.write_calibration(self.calibration)  # 右腕のキャリブレーションを保存
        # self.left_bus.write_calibration(self.calibration)  # 左腕のキャリブレーションを保存

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
        print(f"Calibration saved to {self.calibration_fpath}")

    def configure(self)  -> None:
        """
        Configure the motors for the Dual Scropion Leader Arm.
        Dual Scropion 双腕リーダーアームのモーターを設定する
        """
        self.right_bus.disable_torque()  # 右腕のトルクの無効化
        self.left_bus.disable_torque()  # 左腕のトルクの無効化

        self.right_bus.configure_motors()  # 右腕のモーターを設定
        self.left_bus.configure_motors()  # 左腕のモーターを設定

        for motor in self.right_bus.motors:
            self.right_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
        for motor in self.left_bus.motors:
            self.left_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

    def setup_motors(self) -> None:
        for motor in reversed(self.right_bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.right_bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.right_bus.motors[motor].id}")
        for motor in reversed(self.left_bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.left_bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.left_bus.motors[motor].id}")

    def get_action(self) -> dict[str, float]:
        """
        Read the current action from the Dual Scropion Leader Arm.
        アクションを取得する
        """
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
        if not self.is_connected:
            DeviceNotConnectedError(f"{self} is not connected.")
    
        self.right_bus.disconnect()
        self.left_bus.disconnect()

        logger.info(f"{self} disconnected.")
