#!/usr/bin/env python3
import json
import math
import os
import sys
import time
from pathlib import Path
from threading import Lock

import rclpy
from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


LEFT_ARM_JOINTS = [
    "revolute_1",
    "revolute_2",
    "revolute_3",
    "revolute_4",
    "revolute_5",
    "revolute_6",
    "revolute_7",
    "revolute_8",
]

RIGHT_ARM_JOINTS = [
    "revolute_9",
    "revolute_10",
    "revolute_11",
    "revolute_12",
    "revolute_13",
    "revolute_14",
    "revolute_15",
    "revolute_16",
]

CONTROLLERS = {
    "left_arm_controller": LEFT_ARM_JOINTS,
    "right_arm_controller": RIGHT_ARM_JOINTS,
}

ALL_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

ROS_TO_LEROBOT = {
    "revolute_1": "left_joint0.pos",
    "revolute_2": "left_joint1.pos",
    "revolute_3": "left_joint2.pos",
    "revolute_4": "left_joint3.pos",
    "revolute_5": "left_joint4.pos",
    "revolute_6": "left_joint5.pos",
    "revolute_7": "left_joint6.pos",
    "revolute_8": "left_gripper.pos",
    "revolute_9": "right_joint0.pos",
    "revolute_10": "right_joint1.pos",
    "revolute_11": "right_joint2.pos",
    "revolute_12": "right_joint3.pos",
    "revolute_13": "right_joint4.pos",
    "revolute_14": "right_joint5.pos",
    "revolute_15": "right_joint6.pos",
    "revolute_16": "right_gripper.pos",
}
LEROBOT_TO_ROS = {value: key for key, value in ROS_TO_LEROBOT.items()}

GRIPPER_JOINT_LIMITS = {
    "revolute_8": (-1.91986, 0.45),
    "revolute_16": (-0.45, 1.91986),
}

DEFAULT_JOINT_OFFSETS_DEG = {
    # LeRobot degrees are centered on the calibrated range midpoint. These URDF
    # joints have a large non-zero center, so sign correction alone leaves a
    # visible offset between MoveIt/Isaac and the physical follower.
    "revolute_4": 85.0,
}


def duration_to_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


def parse_optional_json_object(value, parameter_name):
    if value is None or value == "":
        return {}

    try:
        parsed = json.loads(value)
    except json.JSONDecodeError as exc:
        raise ValueError(f"{parameter_name} must be a JSON object") from exc

    if not isinstance(parsed, dict):
        raise ValueError(f"{parameter_name} must be a JSON object")

    return {str(key): float(val) for key, val in parsed.items()}


def parse_optional_float_or_json(value, parameter_name):
    if value is None or value == "":
        return None

    try:
        return float(value)
    except (TypeError, ValueError):
        return parse_optional_json_object(value, parameter_name)


def add_lerobot_to_path(lerobot_root):
    candidates = []
    env_root = os.environ.get("LEROBOT_ROOT", "")
    if env_root:
        candidates.append(Path(env_root).expanduser())
    if lerobot_root:
        candidates.append(Path(lerobot_root).expanduser())
    candidates.append(Path.cwd())
    candidates.extend(Path(__file__).resolve().parents)

    for root in candidates:
        src = root / "src"
        if (src / "lerobot").is_dir():
            sys.path.insert(0, str(src))
            return root

    return None


class FollowerFollowJointTrajectory(Node):
    def __init__(self):
        super().__init__("follower_follow_joint_trajectory")

        self.declare_parameter("lerobot_root", "")
        self.declare_parameter("left_arm_port", "")
        self.declare_parameter("right_arm_port", "")
        self.declare_parameter("robot_id", "")
        self.declare_parameter("calibration_dir", "")
        self.declare_parameter("calibrate_on_connect", False)
        self.declare_parameter("use_degrees", True)
        self.declare_parameter("disable_torque_on_disconnect", True)
        self.declare_parameter(
            "max_relative_target",
            "",
            descriptor=ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("publish_follower_joint_states", True)
        self.declare_parameter("publish_isaac_joint_commands", False)
        self.declare_parameter("joint_command_topic", "/joint_command")
        self.declare_parameter("subscribe_external_joint_states", False)
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("joint_signs", "")
        self.declare_parameter("joint_offsets_deg", "")
        self.declare_parameter("gripper_min_percent", 0.0)
        self.declare_parameter("gripper_max_percent", 100.0)

        lerobot_root = self._string_parameter("lerobot_root")
        add_lerobot_to_path(lerobot_root)

        from lerobot.robots.dual_scorpion_follower import (  # noqa: PLC0415
            DualScorpionFollower,
            DualScorpionFollowerConfig,
        )

        self._left_arm_port = self._string_parameter("left_arm_port")
        self._right_arm_port = self._string_parameter("right_arm_port")
        if not self._left_arm_port or not self._right_arm_port:
            raise ValueError(
                "left_arm_port and right_arm_port are required for follower hardware control"
            )

        robot_id = self._string_parameter("robot_id") or None
        calibration_dir = self._string_parameter("calibration_dir")
        max_relative_target = parse_optional_float_or_json(
            self.get_parameter("max_relative_target").value, "max_relative_target"
        )

        self._use_degrees = self._bool_parameter("use_degrees")
        self._publish_rate_hz = self._float_parameter("publish_rate_hz")
        if self._publish_rate_hz <= 0.0:
            self._publish_rate_hz = 30.0
        self._publish_follower_joint_states = self._bool_parameter(
            "publish_follower_joint_states"
        )
        self._publish_isaac_joint_commands = self._bool_parameter(
            "publish_isaac_joint_commands"
        )
        self._joint_command_topic = self._string_parameter("joint_command_topic")
        self._subscribe_external_joint_states = self._bool_parameter(
            "subscribe_external_joint_states"
        )
        self._joint_states_topic = self._string_parameter("joint_states_topic")

        self._joint_signs = parse_optional_json_object(
            self._string_parameter("joint_signs"), "joint_signs"
        )
        self._joint_offsets_deg = DEFAULT_JOINT_OFFSETS_DEG | parse_optional_json_object(
            self._string_parameter("joint_offsets_deg"), "joint_offsets_deg"
        )
        self._gripper_min_percent = self._float_parameter("gripper_min_percent")
        self._gripper_max_percent = self._float_parameter("gripper_max_percent")
        if self._gripper_min_percent == self._gripper_max_percent:
            raise ValueError("gripper_min_percent and gripper_max_percent must differ")

        self.get_logger().info(f"Using joint signs: {self._joint_signs}")
        self.get_logger().info(f"Using joint offsets in degrees: {self._joint_offsets_deg}")

        config_kwargs = {
            "right_arm_port": self._right_arm_port,
            "left_arm_port": self._left_arm_port,
            "id": robot_id,
            "use_degrees": self._use_degrees,
            "disable_torque_on_disconnect": self._bool_parameter(
                "disable_torque_on_disconnect"
            ),
            "max_relative_target": max_relative_target,
        }
        if calibration_dir:
            config_kwargs["calibration_dir"] = Path(calibration_dir).expanduser()

        self._robot = DualScorpionFollower(DualScorpionFollowerConfig(**config_kwargs))
        calibrate_on_connect = self._bool_parameter("calibrate_on_connect")
        if not calibrate_on_connect and not self._robot.calibration:
            raise RuntimeError(
                "No DualScorpionFollower calibration file was loaded. Run "
                "lerobot-calibrate first, pass the matching robot_id/calibration_dir, "
                "or launch with calibrate_on_connect:=true intentionally."
            )

        self._lock = Lock()
        self._latest_positions = {joint: 0.0 for joint in ALL_JOINTS}
        self._last_state_warning_time = 0.0
        self._last_isaac_warning_time = 0.0

        self.get_logger().info(
            "Connecting DualScorpionFollower "
            f"left={self._left_arm_port} right={self._right_arm_port}"
        )
        self._robot.connect(calibrate=calibrate_on_connect)
        if not self._robot.is_calibrated:
            self._robot.disconnect()
            raise RuntimeError(
                "DualScorpionFollower is not calibrated. Run lerobot-calibrate first, "
                "or launch with calibrate_on_connect:=true intentionally."
            )
        self.read_robot_state()

        self._joint_state_pub = None
        if self._publish_follower_joint_states:
            self._joint_state_pub = self.create_publisher(
                JointState, "joint_states", qos_profile_sensor_data
            )
            self.create_timer(1.0 / self._publish_rate_hz, self.publish_joint_state)

        self._joint_command_pub = None
        if self._publish_isaac_joint_commands:
            self._joint_command_pub = self.create_publisher(
                JointState, self._joint_command_topic, 10
            )
            self.create_timer(5.0, self.warn_if_isaac_disconnected)

        self._external_joint_state_sub = None
        if self._subscribe_external_joint_states:
            self._external_joint_state_sub = self.create_subscription(
                JointState,
                self._joint_states_topic,
                self.external_joint_state_callback,
                qos_profile_sensor_data,
            )

        self._action_servers = []
        for controller_name, joints in CONTROLLERS.items():
            action_name = f"{controller_name}/follow_joint_trajectory"
            self._action_servers.append(
                ActionServer(
                    self,
                    FollowJointTrajectory,
                    action_name,
                    goal_callback=(
                        lambda goal, js=joints, name=controller_name: self.goal_callback(
                            goal, js, name
                        )
                    ),
                    cancel_callback=self.cancel_callback,
                    execute_callback=(
                        lambda handle, js=joints, name=controller_name: self.execute_callback(
                            handle, js, name
                        )
                    ),
                )
            )
            self.get_logger().info(f"Follower trajectory controller ready: {action_name}")

        if self._publish_follower_joint_states:
            self.get_logger().info("Publishing follower hardware state on /joint_states")
        else:
            self.get_logger().info(
                "Follower /joint_states publisher disabled; expecting another source "
                "such as Isaac Sim"
            )
        if self._publish_isaac_joint_commands:
            self.get_logger().info(
                f"Publishing mirrored Isaac joint commands to {self._joint_command_topic}"
            )
        if self._subscribe_external_joint_states:
            self.get_logger().info(
                f"Using external joint states from {self._joint_states_topic}"
            )

    def _string_parameter(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    def _bool_parameter(self, name):
        return self.get_parameter(name).get_parameter_value().bool_value

    def _float_parameter(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def read_robot_state(self):
        try:
            with self._lock:
                observation = self._robot.get_observation()
                for lerobot_key, value in observation.items():
                    ros_joint = LEROBOT_TO_ROS.get(lerobot_key)
                    if ros_joint is not None:
                        self._latest_positions[ros_joint] = self.lerobot_to_ros(
                            ros_joint, float(value)
                        )
        except Exception as exc:  # noqa: BLE001
            now = time.monotonic()
            if now - self._last_state_warning_time > 5.0:
                self._last_state_warning_time = now
                self.get_logger().warn(f"failed to read follower state: {exc}")

    def publish_joint_state(self):
        self.read_robot_state()
        self.publish_cached_joint_state()

    def external_joint_state_callback(self, msg):
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                if name in self._latest_positions:
                    self._latest_positions[name] = position

    def publish_cached_joint_state(self):
        if self._joint_state_pub is None:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ALL_JOINTS)
        with self._lock:
            msg.position = [self._latest_positions[joint] for joint in ALL_JOINTS]
        self._joint_state_pub.publish(msg)

    def warn_if_isaac_disconnected(self):
        if self._joint_command_pub is None:
            return

        now = time.monotonic()
        if (
            self._joint_command_pub.get_subscription_count() == 0
            and now - self._last_isaac_warning_time >= 10.0
        ):
            self._last_isaac_warning_time = now
            self.get_logger().warn(
                f"no subscribers on {self._joint_command_topic}; "
                "Isaac Sim is not receiving mirrored commands"
            )

    def publish_isaac_joint_command(self):
        if self._joint_command_pub is None:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ALL_JOINTS)
        with self._lock:
            msg.position = [self._latest_positions[joint] for joint in ALL_JOINTS]
        self._joint_command_pub.publish(msg)

    def goal_callback(self, goal_request, controller_joints, controller_name):
        requested = list(goal_request.trajectory.joint_names)
        if not requested:
            self.get_logger().warn(f"{controller_name}: empty trajectory joint list")
            return GoalResponse.REJECT

        unknown = sorted(set(requested) - set(controller_joints))
        if unknown:
            self.get_logger().warn(
                f"{controller_name}: trajectory contains joints outside controller: {unknown}"
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle, controller_joints, controller_name):
        goal = goal_handle.request
        trajectory = goal.trajectory
        joint_names = list(trajectory.joint_names)
        points = list(trajectory.points)
        result = FollowJointTrajectory.Result()

        if not points:
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        with self._lock:
            previous_positions = [
                self._latest_positions.get(joint, points[0].positions[index])
                for index, joint in enumerate(joint_names)
            ]

        previous_time = 0.0
        for point in points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "Canceled"
                return result

            target_positions = list(point.positions)
            if len(target_positions) != len(joint_names):
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "Trajectory point position count does not match joint count"
                return result

            target_time = duration_to_seconds(point.time_from_start)
            segment_duration = max(0.0, target_time - previous_time)
            self.interpolate_segment(
                goal_handle,
                joint_names,
                previous_positions,
                target_positions,
                segment_duration,
            )

            previous_positions = target_positions
            previous_time = target_time

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info(f"{controller_name}: follower trajectory command complete")
        return result

    def interpolate_segment(
        self, goal_handle, joint_names, start_positions, target_positions, duration
    ):
        if duration <= 0.0:
            self.send_follower_command(joint_names, target_positions)
            self.publish_feedback(goal_handle, joint_names, target_positions)
            return

        start_time = time.monotonic()
        period = 1.0 / self._publish_rate_hz

        while True:
            elapsed = time.monotonic() - start_time
            ratio = min(1.0, elapsed / duration)
            positions = [
                start + (target - start) * ratio
                for start, target in zip(start_positions, target_positions)
            ]
            self.send_follower_command(joint_names, positions)
            self.publish_feedback(goal_handle, joint_names, positions)

            if ratio >= 1.0:
                break

            time.sleep(min(period, max(0.0, duration - elapsed)))

    def send_follower_command(self, joint_names, ros_positions):
        with self._lock:
            for joint, ros_position in zip(joint_names, ros_positions):
                self._latest_positions[joint] = ros_position

            action = {
                ROS_TO_LEROBOT[joint]: self.ros_to_lerobot(
                    joint, self._latest_positions[joint]
                )
                for joint in ALL_JOINTS
            }

            sent_action = self._robot.send_action(action)
            for lerobot_key, value in sent_action.items():
                ros_joint = LEROBOT_TO_ROS.get(lerobot_key)
                if ros_joint is not None:
                    self._latest_positions[ros_joint] = self.lerobot_to_ros(
                        ros_joint, float(value)
                    )

        self.publish_isaac_joint_command()
        self.publish_cached_joint_state()

    def ros_to_lerobot(self, joint, ros_position):
        if joint in GRIPPER_JOINT_LIMITS:
            return self.gripper_ros_to_percent(joint, ros_position)

        sign = self._joint_signs.get(joint, 1.0)
        offset_deg = self._joint_offsets_deg.get(joint, 0.0)
        if self._use_degrees:
            return math.degrees(ros_position) * sign + offset_deg

        return ros_position * sign + math.radians(offset_deg)

    def lerobot_to_ros(self, joint, lerobot_position):
        if joint in GRIPPER_JOINT_LIMITS:
            return self.gripper_percent_to_ros(joint, lerobot_position)

        sign = self._joint_signs.get(joint, 1.0)
        offset_deg = self._joint_offsets_deg.get(joint, 0.0)
        if self._use_degrees:
            return math.radians((lerobot_position - offset_deg) / sign)

        return (lerobot_position - math.radians(offset_deg)) / sign

    def gripper_ros_to_percent(self, joint, ros_position):
        lower, upper = GRIPPER_JOINT_LIMITS[joint]
        clipped = min(max(ros_position, lower), upper)
        ratio = (clipped - lower) / (upper - lower)
        percent = self._gripper_min_percent + ratio * (
            self._gripper_max_percent - self._gripper_min_percent
        )
        min_percent = min(self._gripper_min_percent, self._gripper_max_percent)
        max_percent = max(self._gripper_min_percent, self._gripper_max_percent)
        return min(max(percent, min_percent), max_percent)

    def gripper_percent_to_ros(self, joint, percent):
        lower, upper = GRIPPER_JOINT_LIMITS[joint]
        ratio = (percent - self._gripper_min_percent) / (
            self._gripper_max_percent - self._gripper_min_percent
        )
        ratio = min(max(ratio, 0.0), 1.0)
        return lower + ratio * (upper - lower)

    def publish_feedback(self, goal_handle, joint_names, positions):
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(joint_names)
        feedback.actual.positions = list(positions)
        feedback.desired.positions = list(positions)
        feedback.error.positions = [0.0 for _ in positions]
        goal_handle.publish_feedback(feedback)

    def destroy_node(self):
        try:
            if getattr(self, "_robot", None) is not None and self._robot.is_connected:
                self._robot.disconnect()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = FollowerFollowJointTrajectory()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
