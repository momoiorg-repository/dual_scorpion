#!/usr/bin/env python3
import time
from threading import Lock

import rclpy
from control_msgs.action import FollowJointTrajectory
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
FALLBACK_JOINT_STATE_FRAME = "isaac_follow_joint_trajectory_fallback"


def duration_to_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


class IsaacFollowJointTrajectory(Node):
    def __init__(self):
        super().__init__("isaac_follow_joint_trajectory")

        self.declare_parameter("joint_command_topic", "/joint_command")
        self.declare_parameter("publish_rate_hz", 60.0)
        self.declare_parameter("publish_fallback_joint_states", False)

        self._joint_command_topic = (
            self.get_parameter("joint_command_topic").get_parameter_value().string_value
        )
        self._publish_rate_hz = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        self._publish_fallback_joint_states = (
            self.get_parameter("publish_fallback_joint_states")
            .get_parameter_value()
            .bool_value
        )
        if self._publish_rate_hz <= 0.0:
            self._publish_rate_hz = 60.0

        self._latest_positions = {joint: 0.0 for joint in ALL_JOINTS}
        self._last_external_joint_state_time = None
        self._last_connectivity_warning_time = 0.0
        self._lock = Lock()
        self._joint_command_pub = self.create_publisher(
            JointState, self._joint_command_topic, 10
        )
        self._joint_state_sub = self.create_subscription(
            JointState,
            "joint_states",
            self.joint_state_callback,
            qos_profile_sensor_data,
        )
        self._fallback_joint_state_pub = None
        if self._publish_fallback_joint_states:
            self._fallback_joint_state_pub = self.create_publisher(
                JointState, "joint_states", qos_profile_sensor_data
            )
            self.create_timer(
                1.0 / min(max(self._publish_rate_hz, 1.0), 60.0),
                self.publish_fallback_joint_state,
            )
        self.create_timer(5.0, self.warn_if_disconnected)
        self._action_servers = []

        for controller_name, joints in CONTROLLERS.items():
            action_name = f"{controller_name}/follow_joint_trajectory"
            self._action_servers.append(
                ActionServer(
                    self,
                    FollowJointTrajectory,
                    action_name,
                    goal_callback=lambda goal, js=joints, name=controller_name: self.goal_callback(
                        goal, js, name
                    ),
                    cancel_callback=self.cancel_callback,
                    execute_callback=lambda handle, js=joints, name=controller_name: self.execute_callback(
                        handle, js, name
                    ),
                )
            )
            self.get_logger().info(f"Isaac trajectory bridge ready: {action_name}")

        self.get_logger().info(
            f"Publishing Isaac joint commands to {self._joint_command_topic}"
        )
        if self._publish_fallback_joint_states:
            self.get_logger().info(
                "Publishing stamped fallback joint_states for MoveIt execution validation"
            )

    def joint_state_callback(self, msg):
        if msg.header.frame_id == FALLBACK_JOINT_STATE_FRAME:
            return

        self._last_external_joint_state_time = time.monotonic()
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                self._latest_positions[name] = position

    def warn_if_disconnected(self):
        warnings = []
        if self._joint_command_pub.get_subscription_count() == 0:
            warnings.append(
                f"no subscribers on {self._joint_command_topic}; Isaac Sim is not receiving commands"
            )

        now = time.monotonic()
        if (
            self._last_external_joint_state_time is None
            or now - self._last_external_joint_state_time > 5.0
        ):
            warnings.append(
                "no recent external /joint_states received; Isaac Sim is not publishing robot state"
            )

        if warnings and now - self._last_connectivity_warning_time >= 10.0:
            self._last_connectivity_warning_time = now
            self.get_logger().warn("; ".join(warnings))

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
        self.get_logger().info(f"{controller_name}: Isaac trajectory command complete")
        return result

    def interpolate_segment(
        self, goal_handle, joint_names, start_positions, target_positions, duration
    ):
        if duration <= 0.0:
            self.publish_command(joint_names, target_positions)
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
            self.publish_command(joint_names, positions)
            self.publish_feedback(goal_handle, joint_names, positions)

            if ratio >= 1.0:
                break

            time.sleep(min(period, max(0.0, duration - elapsed)))

    def publish_command(self, joint_names, positions):
        with self._lock:
            for name, position in zip(joint_names, positions):
                self._latest_positions[name] = position
            command_positions = [
                self._latest_positions[joint] for joint in ALL_JOINTS
            ]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ALL_JOINTS)
        msg.position = command_positions
        self._joint_command_pub.publish(msg)

    def publish_fallback_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FALLBACK_JOINT_STATE_FRAME
        msg.name = list(ALL_JOINTS)

        with self._lock:
            msg.position = [self._latest_positions[joint] for joint in ALL_JOINTS]

        self._fallback_joint_state_pub.publish(msg)

    def publish_feedback(self, goal_handle, joint_names, positions):
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(joint_names)
        feedback.actual.positions = list(positions)
        feedback.desired.positions = list(positions)
        feedback.error.positions = [0.0 for _ in positions]
        goal_handle.publish_feedback(feedback)


def main():
    rclpy.init()
    node = IsaacFollowJointTrajectory()
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
