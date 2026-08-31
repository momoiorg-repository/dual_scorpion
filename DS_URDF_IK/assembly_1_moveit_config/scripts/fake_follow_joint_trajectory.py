#!/usr/bin/env python3
import time
from threading import Lock

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
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

PUBLISH_RATE_HZ = 50.0


def duration_to_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


class FakeFollowJointTrajectory(Node):
    def __init__(self):
        super().__init__("fake_follow_joint_trajectory")
        self._joint_names = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
        self._positions = {joint: 0.0 for joint in self._joint_names}
        self._lock = Lock()
        self._joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_joint_state)
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
            self.get_logger().info(f"Fake controller ready: {action_name}")

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        with self._lock:
            msg.position = [self._positions[joint] for joint in self._joint_names]
        self._joint_state_pub.publish(msg)

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
            previous_positions = [self._positions[joint] for joint in joint_names]

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
        self.get_logger().info(f"{controller_name}: fake trajectory execution complete")
        return result

    def interpolate_segment(
        self, goal_handle, joint_names, start_positions, target_positions, duration
    ):
        if duration <= 0.0:
            self.update_positions(joint_names, target_positions)
            self.publish_feedback(goal_handle, joint_names, target_positions)
            return

        start_time = time.monotonic()
        period = 1.0 / PUBLISH_RATE_HZ

        while True:
            elapsed = time.monotonic() - start_time
            ratio = min(1.0, elapsed / duration)
            positions = [
                start + (target - start) * ratio
                for start, target in zip(start_positions, target_positions)
            ]
            self.update_positions(joint_names, positions)
            self.publish_feedback(goal_handle, joint_names, positions)

            if ratio >= 1.0:
                break

            time.sleep(min(period, max(0.0, duration - elapsed)))

    def update_positions(self, joint_names, positions):
        with self._lock:
            for joint, position in zip(joint_names, positions):
                self._positions[joint] = position

        self.publish_joint_state()

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
    node = FakeFollowJointTrajectory()
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
