"""Display the camera configuration using the shared scorpion_description mesh package."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    directory = Path(__file__).resolve().parent
    urdf = (directory / "scorpion_ros.urdf").read_text(encoding="utf-8")
    gui = LaunchConfiguration("gui")
    publish = LaunchConfiguration("publish_joint_states")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui", default_value="true", description="Open RViz and the joint controls"
            ),
            DeclareLaunchArgument(
                "publish_joint_states",
                default_value="true",
                description="Publish local joint states for standalone viewing",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": ParameterValue(urdf, value_type=str)}],
            ),
            GroupAction(
                condition=IfCondition(publish),
                actions=[
                    Node(
                        package="joint_state_publisher_gui",
                        executable="joint_state_publisher_gui",
                        condition=IfCondition(gui),
                    ),
                    Node(
                        package="joint_state_publisher",
                        executable="joint_state_publisher",
                        condition=UnlessCondition(gui),
                    ),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(directory / "scorpion.rviz")],
                condition=IfCondition(gui),
            ),
        ]
    )
