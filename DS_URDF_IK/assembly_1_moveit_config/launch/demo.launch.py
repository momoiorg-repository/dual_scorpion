from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "assembly_1", package_name="assembly_1_moveit_config"
    ).to_moveit_configs()

    move_group_parameters = [
        moveit_config.to_dict(),
        {
            "use_sim_time": False,
            "allow_trajectory_execution": True,
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
            "monitor_dynamics": False,
        },
    ]

    rviz_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.joint_limits,
        {"use_sim_time": False},
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                output="log",
                arguments=["--frame-id", "world", "--child-frame-id", "root"],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[moveit_config.robot_description, {"use_sim_time": False}],
            ),
            Node(
                package="assembly_1_moveit_config",
                executable="fake_follow_joint_trajectory.py",
                name="fake_follow_joint_trajectory",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                output="screen",
                parameters=move_group_parameters,
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=[
                    "-d",
                    str(moveit_config.package_path / "config" / "moveit.rviz"),
                ],
                parameters=rviz_parameters,
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
