from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "assembly_1", package_name="assembly_1_moveit_config"
    ).to_moveit_configs()

    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    move_group_parameters = [
        moveit_config.to_dict(),
        {
            "use_sim_time": use_sim_time,
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
        {"use_sim_time": use_sim_time},
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("joint_command_topic", default_value="/joint_command"),
            DeclareLaunchArgument("publish_rate_hz", default_value="60.0"),
            DeclareLaunchArgument("publish_fallback_joint_states", default_value="false"),
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
                parameters=[moveit_config.robot_description, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="assembly_1_moveit_config",
                executable="isaac_follow_joint_trajectory.py",
                name="isaac_follow_joint_trajectory",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "joint_command_topic": LaunchConfiguration("joint_command_topic"),
                        "publish_rate_hz": ParameterValue(
                            LaunchConfiguration("publish_rate_hz"), value_type=float
                        ),
                        "publish_fallback_joint_states": ParameterValue(
                            LaunchConfiguration("publish_fallback_joint_states"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                output="screen",
                parameters=move_group_parameters,
            ),
            TimerAction(
                period=3.0,
                actions=[
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
                    )
                ],
            ),
        ]
    )
