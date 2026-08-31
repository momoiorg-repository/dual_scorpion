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
            DeclareLaunchArgument("lerobot_root", default_value=""),
            DeclareLaunchArgument("left_arm_port", default_value=""),
            DeclareLaunchArgument("right_arm_port", default_value=""),
            DeclareLaunchArgument("robot_id", default_value=""),
            DeclareLaunchArgument("calibration_dir", default_value=""),
            DeclareLaunchArgument("calibrate_on_connect", default_value="false"),
            DeclareLaunchArgument("use_degrees", default_value="true"),
            DeclareLaunchArgument("disable_torque_on_disconnect", default_value="true"),
            DeclareLaunchArgument("max_relative_target", default_value=""),
            DeclareLaunchArgument("publish_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("publish_follower_joint_states", default_value="false"),
            DeclareLaunchArgument("subscribe_external_joint_states", default_value="true"),
            DeclareLaunchArgument("joint_states_topic", default_value="joint_states"),
            DeclareLaunchArgument(
                "joint_signs",
                default_value='{"revolute_4": -1}',
            ),
            DeclareLaunchArgument("joint_offsets_deg", default_value=""),
            DeclareLaunchArgument("gripper_min_percent", default_value="0.0"),
            DeclareLaunchArgument("gripper_max_percent", default_value="100.0"),
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
                executable="follower_follow_joint_trajectory.py",
                name="isaac_follower_follow_joint_trajectory",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "joint_command_topic": LaunchConfiguration("joint_command_topic"),
                        "publish_isaac_joint_commands": True,
                        "publish_follower_joint_states": ParameterValue(
                            LaunchConfiguration("publish_follower_joint_states"),
                            value_type=bool,
                        ),
                        "subscribe_external_joint_states": ParameterValue(
                            LaunchConfiguration("subscribe_external_joint_states"),
                            value_type=bool,
                        ),
                        "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                        "lerobot_root": LaunchConfiguration("lerobot_root"),
                        "left_arm_port": LaunchConfiguration("left_arm_port"),
                        "right_arm_port": LaunchConfiguration("right_arm_port"),
                        "robot_id": LaunchConfiguration("robot_id"),
                        "calibration_dir": LaunchConfiguration("calibration_dir"),
                        "calibrate_on_connect": ParameterValue(
                            LaunchConfiguration("calibrate_on_connect"), value_type=bool
                        ),
                        "use_degrees": ParameterValue(
                            LaunchConfiguration("use_degrees"), value_type=bool
                        ),
                        "disable_torque_on_disconnect": ParameterValue(
                            LaunchConfiguration("disable_torque_on_disconnect"),
                            value_type=bool,
                        ),
                        "max_relative_target": ParameterValue(
                            LaunchConfiguration("max_relative_target"), value_type=str
                        ),
                        "publish_rate_hz": ParameterValue(
                            LaunchConfiguration("publish_rate_hz"), value_type=float
                        ),
                        "joint_signs": ParameterValue(
                            LaunchConfiguration("joint_signs"), value_type=str
                        ),
                        "joint_offsets_deg": ParameterValue(
                            LaunchConfiguration("joint_offsets_deg"), value_type=str
                        ),
                        "gripper_min_percent": ParameterValue(
                            LaunchConfiguration("gripper_min_percent"), value_type=float
                        ),
                        "gripper_max_percent": ParameterValue(
                            LaunchConfiguration("gripper_max_percent"), value_type=float
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
