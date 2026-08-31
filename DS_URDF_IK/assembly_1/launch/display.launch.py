from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("assembly_1"))
    urdf_file = package_share / "urdf" / "assembly_1.urdf"
    rviz_config = package_share / "rviz" / "display.rviz"

    robot_description = urdf_file.read_text()

    common_parameters = {
        "robot_description": robot_description,
        "use_sim_time": False,
    }

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[common_parameters],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                parameters=[common_parameters],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(rviz_config)],
                parameters=[{"use_sim_time": False}],
            ),
        ]
    )
