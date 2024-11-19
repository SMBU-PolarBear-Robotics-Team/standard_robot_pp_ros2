import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("standard_robot_pp_ros2"),
        "config",
        "standard_robot_pp_ros2.yaml",
    )

    standard_robot_pp_ros2_node = Node(
        package="standard_robot_pp_ros2",
        executable="standard_robot_pp_ros2_node",
        name="standard_robot_pp_ros2",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription([standard_robot_pp_ros2_node])
