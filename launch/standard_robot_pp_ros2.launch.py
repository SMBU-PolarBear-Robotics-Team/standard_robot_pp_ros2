import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("standard_robot_pp_ros2"),
        "config",
        "standard_robot_pp_ros2.yaml",
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    standard_robot_pp_ros2_node = Node(
        package="standard_robot_pp_ros2",
        executable="standard_robot_pp_ros2_node",
        name="standard_robot_pp_ros2",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[config],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Add the actions to launch all of nodes
    ld.add_action(standard_robot_pp_ros2_node)

    return ld
