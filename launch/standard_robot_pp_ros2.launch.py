# Copyright 2025 SMBU-PolarBear-Robotics-Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
