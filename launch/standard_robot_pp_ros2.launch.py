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
from sdformat_tools.urdf_generator import UrdfGenerator
from xmacro.xmacro4sdf import XMLMacro4sdf


def generate_launch_description():
    # Get the launch directory
    pkg_standard_robot_pp_ros2_dir = get_package_share_directory(
        "standard_robot_pp_ros2"
    )
    pkg_pb2025_robot_description_dir = get_package_share_directory(
        "pb2025_robot_description"
    )

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    config = os.path.join(
        pkg_standard_robot_pp_ros2_dir,
        "config",
        "standard_robot_pp_ros2.yaml",
    )

    xmacro_description = os.path.join(
        pkg_pb2025_robot_description_dir,
        "resource",
        "xmacro",
        "pb2025_sentry_robot.sdf.xmacro",
    )

    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(xmacro_description)

    # Generate SDF from xmacro
    xmacro.generate()
    robot_xml = xmacro.to_string()

    # Generate URDF from SDF
    urdf_generator = UrdfGenerator()
    urdf_generator.parse_from_sdf_string(robot_xml)
    robot_urdf_xml = urdf_generator.to_string()

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    start_standard_robot_pp_ros2_cmd = Node(
        package="standard_robot_pp_ros2",
        executable="standard_robot_pp_ros2_node",
        name="standard_robot_pp_ros2",
        namespace="",
        output="screen",
        parameters=[config],
    )

    start_joint_state_publisher_cmd = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        namespace="",
        parameters=[{"rate": 200, "source_list": ["serial/gimbal_joint_state"]}],
        output="screen",
    )

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        remappings=remappings,
        parameters=[
            {
                "publish_frequency": 200.0,
                "robot_description": robot_urdf_xml,
            }
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Add the actions to launch all of nodes
    ld.add_action(start_standard_robot_pp_ros2_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
