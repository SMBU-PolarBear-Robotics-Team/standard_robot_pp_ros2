import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # config = os.path.join(
    #     get_package_share_directory('ros2_standard_robot_pp'), 'config', 'serial_driver.yaml')

    ros2_standard_robot_pp_node = Node(
        package='ros2_standard_robot_pp',
        executable='ros2_standard_robot_pp_node',
        namespace='',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([ros2_standard_robot_pp_node])
