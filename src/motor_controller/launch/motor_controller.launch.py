"""
Launches the motor controller node.

NOTE: In production the node is started by systemd (see systemd/motor_controller.service).
This launch file exists for local bring-up / unit testing.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('motor_controller')
    params_file = os.path.join(pkg_share, 'config', 'motor_params.yaml')

    return LaunchDescription([
        Node(
            package='motor_controller',
            executable='motor_controller_node',
            name='motor_controller',
            output='screen',
            parameters=[params_file],
        ),
    ])
