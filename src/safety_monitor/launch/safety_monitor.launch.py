"""
Launches the safety monitor node.

NOTE: In production the node is started by systemd (see systemd/safety_monitor.service).
This launch file exists for local bring-up / unit testing.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('safety_monitor')
    params_file = os.path.join(pkg_share, 'config', 'safety_params.yaml')

    return LaunchDescription([
        Node(
            package='safety_monitor',
            executable='safety_monitor_node',
            name='safety_monitor',
            output='screen',
            parameters=[params_file],
        ),
    ])
