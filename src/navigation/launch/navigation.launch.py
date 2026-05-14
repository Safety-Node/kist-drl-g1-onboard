"""navigation.launch.py — launches the UWB-based goto_node.

The Nav2 stack (map_server / amcl / planner / controller / bt_navigator) has been
removed per the 2026-05-14 spec change. See package.xml description for context.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('navigation')
    params_file = os.path.join(pkg_share, 'config', 'goto_params.yaml')

    return LaunchDescription([
        Node(
            package='navigation',
            executable='goto_node',
            name='goto_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
