"""comm_bridge.launch.py — launches outbound and inbound relays."""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('comm_bridge')
    params_file = os.path.join(pkg_share, 'config', 'comm_bridge_params.yaml')

    outbound = Node(
        package='comm_bridge',
        executable='outbound_relay',
        name='outbound_relay',
        output='screen',
        parameters=[params_file],
    )

    inbound = Node(
        package='comm_bridge',
        executable='inbound_relay',
        name='inbound_relay',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([outbound, inbound])
