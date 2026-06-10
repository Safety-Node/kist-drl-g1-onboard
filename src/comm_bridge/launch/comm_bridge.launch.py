"""
Launches outbound and inbound relay nodes.

Loader strategy (revised 2026-05-16):
The relay table is a list-of-dicts (src/dst/type/qos), which ROS 2
parameters cannot represent -- rclpy raises InvalidParameterTypeException
on declare_parameter for list-of-dict. So each relay node loads
comm_bridge_params.yaml directly from its package share path at startup
(see outbound_relay.py / inbound_relay.py docstrings).

Hence NO `parameters=[...]` here. If we passed the yaml as a node
parameter, the node would die in __init__ before the loader could run.
Future scalar-only params (e.g. a single relays_file path) can be added
back as `parameters=[{...}]` if the loader strategy ever splits.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    outbound = Node(
        package='comm_bridge',
        executable='outbound_relay',
        name='outbound_relay',
        output='screen',
    )

    inbound = Node(
        package='comm_bridge',
        executable='inbound_relay',
        name='inbound_relay',
        output='screen',
    )

    return LaunchDescription([outbound, inbound])
