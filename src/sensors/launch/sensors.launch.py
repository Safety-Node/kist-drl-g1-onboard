"""
Launches onboard sensor nodes.

Composition:
- camera_node       sensors.camera_node (pyrealsense2 Python driver) [TASK-32]
- mic_node          sensors.mic_node                                  [TASK-36]
- speaker_node      sensors.speaker_node                              [TASK-31]
- joint_state_node  sensors.joint_state_node                          [TASK-37]
- imu_node          sensors.imu_node                                  [TASK-38]
- uwb_node          sensors.uwb_node                                  [TASK-30]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sensors')
    params_file = os.path.join(pkg_share, 'config', 'sensors_params.yaml')

    # ---------------------------------------------------------------
    # Intel RealSense D435i — Python node via pyrealsense2 SDK
    # ---------------------------------------------------------------
    camera_node = Node(
        package='sensors',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[params_file],
    )

    # ---------------------------------------------------------------
    # Custom Python nodes
    # ---------------------------------------------------------------
    # [TASK-31]
    mic_node = Node(
        package='sensors',
        executable='mic_node',
        name='mic_node',
        output='screen',
        parameters=[params_file],
    )

    # [TASK-36]
    speaker_node = Node(
        package='sensors',
        executable='speaker_node',
        name='speaker_node',
        output='screen',
        parameters=[params_file],
    )

    # [TASK-37]
    joint_state_node = Node(
        package='sensors',
        executable='joint_state_node',
        name='joint_state_node',
        output='screen',
        parameters=[params_file],
    )

    # [TASK-38] 2026-05-22 KIST mail — IMU (base + ankle L/R) for VLA + GearSonic.
    #           2026-05-23 refactor: owns all IMU topics (was split with joint_state_node).
    imu_node = Node(
        package='sensors',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[params_file],
    )

    # [TASK-30]
    uwb_node = Node(
        package='sensors',
        executable='uwb_node',
        name='uwb_node',
        output='screen',
        parameters=[params_file],
    )

    # [TASK-50]
    odom_node = Node(
        package='sensors',
        executable='odom_node',
        name='odom_node',
        output='screen',
        parameters=[params_file],
    )

    # [TASK-51]
    location_node = Node(
        package='sensors',
        executable='location_node',
        name='location_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        camera_node,
        mic_node,
        speaker_node,
        joint_state_node,
        imu_node,
        uwb_node,
        odom_node,
        location_node,
    ])
