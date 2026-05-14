"""sensors.launch.py — launches all onboard sensor nodes.

Composition:
  - camera_node       : realsense2_camera (external C++ driver, IncludeLaunchDescription) [REQ-42]
  - audio_node        : sensors.audio_node                                                 [REQ-42 / REQ-27 / REQ-29]
  - joint_state_node  : sensors.joint_state_node                                           [REQ-42]
  - uwb_node          : sensors.uwb_node                                                   [REQ-37]

Removed (per spec change 2026-05-14):
  - lidar_node : Livox MID-360 dropped in favour of UWB absolute localisation.
  - imu_node   : G1 SDK still emits IMU on its lowstate stream; we publish via joint_state_node
                 (or a future imu_node), but no navigation subscriber consumes it for now.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('sensors')
    params_file = os.path.join(pkg_share, 'config', 'sensors_params.yaml')

    # ---------------------------------------------------------------
    # External driver: Intel RealSense D435i
    # Topic remap: realsense's default /camera/color/... → /onboard/sensors/color/...
    # so it stays inside the /onboard/* prefix our DDS partition filter recognises.
    # ---------------------------------------------------------------
    try:
        realsense_share = get_package_share_directory('realsense2_camera')
        realsense_launch_file = os.path.join(
            realsense_share, 'launch', 'rs_launch.py')
        camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_file),
            launch_arguments={
                'camera_name':       'onboard_camera',
                'camera_namespace':  '/onboard/sensors',
                'enable_color':      'true',
                'enable_depth':      'true',
                'rgb_camera.color_profile':  '1280x720x30',
                'depth_module.depth_profile': '1280x720x30',
                # TODO(REQ-42): tune QoS / pointcloud / align_depth_to_color as needs solidify
            }.items(),
        )
        external_nodes = [camera]
    except Exception:
        # Driver not installed on this host (dev-only build). Skip silently.
        # TODO(REQ-42): make this a hard requirement on the target image.
        external_nodes = []

    # ---------------------------------------------------------------
    # Custom Python nodes
    # ---------------------------------------------------------------
    audio_node = Node(
        package='sensors',
        executable='audio_node',
        name='audio_node',
        output='screen',
        parameters=[params_file],
    )

    joint_state_node = Node(
        package='sensors',
        executable='joint_state_node',
        name='joint_state_node',
        output='screen',
        parameters=[params_file],
    )

    uwb_node = Node(
        package='sensors',
        executable='uwb_node',
        name='uwb_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        *external_nodes,
        audio_node,
        joint_state_node,
        uwb_node,
    ])
