"""
Launches all onboard sensor nodes.

Composition:
- camera_node       realsense2_camera (external C++ driver via IncludeLaunchDescription)  [REQ-42]
- mic_node          sensors.mic_node                                  [REQ-42, REQ-27]
- speaker_node      sensors.speaker_node                              [REQ-29]
- joint_state_node  sensors.joint_state_node (publishes JointState + Imu)  [REQ-42]
- uwb_node          sensors.uwb_node                                  [REQ-37]

Removed (per spec change 2026-05-14):
- lidar_node: Livox MID-360 dropped in favour of UWB absolute localisation.

Environment variables:
  SENSORS_REQUIRE_CAMERA=1       hard-fail the launch if realsense2_camera is
                                 not installed (default: warn and continue).
  SENSORS_REQUIRE_VALID_PARAMS=1 hard-fail the launch if sensors_params.yaml
                                 cannot be parsed (default: warn and fall back
                                 to camera defaults). Recommended for the
                                 deployed NX so a malformed yaml doesn't
                                 silently boot the wrong profile.

Camera parameters (resolution / fps) are read from sensors_params.yaml so
the YAML stays the single source of truth and the launch file does not
duplicate those values.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml


def _load_camera_params(params_file: str) -> dict:
    """Read sensors_params.yaml and return the camera_node ros__parameters block.

    Failure modes (missing file, bad yaml, permission denied) print a visible
    warning so launch-time silence does not mask a misconfigured deployment.
    Set SENSORS_REQUIRE_VALID_PARAMS=1 to promote the warning to a hard fail.
    """
    require_valid = os.environ.get('SENSORS_REQUIRE_VALID_PARAMS', '0') == '1'
    try:
        with open(params_file) as f:
            doc = yaml.safe_load(f) or {}
        return doc.get('camera_node', {}).get('ros__parameters', {})
    except Exception as e:
        msg = (
            f'[sensors.launch.py] WARNING: failed to read camera params from '
            f'{params_file}: {type(e).__name__}: {e}'
        )
        if require_valid:
            raise RuntimeError(
                f'SENSORS_REQUIRE_VALID_PARAMS=1 but {msg}') from e
        print(msg + ' -- falling back to camera defaults.', flush=True)
        return {}


def _format_profile(w, h, fps) -> str:
    return f'{int(w)}x{int(h)}x{int(fps)}'


def generate_launch_description():
    pkg_share = get_package_share_directory('sensors')
    params_file = os.path.join(pkg_share, 'config', 'sensors_params.yaml')

    require_camera = os.environ.get('SENSORS_REQUIRE_CAMERA', '0') == '1'

    # ---------------------------------------------------------------
    # External driver: Intel RealSense D435i
    # Profile args pulled from sensors_params.yaml.
    # ---------------------------------------------------------------
    external_nodes = []
    try:
        realsense_share = get_package_share_directory('realsense2_camera')
        realsense_launch_file = os.path.join(
            realsense_share, 'launch', 'rs_launch.py')

        cam = _load_camera_params(params_file)
        color_profile = _format_profile(
            cam.get('color_width', 1280),
            cam.get('color_height', 720),
            cam.get('color_fps', 30),
        )
        depth_profile = _format_profile(
            cam.get('depth_width', 1280),
            cam.get('depth_height', 720),
            cam.get('depth_fps', 30),
        )

        # Resulting topic names (verify at first integration with `ros2 topic list`):
        #   /onboard/sensors/color/image_raw/compressed
        #   /onboard/sensors/depth/image_raw
        # comm_bridge_params.yaml expects exactly these names. realsense2_camera
        # builds topics as "{camera_namespace}/{camera_name}/<topic>" -- the
        # earlier setting camera_name='onboard_camera' produced an extra
        # "/onboard_camera/" segment that didn't match the relay table, so we
        # drop the per-camera name and keep the prefix flat under
        # /onboard/sensors. We are single-camera, so the disambiguator brings
        # no value here.
        # TODO(REQ-42): if a second camera is ever added, give each a non-empty
        #               camera_name and update comm_bridge_params.yaml accordingly.
        external_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_file),
            launch_arguments={
                'camera_name':                '',
                'camera_namespace':           '/onboard/sensors',
                'enable_color':               str(cam.get('enable_color', True)).lower(),
                'enable_depth':               str(cam.get('enable_depth', True)).lower(),
                'rgb_camera.color_profile':   color_profile,
                'depth_module.depth_profile': depth_profile,
                # TODO(REQ-42): tune QoS / pointcloud / align_depth_to_color as needs solidify
            }.items(),
        ))
    except Exception as e:
        warn = (
            f'realsense2_camera not found — camera disabled. '
            f'Set SENSORS_REQUIRE_CAMERA=1 to enforce. (cause: {e})'
        )
        if require_camera:
            raise RuntimeError(f'SENSORS_REQUIRE_CAMERA=1 but {warn}') from e
        external_nodes = [LogInfo(msg=f'[sensors.launch.py] WARNING: {warn}')]

    # ---------------------------------------------------------------
    # Custom Python nodes
    # ---------------------------------------------------------------
    mic_node = Node(
        package='sensors',
        executable='mic_node',
        name='mic_node',
        output='screen',
        parameters=[params_file],
    )

    speaker_node = Node(
        package='sensors',
        executable='speaker_node',
        name='speaker_node',
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
        mic_node,
        speaker_node,
        joint_state_node,
        uwb_node,
    ])
