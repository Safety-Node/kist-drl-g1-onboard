"""
Launches onboard sensor nodes.

Composition:
- camera_node       realsense2_camera (external C++ driver)  [TASK-32]
- mic_node          sensors.mic_node                          [TASK-36]
- speaker_node      sensors.speaker_node                      [TASK-31]
- joint_state_node  sensors.joint_state_node                  [TASK-37]
- imu_node          sensors.imu_node                          [TASK-38]
- uwb_node          sensors.uwb_node                          [TASK-30]

Env vars:
  SENSORS_REQUIRE_CAMERA=1       hard-fail if realsense2_camera missing.
  SENSORS_REQUIRE_VALID_PARAMS=1 hard-fail if sensors_params.yaml invalid.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml


_CAMERA_INT_KEYS = (
    'color_width', 'color_height', 'color_fps',
    'depth_width', 'depth_height', 'depth_fps',
)


def _load_camera_params(params_file: str) -> dict:
    """Read sensors_params.yaml and return the camera_node ros__parameters block.

    Failure modes (missing file, bad yaml, permission denied, AND non-numeric
    values in numeric fields like color_fps: "thirty") print a visible warning
    so launch-time silence does not mask a misconfigured deployment.
    Set SENSORS_REQUIRE_VALID_PARAMS=1 to promote the warning to a hard fail.

    Int coercion happens inside the try-block so a bad yaml value surfaces here
    and respects the env-var gate, instead of crashing the caller's
    int(cam.get(...)) chain that would bypass the gate.
    """
    require_valid = os.environ.get('SENSORS_REQUIRE_VALID_PARAMS', '0') == '1'
    try:
        with open(params_file) as f:
            doc = yaml.safe_load(f) or {}
        cam = doc.get('camera_node', {}).get('ros__parameters', {}) or {}
        # Coerce numeric fields up-front; if a value is a non-numeric string
        # the ValueError lands in the except below (and the env-var gate applies).
        # Booleans (enable_color, enable_depth) are left to the caller via .lower().
        for key in _CAMERA_INT_KEYS:
            if key in cam:
                cam[key] = int(cam[key])
        return cam
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

        # Topics: /onboard/sensors/camera/{color,depth}/...
        # camera_name must be non-empty: rs_launch.py maps it to the node name
        # (name=LaunchConfiguration('camera_name')), and ROS rejects an empty
        # node name with "Invalid node name: node name must not be empty".
        external_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_file),
            launch_arguments={
                'camera_name':                'camera',
                'camera_namespace':           '/onboard/sensors',
                'enable_color':               str(cam.get('enable_color', True)).lower(),
                'enable_depth':               str(cam.get('enable_depth', True)).lower(),
                'rgb_camera.color_profile':   color_profile,
                'depth_module.depth_profile': depth_profile,
                # Align depth to the color frame for fused color+depth use.
                'align_depth.enable':         'true',
                'pointcloud.enable':          'false',
                'rgb_camera.power_line_frequency': '2',  # 60Hz (Korea); silences range [0,2] warning
                # TODO(REQ-42) [TASK-32]: tune QoS.
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

    return LaunchDescription([
        *external_nodes,
        mic_node,
        speaker_node,
        joint_state_node,
        imu_node,
        uwb_node,
    ])
