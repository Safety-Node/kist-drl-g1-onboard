"""
G1 IMU streaming — base + left/right ankle.

Owns all IMU outputs as a single concern. 2026-05-23 refactor reverses the
2026-05-15 fan-out decision (joint_state_node previously published base IMU
as a side-effect of its lowstate subscription). Now:
- joint_state_node owns JointState only.
- imu_node owns the full IMU surface (base + ankles).
Both nodes subscribe to G1 SDK lowstate independently; DDS multi-subscriber
cost is negligible at the 100 Hz lowstate rate, and ownership becomes
symmetric with the topic prefix (node name ↔ /onboard/sensors/imu/*).

Publications (all sensor_msgs/Imu, BestEffort QoS):
- /onboard/sensors/imu/base         frame_id = base_link  (real data from lowstate)
- /onboard/sensors/imu/ankle_left   frame_id = ankle_left_link   (placeholder — see below)
- /onboard/sensors/imu/ankle_right  frame_id = ankle_right_link  (placeholder — see below)

Ankle IMU source is unconfirmed for G1 SDK. Placeholder messages carry
orientation_covariance[0] = -1 (REP 145 convention: measurement unavailable)
so downstream consumers (GearSonic) can detect and reject them.
Swap placeholder for real reads once SDK source is confirmed (REQ-42).

SDK init order (IMPORTANT): ChannelFactory.Instance().Init() must be called
before rclpy.init(). main() pre-parses sensors_params.yaml to extract
network_interface and domain_id before the ROS node is constructed.

TODO(REQ-42): confirm G1 SDK ankle IMU channel; swap placeholder for real read.
TODO(REQ-42): paired timestamp if both ankles share a frame from the same SDK sample.
"""
import os
import threading

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

_SDK_AVAILABLE = False
try:
    from unitree_sdk2py.core.channel import ChannelFactory, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
    _SDK_AVAILABLE = True
except ImportError:
    pass

# BestEffort QoS — freshness wins for high-rate sensor streams (CONV-001).
_QOS_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# covariance[0] = -1 signals "measurement unavailable" per REP 145.
_COV_UNAVAILABLE = [-1.0] + [0.0] * 8


def _build_imu_placeholder(frame_id: str, stamp) -> Imu:
    """Return an Imu message with identity orientation and zero vectors.

    All three covariance matrices have [0] = -1 to signal that the
    measurement is not available (REP 145).  Downstream consumers
    (GearSonic) must check covariance[0] before trusting the data.
    """
    msg = Imu()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.orientation.w = 1.0  # identity quaternion
    msg.orientation_covariance = _COV_UNAVAILABLE
    msg.angular_velocity_covariance = _COV_UNAVAILABLE
    msg.linear_acceleration_covariance = _COV_UNAVAILABLE
    return msg


class ImuNode(Node):
    def __init__(self) -> None:
        super().__init__('imu_node')

        # --- Parameters -------------------------------------------------------
        self.declare_parameter('publish_rate_hz', 100)
        self.declare_parameter('imu_base_frame_id', 'base_link')
        self.declare_parameter('imu_ankle_left_frame_id', 'ankle_left_link')
        self.declare_parameter('imu_ankle_right_frame_id', 'ankle_right_link')
        # Informational mirrors of the values used for SDK init in main().
        self.declare_parameter('network_interface', 'eth0')
        self.declare_parameter('domain_id', 0)

        rate = self.get_parameter('publish_rate_hz').value
        self._base_frame = self.get_parameter('imu_base_frame_id').value
        self._ankle_l_frame = self.get_parameter('imu_ankle_left_frame_id').value
        self._ankle_r_frame = self.get_parameter('imu_ankle_right_frame_id').value

        # --- Publishers -------------------------------------------------------
        self._pub_base = self.create_publisher(
            Imu, '/onboard/sensors/imu/base', _QOS_SENSOR)
        self._pub_ankle_l = self.create_publisher(
            Imu, '/onboard/sensors/imu/ankle_left', _QOS_SENSOR)
        self._pub_ankle_r = self.create_publisher(
            Imu, '/onboard/sensors/imu/ankle_right', _QOS_SENSOR)

        # --- SDK lowstate subscriber ------------------------------------------
        self._latest_lowstate = None
        self._lowstate_lock = threading.Lock()

        if _SDK_AVAILABLE:
            try:
                sub = ChannelSubscriber('rt/lf/lowstate', unitree_go_msg_dds__LowState_)
                sub.Init(self._on_lowstate, 10)
                self._lowstate_sub = sub  # keep reference so GC doesn't collect it
                self.get_logger().info('imu_node: G1 SDK lowstate subscriber active')
            except Exception as exc:
                self.get_logger().error(
                    f'imu_node: failed to init lowstate subscriber: {exc} — '
                    'base IMU will publish placeholder until resolved')
        else:
            self.get_logger().error(
                'imu_node: unitree_sdk2py not available — '
                'base IMU will publish placeholder (orientation_covariance[0]=-1)')

        # --- Publish timer ----------------------------------------------------
        period = 1.0 / float(rate)
        self._timer = self.create_timer(period, self._publish)
        self.get_logger().info(
            f'imu_node started: {rate} Hz, base_frame={self._base_frame}')

    def _on_lowstate(self, msg) -> None:
        """SDK DDS callback — runs on the SDK thread; cache latest sample."""
        with self._lowstate_lock:
            self._latest_lowstate = msg

    def _publish(self) -> None:
        """Timer callback — publish all 3 IMU topics at publish_rate_hz."""
        now = self.get_clock().now().to_msg()

        # Base IMU — real data when available; skip until first sample.
        with self._lowstate_lock:
            ls = self._latest_lowstate

        if ls is not None:
            imu_state = ls.imu_state
            base_msg = Imu()
            base_msg.header.stamp = now
            base_msg.header.frame_id = self._base_frame

            # G1 SDK quaternion order: [w, x, y, z]
            q = imu_state.quaternion
            base_msg.orientation.w = float(q[0])
            base_msg.orientation.x = float(q[1])
            base_msg.orientation.y = float(q[2])
            base_msg.orientation.z = float(q[3])

            g = imu_state.gyroscope
            base_msg.angular_velocity.x = float(g[0])
            base_msg.angular_velocity.y = float(g[1])
            base_msg.angular_velocity.z = float(g[2])

            a = imu_state.accelerometer
            base_msg.linear_acceleration.x = float(a[0])
            base_msg.linear_acceleration.y = float(a[1])
            base_msg.linear_acceleration.z = float(a[2])

            # Noise covariance uncharacterised; zeros = present but unknown.
            base_msg.orientation_covariance = [0.0] * 9
            base_msg.angular_velocity_covariance = [0.0] * 9
            base_msg.linear_acceleration_covariance = [0.0] * 9

            self._pub_base.publish(base_msg)
        # else: no sample yet — do not publish stale/zero base IMU.

        # Ankle IMUs — placeholder until SDK source confirmed (REQ-42).
        self._pub_ankle_l.publish(
            _build_imu_placeholder(self._ankle_l_frame, now))
        self._pub_ankle_r.publish(
            _build_imu_placeholder(self._ankle_r_frame, now))


def _read_sdk_params_from_yaml() -> tuple[str, int]:
    """Pre-parse sensors_params.yaml for SDK init before rclpy.init().

    Returns (network_interface, domain_id).  Falls back to safe defaults
    so SDK init never blocks on a missing file.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        params_file = os.path.join(
            get_package_share_directory('sensors'), 'config', 'sensors_params.yaml')
        with open(params_file) as f:
            doc = yaml.safe_load(f) or {}
        p = doc.get('imu_node', {}).get('ros__parameters', {})
        return str(p.get('network_interface', 'eth0')), int(p.get('domain_id', 0))
    except Exception:
        return 'eth0', 0


def main(args=None) -> None:
    # SDK init MUST precede rclpy.init() to avoid DDS domain conflicts.
    if _SDK_AVAILABLE:
        network_interface, domain_id = _read_sdk_params_from_yaml()
        try:
            ChannelFactory.Instance().Init(domain_id, network_interface)
        except Exception as exc:
            # Non-fatal: node will warn at runtime and publish placeholders.
            import sys
            print(f'[imu_node] WARNING: SDK ChannelFactory.Init failed: {exc}',
                  file=sys.stderr)

    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
