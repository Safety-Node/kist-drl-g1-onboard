"""
G1 IMU streaming — base + left/right ankle.

Owns all IMU outputs as a single concern. 2026-05-23 refactor: base IMU
moved out of joint_state_node (which now owns JointState only). Both nodes
subscribe to G1 SDK lowstate independently; DDS multi-subscriber cost is
negligible at 100 Hz.

Publications (all sensor_msgs/Imu, BestEffort QoS, ~100 Hz):
  /onboard/sensors/imu/base         frame_id = base_link     — real data
  /onboard/sensors/imu/ankle_left   frame_id = ankle_left_link  — placeholder
  /onboard/sensors/imu/ankle_right  frame_id = ankle_right_link — placeholder

Ankle IMU source is unconfirmed (REQ-42 / TASK-38 open item). Placeholder
publishes zero quaternion + zero vectors so that comm_bridge relay and PC
subscribers can be exercised end-to-end. covariance[0] = -1 signals
"measurement not available" per REP 145.

SDK / ROS 2 CycloneDDS coexistence
-----------------------------------
rmw_cyclonedds_cpp creates CycloneDDS domain 0 when rclpy.init() is called.
ChannelFactory().Init() also calls Domain(id, config) which fails with
DDSException if the domain already exists.  _patch_channel_factory() below
replaces Init() with a version that skips the explicit Domain() creation and
instead creates a DomainParticipant directly on the already-active domain.

imu_node is the ONLY node that needs eth0 + robot peer to reach the G1 SDK.
Adding the robot IP as a unicast peer in the shared cyclonedds.xml would cause
all domain 0 nodes to probe robot ports 7410-7448 simultaneously → EAGAIN storm.
Instead, main() overrides CYCLONEDDS_URI in-process with an imu_node-specific
XML (lo + eth0 + robot peer, MaxAutoParticipantIndex=5) before rclpy.init().
This affects only this process; all other sensor nodes use cyclonedds.xml Domain 0
(lo-only, no robot peer).  G1_ROBOT_IP and G1_NETWORK_IFACE env vars override
the defaults (192.168.123.161 / eth0).  (TASK-32 tracks proper SDK/ROS 2 isolation.)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

import unitree_sdk2py.core.channel as _sdk_ch
from unitree_sdk2py.core.channel import ChannelFactory, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_


def _patch_channel_factory() -> None:
    """Replace ChannelFactory.Init to skip Domain() creation.

    CycloneDDS rejects dds_create_domain() when the domain already exists
    (rmw_cyclonedds_cpp owns it).  We skip straight to DomainParticipant(),
    which succeeds on any already-live domain.
    """
    from cyclonedds.domain import DomainParticipant as _DDP

    def _init(self, id: int, networkInterface=None, qos=None) -> bool:
        if self.__class__._ChannelFactory__initialized:
            return True
        with self.__class__._ChannelFactory__init_lock:
            if self.__class__._ChannelFactory__initialized:
                return True
            try:
                self.__class__._ChannelFactory__participant = _DDP(id)
            except Exception:
                return False
            self.__class__._ChannelFactory__qos = qos
            self.__class__._ChannelFactory__initialized = True
            return True

    _sdk_ch.ChannelFactory.Init = _init


_patch_channel_factory()


_BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# REP 145: covariance[0] == -1  →  measurement not available.
_PLACEHOLDER_COV = [-1.0] + [0.0] * 8


class ImuNode(Node):
    def __init__(self) -> None:
        super().__init__('imu_node')

        # Parameters
        self.declare_parameter('publish_rate_hz', 100)
        self.declare_parameter('imu_base_frame_id', 'base_link')
        self.declare_parameter('imu_ankle_left_frame_id', 'ankle_left_link')
        self.declare_parameter('imu_ankle_right_frame_id', 'ankle_right_link')
        self.declare_parameter('network_interface', 'eth0')
        self.declare_parameter('domain_id', 0)

        rate_hz: int = self.get_parameter('publish_rate_hz').value
        self._frame_base: str = self.get_parameter('imu_base_frame_id').value
        self._frame_ankle_l: str = self.get_parameter('imu_ankle_left_frame_id').value
        self._frame_ankle_r: str = self.get_parameter('imu_ankle_right_frame_id').value
        network_iface: str = self.get_parameter('network_interface').value
        domain_id: int = self.get_parameter('domain_id').value

        # SDK init — must follow super().__init__() / rclpy.init()
        ChannelFactory().Init(domain_id, network_iface)

        # Publishers
        self._pub_base = self.create_publisher(
            Imu, '/onboard/sensors/imu/base', _BEST_EFFORT_QOS)
        self._pub_ankle_l = self.create_publisher(
            Imu, '/onboard/sensors/imu/ankle_left', _BEST_EFFORT_QOS)
        self._pub_ankle_r = self.create_publisher(
            Imu, '/onboard/sensors/imu/ankle_right', _BEST_EFFORT_QOS)

        # Latest lowstate — written by SDK callback, read by timer
        self._lowstate: LowState_ | None = None

        # G1 SDK lowstate subscriber (independent from joint_state_node)
        self._sub = ChannelSubscriber('rt/lf/lowstate', LowState_)
        self._sub.Init(self._on_lowstate, 10)

        # Publish timer
        period = 1.0 / rate_hz
        self._timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            f'imu_node ready — {rate_hz} Hz, iface={network_iface}, domain={domain_id}')

    # ------------------------------------------------------------------ #
    # SDK callback                                                         #
    # ------------------------------------------------------------------ #

    def _on_lowstate(self, msg: LowState_) -> None:
        self._lowstate = msg

    # ------------------------------------------------------------------ #
    # Timer callback                                                       #
    # ------------------------------------------------------------------ #

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._publish_base(stamp)
        self._publish_ankle_placeholder(stamp, self._pub_ankle_l, self._frame_ankle_l)
        self._publish_ankle_placeholder(stamp, self._pub_ankle_r, self._frame_ankle_r)

    def _publish_base(self, stamp: Time) -> None:
        msg = Imu()
        msg.header = Header(stamp=stamp, frame_id=self._frame_base)

        if self._lowstate is None:
            # No SDK data yet — emit placeholder so topic is visible
            msg.orientation_covariance = list(_PLACEHOLDER_COV)
            msg.angular_velocity_covariance = list(_PLACEHOLDER_COV)
            msg.linear_acceleration_covariance = list(_PLACEHOLDER_COV)
            self._pub_base.publish(msg)
            return

        imu = self._lowstate.imu_state

        # G1 SDK quaternion order: [w, x, y, z]
        msg.orientation.w = float(imu.quaternion[0])
        msg.orientation.x = float(imu.quaternion[1])
        msg.orientation.y = float(imu.quaternion[2])
        msg.orientation.z = float(imu.quaternion[3])
        msg.orientation_covariance = [0.0] * 9

        msg.angular_velocity.x = float(imu.gyroscope[0])
        msg.angular_velocity.y = float(imu.gyroscope[1])
        msg.angular_velocity.z = float(imu.gyroscope[2])
        msg.angular_velocity_covariance = [0.0] * 9

        msg.linear_acceleration.x = float(imu.accelerometer[0])
        msg.linear_acceleration.y = float(imu.accelerometer[1])
        msg.linear_acceleration.z = float(imu.accelerometer[2])
        msg.linear_acceleration_covariance = [0.0] * 9

        self._pub_base.publish(msg)

    def _publish_ankle_placeholder(
        self,
        stamp: Time,
        pub,
        frame_id: str,
    ) -> None:
        # TODO(REQ-42) [TASK-38]: replace with real ankle IMU once SDK source confirmed.
        msg = Imu()
        msg.header = Header(stamp=stamp, frame_id=frame_id)
        msg.orientation.w = 1.0  # identity quaternion
        msg.orientation_covariance = list(_PLACEHOLDER_COV)
        msg.angular_velocity_covariance = list(_PLACEHOLDER_COV)
        msg.linear_acceleration_covariance = list(_PLACEHOLDER_COV)
        pub.publish(msg)


def main(args=None) -> None:
    import os
    import textwrap

    # Override CYCLONEDDS_URI for this process only so the G1 SDK participant
    # (domain 0) can discover the robot over eth0 via unicast.  MaxAutoParticipant
    # Index=5 limits probes to 6 ports (7410-7420), avoiding the EAGAIN storm that
    # the global cyclonedds.xml robot peer caused on all domain 0 nodes.
    robot_ip = os.getenv('G1_ROBOT_IP', '192.168.123.161')
    iface    = os.getenv('G1_NETWORK_IFACE', 'eth0')
    os.environ['CYCLONEDDS_URI'] = textwrap.dedent(f"""\
        <CycloneDDS xmlns="https://cdds.io/config">
          <Domain Id="0">
            <General>
              <Interfaces>
                <NetworkInterface name="lo"      multicast="false"/>
                <NetworkInterface name="{iface}" multicast="false"/>
              </Interfaces>
              <AllowMulticast>false</AllowMulticast>
            </General>
            <Discovery>
              <Peers>
                <Peer address="localhost"/>
                <Peer address="{robot_ip}"/>
              </Peers>
              <ParticipantIndex>auto</ParticipantIndex>
              <MaxAutoParticipantIndex>5</MaxAutoParticipantIndex>
            </Discovery>
          </Domain>
        </CycloneDDS>""")

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
