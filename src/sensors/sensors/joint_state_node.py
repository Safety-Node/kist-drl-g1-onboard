"""
G1 SDK lowstate → JointState publisher.

2026-05-23 refactor: this node now owns ONLY
/onboard/sensors/joint_states. Base IMU (formerly fanned out here per the
2026-05-15 decision) has moved to `imu_node` for symmetric ownership with
the ankle IMU topics — see REQ-42 v2026-05-23. Both nodes subscribe to
G1 SDK lowstate independently; the DDS multi-subscriber cost is
negligible at the SDK lowstate rate.

Domain: SDK shares ROS 2 domain 0 — _patch_channel_factory() joins the domain
        rclpy already created, so domain_id MUST match the ROS domain. Raw G1
        channels (rt/lf/lowstate, ...) stay visible in ros2 topic list on the NX
        by design; comm_bridge gates the workstation (domain-id-strategy).

Source: unitree_sdk2py ChannelSubscriber on 'rt/lowstate' (unitree_hg.LowState_).
        Callback runs in the SDK thread; the ROS timer publishes the latest
        cached frame at publish_rate_hz. 로봇 없이 돌릴 때는 별도 fake
        publisher 스크립트가 rt/lowstate에 가짜 프레임을 쏘면 됨.
"""

from typing import List, Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

import unitree_sdk2py.core.channel as _sdk_ch
from unitree_sdk2py.core.channel import ChannelFactory, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


def _patch_channel_factory() -> None:
    """Replace ChannelFactory.Init to skip Domain() creation.

    rmw_cyclonedds_cpp and unitree_sdk2py share libddsc.so. Calling
    Domain(0, config) after rclpy.init() raises DDSException because domain 0
    already exists. We join the active domain via DomainParticipant() instead.
    Mirrors imu_node.
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


# Sensor-stream QoS — matches imu_node and the comm_bridge outbound relay
# subscriber (sensor → best_effort, freshness wins; see comm_bridge_params.yaml).
_BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def to_joint_state(lowstate, joint_names: Sequence[str], stamp) -> JointState:
    js = JointState()
    js.header.stamp = stamp
    js.name = list(joint_names)
    js.position = [float(lowstate.motor_state[i].q) for i in range(len(joint_names))]
    js.velocity = [float(lowstate.motor_state[i].dq) for i in range(len(joint_names))]
    js.effort = [float(lowstate.motor_state[i].tau_est) for i in range(len(joint_names))]
    return js


# -----------------------------------------------------------------------------
# Node
# -----------------------------------------------------------------------------
class JointStateNode(Node):

    def __init__(self) -> None:
        super().__init__('joint_state_node')

        self.declare_parameter('publish_rate_hz', 100)
        self.declare_parameter(
            'joint_names', [f'joint_{i}' for i in range(29)])
        self.declare_parameter('domain_id', 0)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self._joint_names: List[str] = [
            str(n) for n in self.get_parameter('joint_names').value]
        domain = int(self.get_parameter('domain_id').value)

        # Join the domain already created by rclpy.init() (see patch above).
        ChannelFactory().Init(domain)

        self._latest: Optional[LowState_] = None
        self._sub = ChannelSubscriber('rt/lowstate', LowState_)
        self._sub.Init(self._on_lowstate, 10)

        self._joint_pub = self.create_publisher(
            JointState, '/onboard/sensors/joint_states', _BEST_EFFORT_QOS)

        period = 1.0 / rate if rate > 0 else 0.01
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f'joint_state_node started (SDK rt/lowstate, {rate:.1f} Hz, '
            f'{len(self._joint_names)} joints, '
            f'domain={domain})')

    def _on_lowstate(self, msg: LowState_) -> None:
        self._latest = msg

    def _tick(self) -> None:
        ls = self._latest
        if ls is None:
            return  # 아직 첫 프레임 수신 전 — silently skip
        stamp = self.get_clock().now().to_msg()
        js = to_joint_state(ls, self._joint_names, stamp)
        self._joint_pub.publish(js)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
