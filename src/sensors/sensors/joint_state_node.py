"""
G1 SDK lowstate → JointState publisher.

2026-05-23 refactor: this node now owns ONLY
/onboard/sensors/joint_states. Base IMU (formerly fanned out here per the
2026-05-15 decision) has moved to `imu_node` for symmetric ownership with
the ankle IMU topics — see REQ-42 v2026-05-23. Both nodes subscribe to
G1 SDK lowstate independently; the DDS multi-subscriber cost is
negligible at the SDK lowstate rate.

Domain: SDK shares ROS 2 domain 0 — a second DomainParticipant is created
        on domain 0 (CycloneDDS supports multiple participants per domain
        per process) and the DataReader is polled inside the timer callback.
        This replaces ChannelSubscriber whose internal thread busy-polls the
        DDS reader with no sleep, consuming an entire CPU core even when no
        robot data arrives.  Polling in the timer adds zero threads and uses
        CPU only proportional to the publish rate.

Source: DataReader on 'rt/lowstate' (unitree_hg.LowState_), polled at
        publish_rate_hz. 로봇 없이 돌릴 때는 별도 fake publisher 스크립트가
        rt/lowstate에 가짜 프레임을 쏘면 됨.
"""

from typing import List, Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import DataReader
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


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

        # CycloneDDS reader polled in the timer — no background thread.
        _dp = DomainParticipant(domain)
        _topic = Topic(_dp, 'rt/lowstate', LowState_)
        self._dds_reader: DataReader = DataReader(_dp, _topic)
        self._latest: Optional[LowState_] = None

        self._joint_pub = self.create_publisher(
            JointState, '/onboard/sensors/joint_states', _BEST_EFFORT_QOS)

        period = 1.0 / rate if rate > 0 else 0.01
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f'joint_state_node started (SDK rt/lowstate, {rate:.1f} Hz, '
            f'{len(self._joint_names)} joints, '
            f'domain={domain})')

    def _tick(self) -> None:
        samples = self._dds_reader.take(N=1)
        if samples:
            self._latest = samples[0]

        if self._latest is None:
            return  # 아직 첫 프레임 수신 전 — silently skip
        stamp = self.get_clock().now().to_msg()
        js = to_joint_state(self._latest, self._joint_names, stamp)
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
