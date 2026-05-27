"""
G1 SDK lowstate → JointState publisher.

2026-05-23 refactor: this node now owns ONLY
/onboard/sensors/joint_states. Base IMU (formerly fanned out here per the
2026-05-15 decision) has moved to `imu_node` for symmetric ownership with
the ankle IMU topics — see REQ-42 v2026-05-23. Both nodes subscribe to
G1 SDK lowstate independently; the DDS multi-subscriber cost is
negligible at the SDK lowstate rate.

Trap: G1 SDK opens its own DDS participant. If sharing ROS 2 domain_id, raw G1
      channels (rt/lf/lowstate, rt/lowcmd, rt/arm_sdk) leak into ros2 topic list.
      Pin sensors_params.yaml domain_id to a separate value (see TASK-32).

Source: unitree_sdk2py ChannelSubscriber on 'rt/lowstate' (unitree_hg.LowState_).
        Callback runs in the SDK thread; the ROS timer publishes the latest
        cached frame at publish_rate_hz. 로봇 없이 돌릴 때는 별도 fake
        publisher 스크립트가 rt/lowstate에 가짜 프레임을 쏘면 됨.

TODO(REQ-42) [TASK-32]: motor_state 35 슬롯 중 G1 활성 29개 정확한 인덱스 확정
                        (현재는 0..N-1로 가정, joint_names 길이만큼만 사용).
"""
from typing import List, Optional, Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


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
        self.declare_parameter('network_interface', '')
        self.declare_parameter('domain_id', 0)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self._joint_names: List[str] = [
            str(n) for n in self.get_parameter('joint_names').value]
        nic = str(self.get_parameter('network_interface').value)
        domain = int(self.get_parameter('domain_id').value)

        if nic:
            ChannelFactoryInitialize(domain, nic)
        else:
            ChannelFactoryInitialize(domain)

        self._latest: Optional[LowState_] = None
        self._sub = ChannelSubscriber('rt/lowstate', LowState_)
        self._sub.Init(self._on_lowstate, 10)

        self._joint_pub = self.create_publisher(
            JointState, '/onboard/sensors/joint_states', 10)

        period = 1.0 / rate if rate > 0 else 0.01
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f'joint_state_node started (SDK rt/lowstate, {rate:.1f} Hz, '
            f'{len(self._joint_names)} joints, '
            f'domain={domain}, nic={nic or "default"})')

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
