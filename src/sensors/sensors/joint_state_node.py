"""
G1 SDK lowstate → JointState publisher.

2026-05-23 refactor: this node now owns ONLY
/onboard/sensors/joint_states. Base IMU (formerly fanned out here per the
2026-05-15 decision) has moved to `imu_node` for symmetric ownership with
the ankle IMU topics — see REQ-42 v2026-05-23. Both nodes subscribe to
G1 SDK lowstate independently; the DDS multi-subscriber cost is
negligible at 100 Hz.

Trap: G1 SDK opens its own DDS participant. If sharing ROS 2 domain_id, raw G1
      channels (rt/lf/lowstate, rt/lowcmd, rt/arm_sdk) leak into ros2 topic list.
      Pin sensors_params.yaml domain_id to a separate value (see TASK-32).

TODO(REQ-42) [TASK-37]: lowstate subscribe + JointState publish at publish_rate_hz.
"""
import rclpy
from rclpy.node import Node


class JointStateNode(Node):
    def __init__(self) -> None:
        super().__init__('joint_state_node')
        # TODO(REQ-42) [TASK-37]: declare params (publish_rate_hz, domain_id),
        #                          init unitree_sdk2_python lowstate subscriber,
        #                          JointState publish timer.
        self.get_logger().info('joint_state_node started (TBD — publishes JointState only)')


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
