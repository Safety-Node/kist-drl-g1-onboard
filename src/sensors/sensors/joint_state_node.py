"""
G1 SDK lowstate → /onboard/sensors/joint_states (JointState) + /onboard/sensors/imu/data (Imu).
Single lowstate subscription fanned out (saves CPU + avoids DDS reader race).

Trap: G1 SDK opens its own DDS participant. If sharing ROS 2 domain_id, raw G1
      channels (rt/lf/lowstate, rt/lowcmd, rt/arm_sdk) leak into ros2 topic list.
      Pin sensors_params.yaml domain_id to a separate value (see TASK-32).

TODO(REQ-42) [TASK-32]: lowstate subscribe + JointState/Imu fan-out at publish_rate_hz.
"""
import rclpy
from rclpy.node import Node


class JointStateNode(Node):
    def __init__(self) -> None:
        super().__init__('joint_state_node')
        # TODO(REQ-42) [TASK-32]: declare params (publish_rate_hz, domain_id, imu_frame_id),
        #                          init unitree_sdk2_python lowstate subscriber, fan-out timer.
        self.get_logger().info('joint_state_node started (TBD — publishes JointState + Imu)')


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
