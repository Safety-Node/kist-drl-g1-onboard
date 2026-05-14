"""joint_state_node — publishes JointState read from the G1 SDK low-state stream.

TODO(REQ-42): subscribe to G1 SDK `rt/lf/lowstate` (or equivalent) via unitree_sdk2_python.
TODO(REQ-42): publish sensor_msgs/JointState on /onboard/sensors/joint_states at ~100 Hz.

Fields per joint: q, dq, tau_est (mapped to JointState.position/velocity/effort).
"""
import rclpy
from rclpy.node import Node


class JointStateNode(Node):
    def __init__(self) -> None:
        super().__init__('joint_state_node')
        # TODO(REQ-42): declare parameters (publish_rate_hz, network_interface, domain_id)
        # TODO(REQ-42): initialise unitree_sdk2_python channel subscriber
        # TODO(REQ-42): create publisher on /onboard/sensors/joint_states (sensor_msgs/JointState)
        # TODO(REQ-42): create timer at publish_rate_hz
        self.get_logger().info('joint_state_node started (TBD)')


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
