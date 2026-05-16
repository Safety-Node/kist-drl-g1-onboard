"""
Outbound relay /onboard/* → /bridge/*.
Loader: yaml.safe_load (list-of-dict cannot be a ROS 2 param).

TODO(REQ-32) [TASK-28]: implement relay (loader + dynamic msg import + sub/pub + echo guard).
TODO(REQ-33) [TASK-28]: QoS string → QoSProfile translation (best_effort, reliable).
"""
import rclpy
from rclpy.node import Node


class OutboundRelay(Node):
    def __init__(self) -> None:
        super().__init__('outbound_relay')
        self.get_logger().info('outbound_relay started (TBD)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OutboundRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
