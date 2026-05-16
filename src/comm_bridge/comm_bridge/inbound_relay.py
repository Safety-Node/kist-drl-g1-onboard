"""
Inbound relay /bridge/cmd/* → /onboard/cmd/* (RELIABLE only).

Loader: yaml.safe_load (list-of-dict cannot be a ROS 2 param).
Warn if any yaml entry's qos != reliable.

TODO(REQ-32) [TASK-28]: implement relay (loader + dynamic msg import + sub/pub + echo guard).
TODO(REQ-33, REQ-38) [TASK-28]: apply RELIABLE QoS, KEEP_LAST(depth=10);
                                 warn on yaml qos != reliable.
"""
import rclpy
from rclpy.node import Node


class InboundRelay(Node):
    def __init__(self) -> None:
        super().__init__('inbound_relay')
        self.get_logger().info('inbound_relay started (TBD)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InboundRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
