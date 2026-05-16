"""
Inbound relay /bridge/cmd/* → /onboard/cmd/* (RELIABLE only — warn if yaml says otherwise).
Loader: yaml.safe_load (list-of-dict cannot be a ROS 2 param).

TODO(REQ-32) [TASK-XX]: implement relay (loader + dynamic msg import + sub/pub + echo guard).
TODO(REQ-33, REQ-38) [TASK-XX]: apply RELIABLE QoS, KEEP_LAST(depth=10); warn on yaml qos!=reliable.

Suggestion: https://www.notion.so/comm_bridge-355b39de7dd781d1b207f006610c3906
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
