"""
Forwards /bridge/* commands to /onboard/* with RELIABLE QoS.

All command channels are RELIABLE — control messages must not be dropped.
The per-relay yaml schema still carries `qos` for symmetry with the outbound
relay, but for the inbound path it is expected to be "reliable" on every entry.

TODO(REQ-32): load 'relays' parameter (list of dicts: src, dst, type, qos).
TODO(REQ-32): dynamically import message classes from the 'type' string.
TODO(REQ-32): wire create_subscription(src) → create_publisher(dst) per entry.
TODO(REQ-33, REQ-38): apply QoS RELIABLE, KEEP_LAST(depth=10).
"""
import rclpy
from rclpy.node import Node


class InboundRelay(Node):
    def __init__(self) -> None:
        super().__init__('inbound_relay')
        # TODO(REQ-32): declare 'relays' parameter (list of dicts with src/dst/type/qos)
        # TODO(REQ-32): dynamic import each entry's message class
        # TODO(REQ-32): wire pub/sub pair under a RELIABLE QoS profile
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
