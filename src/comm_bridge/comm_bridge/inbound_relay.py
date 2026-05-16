"""
Forwards /bridge/* commands to /onboard/* with RELIABLE QoS.

Design (revised 2026-05-16 review):
- Same loader strategy as outbound_relay: read comm_bridge_params.yaml from
  share/comm_bridge/config/ at startup (no rclpy parameter for the relay
  table -- list-of-dicts is unsupported by declare_parameter).
- All command channels are RELIABLE; the per-relay yaml schema still carries
  `qos` for symmetry with the outbound side, but for inbound it is expected
  to be "reliable" on every entry (the loader can warn otherwise).

TODO(REQ-32): load relay table from share/comm_bridge/config/comm_bridge_params.yaml
              (yaml.safe_load -> validate keys src/dst/type/qos; warn if qos != reliable).
TODO(REQ-32): per-entry echo-loop guard.
TODO(REQ-32): dynamically import message classes from the 'type' string.
TODO(REQ-32): wire create_subscription(src) -> create_publisher(dst) per entry.
TODO(REQ-33, REQ-38): apply QoS RELIABLE, KEEP_LAST(depth=10).
"""
import rclpy
from rclpy.node import Node


class InboundRelay(Node):
    def __init__(self) -> None:
        super().__init__('inbound_relay')
        # TODO(REQ-32): resolve share/comm_bridge/config/comm_bridge_params.yaml,
        #               yaml.safe_load it, pick the inbound_relay block
        # TODO(REQ-32): dynamic import each entry's message class
        # TODO(REQ-32): wire pub/sub pair under a RELIABLE QoS profile;
        #               warn if any entry's qos != "reliable"
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
