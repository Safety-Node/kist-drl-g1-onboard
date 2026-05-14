"""outbound_relay — forwards /onboard/* topics to /bridge/* with BEST_EFFORT QoS.

TODO(REQ-32): load relay table from comm_bridge_params.yaml.
TODO(REQ-32): per-entry create_subscription (/onboard/...) → create_publisher (/bridge/...).
TODO(REQ-33): apply QoS profile BEST_EFFORT, KEEP_LAST(depth=1) for sensor streams.
TODO(REQ-33): verify CycloneDDS partition assignment keeps /onboard/ topics LAN-only.
"""
import rclpy
from rclpy.node import Node


class OutboundRelay(Node):
    def __init__(self) -> None:
        super().__init__('outbound_relay')
        # TODO(REQ-32): declare 'relays' parameter (list of dicts)
        # TODO(REQ-32): dynamically import message types from 'type' string
        # TODO(REQ-32): wire subscriptions to publishers
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
