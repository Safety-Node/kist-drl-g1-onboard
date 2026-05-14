"""inbound_relay — forwards /bridge/* commands to /onboard/* with RELIABLE QoS.

TODO(REQ-32): load relay table from comm_bridge_params.yaml.
TODO(REQ-32): per-entry create_subscription (/bridge/cmd/...) → create_publisher (/onboard/cmd/...).
TODO(REQ-33, REQ-38): apply QoS profile RELIABLE, KEEP_LAST(depth=10) for control commands.
"""
import rclpy
from rclpy.node import Node


class InboundRelay(Node):
    def __init__(self) -> None:
        super().__init__('inbound_relay')
        # TODO(REQ-32): declare 'relays' parameter (list of dicts)
        # TODO(REQ-32): dynamically import message types from 'type' string
        # TODO(REQ-32): wire subscriptions to publishers
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
