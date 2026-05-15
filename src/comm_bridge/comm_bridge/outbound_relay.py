"""
Forwards /onboard/* topics to /bridge/* with per-relay QoS.

Reads a relay table from comm_bridge_params.yaml. Each entry has src, dst, type,
and qos ("best_effort" | "reliable"). Sensor streams use BEST_EFFORT (freshness
wins) and state reports use RELIABLE (transitions must not be dropped).

TODO(REQ-32): load 'relays' parameter (list of dicts: src, dst, type, qos).
TODO(REQ-32): dynamically import message classes from the 'type' string
              (e.g. "sensor_msgs/msg/Image" → from sensor_msgs.msg import Image).
TODO(REQ-32): wire create_subscription(src) → create_publisher(dst) per entry.
TODO(REQ-33): translate qos strings to rclpy.qos.QoSProfile:
                best_effort → BEST_EFFORT, KEEP_LAST(depth=1)
                reliable    → RELIABLE,    KEEP_LAST(depth=10)
TODO(REQ-33): verify CycloneDDS partition filter keeps /onboard/ off the wire
              (only /bridge/* should cross the NX↔PC LAN segment).
"""
import rclpy
from rclpy.node import Node


class OutboundRelay(Node):
    def __init__(self) -> None:
        super().__init__('outbound_relay')
        # TODO(REQ-32): declare 'relays' parameter (list of dicts with src/dst/type/qos)
        # TODO(REQ-32): dynamic import each entry's message class
        # TODO(REQ-33): translate qos string → QoSProfile and wire pub/sub pair
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
