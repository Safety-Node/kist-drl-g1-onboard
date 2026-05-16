"""
Forwards /onboard/* topics to /bridge/* with per-relay QoS.

Design (revised 2026-05-16 review):
- The relay table lives in comm_bridge_params.yaml. ROS 2 parameters cannot
  carry a list-of-dicts, so the node loads the YAML directly from the
  package share path (ament_index_python.get_package_share_directory) at
  startup; nothing is declared as a ros2 param. This keeps the YAML schema
  human-readable and avoids the four-parallel-list workaround.
- DDS partition isolation for /onboard/* is set in config/cyclonedds.xml
  (NetworkPartition "OnboardLocal" -> 127.0.0.1). The relay subscribes to
  /onboard/* on loopback and re-publishes on /bridge/* over the LAN.

TODO(REQ-32): load relay table from share/comm_bridge/config/comm_bridge_params.yaml
              (yaml.safe_load -> validate keys src/dst/type/qos).
TODO(REQ-32): per-entry guard against echo loops -- reject any relay whose
              dst would re-enter the source prefix (/onboard <-> /onboard or
              /bridge <-> /bridge).
TODO(REQ-32): dynamically import message classes from the 'type' string
              (e.g. "sensor_msgs/msg/Image" -> from sensor_msgs.msg import Image).
TODO(REQ-32): wire create_subscription(src) -> create_publisher(dst) per entry.
TODO(REQ-33): translate qos strings to rclpy.qos.QoSProfile:
                best_effort -> BEST_EFFORT, KEEP_LAST(depth=1)
                reliable    -> RELIABLE,    KEEP_LAST(depth=10)
TODO(REQ-33): verify CycloneDDS partition filter at first integration --
              run `ros2 topic list` on NX and PC, confirm /onboard/* is
              visible only on NX.
"""
import rclpy
from rclpy.node import Node


class OutboundRelay(Node):
    def __init__(self) -> None:
        super().__init__('outbound_relay')
        # TODO(REQ-32): resolve share/comm_bridge/config/comm_bridge_params.yaml,
        #               yaml.safe_load it, pick the outbound_relay block
        # TODO(REQ-32): for each entry: dynamic import message class, build QoS,
        #               create subscription -> create publisher
        # TODO(REQ-32): echo-loop guard before wiring (raise on suspicious entries)
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
