"""
G1 odometry relay — /dog_odom → /onboard/sensors/odom.

The G1 locomotion controller publishes odometry on /dog_odom
(nav_msgs/Odometry, ~100 Hz).  This node subscribes to that topic
and re-publishes it on the standard onboard namespace so comm_bridge
can relay it to the workstation PC.

No data transformation is applied — the SDK-provided pose/twist values
are forwarded as-is (frame_id: odom, child_frame_id: robot_center).

Publications
------------
  /onboard/sensors/odom  nav_msgs/Odometry  BestEffort  ~100 Hz
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from nav_msgs.msg import Odometry


_BE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# /dog_odom is published by the G1 locomotion controller with BEST_EFFORT
_SUB_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10,
)


class OdomNode(Node):
    """Relays G1 SDK odometry to /onboard/sensors/odom."""

    def __init__(self) -> None:
        super().__init__('odom_node')

        self.declare_parameter('source_topic', '/dog_odom')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'robot_center')

        source = self.get_parameter('source_topic').value
        self._frame_id = self.get_parameter('frame_id').value
        self._child_frame_id = self.get_parameter('child_frame_id').value

        self._pub = self.create_publisher(
            Odometry, '/onboard/sensors/odom', _BE_QOS)

        self._sub = self.create_subscription(
            Odometry, source, self._on_odom, _SUB_QOS)

        self.get_logger().info(
            f'odom_node ready — relay {source} → /onboard/sensors/odom')

    def _on_odom(self, msg: Odometry) -> None:
        # Override frame ids if configured differently from SDK defaults
        msg.header.frame_id = self._frame_id
        msg.child_frame_id = self._child_frame_id
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
