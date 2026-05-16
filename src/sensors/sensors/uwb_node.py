"""
UWB beacon receiver → /onboard/sensors/uwb/pose (geometry_msgs/PoseStamped).

Replaces SLAM-based localisation (REQ-37).

Trap: anchor table is dict-of-list — load via self.get_parameters_by_prefix('anchors').
      Quaternion stays identity (no yaw source); no TF broadcast.

TODO(REQ-37, REQ-42) [TASK-30]: vendor SDK + transport + pose publisher.
TODO(REQ-37) [TASK-30]: outlier rejection + sample-sequence dedup.
"""
import rclpy
from rclpy.node import Node


class UwbNode(Node):
    def __init__(self) -> None:
        super().__init__('uwb_node')
        # TODO(REQ-37) [TASK-30]: declare params + load anchors via get_parameters_by_prefix.
        # TODO(REQ-37) [TASK-30]: open vendor transport, publish PoseStamped at publish_rate_hz.
        self.get_logger().info('uwb_node started (TBD)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UwbNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
