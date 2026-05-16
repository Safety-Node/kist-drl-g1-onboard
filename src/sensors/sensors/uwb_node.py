"""
UWB beacon receiver / absolute pose publisher.

Replaces SLAM-based localisation (REQ-37). Reads UWB tag position from a
vendor module (serial or UDP -- TBD) and publishes the robot's absolute
pose in the 'map' frame.

Scope (decided 2026-05-16, sensors review):
- This node publishes geometry_msgs/PoseStamped only.
- No TF broadcast. UWB gives reliable (x, y) but no yaw -- a full 6-DoF
  TF requires fusion with another orientation source (typically IMU),
  and TF cannot carry a partial frame. The only NX consumer today is
  goto_node (a P-controller that needs Pose, not TF). If RViz or any
  TF-dependent consumer is added later, introduce a separate fusion
  node (uwb_node + joint_state_node.Imu -> map->base_link broadcaster)
  rather than coupling sensor publishers to each other here.

TODO(REQ-37): pick UWB vendor SDK / wire protocol (Nooploop, Decawave, Sewio, …).
TODO(REQ-37): subscribe to UWB tag stream, parse anchor distances, run trilateration if
              the module does not already publish a fused position.
TODO(REQ-37): publish geometry_msgs/PoseStamped on /onboard/sensors/uwb/pose at
              publish_rate_hz. Set quaternion to identity (no yaw source on NX today);
              downstream consumers must treat yaw as unknown.
TODO(REQ-37): apply outlier rejection + smoothing (median or 1€ filter) to tame UWB
              multipath jitter before publishing.
TODO(REQ-37): load anchor table via self.get_parameters_by_prefix('anchors') --
              declare_parameter cannot enumerate dynamic dict keys, but
              dict-of-list yaml IS supported via the prefix accessor (unlike
              the list-of-dict case that bit comm_bridge). Iterate the result
              into a {anchor_id: [x, y, z]} map at startup.
"""
import rclpy
from rclpy.node import Node


class UwbNode(Node):
    def __init__(self) -> None:
        super().__init__('uwb_node')

        # TODO(REQ-37): declare params (transport, serial_port, serial_baud,
        #               udp_listen_port, publish_rate_hz, frame_id, child_frame_id)
        # TODO(REQ-37): load anchor table via get_parameters_by_prefix('anchors')
        # TODO(REQ-37): open transport (serial.Serial or socket) — keep blocking read in a
        #               background thread so this node's executor stays responsive
        # TODO(REQ-37): create publisher /onboard/sensors/uwb/pose (geometry_msgs/PoseStamped)
        # TODO(REQ-37): create timer at publish_rate_hz to drain the latest sample and publish

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
