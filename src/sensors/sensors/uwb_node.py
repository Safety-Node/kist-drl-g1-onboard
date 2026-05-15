"""
UWB beacon receiver / absolute pose publisher.

Replaces SLAM-based localisation (REQ-37). Reads UWB tag position from a
vendor module (serial or UDP -- TBD) and publishes the robot's absolute
pose in the 'map' frame.

TODO(REQ-37): pick UWB vendor SDK / wire protocol (Nooploop, Decawave, Sewio, …).
TODO(REQ-37): subscribe to UWB tag stream, parse anchor distances, run trilateration if
              the module does not already publish a fused position.
TODO(REQ-37): publish geometry_msgs/PoseStamped on /onboard/sensors/uwb/pose at
              publish_rate_hz.
TODO(REQ-37): also broadcast TF (map → base_link) so downstream consumers
              (e.g. RViz, future Nav2 fall-back) get a coherent transform tree.
TODO(REQ-37): apply outlier rejection + smoothing (median or 1€ filter) to tame UWB
              multipath jitter before publishing.
"""
import rclpy
from rclpy.node import Node


class UwbNode(Node):
    def __init__(self) -> None:
        super().__init__('uwb_node')

        # TODO(REQ-37): declare params (transport, serial_port, serial_baud,
        #               udp_listen_port, publish_rate_hz, frame_id, child_frame_id, anchors)
        # TODO(REQ-37): open transport (serial.Serial or socket) — keep blocking read in a
        #               background thread so this node's executor stays responsive
        # TODO(REQ-37): create publisher /onboard/sensors/uwb/pose (geometry_msgs/PoseStamped)
        # TODO(REQ-37): create TF broadcaster (map → base_link)
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
