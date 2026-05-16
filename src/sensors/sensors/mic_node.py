"""
ALSA mic capture → /onboard/sensors/audio/pcm (g1_onboard_msgs/AudioPCM, 16k/16-bit/mono).

Trap: channels=1 mono is intentional (G1 4-mic mix-down — see TASK-31).

TODO(REQ-42, REQ-27) [TASK-31]: ALSA capture + int16 LE packing + shutdown order.
"""
import rclpy
from rclpy.node import Node


class MicNode(Node):
    def __init__(self) -> None:
        super().__init__('mic_node')
        # TODO(REQ-42, REQ-27) [TASK-31]: declare params, open ALSA capture, publish loop.
        self.get_logger().info('mic_node started (TBD)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
