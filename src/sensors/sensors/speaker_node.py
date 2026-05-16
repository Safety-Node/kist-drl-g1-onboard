"""
ALSA speaker playback ← /onboard/audio/playback (g1_onboard_msgs/AudioPCM).

Publishes /onboard/audio/speaker_state (g1_onboard_msgs/SpeakerState).

Trap: subscribes /onboard/audio/playback (NOT /onboard/sensors/audio/playback —
      mic/speaker namespace asymmetry — see TASK-31).
Trap: queue overflow = drop OLDEST; max_queue_depth < 256 (SpeakerState.queue_depth uint8).

TODO(REQ-29) [TASK-31]: playback queue + writer thread + SpeakerState + shutdown drain.
"""
import rclpy
from rclpy.node import Node


class SpeakerNode(Node):
    def __init__(self) -> None:
        super().__init__('speaker_node')
        # TODO(REQ-29) [TASK-36]: declare params, open ALSA playback, queue + writer thread.
        self.get_logger().info('speaker_node started (TBD)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
