"""audio_node — ALSA mic capture + speaker playback.

TODO(REQ-42, REQ-27): publish 4-mic array PCM on /onboard/sensors/audio/pcm.
TODO(REQ-29): subscribe /onboard/sensors/audio/playback and play to speaker.
TODO(REQ-29): publish /onboard/audio/speaker_state (String) — used by STT to block self-pickup.

Real-time notes:
  - ALSA capture is blocking; run capture loop in a background thread.
  - QoS: BEST_EFFORT for outbound PCM (freshness wins over reliability).
"""
import rclpy
from rclpy.node import Node


class AudioNode(Node):
    def __init__(self) -> None:
        super().__init__('audio_node')
        # TODO(REQ-42): declare parameters (capture_device, playback_device, sample_rate, channels, chunk_ms)
        # TODO(REQ-42, REQ-27): create publisher on /onboard/sensors/audio/pcm (kist_drl_g1_msgs/AudioPCM)
        # TODO(REQ-29): create subscriber on /onboard/sensors/audio/playback (kist_drl_g1_msgs/AudioPCM)
        # TODO(REQ-29): create publisher on /onboard/audio/speaker_state (std_msgs/String)
        # TODO(REQ-42): open ALSA capture + playback handles
        self.get_logger().info('audio_node started (TBD)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AudioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
