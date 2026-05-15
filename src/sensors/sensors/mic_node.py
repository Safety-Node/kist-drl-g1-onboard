"""mic_node — ALSA microphone capture publisher.

Publishes fixed 16 kHz / 16-bit / mono PCM chunks from the 4-mic array to
/onboard/sensors/audio/pcm as kist_drl_g1_msgs/AudioPCM. The fixed sample
rate keeps speaker_node and the PC-side STT Provider simple — any rate
mismatch is the publisher's responsibility (e.g. TTS Provider resamples
its 24 kHz CLOVA output to 16 kHz before sending to /audio/playback).

Split from the original audio_node (single-responsibility — capture only).

TODO(REQ-42, REQ-27): declare params (capture_device, sample_rate=16000,
                      channels=1, chunk_ms=20).
TODO(REQ-42, REQ-27): open ALSA capture handle; ALSA reads block, so run
                      the read loop on a background thread.
TODO(REQ-42, REQ-27): publish AudioPCM at chunk cadence; QoS BEST_EFFORT,
                      KEEP_LAST(depth=1) — freshness wins over reliability.
"""
import rclpy
from rclpy.node import Node


class MicNode(Node):
    def __init__(self) -> None:
        super().__init__('mic_node')
        # TODO(REQ-42, REQ-27): declare params (capture_device, sample_rate, channels, chunk_ms)
        # TODO(REQ-42, REQ-27): open ALSA capture handle (pyalsaaudio.PCM(PCM_CAPTURE, ...))
        # TODO(REQ-42, REQ-27): spawn background thread for blocking ALSA reads
        # TODO(REQ-42, REQ-27): create publisher /onboard/sensors/audio/pcm (kist_drl_g1_msgs/AudioPCM)
        # TODO(REQ-42, REQ-27): timestamp each chunk and tag sample_rate / channels / bit_depth
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
