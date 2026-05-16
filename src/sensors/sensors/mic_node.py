"""
ALSA microphone capture publisher.

Publishes fixed 16 kHz / 16-bit / mono PCM chunks from the 4-mic array to
/onboard/sensors/audio/pcm as g1_onboard_msgs/AudioPCM. The fixed sample
rate keeps speaker_node and the PC-side STT Provider simple -- any rate
mismatch is the publisher's responsibility (e.g. TTS Provider resamples
its 24 kHz CLOVA output to 16 kHz before sending to /audio/playback).

Channel choice (channels=1, decided 2026-05-16):
  ALSA mixes the G1 4-mic array down to mono at capture time. We GIVE UP
  beamforming and direction-of-arrival noise suppression in exchange for:
    - a single PCM stream that STT (CLOVA) accepts unchanged
    - a payload small enough that BEST_EFFORT QoS over the LAN is trivial
  The G1 array hardware does some onboard AEC already, and the demo
  environment is acoustically simple (single speaker, fixed distance), so
  the multi-channel features would be overkill for the current scope.
  If a beamformed front-end becomes useful later, swap this node for a
  4-channel capture + beamformer pair that still emits a 16 kHz mono
  AudioPCM on the same topic -- downstream code does not change.

Split from the original audio_node (single-responsibility -- capture only).

TODO(REQ-42, REQ-27): declare params (capture_device, sample_rate=16000,
                      channels=1, chunk_ms=20).
TODO(REQ-42, REQ-27): open ALSA capture handle; ALSA reads block, so run
                      the read loop on a background thread.
TODO(REQ-42, REQ-27): publish AudioPCM at chunk cadence; QoS BEST_EFFORT,
                      KEEP_LAST(depth=1) -- freshness wins over reliability.
TODO(REQ-42, REQ-27): on shutdown -- signal the capture thread to stop,
                      join it, then close the ALSA handle. Mirror of
                      speaker_node's "drain queue + close cleanly" path.
"""
import rclpy
from rclpy.node import Node


class MicNode(Node):
    def __init__(self) -> None:
        super().__init__('mic_node')
        # TODO(REQ-42, REQ-27): declare params (capture_device, sample_rate, channels, chunk_ms)
        # TODO(REQ-42, REQ-27): open ALSA capture handle (pyalsaaudio.PCM(PCM_CAPTURE, ...))
        # TODO(REQ-42, REQ-27): spawn background thread for blocking ALSA reads
        # TODO(REQ-42, REQ-27): create publisher /onboard/sensors/audio/pcm
        #                       (g1_onboard_msgs/AudioPCM)
        # TODO(REQ-42, REQ-27): timestamp each chunk and tag sample_rate / channels / bit_depth
        # TODO(REQ-42, REQ-27): pack ALSA's int16 frames into AudioPCM.data via
        #                       np.asarray(frames, dtype=np.int16).tobytes() --
        #                       little-endian native on x86_64 / aarch64 matches
        #                       the AudioPCM contract ("signed LE int16 packed
        #                       into uint8[]"). Verify endianness explicitly if
        #                       this ever runs on a big-endian platform.
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
