"""speaker_node — ALSA speaker playback subscriber + state publisher.

Subscribes /onboard/sensors/audio/playback (kist_drl_g1_msgs/AudioPCM),
pushes each chunk to the ALSA playback device at a fixed 16 kHz / 16-bit /
mono pipeline rate, and emits kist_drl_g1_msgs/SpeakerState on
/onboard/audio/speaker_state so the PC-side STT Provider can mute its
input while we're playing.

Senders that produce a different native rate (e.g. CLOVA TTS at 24 kHz)
MUST resample to 16 kHz before publishing — see AudioPCM.msg.

Split from the original audio_node (single-responsibility — playback only).

TODO(REQ-29): declare params (playback_device, sample_rate=16000, channels=1).
TODO(REQ-29): open ALSA playback handle once at startup; serialise writes
              through an internal queue (single-writer thread).
TODO(REQ-29): subscribe /onboard/sensors/audio/playback; QoS RELIABLE,
              KEEP_LAST(depth=10) — do not drop TTS audio.
TODO(REQ-29): publish SpeakerState (playing, current_chunk_id, queue_depth)
              on every push/pop transition.
TODO(REQ-29): on shutdown, drain the queue then close the device cleanly.
"""
import rclpy
from rclpy.node import Node


class SpeakerNode(Node):
    def __init__(self) -> None:
        super().__init__('speaker_node')
        # TODO(REQ-29): declare params (playback_device, sample_rate, channels)
        # TODO(REQ-29): open ALSA playback handle (pyalsaaudio.PCM(PCM_PLAYBACK, ...))
        # TODO(REQ-29): create subscriber /onboard/sensors/audio/playback
        # TODO(REQ-29): create publisher /onboard/audio/speaker_state (kist_drl_g1_msgs/SpeakerState)
        # TODO(REQ-29): spawn writer thread + chunk queue; emit SpeakerState on push/pop
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
