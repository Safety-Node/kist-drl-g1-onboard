"""
ALSA speaker playback subscriber + state publisher.

Subscribes /onboard/audio/playback (g1_onboard_msgs/AudioPCM), pushes each
chunk to the ALSA playback device at a fixed 16 kHz / 16-bit / mono
pipeline rate, and emits g1_onboard_msgs/SpeakerState on
/onboard/audio/speaker_state so the PC-side STT Provider can mute its
input while we're playing.

Audio namespace convention (decided 2026-05-16, sensors review):
  mic     = sensor (input)     -> /onboard/sensors/audio/pcm
  speaker = actuator (output)  -> /onboard/audio/playback
  speaker state                -> /onboard/audio/speaker_state
The asymmetric prefix (sensors/ vs no sensors/) is intentional: speaker
is not a sensor, and keeping its inputs/outputs under /onboard/audio/
keeps the topic tree readable.

Senders that produce a different native rate (e.g. CLOVA TTS at 24 kHz)
MUST resample to 16 kHz before publishing -- see AudioPCM.msg.

Split from the original audio_node (single-responsibility -- playback only).

TODO(REQ-29): declare params (playback_device, sample_rate=16000, channels=1,
              max_queue_depth=50).
TODO(REQ-29): open ALSA playback handle once at startup; serialise writes
              through an internal queue (single-writer thread).
TODO(REQ-29): subscribe /onboard/audio/playback; QoS RELIABLE,
              KEEP_LAST(depth=10) -- do not drop TTS audio in transit.
TODO(REQ-29): publish SpeakerState (playing, current_chunk_id, queue_depth)
              on every push/pop transition.
TODO(REQ-29): bound the internal queue to max_queue_depth (default 50).
              On overflow, drop OLDEST chunk and emit SpeakerState with
              queue_depth at the cap. Rationale: TTS values recency over
              completeness (a stale system prompt mid-playback is worse than
              the latest message arriving on time). SpeakerState.queue_depth
              is uint8 so the cap must stay < 256.
TODO(REQ-29): on shutdown, drain the queue then close the device cleanly.
"""
import rclpy
from rclpy.node import Node


class SpeakerNode(Node):
    def __init__(self) -> None:
        super().__init__('speaker_node')
        # TODO(REQ-29): declare params (playback_device, sample_rate, channels,
        #               max_queue_depth)
        # TODO(REQ-29): open ALSA playback handle (pyalsaaudio.PCM(PCM_PLAYBACK, ...))
        # TODO(REQ-29): create subscriber /onboard/audio/playback
        # TODO(REQ-29): create publisher /onboard/audio/speaker_state
        #               (g1_onboard_msgs/SpeakerState)
        # TODO(REQ-29): spawn writer thread + bounded chunk queue
        #               (drop-oldest on overflow); emit SpeakerState on push/pop
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
