# AudioPCM — raw PCM audio chunk used between sensors/{mic,speaker}_node and comm_bridge.
#
# Pipeline format (LOCKED 2026-05-15 per sensors review):
#   sample_rate = 16000  (Hz)
#   channels    = 1      (mono)
#   bit_depth   = 16     (signed little-endian)
#
# Publishers that produce a different native rate (e.g. CLOVA TTS at 24 kHz)
# MUST resample to 16 kHz / 16-bit / mono before publishing. The fields below
# remain in the message so a future "mixed rate" path can be opted into
# without re-spinning the wire format, but the canonical pipeline value is
# fixed and consumers (speaker_node, STT Provider) assume the values above.
#
# header     : standard stamped header (frame_id may be unused for audio)
# sample_rate: samples per second (Hz). Pipeline: 16000.
# channels   : channel count. Pipeline: 1.
# bit_depth  : sample width in bits. Pipeline: 16.
# data       : interleaved PCM samples as little-endian signed int16, packed into bytes.

std_msgs/Header header
uint32 sample_rate
uint8  channels
uint8  bit_depth
uint8[] data
