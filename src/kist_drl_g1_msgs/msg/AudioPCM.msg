# AudioPCM — raw PCM audio chunk used between sensors/audio_node and comm_bridge.
# TODO(REQ-27, REQ-29): finalise sample format. Current draft assumes 16 kHz / 16-bit / mono.
#
# header     : standard stamped header (frame_id may be unused for audio)
# sample_rate: samples per second (Hz). Expected: 16000.
# channels   : channel count. Expected: 1 (mono mic capture) or 2 (stereo speaker).
# bit_depth  : sample width in bits. Expected: 16.
# data       : interleaved PCM samples as little-endian signed int16, packed into bytes.

std_msgs/Header header
uint32 sample_rate
uint8  channels
uint8  bit_depth
uint8[] data
