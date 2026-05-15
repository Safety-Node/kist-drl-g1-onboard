# SpeakerState — speaker playback state.
# TODO(REQ-29): emit on every chunk push / pop transition.
#
# STT Provider on the PC subscribes to this so it can mute the input stream while
# `playing == true` to avoid self-pickup (echo cancellation fall-back).

std_msgs/Header header

bool   playing
uint32 current_chunk_id      # 0 when idle
uint8  queue_depth           # pending playback chunks in the audio_node buffer
