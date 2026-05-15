# ActionChunk — sequence of JointCmd steps from VLA inference.
# TODO(REQ-34, REQ-38): tune step_dt + chunk_id wrap behaviour once VLA is integrated.
#
# motor_controller pushes chunks into joint_buf and runs QueueAggregate.crossfade()
# when a new chunk overlaps the tail of the previous one (SmolVLA §3.3).

std_msgs/Header header

# Monotonically-increasing identifier so motor_controller can detect chunk
# transitions and pair them in the crossfade buffer.
uint32 chunk_id

# Time delta between consecutive steps within this chunk (s). Typically 0.05 (= 20 Hz).
float32 step_dt

# Joint commands in execution order. Length = chunk_size (default 30 per SmolVLA).
JointCmd[] steps
