# BufState — motor_controller ring-buffer telemetry.
# TODO(REQ-34, REQ-38): wire into _control_loop with fill ratio + cadence measurement.
#
# Published periodically so comm_bridge can forward this to the PC monitor.

std_msgs/Header header

float32 velocity_fill        # velocity_buf fill ratio, 0.0–1.0
float32 joint_fill           # joint_buf    fill ratio, 0.0–1.0
float32 control_rate_hz      # measured loop frequency (target: 20.0)
uint32  last_cmd_age_ms      # ms since the most recent JointCmd landed (any producer).
                             # Renamed from last_chunk_age_ms (2026-05-16): wire is
                             # step-by-step now, "last cmd" is the meaningful health
                             # signal; chunk-specific aging would only matter for
                             # VLA-producer monitoring and can be derived from
                             # JointCmd.chunk_id transitions if needed.

# Cumulative counters since process start
uint32  underrun_count       # pop on an empty buffer
uint32  overrun_count        # push on a full buffer
