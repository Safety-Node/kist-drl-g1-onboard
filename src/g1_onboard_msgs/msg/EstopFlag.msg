# EstopFlag — structured E-STOP event from safety_monitor.
# TODO(REQ-35): emit on every state transition; latched until explicitly cleared.
#
# This is the *DDS* mirror of the shared-memory flag (`safety_flag`) that
# motor_controller polls directly. The DDS path is for PC-side UI / logging.

std_msgs/Header header

bool active                   # true while latched

# REASON_* enum
#
# Trigger semantics (decided 2026-05-16):
#   REASON_NONE           : default value; not a real reason. Set when active=false.
#   REASON_JOINT_LIMIT    : a joint q / dq / tau in an inbound JointCmd was out of
#                           the per-joint range from safety_params.yaml.
#   REASON_VELOCITY       : an inbound Twist exceeded max_linear_x/y or max_angular_z.
#   REASON_PROXIMITY      : RealSense Depth proximity check tripped (front-cone
#                           pixel count above threshold within N metres).
#   REASON_COMMS_TIMEOUT  : an *external input stream* (cmd_vel / cmd/arm) was
#                           silent longer than safety_params.yaml comms_timeout_s.
#                           "Other side stopped talking."
#   REASON_WATCHDOG       : safety_monitor's *own* main loop overran its budget
#                           or failed to publish heartbeat in time. "We stopped
#                           ticking." Distinct from COMMS_TIMEOUT (which is
#                           about the upstream producer, not us).
#   REASON_MANUAL         : operator-triggered (CLI, GUI button, or a future
#                           hardware E-STOP button).
#   REASON_SELF_COLLISION : *RESERVED for future use.* Self-collision check is
#                           not implemented today (no collision model on NX);
#                           reason code reserved so the wire format does not
#                           need a renumber once it lands.
#   REASON_MALFORMED_CMD  : a JointCmd violated the array-length invariant
#                           (joint_names / q / dq / kp / kd / tau_ff lengths
#                           must all match) or carried out-of-range enum values
#                           (mode, weight outside [0,1]). This is a producer
#                           bug, not a physical safety event, but we surface it
#                           via E-STOP so the failure is visible end-to-end.
uint8 REASON_NONE           = 0
uint8 REASON_JOINT_LIMIT    = 1
uint8 REASON_VELOCITY       = 2
uint8 REASON_PROXIMITY      = 3
uint8 REASON_COMMS_TIMEOUT  = 4
uint8 REASON_WATCHDOG       = 5
uint8 REASON_MANUAL         = 6
uint8 REASON_SELF_COLLISION = 7
uint8 REASON_MALFORMED_CMD  = 8

uint8 reason
string detail                 # human-readable: "knee_pitch over 2.5rad", "18pt within 25cm"
