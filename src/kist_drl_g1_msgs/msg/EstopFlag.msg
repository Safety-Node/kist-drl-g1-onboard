# EstopFlag — structured E-STOP event from safety_monitor.
# TODO(REQ-35): emit on every state transition; latched until explicitly cleared.
#
# This is the *DDS* mirror of the shared-memory flag (`safety_flag`) that
# motor_controller polls directly. The DDS path is for PC-side UI / logging.

std_msgs/Header header

bool active                   # true while latched

uint8 REASON_NONE           = 0
uint8 REASON_JOINT_LIMIT    = 1
uint8 REASON_VELOCITY       = 2
uint8 REASON_PROXIMITY      = 3
uint8 REASON_COMMS_TIMEOUT  = 4
uint8 REASON_WATCHDOG       = 5
uint8 REASON_MANUAL         = 6
uint8 REASON_SELF_COLLISION = 7

uint8 reason
string detail                 # human-readable: "knee_pitch over 2.5rad", "18pt within 25cm"
