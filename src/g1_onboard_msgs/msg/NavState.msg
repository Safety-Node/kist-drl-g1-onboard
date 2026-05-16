# NavState — high-level state of navigation/goto_node.
# State machine details: see navigation/goto_node.py docstring.

std_msgs/Header header

# STATUS_* values
#   IDLE       no active goal
#   MOVING     goal accepted, P-controller driving cmd_vel
#   REACHED    within position tolerance (yaw_tolerance reserved — UWB has no yaw)
#   FAILED     watchdog / out-of-bounds / abort. Sticky — new nav_goal required;
#              UWB recovery does NOT auto-resume.
#   CANCELED   preempted by a NEW nav_goal while previously MOVING.
#              Emitted ONLY for live-goal preemption. Transitions from REACHED /
#              FAILED bypass CANCELED (terminal states have no active goal to cancel).
#              No external /onboard/cmd/nav_cancel API — external cancel via
#              safety_monitor E-STOP.
uint8 STATUS_IDLE     = 0
uint8 STATUS_MOVING   = 1
uint8 STATUS_REACHED  = 2
uint8 STATUS_FAILED   = 3
uint8 STATUS_CANCELED = 4

uint8 status

# Goal context (target_name always populated while status != IDLE)
string  target_name          # named_goal key (e.g. "refrigerator")
float32 distance_to_goal_m
float32 yaw_error_rad        # RESERVED, currently always 0.0 (no yaw source on NX)

# Failure detail (meaningful when status in {FAILED, CANCELED})
string error_message
