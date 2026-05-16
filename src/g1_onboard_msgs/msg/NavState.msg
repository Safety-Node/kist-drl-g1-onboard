# NavState — high-level state of navigation/goto_node.
# TODO(REQ-30): align field names with Move Connector's status schema on the PC side.
#
# Replaces the std_msgs/String hack on /onboard/nav/state.

std_msgs/Header header

uint8 STATUS_IDLE     = 0   # no active goal
uint8 STATUS_MOVING   = 1   # goal accepted, P-controller driving cmd_vel
uint8 STATUS_REACHED  = 2   # within position + yaw tolerance
uint8 STATUS_FAILED   = 3   # watchdog / out-of-bounds / abort
uint8 STATUS_CANCELED = 4   # external cancel request

uint8 status

# Goal context
# Post-2026-05-16 (decision 5a reverted): nav_goal is always a named_goal text
# key on /onboard/cmd/nav_goal; goto_node resolves it via named_goals.yaml.
# target_name is therefore always populated while status != IDLE. Reserved
# empty for a hypothetical future direct-PoseStamped path (none today).
string  target_name          # named_goal key (e.g. "refrigerator")
float32 distance_to_goal_m   # straight-line distance to goal in metres
float32 yaw_error_rad

# Failure detail (only meaningful when status == FAILED / CANCELED)
string error_message
