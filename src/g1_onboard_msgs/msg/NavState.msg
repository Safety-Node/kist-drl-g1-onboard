# NavState — high-level state of navigation/goto_node.
# TODO(REQ-30): align field names with Move Connector's status schema on the PC side.
#
# Replaces the std_msgs/String hack on /onboard/nav/state.

std_msgs/Header header

uint8 STATUS_IDLE     = 0   # no active goal
uint8 STATUS_MOVING   = 1   # goal accepted, P-controller driving cmd_vel
uint8 STATUS_REACHED  = 2   # within position tolerance
                            # (yaw_tolerance is reserved -- see yaw_error_rad below)
uint8 STATUS_FAILED   = 3   # watchdog / out-of-bounds / abort.
                            # Sticky -- a fresh nav_goal is required to leave
                            # this state; UWB recovery does NOT auto-resume.
uint8 STATUS_CANCELED = 4   # PREEMPTED BY A NEW NAV_GOAL (2026-05-16 navigation
                            # review #4, #5). There is no external /onboard/cmd/
                            # nav_cancel API by design; external cancellation
                            # flows through safety_monitor E-STOP.
                            # When a new goal arrives while MOVING, the node
                            # emits NavState(CANCELED, "preempted by '<new>'")
                            # for the old goal followed by NavState(MOVING, ...)
                            # for the new one, so the PC trace stays continuous.

uint8 status

# Goal context
# Post-2026-05-16 (decision 5a reverted): nav_goal is always a named_goal text
# key on /onboard/cmd/nav_goal; goto_node resolves it via named_goals.yaml.
# target_name is therefore always populated while status != IDLE. Reserved
# empty for a hypothetical future direct-PoseStamped path (none today).
string  target_name          # named_goal key (e.g. "refrigerator")
float32 distance_to_goal_m   # straight-line distance to goal in metres
float32 yaw_error_rad        # RESERVED, currently always 0.0 (2026-05-16
                             # navigation review #1). UWB has no yaw source on
                             # NX today, so goto_node ignores yaw control;
                             # kp_angular=0 in goto_params.yaml. Kept in the
                             # schema so a future IMU-fusion node can fill it
                             # without an ICD break.

# Failure detail (only meaningful when status == FAILED / CANCELED)
string error_message
