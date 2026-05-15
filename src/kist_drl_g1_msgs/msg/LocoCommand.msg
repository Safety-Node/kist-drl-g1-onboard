# LocoCommand — discrete G1 LocoClient action (anything other than continuous Move).
# TODO(REQ-34): finalise action set after demo-script walk-through.
#
# Continuous walking commands stay on geometry_msgs/Twist (see /onboard/safety/validated_twist).
# This message is for one-shot LocoClient API calls that change the FSM:
# StandUp, BalanceStand, SitDown, Damp, ZeroTorque, etc.
#
# Typical demo flow consumers:
#   - System start-up:  ACTION_STAND_UP  → ACTION_BALANCE_STAND
#   - Idle posture:     ACTION_BALANCE_STAND (held while VLA manipulates arms)
#   - Demo shutdown:    ACTION_SIT_DOWN or ACTION_DAMP
#   - safety_monitor E-STOP path can publish ACTION_DAMP independently of comm_bridge.
#
# motor_controller subscribes to this and translates each value into the matching
# unitree_sdk2_python LocoClient method call. Returns are reported via NavState
# (status) or BufState (operational telemetry).

std_msgs/Header header

uint8 ACTION_DAMP          = 0   # passive — zero torque + crouch (E-STOP path)
uint8 ACTION_ZERO_TORQUE   = 1   # zero torque without crouch (gentle hand-off)
uint8 ACTION_STAND_UP      = 2   # lying / damped → upright (takes a few seconds)
uint8 ACTION_BALANCE_STAND = 3   # locked stance, ready to walk or manipulate
uint8 ACTION_LOW_STAND     = 4   # half-crouch standing posture
uint8 ACTION_HIGH_STAND    = 5   # full-height standing posture
uint8 ACTION_SIT_DOWN      = 6   # controlled sit-down

uint8 action

# Optional integer parameter (e.g. gait type, balance mode index).
# Most actions ignore this. Senders can leave it 0.
int32 param
