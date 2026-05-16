# JointCmd — joint-space command for either rt/arm_sdk (upper-body blended) or
# rt/lowcmd (29-motor full-body override). Replaces the sensor_msgs/JointState
# hack used for command channels.
#
# Routing convention (decided 2026-05-15):
#   /onboard/cmd/arm  → motor_controller forwards to rt/arm_sdk; `weight` respected.
#   /onboard/cmd/low  → motor_controller forwards to rt/lowcmd;  `weight` ignored.
#
# JointState semantically means "current state"; this message is a "target
# command" with PD gains, feed-forward torque, and the rt/arm_sdk blending weight.
#
# TODO(REQ-34): finalise gain / mode / weight policy once VLA Provider integrates.

std_msgs/Header header

# Joint name → vector index. All arrays below must share this length.
string[]  joint_names

# Target trajectory point (one value per joint)
float64[] q          # target position (rad)
float64[] dq         # target velocity (rad/s)
float64[] kp         # P gain
float64[] kd         # D gain
float64[] tau_ff     # feed-forward torque (Nm)

# Execution mode
uint8 MODE_POSITION = 0    # use q + kp/kd, ignore tau_ff
uint8 MODE_TORQUE   = 1    # use tau_ff only
uint8 MODE_HYBRID   = 2    # use all of q, dq, kp, kd, tau_ff

uint8 mode

# rt/arm_sdk blending weight, forwarded to motor_cmd[29].q on G1 SDK.
# Range [0.0, 1.0]:
#   0.0 = G1 motion controller drives the arms (our q/dq/... ignored)
#   1.0 = these JointCmd values drive the arms fully
#   in-between = linear blend per G1 SDK formula
#     actual = motion_cmd * (1 - weight) + user_cmd * weight
#
# Typical use:
#   - VLA inference normal:  weight = 1.0
#   - Start-up / hand-back:  ramp 0.0 ↔ 1.0 over ~2 s (linspace(0,1,101) × 20 ms)
#   - safety_monitor can clamp to 0.0 to force motion-controller fallback
#
# Default (zero-initialised) means "let motion controller drive" — VLA Provider
# MUST set this to 1.0 explicitly for any meaningful arm motion.
float32 weight
