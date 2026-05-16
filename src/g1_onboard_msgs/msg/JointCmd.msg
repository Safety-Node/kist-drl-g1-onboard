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
# Wire format (decided 2026-05-16, option (a') from msg review):
#   Producers publish step-by-step JointCmd. ActionChunk (chunk-as-wire) was
#   rejected because chunk-level safety validation has a structural timing
#   problem (validating step[29] against current state when it executes 1.5 s
#   later defeats REQ-35). Chunk metadata (chunk_id, step_index) travels with
#   each step so downstream can detect chunk transitions for optional
#   crossfade or telemetry without forcing all producers to emit chunks.
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

# INVARIANT (enforced by safety_monitor at runtime; ROS IDL cannot express it):
#   len(joint_names) == len(q) == len(dq) == len(kp) == len(kd) == len(tau_ff)
# A length mismatch is malformed, not a safety violation per se -- safety_monitor
# rejects with EstopFlag.REASON_MALFORMED_CMD.

# Execution mode
#   POSITION = PD on (q, dq) with kp/kd, no feed-forward.
#   HYBRID   = PD on (q, dq) with kp/kd, plus tau_ff feed-forward torque.
#   TORQUE   = pure torque (tau_ff), PD inputs (q, dq, kp, kd) ignored.
# dq is the velocity reference for the PD law in POSITION and HYBRID
# (G1 SDK rt/arm_sdk computes tau = kp*(q* - q) + kd*(dq* - dq) + tau_ff
# with tau_ff = 0 in POSITION).
uint8 MODE_POSITION = 0    # use q, dq, kp, kd; ignore tau_ff
uint8 MODE_TORQUE   = 1    # use tau_ff only; ignore q, dq, kp, kd
uint8 MODE_HYBRID   = 2    # use q, dq, kp, kd, tau_ff (PD + feed-forward)

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
#
# Out-of-range values (weight outside [0.0, 1.0]) are malformed and trigger
# EstopFlag.REASON_MALFORMED_CMD at safety_monitor.
#
# MODE_TORQUE × weight policy (decided 2026-05-16):
#   weight is a position-blending concept (rt/arm_sdk linear-interpolates the
#   target position). It is undefined for pure-torque control. motor_controller
#   FORCES weight = 1.0 internally when mode == MODE_TORQUE, regardless of the
#   sender value. Senders SHOULD leave weight at 1.0 with MODE_TORQUE; values
#   other than 1.0 are silently coerced (not flagged as malformed -- weight is
#   simply unused in that mode).
float32 weight

# ---------- Chunk metadata (option (a') wire) ----------
# Producers that emit VLA action chunks (e.g. SmolVLA on PC) tag each step
# with the chunk it belongs to. Producers without chunk semantics (teleop,
# scripted demos, CLI debug `ros2 topic pub`) leave both fields at 0.
#
#   chunk_id == 0 : "no chunk context" (single-shot or continuous stream)
#   chunk_id != 0 : monotonically-increasing per producer. On wraparound the
#                   value SKIPS 0 (goes from 2^32-1 -> 1) to preserve the
#                   sentinel meaning of chunk_id == 0. At 1.5 s per chunk this
#                   takes ~204 years, but the skip rule is cheap insurance
#                   against producers that mistakenly increment per-step.
#   step_index    : 0-based index within the chunk (0 = chunk start)
#
# motor_controller may use (step_index == 0) to detect chunk boundaries and
# trigger optional NX-side crossfade (queue_aggregate.crossfade), but the
# canonical expectation is that the producer (PC VLA) does crossfade before
# publishing -- crossfade lives next to inference. NX-side is a fallback hook.
uint32 chunk_id
uint16 step_index
