"""
100 Hz control loop driving the G1 SDK (REQ-34 v2026-05-22).

Subscriptions:
- /onboard/safety/validated_joint_chunk (JointCmdChunk)
                                                 → unpack into joint_buf
                                                   (arm + low both arrive here;
                                                    distinguished by joint_names;
                                                    carries chunk_id; per-step
                                                    step_index inside the chunk
                                                    is for trace/log only)
- /onboard/safety/estop (EstopFlag)              structured DDS context
- /onboard/cmd/loco (LocoCommand)                LocoClient FSM dispatch (no buffer)
- POSIX SHM byte 'safety_flag'                   zero-latency E-STOP poll

Publications:
- /onboard/motor/buf_state (BufState)            telemetry → comm_bridge → PC

G1 SDK targets:
- rt/arm_sdk          upper-body joints, weight = motor_cmd[29].q (from /cmd/arm)
- rt/lowcmd           lower-body joints, weight ignored (from /cmd/low, NEW 2026-05-22)
- LocoClient action   discrete FSM (Damp / StandUp / BalanceStand / SitDown / ...)
                      via LocoCommand channel — usage scope TBD (likely
                      demo entry/exit + posture transitions; not finalized)

3 dispatch modes (queue does not carry an enum; dispatcher routes by joint_names):
- JOINT_CMD_ARM → rt/arm_sdk publish (weight ramped from arm_default_weight)
- JOINT_CMD_LOW → rt/lowcmd publish (weight ignored)
- LOCO_CMD      → LocoClient.<method>() from LocoCommand.action
- ESTOP         → LocoClient.Damp() + arm weight 1.0→0.0 ramp + flush

2026-05-22 KIST mail:
- VELOCITY_CMD mode / LocoClient.Move(vx, vy, vyaw) dropped — walking is now
  low-level VLA via /cmd/low. velocity_buf removed.
- Loop rate 20 Hz → 100 Hz. Existing 20 ms ramp / busy-wait hybrid timer /
  overrun thresholds rerated against the 10 ms period (linspace(0,1,N) ramp
  count doubles to preserve ~2.0 s envelope).

2026-05-26 wire reversal (CONV-006 REVISED — workstation repo):
- /bridge/cmd/{arm,low} now carries JointCmdChunk (not single-step JointCmd).
  Chunk-handling responsibility moves from PC to here. The PC publishes a
  full action_horizon-step chunk per inference (~15 Hz); this node unpacks
  each chunk's steps[] into joint_buf and paces them at 100 Hz.
- Chunk-boundary crossfade is now CANONICAL on NX (queue_aggregate.crossfade()
  default ON). On chunk_id transition we keep the tail of the previous
  chunk still in joint_buf, crossfade it with the head of the new chunk,
  then drop any remaining old-chunk tail (mid-chunk preemption).
- Empty queue (underflow) holds the last popped step at 100 Hz so NX never
  silently stops publishing during inference lag.
- Non-chunk producers (teleop, CLI) MUST use a separate single-step
  JointCmd topic per JointCmdChunk.msg comment — chunk_id==0 is no longer
  meaningful inside this node.

Traps:
- estop_loco_action is a yaml string → method name. Validate with hasattr at
  startup; a typo otherwise blows up only on the first E-STOP.
- crossfade is CANONICAL here (CONV-006 REVISED 2026-05-26). The yaml knobs
  chunk_size / crossfade_threshold_g previously labelled "fallback path" are
  now the live config for the canonical path — re-validate defaults when
  wiring.
- arm vs low routing is on joint_names — joint_name → SDK target mapping must
  be locked in motor_params.yaml; unknown joint name in a chunk = MALFORMED.

Real-time strategy:
- gc.disable() before main loop          (TODO(REQ-38))
- busy-wait hybrid timer (80/20) @100Hz  (TODO(REQ-38))
- SHM polled every tick                  (TODO(REQ-35))
- systemd CPUAffinity=1, Nice=-20

TODO(REQ-34) [TASK-34]: declare params + init unitree_sdk2_python (LocoClient + rt/arm_sdk + rt/lowcmd).
TODO(REQ-35) [TASK-34]: validate estop_loco_action at startup; open SHM safety_flag.
TODO(REQ-34) [TASK-34]: subscribe JointCmdChunk on arm + low; loco dispatch.
TODO(REQ-34) [TASK-34]: chunk-receipt path — detect chunk_id transition vs
                        last seen; on transition, call crossfade(old_tail,
                        new_chunk.steps), drop previous chunk tail, push
                        blended sequence into joint_buf. No transition →
                        append steps directly.
TODO(REQ-34, REQ-38) [TASK-34]: 100 Hz control loop (busy-wait hybrid, gc.disable,
                                 SHM poll, pop+execute, BufState publish,
                                 empty-queue last-step republish).
TODO(REQ-35) [TASK-34]: ESTOP path — LocoClient.<estop_loco_action>() + arm weight
                         1.0→0.0 ramp + action_queue.flush().
TODO(REQ-34): re-rate arm weight ramp step count for 100 Hz period.
"""
import rclpy
from rclpy.node import Node

from .action_queue import ActionQueue
from .queue_aggregate import crossfade  # noqa: F401  (canonical chunk crossfade; CONV-006 REVISED)


class MotorControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('motor_controller')
        # TODO(REQ-34) [TASK-34]: wire everything (see module docstring TODO list).
        # NOTE: ActionQueue(slots=64) currently hard-coded; motor_params.yaml
        # ring_buffer_slots is not read yet — keep in sync until param wiring lands.
        self._queue = ActionQueue(slots=64)
        self.get_logger().info('motor_controller_node started (TBD)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
