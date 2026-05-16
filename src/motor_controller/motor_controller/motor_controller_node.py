"""
20 Hz control loop driving the G1 SDK.

Subscriptions:
- /onboard/safety/validated_twist (Twist)        → velocity_buf
- /onboard/safety/validated_joint (JointCmd)     → joint_buf
                                                   (carries chunk_id + step_index;
                                                    chunk_id==0 = non-chunk producer)
- /onboard/safety/estop (EstopFlag)              structured DDS context
- /onboard/cmd/loco (LocoCommand)                LocoClient FSM dispatch (no buffer)
- POSIX SHM byte 'safety_flag'                   zero-latency E-STOP poll

Publications:
- /onboard/motor/buf_state (BufState)            telemetry → comm_bridge → PC

G1 SDK targets:
- rt/arm_sdk          upper-body joints, weight = motor_cmd[29].q
- LocoClient.Move     walking (continuous)
- LocoClient action   discrete FSM (Damp / StandUp / BalanceStand / SitDown / ...)
- rt/lowcmd           29-motor debug (gated, /onboard/cmd/low not exposed today)

4 modes (dispatcher concern; queue does not carry an enum):
- VELOCITY_CMD  → LocoClient.Move(vx, vy, vyaw)
- JOINT_CMD     → rt/arm_sdk publish (weight ramped from arm_default_weight)
- LOCO_CMD      → LocoClient.<method>() from LocoCommand.action
- ESTOP         → LocoClient.Damp() + arm weight 1.0→0.0 ramp + flush

Traps:
- estop_loco_action is a yaml string → method name. Validate with hasattr at
  startup; a typo otherwise blows up only on the first E-STOP.
- validated_twist may stream zero-Twists continuously; deadband or last-cache
  before each LocoClient.Move to avoid jitter inside G1's walking controller.
- crossfade lives at the PC VLA Provider per (a') wire decision; NX
  queue_aggregate is OFF by default (chunk_size / crossfade_threshold_g
  in yaml are scaffold for the fallback path).

Real-time strategy:
- gc.disable() before main loop          (TODO(REQ-38))
- busy-wait hybrid timer (80/20)         (TODO(REQ-38))
- SHM polled every tick                  (TODO(REQ-35))
- systemd CPUAffinity=1, Nice=-20

TODO(REQ-34) [TASK-34]: declare params + init unitree_sdk2_python (LocoClient + rt/arm_sdk).
TODO(REQ-35) [TASK-34]: validate estop_loco_action at startup; open SHM safety_flag.
TODO(REQ-34) [TASK-34]: subscribers wired with push_velocity / push_joint / loco dispatch.
TODO(REQ-34, REQ-38) [TASK-34]: 20 Hz control loop (busy-wait hybrid, gc.disable,
                                 SHM poll, pop+execute, BufState publish).
TODO(REQ-35) [TASK-34]: ESTOP path — LocoClient.<estop_loco_action>() + arm weight
                         1.0→0.0 ramp + action_queue.flush().
TODO(REQ-34) [TASK-34]: optional NX crossfade if step_index==0 with new chunk_id.
"""
import rclpy
from rclpy.node import Node

from .action_queue import ActionQueue
from .queue_aggregate import crossfade  # noqa: F401  (optional NX-side fallback)


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
