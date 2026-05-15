"""motor_controller_node — 20 Hz control loop driving the G1 SDK.

Subscribes (DDS):
  /onboard/safety/validated_twist  (geometry_msgs/Twist)        [safety_monitor → velocity_buf]
  /onboard/safety/validated_joint  (kist_drl_g1_msgs/JointCmd)  [safety_monitor → joint_buf]
  /onboard/safety/estop            (kist_drl_g1_msgs/EstopFlag) [structured context for PC; we still act on SHM]
  /onboard/cmd/loco                (kist_drl_g1_msgs/LocoCommand) [comm_bridge → LocoClient action dispatch]

Side channel:
  POSIX shared-memory byte at `estop_shm_name` ("safety_flag")  — polled every
  control tick by motor_controller; this is the zero-latency E-STOP path.
  The DDS EstopFlag mirror is for structured logging + PC monitoring.

Publishes:
  /onboard/motor/buf_state         (kist_drl_g1_msgs/BufState)  [→ comm_bridge → PC]

G1 SDK destinations:
  - rt/arm_sdk         (upper-body joints, blended via motor_cmd[29].q weight)
  - rt/lowcmd          (29-motor debug / full-body override — optional, not wired today)
  - LocoClient.Move()  (walking — high-level RPC, not a topic)
  - LocoClient.{Damp,StandUp,BalanceStand,SitDown,...}() (discrete FSM actions)

Modes (per design — handled in CommandExecutor):
  - VELOCITY_CMD  → LocoClient.Move(vx, vy, vyaw)
  - JOINT_CMD     → rt/arm_sdk publish (upper-body; weight ramped from arm_default_weight)
  - LOCO_CMD      → LocoClient.<method>() dispatch from LocoCommand.action enum
  - ESTOP         → LocoClient.Damp() + arm weight 1.0 → 0.0 ramp + buffer flush

Real-time strategy:
  - gc.disable() before main loop                          (TODO(REQ-38))
  - busy-wait hybrid timer (80% sleep + 20% busy)          (TODO(REQ-38))
  - SHM polled every tick (~50 ms today)                   (TODO(REQ-35))
  - systemd CPUAffinity=1, Nice=-20                        (see systemd/motor_controller.service)
"""
import rclpy
from rclpy.node import Node

from .action_queue import ActionQueue
from .queue_aggregate import crossfade  # noqa: F401  (used once VLA chunks land)


class MotorControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('motor_controller')

        # TODO(REQ-34): declare params (control_rate_hz, busy_wait_fraction,
        #               ring_buffer_slots, chunk_size, crossfade_threshold_g,
        #               arm_default_weight, arm_weight_ramp_steps, arm_weight_ramp_dt,
        #               estop_loco_action, network_interface, domain_id, estop_shm_name)
        # TODO(REQ-34): init unitree_sdk2_python — LocoClient handle + rt/arm_sdk publisher
        #               (+ optional rt/lowcmd publisher behind a debug flag)
        # TODO(REQ-35): open shared-memory 'safety_flag' read-only
        # TODO(REQ-34): create publisher /onboard/motor/buf_state (kist_drl_g1_msgs/BufState)
        # TODO(REQ-34): subscribe /onboard/safety/validated_twist (Twist) → velocity_buf
        # TODO(REQ-34): subscribe /onboard/safety/validated_joint (JointCmd) → joint_buf
        # TODO(REQ-35): subscribe /onboard/safety/estop (EstopFlag) — structured context
        # TODO(REQ-34): subscribe /onboard/cmd/loco (LocoCommand) — LocoClient dispatch
        # TODO(REQ-34, REQ-38): launch 20 Hz control loop thread (busy-wait hybrid timer)
        # TODO(REQ-34): start weight ramp 0.0 → arm_default_weight when the first
        #               validated_joint arrives; ramp back to 0.0 on ESTOP / shutdown

        # ActionQueue construction will switch to the declared param once params are wired.
        self._queue = ActionQueue(slots=64)   # TODO(REQ-34): use ring_buffer_slots param
        self.get_logger().info('motor_controller_node started (TBD)')

    # TODO(REQ-34, REQ-38): def _control_loop(self) -> None
    #                       20 Hz busy-wait hybrid; per tick:
    #                         1. read SHM safety_flag — if set, _execute_estop()
    #                         2. pop_velocity → _execute_velocity (LocoClient.Move)
    #                         3. pop_joint    → _execute_joint    (rt/arm_sdk publish, with weight)
    #                         4. publish BufState (fill ratios + control rate + counters)
    # TODO(REQ-34): def _on_validated_twist(self, msg) -> None  ← push to velocity_buf
    # TODO(REQ-34): def _on_validated_joint(self, msg) -> None  ← push to joint_buf (crossfade if overlap)
    # TODO(REQ-34): def _on_loco_command(self, msg) -> None     ← dispatch by LocoCommand.action
    # TODO(REQ-35): def _on_estop_flag(self, msg) -> None       ← cache reason/detail for telemetry
    # TODO(REQ-34): def _execute_velocity(self, cmd) -> None    ← LocoClient.Move(vx, vy, vyaw)
    # TODO(REQ-34): def _execute_joint(self, cmd) -> None       ← rt/arm_sdk publish (weight=current_weight)
    # TODO(REQ-35): def _execute_estop(self) -> None
    #                       1. LocoClient.<estop_loco_action>()  (default: Damp)
    #                       2. start arm weight ramp 1.0 → 0.0 over ramp_steps × ramp_dt
    #                       3. action_queue.flush()


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
