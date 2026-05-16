"""
20 Hz control loop driving the G1 SDK.

Subscribes (DDS):
- /onboard/safety/validated_twist (Twist)        -- safety -> velocity_buf
- /onboard/safety/validated_joint (JointCmd)     -- safety -> joint_buf
                                                    JointCmd carries chunk_id +
                                                    step_index; chunk_id == 0
                                                    means non-chunked producer
                                                    (teleop / scripted / CLI).
- /onboard/safety/estop (EstopFlag)              -- structured context (PC mirror)
- /onboard/cmd/loco (LocoCommand)                -- LocoClient action dispatch

Side channel:
  POSIX shared-memory byte at estop_shm_name ("safety_flag") polled every
  control tick (~250 Hz). The DDS EstopFlag is the PC-facing mirror; this
  byte is the zero-latency path that actually triggers Damp().

Publishes:
- /onboard/motor/buf_state (BufState) -> comm_bridge -> PC

G1 SDK destinations:
- rt/arm_sdk          upper-body joints, blended via motor_cmd[29].q weight
- rt/lowcmd           29-motor debug / full-body override (optional)
- LocoClient.Move()   walking (high-level RPC, not a topic)
- LocoClient action   discrete FSM (Damp/StandUp/BalanceStand/SitDown/...)

Modes (CommandExecutor):
- VELOCITY_CMD  -> LocoClient.Move(vx, vy, vyaw)
- JOINT_CMD     -> rt/arm_sdk publish (weight ramped from arm_default_weight)
- LOCO_CMD      -> LocoClient.<method>() dispatch from LocoCommand.action
- ESTOP         -> LocoClient.Damp() + arm weight 1.0 -> 0.0 ramp + flush

Real-time strategy:
- gc.disable() before main loop                  (TODO(REQ-38))
- busy-wait hybrid timer (80% sleep + 20% busy)  (TODO(REQ-38))
- SHM polled every tick (~50 ms today)           (TODO(REQ-35))
- systemd CPUAffinity=1, Nice=-20  (see systemd/motor_controller.service)
"""
import rclpy
from rclpy.node import Node

from .action_queue import ActionQueue
from .queue_aggregate import crossfade  # noqa: F401  (optional NX-side fallback;
                                        # canonical crossfade lives at PC VLA Provider
                                        # per 2026-05-16 msg review option (a'))


class MotorControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('motor_controller')

        # TODO(REQ-34): declare params (control_rate_hz, busy_wait_fraction,
        #               ring_buffer_slots, chunk_size, crossfade_threshold_g,
        #               arm_default_weight, arm_weight_ramp_steps, arm_weight_ramp_dt,
        #               estop_loco_action, network_interface, domain_id, estop_shm_name)
        # TODO(REQ-34): init unitree_sdk2_python — LocoClient handle + rt/arm_sdk publisher
        #               (+ optional rt/lowcmd publisher behind a debug flag)
        # TODO(REQ-35): validate estop_loco_action at startup via
        #               hasattr(loco_client, estop_loco_action). A yaml typo
        #               ("Damp" -> "damp") would otherwise raise AttributeError
        #               at the worst possible moment -- the first E-STOP.
        #               Fail fast at construction instead.
        # TODO(REQ-35): open shared-memory 'safety_flag' read-only
        # TODO(REQ-34): create publisher /onboard/motor/buf_state (g1_onboard_msgs/BufState)
        # TODO(REQ-34): subscribe /onboard/safety/validated_twist (Twist) → push_velocity
        # TODO(REQ-34): subscribe /onboard/safety/validated_joint (JointCmd) → push_joint
        # TODO(REQ-35): subscribe /onboard/safety/estop (EstopFlag) — structured context
        # TODO(REQ-34): subscribe /onboard/cmd/loco (LocoCommand) — LocoClient dispatch
        # TODO(REQ-34, REQ-38): launch 20 Hz control loop thread (busy-wait hybrid timer)
        # TODO(REQ-34): start weight ramp 0.0 → arm_default_weight when the first
        #               validated_joint arrives; ramp back to 0.0 on ESTOP / shutdown

        # ActionQueue construction will switch to the declared param once params are wired.
        # NOTE(REQ-34): the hard-coded 64 happens to match motor_params.yaml's
        #               ring_buffer_slots, but it is NOT read from yaml yet. Any
        #               change to the yaml has no effect until the param wiring
        #               lands -- see the matching comment in motor_params.yaml.
        self._queue = ActionQueue(slots=64)
        self.get_logger().info('motor_controller_node started (TBD)')

    # TODO(REQ-34, REQ-38): def _control_loop(self) -> None
    #     20 Hz busy-wait hybrid; per tick:
    #       1. read SHM safety_flag -- if set, _execute_estop()
    #       2. pop_velocity -> _execute_velocity (LocoClient.Move)
    #       3. pop_joint    -> _execute_joint    (rt/arm_sdk publish, with weight)
    #       4. publish BufState (fill ratios + control rate + counters)
    # TODO(REQ-34): def _on_validated_twist(self, msg)  -- wrap in VelocityCommand,
    #               push_velocity.
    #               Deadband / cache policy: if safety_monitor publishes
    #               validated_twist every tick (including zero-Twist while idle),
    #               LocoClient.Move() gets called continuously and may surface
    #               jitter inside G1's walking controller. Decide at integration
    #               time whether to (a) deadband near-zero Twists, (b) skip
    #               dispatch when Twist equals the last sent one, or (c) rely
    #               on safety_monitor publishing only on change. (c) is the
    #               cleanest but requires confirming the safety_monitor cadence.
    # TODO(REQ-34): def _on_validated_joint(self, msg)  -- wrap in JointCommand,
    #               push_joint (crossfade if optional NX-side fallback is enabled
    #               and msg.step_index == 0 with a new msg.chunk_id).
    # TODO(REQ-34): def _on_loco_command(self, msg)     -- dispatch by LocoCommand.action
    #                                                    (one-shot RPC, bypasses queue)
    # TODO(REQ-35): def _on_estop_flag(self, msg)       -- cache reason/detail for telemetry
    # TODO(REQ-34): def _execute_velocity(self, cmd)    -- LocoClient.Move(vx, vy, vyaw)
    # TODO(REQ-34): def _execute_joint(self, cmd)       -- rt/arm_sdk publish (current_weight)
    # TODO(REQ-35): def _execute_estop(self) -> None
    #               1. LocoClient.<estop_loco_action>()  (default: Damp)
    #               2. start arm weight ramp 1.0 -> 0.0 over ramp_steps * ramp_dt
    #               3. action_queue.flush()


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
