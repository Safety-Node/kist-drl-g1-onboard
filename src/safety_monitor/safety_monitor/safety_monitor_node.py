"""
Validates all motion commands and emits E-STOP within 200 ms (REQ-35).

Subscriptions:
- /onboard/navigation/cmd_vel        (Twist)        navigation/goto_node
- /onboard/cmd/arm                   (JointCmd)     comm_bridge inbound
- /onboard/sensors/depth/image_raw   (Image)        sensors RealSense Depth
- /onboard/sensors/joint_states      (JointState)   sensors joint_state_node

Publications:
- /onboard/safety/validated_twist    (Twist)        → motor_controller velocity_buf
- /onboard/safety/validated_joint    (JointCmd)     → motor_controller joint_buf
- /onboard/safety/estop              (EstopFlag)    structured DDS + 1 Hz heartbeat
- POSIX SHM byte (estop_shm_name)    → motor_controller zero-latency poll

By-design exclusions: /onboard/cmd/loco bypasses this node (motor_controller
dispatches LocoClient FSM directly). /onboard/cmd/low (rt/lowcmd) is unrouted
today — if exposed, must route through here too with expanded joint_limits.

Traps:
- joint_limits in yaml is dict-of-list → load via get_parameters_by_prefix.
- comms watchdog is per-stream (cmd_vel_timeout_s ≠ cmd_arm_timeout_s) because
  cmd_vel idle (BalanceStand) is legitimate but cmd/arm idle during VLA is not.
- Unknown joint_names in JointCmd → REASON_MALFORMED_CMD (not JOINT_LIMIT).
- State-stream staleness (joint_states / depth) silently corrupts checks;
  separate watchdog needed (reason code policy TBD: reuse COMMS_TIMEOUT vs new SENSOR_TIMEOUT).
- Self-watchdog: this node's own loop overrun → REASON_WATCHDOG (distinct from
  COMMS_TIMEOUT which is "upstream stopped talking").

TODO(REQ-35) [TASK-33]: declare params + load joint_limits via prefix accessor.
TODO(REQ-35) [TASK-33]: SHM byte open, all subscribers + publishers wired.
TODO(REQ-35) [TASK-33]: validation pipeline (joint / velocity / proximity / rate / comms).
TODO(REQ-35) [TASK-33]: per-stream comms watchdog + state-stream staleness watchdog.
TODO(REQ-35) [TASK-33]: self-watchdog (loop overrun → REASON_WATCHDOG + SHM set).
TODO(REQ-35) [TASK-33]: EstopFlag heartbeat at estop_heartbeat_hz.
TODO(REQ-35) [TASK-33]: gc.disable() after steady-state warm-up.
"""
import rclpy
from rclpy.node import Node


class SafetyMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('safety_monitor')
        # TODO(REQ-35) [TASK-33]: wire everything (see module docstring TODO list).
        self.get_logger().info('safety_monitor_node started (TBD)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
