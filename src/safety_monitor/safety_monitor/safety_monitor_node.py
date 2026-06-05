"""
Validates all motion commands and emits E-STOP within 200 ms (REQ-35).

Subscriptions:
- /onboard/cmd/arm                   (JointCmd)     comm_bridge inbound (rt/arm_sdk path)
- /onboard/cmd/low                   (JointCmd)     comm_bridge inbound (rt/lowcmd path, NEW 2026-05-22)
- /onboard/sensors/depth/image_raw   (Image)        sensors RealSense Depth
- /onboard/sensors/joint_states      (JointState)   sensors joint_state_node

Publications:
- /onboard/safety/validated_joint    (JointCmd)     → motor_controller joint_buf
                                                      (arm + low share the topic;
                                                       distinguished by joint_names)
- /onboard/safety/estop              (EstopFlag)    structured DDS + 1 Hz heartbeat
- POSIX SHM byte (estop_shm_name)    → motor_controller zero-latency poll

By-design exclusions: /onboard/cmd/loco bypasses this node (motor_controller
dispatches LocoClient FSM directly).

2026-05-22 KIST mail (later partly reversed — see 2026-05-26 below):
- Validation now runs at 100 Hz (REQ-34 v2026-05-22) — joint_limits / velocity
  bounds / proximity check / rate watchdog all need rerating; E-STOP budget
  remains 200 ms.

2026-05-26 KIST 회의 (workstation CONV-012, partly reverses 2026-05-22):
- cmd_vel watchdog active 다시: PC NavigationProvider Twist 부활했으므로
  /onboard/cmd/vel 도 per-stream comms watchdog 대상이다.
  validated_twist 는 여전히 dropped — motor_controller 가 PC Twist 받아
  LocoClient.Move 직접 호출 (no safety-side velocity validation in this
  iteration; safety budget covered by motor_controller's own LocoClient
  dispatch + the existing joint-level safety_monitor on arm/low).

Traps:
- joint_limits in yaml is dict-of-list → load via get_parameters_by_prefix.
- comms watchdog is per-stream (cmd_arm_timeout_s ≠ cmd_low_timeout_s) — both
  active during VLA execution; either idle implies an upstream stall.
- Unknown joint_names in JointCmd → REASON_MALFORMED_CMD (not JOINT_LIMIT).
- State-stream staleness (joint_states / depth) silently corrupts checks;
  separate watchdog needed (reason code policy TBD: reuse COMMS_TIMEOUT vs new SENSOR_TIMEOUT).
- Self-watchdog: this node's own loop overrun → REASON_WATCHDOG (distinct from
  COMMS_TIMEOUT which is "upstream stopped talking").

TODO(REQ-35) [TASK-33]: SHM byte open, all subscribers + publishers wired.
TODO(REQ-35) [TASK-33]: validation pipeline (joint / proximity / rate / comms).
                         Velocity-value validation dropped per CONV-012 2026-05-26;
                         cmd_vel kept as watchdog-only stream.
TODO(REQ-35) [TASK-33]: per-stream comms watchdog + state-stream staleness watchdog.
TODO(REQ-35) [TASK-33]: self-watchdog (loop overrun → REASON_WATCHDOG + SHM set).
TODO(REQ-35) [TASK-33]: EstopFlag heartbeat at estop_heartbeat_hz.
TODO(REQ-35) [TASK-33]: gc.disable() after steady-state warm-up.
TODO(REQ-35): re-rate validation budget for 100 Hz loop (was 20 Hz).
"""
import rclpy
from rclpy.node import Node


class SafetyMonitorNode(Node):
    def __init__(self) -> None:
        # automatically_declare_parameters_from_overrides=True so the
        # joint_limits.<name> entries from yaml are auto-declared and visible
        # to get_parameters_by_prefix(). yaml is the single source of truth;
        # launch + systemd always pass --params-file.
        super().__init__(
            'safety_monitor',
            automatically_declare_parameters_from_overrides=True,
        )

        self._loop_rate_hz: int = self.get_parameter('loop_rate_hz').value
        self._estop_shm_name: str = self.get_parameter('estop_shm_name').value
        self._estop_heartbeat_hz: float = self.get_parameter('estop_heartbeat_hz').value
        self._proximity_min_dist_m: float = self.get_parameter('proximity_min_dist_m').value
        self._proximity_depth_pixel_thresh: int = self.get_parameter(
            'proximity_depth_pixel_thresh').value
        self._cmd_arm_timeout_s: float = self.get_parameter('cmd_arm_timeout_s').value
        self._cmd_vel_timeout_s: float = self.get_parameter('cmd_vel_timeout_s').value
        self._joint_rate_of_change_limit: float = self.get_parameter(
            'joint_rate_of_change_limit').value

        # joint_limits — yaml dict-of-list; flat ROS param store needs prefix accessor.
        # Shape: {joint_name: (q_min, q_max, dq_max, tau_max)}
        limit_params = self.get_parameters_by_prefix('joint_limits')
        self._joint_limits: dict[str, tuple[float, float, float, float]] = {
            name: tuple(p.value) for name, p in limit_params.items()
        }

        # TODO(REQ-35) [TASK-33]: wire SHM / subs / pubs / validation / watchdogs
        #                          (see module docstring TODO list).
        self.get_logger().info(
            f'safety_monitor ready (params loaded, '
            f'{len(self._joint_limits)} joints, '
            f'loop={self._loop_rate_hz} Hz)'
        )


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
