"""
Validates motion commands and emits E-STOP (REQ-35). [TASK-33]

Minimal/bypass scope (demo): subscribe /onboard/cmd/{arm,low} (JointCmd),
validate, forward to /onboard/safety/validated_joint (JointCmd) →
motor_controller. arm + low share the topic; motor_controller routes by
joint_names. velocity (Twist) is NOT validated here (CONV-012 2026-05-26) —
motor_controller drives LocoClient.Move directly.

Validation implemented:
- array-length invariant (names == q == dq == kp == kd == tau_ff),
  weight in [0,1], mode in {0,1,2}, known joint_names → else MALFORMED (drop).
- joint_limits clamp: q/dq/tau_ff coerced into yaml range, forwarded (non-fatal).

E-STOP path: EstopFlag DDS (event + estop_heartbeat_hz heartbeat) mirrored to
the POSIX SHM 'safety_flag' byte motor_controller polls. estop_latch=true keeps
it latched until restart.

Deferred TODO(REQ-35) [TASK-33]: proximity (depth), joint rate-of-change,
per-stream comms watchdog, joint_states/depth staleness, self-watchdog,
100 Hz loop + gc.disable, SHM-write latency re-rating.

Traps:
- joint_limits in yaml is dict-of-list → load via get_parameters_by_prefix.
- SHM uses multiprocessing.shared_memory; its resource_tracker may warn/unlink
  across processes. Revisit (posix_ipc) if motor_controller integration hits it.
- /onboard/cmd/loco bypasses this node by design (motor_controller FSM direct).
"""
from multiprocessing import shared_memory

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from g1_onboard_msgs.msg import EstopFlag, JointCmd


_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class SafetyMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__(
            'safety_monitor',
            automatically_declare_parameters_from_overrides=True,
        )

        self._estop_shm_name: str = self.get_parameter('estop_shm_name').value
        self._estop_heartbeat_hz: float = self.get_parameter('estop_heartbeat_hz').value
        self._estop_latch: bool = self.get_parameter('estop_latch').value
        # Loaded for deferred checks (proximity / watchdog / rate / loop).
        self._loop_rate_hz: int = self.get_parameter('loop_rate_hz').value
        self._proximity_min_dist_m: float = self.get_parameter('proximity_min_dist_m').value
        self._proximity_depth_pixel_thresh: int = self.get_parameter('proximity_depth_pixel_thresh').value
        self._cmd_arm_timeout_s: float = self.get_parameter('cmd_arm_timeout_s').value
        self._cmd_vel_timeout_s: float = self.get_parameter('cmd_vel_timeout_s').value
        self._joint_rate_of_change_limit: float = self.get_parameter('joint_rate_of_change_limit').value

        # joint_limits: {name: (q_min, q_max, dq_max, tau_max)}
        self._joint_limits: dict[str, tuple[float, float, float, float]] = {
            name: tuple(p.value)
            for name, p in self.get_parameters_by_prefix('joint_limits').items()
        }

        self._estop_active = False
        self._estop_reason = EstopFlag.REASON_NONE
        self._estop_detail = ''
        self._shm = self._open_shm(self._estop_shm_name)

        self._validated_pub = self.create_publisher(
            JointCmd, '/onboard/safety/validated_joint', _RELIABLE_QOS)
        self._estop_pub = self.create_publisher(
            EstopFlag, '/onboard/safety/estop', _RELIABLE_QOS)
        self.create_subscription(JointCmd, '/onboard/cmd/arm', self._on_cmd, _RELIABLE_QOS)
        self.create_subscription(JointCmd, '/onboard/cmd/low', self._on_cmd, _RELIABLE_QOS)

        hb = self._estop_heartbeat_hz if self._estop_heartbeat_hz > 0 else 1.0
        self.create_timer(1.0 / hb, self._publish_estop)

        self.get_logger().info(
            f'safety_monitor ready (minimal validate+clamp+estop; '
            f'{len(self._joint_limits)} joints, latch={self._estop_latch})')

    def _open_shm(self, name: str) -> shared_memory.SharedMemory:
        try:
            shm = shared_memory.SharedMemory(name=name, create=True, size=1)
        except FileExistsError:
            shm = shared_memory.SharedMemory(name=name, create=False)
        shm.buf[0] = 0
        return shm

    def _on_cmd(self, cmd: JointCmd) -> None:
        if self._estop_active and self._estop_latch:
            return  # latched — drop until restart (TODO: reset path)
        ok, reason, detail, out = self._validate_and_clamp(cmd)
        if not ok:
            self._trigger_estop(reason, detail)
            return
        if self._estop_active:  # non-latching: clean command clears
            self._clear_estop()
        self._validated_pub.publish(out)

    def _validate_and_clamp(self, cmd: JointCmd):
        n = len(cmd.joint_names)
        for fld in (cmd.q, cmd.dq, cmd.kp, cmd.kd, cmd.tau_ff):
            if len(fld) != n:
                return False, EstopFlag.REASON_MALFORMED_CMD, f'array length mismatch (names={n})', None
        if not 0.0 <= cmd.weight <= 1.0:
            return False, EstopFlag.REASON_MALFORMED_CMD, f'weight {cmd.weight} out of [0,1]', None
        if cmd.mode not in (JointCmd.MODE_POSITION, JointCmd.MODE_TORQUE, JointCmd.MODE_HYBRID):
            return False, EstopFlag.REASON_MALFORMED_CMD, f'mode {cmd.mode} invalid', None
        for name in cmd.joint_names:
            if name not in self._joint_limits:
                return False, EstopFlag.REASON_MALFORMED_CMD, f'unknown joint {name}', None

        q, dq, tau = list(cmd.q), list(cmd.dq), list(cmd.tau_ff)
        for i, name in enumerate(cmd.joint_names):
            q_min, q_max, dq_max, tau_max = self._joint_limits[name]
            q[i] = _clamp(q[i], q_min, q_max)
            dq[i] = _clamp(dq[i], -dq_max, dq_max)
            tau[i] = _clamp(tau[i], -tau_max, tau_max)

        out = JointCmd()
        out.header = cmd.header
        out.joint_names = list(cmd.joint_names)
        out.q, out.dq, out.tau_ff = q, dq, tau
        out.kp, out.kd = list(cmd.kp), list(cmd.kd)
        out.mode, out.weight = cmd.mode, cmd.weight
        out.chunk_id, out.step_index = cmd.chunk_id, cmd.step_index
        return True, EstopFlag.REASON_NONE, '', out

    def _trigger_estop(self, reason: int, detail: str) -> None:
        self._estop_active = True
        self._estop_reason = reason
        self._estop_detail = detail
        self._shm.buf[0] = 1
        self.get_logger().warn(f'E-STOP reason={reason} detail={detail!r}')
        self._publish_estop()

    def _clear_estop(self) -> None:
        self._estop_active = False
        self._estop_reason = EstopFlag.REASON_NONE
        self._estop_detail = ''
        self._shm.buf[0] = 0

    def _publish_estop(self) -> None:
        msg = EstopFlag()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.active = self._estop_active
        msg.reason = self._estop_reason
        msg.detail = self._estop_detail
        self._estop_pub.publish(msg)

    def destroy_node(self) -> None:
        try:
            self._shm.buf[0] = 0
            self._shm.close()
            self._shm.unlink()
        except Exception:
            pass
        super().destroy_node()


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
