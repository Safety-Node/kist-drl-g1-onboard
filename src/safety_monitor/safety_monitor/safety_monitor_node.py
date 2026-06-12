"""
Validates motion commands and emits E-STOP (REQ-35). [TASK-33]

Subscribe /onboard/cmd/{arm,low} (JointCmdChunk), validate every step, forward
to /onboard/safety/validated_joint_chunk (JointCmdChunk) → motor_controller.
arm + low share the topic; motor_controller routes by joint_names. velocity
(Twist) is NOT handled here (CONV-012 2026-05-26) — motor_controller drives
LocoClient.Move directly, so cmd_vel watchdog is motor_controller's concern.

Wire is JointCmdChunk (2026-05-26 reversal): PC VLAProvider emits one chunk of
action_horizon steps per inference; NX paces at 100 Hz. We validate each step
and republish the chunk in shape (chunk_id/header preserved). A single bad step
E-STOPs and drops the WHOLE chunk.

Checks:
- malformed (per step): array-length invariant, weight in [0,1], mode in {0,1,2},
  known joint_names → REASON_MALFORMED_CMD (drop chunk).
- joint_limit (per step): q/dq/tau out of yaml range → REASON_JOINT_LIMIT if
  joint_limit_estop else clamp into range and forward.
- rate-of-change (per chunk): |Δq| between consecutive steps over
  joint_rate_of_change_limit → REASON_JOINT_LIMIT (chunk-internal only;
  chunk boundary is motor_controller crossfade's job).
- comms watchdog (loop): cmd/arm, cmd/low silent past per-stream timeout (armed
  on first message) → REASON_COMMS_TIMEOUT.
- self-watchdog (loop): loop interval > loop_overrun_factor*period → REASON_WATCHDOG.
- proximity (depth, opt-in): close depth pixels in the front ROI exceed threshold
  → REASON_PROXIMITY. Off unless proximity_enable.

E-STOP path: EstopFlag DDS (event + estop_heartbeat_hz heartbeat) mirrored to the
POSIX SHM 'safety_flag' byte motor_controller polls. estop_latch=true latches
until restart. gc.disable() in main() removes GC jitter from the 200 ms budget.

Traps:
- joint_limits in yaml is dict-of-list → load via get_parameters_by_prefix.
- SHM uses multiprocessing.shared_memory; resource_tracker may warn across
  processes. Revisit (posix_ipc) if motor_controller integration hits it.
- /onboard/cmd/loco bypasses this node by design (motor_controller FSM direct).
- depth_topic defaults to the ICD name; real camera publishes under camera/* —
  remap via param. No depth → proximity is fail-open (cannot check).
"""
import gc
from multiprocessing import shared_memory

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from g1_onboard_msgs.msg import EstopFlag, JointCmd, JointCmdChunk


_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
# Sensor stream — freshness wins (matches comm_bridge image relay).
_BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
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
        self._loop_rate_hz: int = self.get_parameter('loop_rate_hz').value
        self._loop_overrun_factor: float = self.get_parameter('loop_overrun_factor').value
        self._joint_limit_estop: bool = self.get_parameter('joint_limit_estop').value
        self._cmd_arm_timeout_s: float = self.get_parameter('cmd_arm_timeout_s').value
        self._cmd_low_timeout_s: float = self.get_parameter('cmd_low_timeout_s').value
        self._proximity_enable: bool = self.get_parameter('proximity_enable').value
        self._proximity_min_dist_m: float = self.get_parameter('proximity_min_dist_m').value
        self._proximity_depth_pixel_thresh: int = self.get_parameter('proximity_depth_pixel_thresh').value
        self._proximity_roi_frac: float = self.get_parameter('proximity_roi_frac').value
        self._depth_topic: str = self.get_parameter('depth_topic').value
        self._joint_rate_limit: float = self.get_parameter('joint_rate_of_change_limit').value

        # joint_limits: {name: (q_min, q_max, dq_max, tau_max)}
        self._joint_limits: dict[str, tuple[float, float, float, float]] = {
            name: tuple(p.value)
            for name, p in self.get_parameters_by_prefix('joint_limits').items()
        }

        self._estop_active = False
        self._estop_reason = EstopFlag.REASON_NONE
        self._estop_detail = ''
        self._shm = self._open_shm(self._estop_shm_name)

        # comms watchdog: stream → last-rx seconds (key present == armed).
        self._last_rx: dict[str, float] = {}
        self._stream_timeout = {
            'arm': self._cmd_arm_timeout_s,
            'low': self._cmd_low_timeout_s,
        }
        self._last_loop_s: float | None = None

        self._validated_pub = self.create_publisher(
            JointCmdChunk, '/onboard/safety/validated_joint_chunk', _RELIABLE_QOS)
        self._estop_pub = self.create_publisher(
            EstopFlag, '/onboard/safety/estop', _RELIABLE_QOS)
        self.create_subscription(
            JointCmdChunk, '/onboard/cmd/arm', lambda m: self._on_chunk(m, 'arm'), _RELIABLE_QOS)
        self.create_subscription(
            JointCmdChunk, '/onboard/cmd/low', lambda m: self._on_chunk(m, 'low'), _RELIABLE_QOS)
        if self._proximity_enable:
            self.create_subscription(Image, self._depth_topic, self._on_depth, _BEST_EFFORT_QOS)

        rate = self._loop_rate_hz if self._loop_rate_hz > 0 else 100
        self._loop_period = 1.0 / rate
        self.create_timer(self._loop_period, self._loop)
        hb = self._estop_heartbeat_hz if self._estop_heartbeat_hz > 0 else 1.0
        self.create_timer(1.0 / hb, self._publish_estop)

        self.get_logger().info(
            f'safety_monitor ready ({len(self._joint_limits)} joints, '
            f'{rate}Hz loop, joint_limit_estop={self._joint_limit_estop}, '
            f'proximity={self._proximity_enable}, latch={self._estop_latch})')

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _open_shm(self, name: str) -> shared_memory.SharedMemory:
        try:
            shm = shared_memory.SharedMemory(name=name, create=True, size=1)
        except FileExistsError:
            shm = shared_memory.SharedMemory(name=name, create=False)
        shm.buf[0] = 0
        return shm

    def _on_chunk(self, chunk: JointCmdChunk, stream: str) -> None:
        self._last_rx[stream] = self._now_s()  # arm watchdog (even if dropped below)
        if self._estop_active and self._estop_latch:
            return  # latched — drop until restart (TODO: reset path)
        out_steps = []
        prev_q: dict[str, float] = {}  # joint → last step's q (within this chunk)
        for i, step in enumerate(chunk.steps):
            ok, reason, detail, out = self._validate_and_clamp(step)
            if not ok:
                self._trigger_estop(reason, f'chunk {chunk.chunk_id} step {i}: {detail}')
                return  # one bad step drops the whole chunk
            rate = self._check_rate(step, prev_q)
            if rate is not None:
                name, dq = rate
                self._trigger_estop(
                    EstopFlag.REASON_JOINT_LIMIT,
                    f'chunk {chunk.chunk_id} step {i}: rate {name} dq {dq:.3f} > {self._joint_rate_limit:.3f}')
                return
            for n, q in zip(step.joint_names, step.q):
                prev_q[n] = q
            out_steps.append(out)
        if self._estop_active:  # non-latching: clean chunk clears
            self._clear_estop()
        out_chunk = JointCmdChunk()
        out_chunk.header = chunk.header
        out_chunk.chunk_id = chunk.chunk_id
        out_chunk.steps = out_steps
        self._validated_pub.publish(out_chunk)

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
            if self._joint_limit_estop:
                if not q_min <= q[i] <= q_max:
                    return False, EstopFlag.REASON_JOINT_LIMIT, f'{name} q {q[i]:.3f} out of [{q_min:.3f},{q_max:.3f}]', None
                if abs(dq[i]) > dq_max:
                    return False, EstopFlag.REASON_JOINT_LIMIT, f'{name} dq {dq[i]:.3f} over {dq_max:.3f}', None
                if abs(tau[i]) > tau_max:
                    return False, EstopFlag.REASON_JOINT_LIMIT, f'{name} tau {tau[i]:.3f} over {tau_max:.3f}', None
            else:
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

    def _check_rate(self, step, prev_q: dict):
        """Δq/tick between consecutive chunk steps; return (joint, dq) on violation."""
        for name, q in zip(step.joint_names, step.q):
            if name in prev_q and abs(q - prev_q[name]) > self._joint_rate_limit:
                return name, q - prev_q[name]
        return None

    def _on_depth(self, img: Image) -> None:
        if self._estop_active:
            return
        if img.encoding not in ('16UC1', 'mono16'):
            return  # only raw 16-bit depth (mm) supported
        buf = np.frombuffer(bytes(img.data), dtype=np.uint16)
        if buf.size != img.width * img.height:
            return
        frame = buf.reshape(img.height, img.width)
        f = self._proximity_roi_frac
        y0, y1 = int(img.height * (1 - f) / 2), int(img.height * (1 + f) / 2)
        x0, x1 = int(img.width * (1 - f) / 2), int(img.width * (1 + f) / 2)
        roi = frame[y0:y1, x0:x1]
        thr_mm = self._proximity_min_dist_m * 1000.0
        close = int(np.count_nonzero((roi > 0) & (roi < thr_mm)))
        if close > self._proximity_depth_pixel_thresh:
            self._trigger_estop(
                EstopFlag.REASON_PROXIMITY,
                f'{close}px within {self._proximity_min_dist_m}m')

    def _loop(self) -> None:
        now = self._now_s()
        # self-watchdog: loop overran its period (skip first tick).
        if self._last_loop_s is not None and not self._estop_active:
            dt = now - self._last_loop_s
            if dt > self._loop_overrun_factor * self._loop_period:
                self._trigger_estop(
                    EstopFlag.REASON_WATCHDOG, f'loop overrun {dt * 1e3:.1f}ms')
        self._last_loop_s = now
        # comms watchdog: armed stream silent past its timeout.
        if not self._estop_active:
            for stream, last in self._last_rx.items():
                if now - last > self._stream_timeout[stream]:
                    self._trigger_estop(
                        EstopFlag.REASON_COMMS_TIMEOUT,
                        f'{stream} silent {now - last:.2f}s')
                    break

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
    gc.disable()  # remove GC jitter from the 200 ms E-STOP budget (REQ-35)
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
