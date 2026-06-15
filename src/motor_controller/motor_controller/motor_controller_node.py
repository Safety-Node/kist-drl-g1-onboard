"""
100 Hz control loop driving the G1 SDK (REQ-34) [TASK-34].

Subscriptions:
- /onboard/cmd/vel (Twist)                       → velocity_buf → LocoClient.Move
- /onboard/cmd/loco (LocoCommand)                LocoClient FSM dispatch (no buffer)
- /onboard/safety/estop (EstopFlag)              E-STOP DDS context
- /onboard/safety/validated_joint_chunk (JointCmdChunk)  joint path — TBD (chunk)
- POSIX SHM byte 'safety_flag'                   zero-latency E-STOP poll

Publications:
- /onboard/motor/buf_state (BufState)            telemetry → comm_bridge → PC

Dispatch:
- VELOCITY → LocoClient.Move(vx,vy,vyaw)  (continuous walking; PC NavigationProvider)
- LOCO     → LocoClient.<method>() from LocoCommand.action  (discrete FSM, bypasses queue)
- ESTOP    → flush + LocoClient.<estop_loco_action>()  (SHM byte OR DDS active)
- JOINT    → rt/arm_sdk / rt/lowcmd publish from joint_buf  — TBD (safety_monitor chunk path)

E-STOP triggers (rising edge in _control_loop): SHM 'safety_flag'==1 OR EstopFlag.active.
Both are set by safety_monitor; this node only honours them (no internal estop logic).

dry_run / SDK: _init_sdk falls back to dry_run (log-only) if dry_run=true or the SDK
fails to connect, so the node runs on the bench without a robot.

Real-time: gc.disable() in main(); SHM polled every tick. busy_wait_fraction /
systemd Nice/CPUAffinity are TODO(REQ-38).

Traps:
- estop_loco_action is a yaml string → LocoClient method; validated at startup.
- ChannelFactory shares libddsc with rclpy → _patch_channel_factory joins the
  existing domain (Domain() after rclpy.init raises). Mirrors imu_node.

TBD (chunk path, unused in the velocity-only demo):
- _on_chunk: chunk_id-transition detect → crossfade(old_tail, new) → joint_buf.
- _dispatch_joint: pop joint_buf → rt/arm_sdk/rt/lowcmd, empty-queue last-step hold.
- _enter_estop arm weight 1.0→0.0 ramp.
"""
import gc
from multiprocessing import shared_memory

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from g1_onboard_msgs.msg import BufState, EstopFlag, JointCmdChunk, LocoCommand

import unitree_sdk2py.core.channel as _sdk_ch
from unitree_sdk2py.core.channel import ChannelFactory
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

from .action_queue import ActionQueue, VelocityCommand
from .queue_aggregate import crossfade  # noqa: F401  (canonical chunk crossfade, TBD)


_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# LocoCommand.action → LocoClient method name (verified against g1_loco_client).
_LOCO_METHOD = {
    LocoCommand.ACTION_DAMP:          'Damp',
    LocoCommand.ACTION_ZERO_TORQUE:   'ZeroTorque',
    LocoCommand.ACTION_STAND_UP:      'Lie2StandUp',   # TODO(REQ-34): confirm vs Squat2StandUp per start posture
    LocoCommand.ACTION_BALANCE_STAND: 'BalanceStand',  # takes balance_mode arg (uses LocoCommand.param)
    LocoCommand.ACTION_LOW_STAND:     'LowStand',
    LocoCommand.ACTION_HIGH_STAND:    'HighStand',
    LocoCommand.ACTION_SIT_DOWN:      'Sit',
}


def _patch_channel_factory() -> None:
    """Join rclpy's existing DDS domain instead of creating one (shared libddsc).

    Mirrors imu_node/joint_state_node: Domain(0) after rclpy.init() raises, so
    ChannelFactory.Init joins via DomainParticipant. LocoClient then runs its RPC
    channels on that participant.
    """
    from cyclonedds.domain import DomainParticipant as _DDP

    def _init(self, id: int, networkInterface=None, qos=None) -> bool:
        if self.__class__._ChannelFactory__initialized:
            return True
        with self.__class__._ChannelFactory__init_lock:
            if self.__class__._ChannelFactory__initialized:
                return True
            try:
                self.__class__._ChannelFactory__participant = _DDP(id)
            except Exception:
                return False
            self.__class__._ChannelFactory__qos = qos
            self.__class__._ChannelFactory__initialized = True
            return True

    _sdk_ch.ChannelFactory.Init = _init


class MotorControllerNode(Node):
    def __init__(self) -> None:
        super().__init__(
            'motor_controller',
            automatically_declare_parameters_from_overrides=True,
        )
        self._control_rate_hz: int = self.get_parameter('control_rate_hz').value
        self._ring_slots: int = self.get_parameter('ring_buffer_slots').value
        self._estop_loco_action: str = self.get_parameter('estop_loco_action').value
        self._network_interface: str = self.get_parameter('network_interface').value
        self._domain_id: int = self.get_parameter('domain_id').value
        self._estop_shm_name: str = self.get_parameter('estop_shm_name').value
        self._dry_run: bool = self.get_parameter('dry_run').value
        self._buf_state_rate_hz: float = self.get_parameter('buf_state_rate_hz').value

        if not hasattr(LocoClient, self._estop_loco_action):
            raise RuntimeError(
                f'estop_loco_action {self._estop_loco_action!r} is not a LocoClient method')

        self._queue = ActionQueue(slots=self._ring_slots)
        self._shm = self._open_shm(self._estop_shm_name)
        self._estop_active = False
        self._dds_estop = False
        self._tick_count = 0
        self._last_joint_pop_ns = 0  # set by chunk path (TBD)

        self._loco = self._init_sdk()

        self._buf_pub = self.create_publisher(BufState, '/onboard/motor/buf_state', _RELIABLE_QOS)
        self.create_subscription(Twist, '/onboard/cmd/vel', self._on_cmd_vel, _RELIABLE_QOS)
        self.create_subscription(LocoCommand, '/onboard/cmd/loco', self._on_cmd_loco, _RELIABLE_QOS)
        self.create_subscription(EstopFlag, '/onboard/safety/estop', self._on_estop, _RELIABLE_QOS)
        self.create_subscription(
            JointCmdChunk, '/onboard/safety/validated_joint_chunk', self._on_chunk, _RELIABLE_QOS)

        period = 1.0 / (self._control_rate_hz if self._control_rate_hz > 0 else 100)
        self._control_period = period
        self.create_timer(period, self._control_loop)
        bs = self._buf_state_rate_hz if self._buf_state_rate_hz > 0 else 10.0
        self._buf_state_period = 1.0 / bs
        self.create_timer(self._buf_state_period, self._publish_buf_state)

        self.get_logger().info(
            f'motor_controller ready ({self._control_rate_hz}Hz, slots={self._ring_slots}, '
            f'estop_action={self._estop_loco_action}, dry_run={self._loco is None})')

    # --- setup ---LocoClient
    def _init_sdk(self) -> LocoClient | None:
        if self._dry_run:
            self.get_logger().warn('dry_run=true — SDK disabled, dispatch logs only')
            return None
        try:
            _patch_channel_factory()
            ChannelFactory().Init(self._domain_id, self._network_interface)
            loco = LocoClient()
            loco.SetTimeout(10.0)
            loco.Init()
            return loco
        except Exception as e:  # no robot / SDK error → dry-run, node stays up
            self.get_logger().error(f'SDK init failed ({e}) — falling back to dry_run')
            return None

    def _open_shm(self, name: str) -> shared_memory.SharedMemory:
        # safety_monitor owns this byte; create-or-attach so motor runs without it.
        try:
            shm = shared_memory.SharedMemory(name=name, create=False)
        except FileNotFoundError:
            shm = shared_memory.SharedMemory(name=name, create=True, size=1)
            shm.buf[0] = 0
        return shm

    # --- producers (subscriptions) ---
    def _on_cmd_vel(self, twist: Twist) -> None:
        self._queue.push_velocity(VelocityCommand(twist=twist))

    def _on_cmd_loco(self, msg: LocoCommand) -> None:
        name = _LOCO_METHOD.get(msg.action)
        if name is None:
            self.get_logger().warn(f'unknown loco action {msg.action}')
            return
        if self._loco is None:
            self.get_logger().info(f'[dry] loco {name}(param={msg.param})')
            return
        fn = getattr(self._loco, name)
        fn(msg.param) if msg.action == LocoCommand.ACTION_BALANCE_STAND else fn()

    def _on_estop(self, flag: EstopFlag) -> None:
        self._dds_estop = flag.active  # SHM byte is the fast path; this is DDS context

    def _on_chunk(self, chunk: JointCmdChunk) -> None:
        # TODO(REQ-34) [TASK-34]: chunk path (safety_monitor) — detect chunk_id
        # transition, crossfade(old_tail, new) on overlap, unpack steps into
        # joint_buf. Unused in the velocity-only demo.
        pass

    # --- control loop ---
    def _control_loop(self) -> None:
        self._tick_count += 1
        estop = bool(self._shm.buf[0]) or self._dds_estop
        if estop and not self._estop_active:  # rising edge — one-shot
            self._enter_estop()
        self._estop_active = estop
        if estop:
            return
        vc = self._queue.pop_velocity()
        if vc is not None:
            self._loco_move(vc.twist)
        self._dispatch_joint()

    def _loco_move(self, twist: Twist) -> None:
        vx, vy, vyaw = twist.linear.x, twist.linear.y, twist.angular.z
        if self._loco is None:
            self.get_logger().info(f'[dry] Move({vx:.2f},{vy:.2f},{vyaw:.2f})')
            return
        self._loco.Move(vx, vy, vyaw, True)  # continuous velocity stream

    def _dispatch_joint(self) -> None:
        # TODO(REQ-34) [TASK-34]: pop joint_buf → rt/arm_sdk / rt/lowcmd publish,
        # empty-queue last-step republish. Fed by the chunk path (TBD).
        pass

    def _enter_estop(self) -> None:
        self._queue.flush()
        self.get_logger().warn(f'E-STOP — flush + LocoClient.{self._estop_loco_action}()')
        if self._loco is not None:
            getattr(self._loco, self._estop_loco_action)()
        # TODO(REQ-35) [TASK-34]: arm weight 1.0→0.0 ramp (arm/chunk path, TBD).

    # --- telemetry ---
    def _publish_buf_state(self) -> None:
        vfill, jfill = self._queue.fill_ratio()
        measured = self._tick_count / self._buf_state_period
        self._tick_count = 0
        msg = BufState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.velocity_fill = vfill
        msg.joint_fill = jfill
        msg.control_rate_hz = float(measured)
        age_ns = self.get_clock().now().nanoseconds - self._last_joint_pop_ns
        msg.last_cmd_age_ms = int(age_ns / 1e6) if self._last_joint_pop_ns else 0
        msg.underrun_count = self._queue.underrun_count
        msg.overrun_count = self._queue.overrun_count
        self._buf_pub.publish(msg)

    def destroy_node(self) -> None:
        try:
            self._shm.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    gc.disable()  # remove GC jitter from the 100 Hz loop (REQ-38)
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
