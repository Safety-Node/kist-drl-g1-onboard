"""
100 Hz control loop driving the G1 SDK (REQ-34) [TASK-34].

Subscriptions:
- /onboard/cmd/vel (Twist)                       → velocity_buf → LocoClient.Move
- /onboard/cmd/loco (LocoCommand)                LocoClient FSM dispatch (no buffer)
- /onboard/safety/estop (EstopFlag)              E-STOP DDS context
- /onboard/safety/validated_joint_chunk (JointCmdChunk)  → joint_buf → rt/arm_sdk
- POSIX SHM byte 'safety_flag'                   zero-latency E-STOP poll

Publications:
- /onboard/motor/buf_state (BufState)            telemetry → comm_bridge → PC

Dispatch:
- VELOCITY → LocoClient.Move(vx,vy,vyaw)  (continuous walking)
- LOCO     → LocoClient.<method>() from LocoCommand.action  (discrete FSM, no buffer)
- ESTOP    → flush + LocoClient.<estop_loco_action>()  (SHM byte OR DDS active)
- JOINT    → rt/arm_sdk from joint_buf (weight=motor_cmd[29].q). ARM/WAIST only
            (idx 12-28); legs owned by LocoClient. Empty queue holds last step.

E-STOP triggers (rising edge): SHM 'safety_flag'==1 OR EstopFlag.active — both set
by safety_monitor; this node only honours them (no internal estop logic).

dry_run / SDK: _init_sdk falls back to log-only if dry_run or SDK fails. arm_sdk
init is isolated — its failure disables the arm path but never LocoClient.

Traps:
- estop_loco_action is a yaml string → LocoClient method; validated at startup.
- _patch_channel_factory joins rclpy's domain (shared libddsc). network_interface
  is ignored by the patch — robot link decided by cyclonedds.xml.

TODO (not needed for the velocity + fixed-arm demo; see Notion tasks):
- [A] chunk crossfade (chunk_id transition) + arm weight ramp (0↔1, ~2 s).
- [B] busy-wait 100 Hz timer + systemd unit (CPUAffinity/Nice/Restart/MEMLOCK).
- [C] rt/lowcmd (low/whole-body) path — arm_sdk only for now.
- [D] HW-integration checks (tick jitter, ramp timing, overflow, typo-raise).
"""
import gc
from multiprocessing import shared_memory

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from g1_onboard_msgs.msg import BufState, EstopFlag, JointCmdChunk, LocoCommand

import unitree_sdk2py.core.channel as _sdk_ch
from unitree_sdk2py.core.channel import ChannelFactory, ChannelPublisher
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

from .action_queue import ActionQueue, JointCommand, VelocityCommand
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

# G1 29-DOF joint name → motor index (matches sensors joint_state_node order +
# unitree_sdk2py G1JointIndex). rt/arm_sdk uses motor_cmd[29].q as the blend weight.
_G1_JOINT_INDEX = {
    'left_hip_pitch': 0, 'left_hip_roll': 1, 'left_hip_yaw': 2, 'left_knee': 3,
    'left_ankle_pitch': 4, 'left_ankle_roll': 5,
    'right_hip_pitch': 6, 'right_hip_roll': 7, 'right_hip_yaw': 8, 'right_knee': 9,
    'right_ankle_pitch': 10, 'right_ankle_roll': 11,
    'waist_yaw': 12, 'waist_roll': 13, 'waist_pitch': 14,
    'left_shoulder_pitch': 15, 'left_shoulder_roll': 16, 'left_shoulder_yaw': 17,
    'left_elbow': 18, 'left_wrist_roll': 19, 'left_wrist_pitch': 20, 'left_wrist_yaw': 21,
    'right_shoulder_pitch': 22, 'right_shoulder_roll': 23, 'right_shoulder_yaw': 24,
    'right_elbow': 25, 'right_wrist_roll': 26, 'right_wrist_pitch': 27, 'right_wrist_yaw': 28,
}
_ARM_SDK_WEIGHT_IDX = 29              # kNotUsedJoint — motor_cmd[29].q = enable/blend weight
_ARM_SDK_INDICES = set(range(12, 29))  # waist + both arms; legs (0-11) owned by LocoClient


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
        self._last_joint_pop_ns = 0
        self._arm_pub = None      # rt/arm_sdk ChannelPublisher (None in dry-run / on failure)
        self._arm_cmd = None      # persistent LowCmd_ reused each write
        self._crc = None
        self._last_joint = None   # last popped JointCommand (underflow hold)

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

    # --- setup ---
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
        except Exception as e:  # no robot / SDK error → dry-run, node stays up
            self.get_logger().error(f'SDK init failed ({e}) — falling back to dry_run')
            return None
        # arm_sdk publisher — isolated so its failure never disables LocoClient.
        try:
            self._arm_pub = ChannelPublisher('rt/arm_sdk', LowCmd_)
            self._arm_pub.Init()
            self._arm_cmd = unitree_hg_msg_dds__LowCmd_()
            self._crc = CRC()
        except Exception as e:
            self._arm_pub = None
            self.get_logger().error(f'arm_sdk init failed ({e}) — arm joint path disabled, loco OK')
        return loco

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
        # Unpack steps into joint_buf; control loop paces them to rt/arm_sdk at 100 Hz.
        # TODO(REQ-34) [TASK-34]: chunk_id-transition crossfade (TBD) — simple append for now.
        for step in chunk.steps:
            self._queue.push_joint(JointCommand(joint=step))

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
        jc = self._queue.pop_joint()
        if jc is not None:
            self._last_joint = jc
            self._last_joint_pop_ns = self.get_clock().now().nanoseconds
        else:
            jc = self._last_joint  # underflow — hold last step
        if jc is None:
            return  # no arm command yet → arm_sdk stays silent, LocoClient owns the arm
        self._publish_arm(jc.joint)

    def _publish_arm(self, cmd) -> None:
        # arm joints only (waist + arms); legs skipped — LocoClient owns them.
        if self._arm_pub is None:  # dry-run / arm disabled — log the mapping
            mapped = [(n, _G1_JOINT_INDEX.get(n), round(q, 3))
                      for n, q in zip(cmd.joint_names, cmd.q)
                      if _G1_JOINT_INDEX.get(n) in _ARM_SDK_INDICES]
            self.get_logger().info(f'[dry] arm_sdk weight={cmd.weight:.2f} {mapped}')
            return
        self._arm_cmd.motor_cmd[_ARM_SDK_WEIGHT_IDX].q = float(cmd.weight)  # enable/blend
        for i, name in enumerate(cmd.joint_names):
            idx = _G1_JOINT_INDEX.get(name)
            if idx not in _ARM_SDK_INDICES:
                continue  # unknown or leg joint — skip (loco owns legs)
            m = self._arm_cmd.motor_cmd[idx]
            m.q = float(cmd.q[i]); m.dq = float(cmd.dq[i])
            m.kp = float(cmd.kp[i]); m.kd = float(cmd.kd[i]); m.tau = float(cmd.tau_ff[i])
        self._arm_cmd.crc = self._crc.Crc(self._arm_cmd)
        self._arm_pub.Write(self._arm_cmd)

    def _enter_estop(self) -> None:
        self._queue.flush()
        self._last_joint = None
        self.get_logger().warn(f'E-STOP — flush + LocoClient.{self._estop_loco_action}()')
        if self._loco is not None:
            getattr(self._loco, self._estop_loco_action)()
        # Release arm_sdk (weight→0) so LocoClient reclaims the arm. (1→0 ramp = TBD)
        if self._arm_pub is not None:
            self._arm_cmd.motor_cmd[_ARM_SDK_WEIGHT_IDX].q = 0.0
            self._arm_cmd.crc = self._crc.Crc(self._arm_cmd)
            self._arm_pub.Write(self._arm_cmd)

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
