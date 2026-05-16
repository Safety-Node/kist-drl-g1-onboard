"""
Lock-free Ring Buffer for safety-validated motor commands. [TASK-34]

Two SPSC FIFOs (per actuator surface):
  velocity_buf  : LocoClient.Move commands       (producer: _on_validated_twist)
  joint_buf     : rt/arm_sdk joint commands      (producer: _on_validated_joint)
  consumer (both)                                 : control loop

The 4-mode taxonomy (VELOCITY / JOINT / LOCO / ESTOP) is a dispatcher
concern, not a queue concern: LOCO_CMD is a one-shot RPC, ESTOP is an
event (flush + direct dispatch). Neither flows through this queue, so
there is no enum here — the dataclass type IS the discriminator.

Overflow policy: deque(maxlen=slots) drops OLDEST silently. push_*
checks len()==slots BEFORE appending so per-buf overrun counters can
record the eviction for /onboard/motor/buf_state.

Counter ownership (kept single-writer to survive MultiThreadedExecutor
without locks or GIL reliance):
  _velocity_overrun_count : velocity producer
  _joint_overrun_count    : joint producer
  _underrun_count         : control loop (sole popper)

TODO(REQ-34) [TASK-34]: implement push / pop / flush as lock-free SPSC.
"""
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque


@dataclass
class VelocityCommand:
    """Walk command awaiting LocoClient.Move(vx, vy, vyaw)."""
    twist: Any   # geometry_msgs.msg.Twist (typed Any to keep this module ROS-free)
    stamp_ns: int = 0


@dataclass
class JointCommand:
    """Joint command awaiting rt/arm_sdk publish. Carries chunk_id / step_index / weight."""
    joint: Any   # g1_onboard_msgs.msg.JointCmd
    stamp_ns: int = 0


class ActionQueue:
    """Velocity + joint commands in separate ring buffers."""

    def __init__(self, slots: int = 64) -> None:
        self._slots = slots
        self._velocity_buf: Deque[VelocityCommand] = deque(maxlen=slots)
        self._joint_buf: Deque[JointCommand] = deque(maxlen=slots)
        self._velocity_overrun_count: int = 0
        self._joint_overrun_count: int = 0
        self._underrun_count: int = 0

    def push_velocity(self, cmd: VelocityCommand) -> None:
        raise NotImplementedError('TODO(REQ-34) [TASK-34]: push_velocity')

    def push_joint(self, cmd: JointCommand) -> None:
        raise NotImplementedError('TODO(REQ-34) [TASK-34]: push_joint')

    def pop_velocity(self) -> VelocityCommand | None:
        raise NotImplementedError('TODO(REQ-34) [TASK-34]: pop_velocity')

    def pop_joint(self) -> JointCommand | None:
        raise NotImplementedError('TODO(REQ-34) [TASK-34]: pop_joint')

    def flush(self) -> None:
        """E-STOP path: drop every pending command atomically (counters untouched)."""
        raise NotImplementedError('TODO(REQ-34) [TASK-34]: flush')

    def fill_ratio(self) -> tuple[float, float]:
        return (len(self._velocity_buf) / self._slots,
                len(self._joint_buf) / self._slots)

    @property
    def overrun_count(self) -> int:
        """Sum of per-buf overruns. Each underlying counter is single-writer."""
        return self._velocity_overrun_count + self._joint_overrun_count

    @property
    def underrun_count(self) -> int:
        return self._underrun_count
