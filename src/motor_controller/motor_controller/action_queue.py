"""
Lock-free Ring Buffer for safety-validated motor commands.

Two independent FIFOs, one per actuator surface:
  - velocity_buf  : LocoClient.Move(vx, vy, vyaw) commands           (hardware handles inertia)
  - joint_buf     : rt/arm_sdk joint commands (upper-body, weight)   (VLA action chunks)

The queue stores ONLY commands that need buffering. The 4-mode taxonomy
in motor_controller_node (VELOCITY_CMD / JOINT_CMD / LOCO_CMD / ESTOP) is
a DISPATCHER concern, not a queue concern:

  - LOCO_CMD : one-shot LocoClient.<method>() RPCs do not buffer; they
               dispatch directly from the LocoCommand subscription callback.
  - ESTOP    : the dispatcher calls flush() + _execute_estop() directly;
               there is no ESTOP MotorCommand carried through this queue.

Type model (decided 2026-05-16, motor_controller review):
  VelocityCommand and JointCommand are sibling dataclasses -- the Python
  class IS the discriminator (tagged union by type). No CmdMode enum,
  no `payload: Any`. Dispatchers use isinstance / match-case rather
  than reading a mode field. The queue's API splits into push_velocity /
  push_joint so the producer side cannot accidentally cross-route.

Overflow policy (decided 2026-05-16):
  Both deques use maxlen=slots, which silently drops the oldest entry on
  overflow. To still count overflow events for /onboard/motor/buf_state
  telemetry, push_velocity / push_joint check len(buf) == slots BEFORE
  appending and increment overrun_count -- without the explicit check,
  deque eviction is invisible.

TODO(REQ-34): implement push / pop / flush with a single-producer
              single-consumer SPSC pattern (atomic head/tail).
TODO(REQ-34): expose buffer fill ratio for /onboard/motor/buf_state monitoring.
"""
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque


@dataclass
class VelocityCommand:
    """Walk command awaiting LocoClient.Move(vx, vy, vyaw) dispatch."""
    # geometry_msgs.msg.Twist -- typed as Any so this module stays free of a
    # hard ROS import (helps unit-testability and keeps cold-start fast).
    twist: Any
    stamp_ns: int = 0


@dataclass
class JointCommand:
    """Joint command awaiting rt/arm_sdk publish.

    Carries the full g1_onboard_msgs/JointCmd so chunk_id / step_index and
    weight reach the dispatcher without re-packing.
    """
    # g1_onboard_msgs.msg.JointCmd
    joint: Any
    stamp_ns: int = 0


class ActionQueue:
    """Velocity and joint commands queued in separate ring buffers.

    Dispatch is by method, not by a tagged value -- push_velocity vs
    push_joint makes the producer's intent explicit and prevents the
    "single push() with mode field" ambiguity flagged in the
    2026-05-16 review.
    """

    def __init__(self, slots: int = 64) -> None:
        self._slots = slots
        # maxlen makes deque drop-oldest on overflow (our chosen policy);
        # the explicit length check inside push_* increments overrun_count
        # for telemetry. Without it, drop-oldest would be invisible.
        self._velocity_buf: Deque[VelocityCommand] = deque(maxlen=slots)
        self._joint_buf: Deque[JointCommand] = deque(maxlen=slots)
        self._overrun_count: int = 0   # combined across both bufs (matches BufState schema)
        self._underrun_count: int = 0  # incremented on pop from empty buf

    # TODO(REQ-34): make push/pop lock-free (SPSC, atomic head/tail).
    def push_velocity(self, cmd: VelocityCommand) -> None:
        """Append to velocity_buf. Overflow policy: drop-oldest (via deque maxlen).

        Increments overrun_count when the buffer was full at push time --
        the producer sees no error, but BufState.overrun_count reflects
        the dropped-oldest event for diagnostics.
        """
        raise NotImplementedError('TODO(REQ-34): implement push_velocity')

    def push_joint(self, cmd: JointCommand) -> None:
        """Append to joint_buf. Overflow policy: drop-oldest (via deque maxlen).

        Same overrun_count semantics as push_velocity. The dispatcher MAY
        also peek at cmd.joint.chunk_id / step_index to invoke
        queue_aggregate.crossfade() at chunk boundaries (optional NX-side
        fallback; canonical crossfade is on the PC).
        """
        raise NotImplementedError('TODO(REQ-34): implement push_joint')

    def pop_velocity(self) -> VelocityCommand | None:
        """Pop oldest velocity command. None when empty; increments underrun_count."""
        raise NotImplementedError('TODO(REQ-34): implement pop_velocity')

    def pop_joint(self) -> JointCommand | None:
        """Pop oldest joint command. None when empty; increments underrun_count."""
        raise NotImplementedError('TODO(REQ-34): implement pop_joint')

    def flush(self) -> None:
        """E-STOP path: drop every pending command atomically.

        Counters (overrun_count / underrun_count) are NOT touched by flush --
        flushed commands were never executed, but they were not dropped due
        to overflow either. Keep the diagnostics clean.
        """
        raise NotImplementedError('TODO(REQ-34): implement flush')

    def fill_ratio(self) -> tuple[float, float]:
        """Return (velocity_fill, joint_fill) in [0.0, 1.0] for telemetry."""
        return (len(self._velocity_buf) / self._slots,
                len(self._joint_buf) / self._slots)

    @property
    def overrun_count(self) -> int:
        """Pushes that landed on a full buffer (oldest entry was evicted)."""
        return self._overrun_count

    @property
    def underrun_count(self) -> int:
        """Pops that found the buffer empty."""
        return self._underrun_count
