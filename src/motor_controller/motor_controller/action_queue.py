"""
Lock-free Ring Buffer for safety-validated motor commands.

Two independent FIFOs:
  - velocity_buf  : LocoClient.Move(vx, vy, vyaw) commands           (hardware handles inertia)
  - joint_buf     : rt/arm_sdk joint commands (upper-body, weight)   (VLA action chunks)

ESTOP and LocoCommand bypass this queue:
  - ESTOP        : motor_controller calls _execute_estop() immediately, which calls
                   flush() on this queue to drop all pending commands atomically.
  - LocoCommand  : routed straight to LocoClient method (no buffering for one-shot RPCs).

TODO(REQ-34): implement push / pop / flush with a single-producer single-consumer SPSC pattern.
TODO(REQ-34): expose buffer fill ratio for /onboard/motor/buf_state monitoring.
"""
from collections import deque
from dataclasses import dataclass
from enum import auto, Enum
from typing import Any, Deque


class CmdMode(Enum):
    VELOCITY_CMD = auto()
    JOINT_CMD = auto()
    ESTOP = auto()


@dataclass
class MotorCommand:
    mode: CmdMode
    payload: Any   # geometry_msgs/Twist for VELOCITY, joint vector for JOINT, None for ESTOP
    stamp_ns: int = 0


class ActionQueue:
    """Velocity and joint commands queued in separate ring buffers."""

    def __init__(self, slots: int = 64) -> None:
        self._slots = slots
        self._velocity_buf: Deque[MotorCommand] = deque(maxlen=slots)
        self._joint_buf: Deque[MotorCommand] = deque(maxlen=slots)

    # TODO(REQ-34): make push/pop lock-free (SPSC, atomic head/tail).
    def push(self, cmd: MotorCommand) -> None:
        raise NotImplementedError('TODO(REQ-34): implement push')

    def pop_velocity(self) -> MotorCommand | None:
        raise NotImplementedError('TODO(REQ-34): implement pop_velocity')

    def pop_joint(self) -> MotorCommand | None:
        raise NotImplementedError('TODO(REQ-34): implement pop_joint')

    def flush(self) -> None:
        """E-STOP path: drop every pending command atomically."""
        raise NotImplementedError('TODO(REQ-34): implement flush')

    def fill_ratio(self) -> tuple[float, float]:
        """Return (velocity_fill, joint_fill) in [0.0, 1.0] for telemetry."""
        return (len(self._velocity_buf) / self._slots,
                len(self._joint_buf) / self._slots)
