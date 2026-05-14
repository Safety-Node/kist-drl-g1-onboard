"""motor_controller_node — 20Hz control loop driving the G1 SDK.

Subscribes:
  - /onboard/safety/validated_cmd  (DDS + shared memory)   [from safety_monitor]
  - /onboard/safety/estop          (DDS)                   [from safety_monitor]
  - shared-memory 'safety_flag' byte                       [zero-latency E-STOP path]

Publishes:
  - /onboard/motor/buf_state       (status / telemetry)    [to comm_bridge → PC]

Modes (per spec):
  - VELOCITY_CMD  → LocoClient.Move(vx, vy, vyaw)
  - JOINT_CMD     → rt/lowcmd (chunked, blended via QueueAggregate)
  - ESTOP         → Damp() + buffer flush

Real-time strategy:
  - gc.disable() before main loop                          (TODO(REQ-38))
  - busy-wait hybrid timer (80% sleep + 20% busy)          (TODO(REQ-38))
  - systemd CPUAffinity=1, Nice=-20                        (see systemd/motor_controller.service)
"""
import rclpy
from rclpy.node import Node

from .action_queue import ActionQueue
from .queue_aggregate import crossfade  # noqa: F401  (used once VLA chunks land)


class MotorControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('motor_controller')

        # TODO(REQ-34): declare params (control_rate_hz, ring_buffer_slots, chunk_size, …)
        # TODO(REQ-34): construct ActionQueue from params
        # TODO(REQ-34): init unitree_sdk2_python LocoClient / lowcmd publisher
        # TODO(REQ-35): open shared-memory 'safety_flag' read-only
        # TODO(REQ-34): create publisher /onboard/motor/buf_state
        # TODO(REQ-34, REQ-35): subscribe /onboard/safety/validated_cmd, /onboard/safety/estop
        # TODO(REQ-34, REQ-38): launch 20 Hz control loop thread (busy-wait hybrid)

        self._queue = ActionQueue(slots=64)
        self.get_logger().info('motor_controller_node started (TBD)')

    # TODO(REQ-34, REQ-38): def _control_loop(self) -> None  ← 20 Hz busy-wait hybrid
    # TODO(REQ-34): def _on_validated_cmd(self, msg) -> None
    # TODO(REQ-35): def _on_estop(self, msg) -> None
    # TODO(REQ-34): def _execute_velocity(self, cmd) -> None  # LocoClient.Move
    # TODO(REQ-34): def _execute_joint(self, cmd) -> None     # rt/lowcmd
    # TODO(REQ-35): def _execute_estop(self) -> None          # Damp() + flush


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
