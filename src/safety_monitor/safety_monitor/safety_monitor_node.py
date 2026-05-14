"""safety_monitor_node — validates all motion commands and emits E-STOP within 200 ms.

Responsibilities (all TBD — see TODO markers below):
  - subscribe /onboard/navigation/cmd_vel (Twist)               [from navigation/goto_node]
  - subscribe /onboard/cmd/joint (JointState/JointCmd)          [from comm_bridge]
  - subscribe /onboard/sensors/depth/image_raw                  [from sensors — RealSense Depth]
  - subscribe /onboard/sensors/joint_states                     [from sensors]
  - publish   /onboard/safety/validated_cmd                     [DDS, → motor_controller]
  - publish   /onboard/safety/estop                             [DDS, → motor_controller]
  - mirror E-STOP through a shared-memory flag (POSIX shm)      [zero-IPC-latency path]

Note (2026-05-14 spec change):
  - LiDAR PointCloud was removed from the input list. Proximity E-STOP relies on RealSense
    Depth only (still satisfies REQ-35 proximity check, with reduced FOV / range).

Real-time strategy (per spec):
  - gc.disable() before entering the main loop          (TODO(REQ-35))
  - shared-memory flag write < 0.01 ms                  (TODO(REQ-35))
  - systemd CPUAffinity=0, Nice=-20                     (see systemd/safety_monitor.service)
"""
import rclpy
from rclpy.node import Node


class SafetyMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('safety_monitor')

        # TODO(REQ-35): declare params (loop_rate_hz, estop_shm_name, joint_limits, …)
        # TODO(REQ-35): open / create shared-memory segment for E-STOP flag
        # TODO(REQ-35): subscribe inputs (cmd_vel, joint_cmd, depth, joint_state)
        # TODO(REQ-35): publish outputs (validated_cmd, estop)
        # TODO(REQ-35): main 100 Hz timer running validation pipeline
        # TODO(REQ-35): call gc.disable() once steady-state warm-up is done

        self.get_logger().info('safety_monitor_node started (TBD)')

    # TODO(REQ-35): def _check_joint_limits(self, cmd) -> bool
    # TODO(REQ-35): def _check_velocity_limits(self, twist) -> bool
    # TODO(REQ-35): def _check_proximity(self, depth) -> bool   # RealSense Depth only
    # TODO(REQ-35): def _check_self_collision(self, target_q, current_q) -> bool
    # TODO(REQ-35): def _trigger_estop(self, reason: str) -> None


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
