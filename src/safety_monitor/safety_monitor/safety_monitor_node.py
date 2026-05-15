"""safety_monitor_node — validates all motion commands and emits E-STOP within 200 ms.

Subscriptions:
  /onboard/navigation/cmd_vel        (geometry_msgs/Twist)        [navigation/goto_node]
  /onboard/cmd/arm                   (kist_drl_g1_msgs/JointCmd)  [comm_bridge inbound relay, rt/arm_sdk path]
  /onboard/sensors/depth/image_raw   (sensor_msgs/Image)          [sensors/camera — RealSense Depth]
  /onboard/sensors/joint_states      (sensor_msgs/JointState)     [sensors/joint_state_node]

Publications:
  /onboard/safety/validated_twist    (geometry_msgs/Twist)        [→ motor_controller velocity_buf]
  /onboard/safety/validated_joint    (kist_drl_g1_msgs/JointCmd)  [→ motor_controller joint_buf]
  /onboard/safety/estop              (kist_drl_g1_msgs/EstopFlag) [structured DDS mirror; also broadcast at heartbeat]

Side channel:
  POSIX shared-memory byte at `estop_shm_name` ("safety_flag") — motor_controller
  polls this every control tick (~250 Hz). The DDS topic is the PC-facing
  structured mirror; this byte is the zero-latency path.

Validation checks (REQ-35):
  - Joint limit       (per-joint q_min/q_max, |dq| ≤ dq_max, |τ| ≤ tau_max)
  - Velocity limit    (|linear.x|, |linear.y|, |angular.z| ≤ max_*)
  - Proximity         (RealSense Depth — count pixels closer than proximity_min_dist_m)
  - Self-collision    (placeholder; FK + coarse collision model)
  - Rate of change    (per-joint Δq per tick ≤ joint_rate_of_change_limit)
  - Comms timeout     (no fresh cmd in comms_timeout_s → EstopFlag.REASON_COMMS_TIMEOUT)

Note (2026-05-14 spec change):
  LiDAR PointCloud was removed. Proximity E-STOP uses Depth only; FOV/range
  reduced — re-evaluate before any non-demo deployment.

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

        # TODO(REQ-35): declare params (loop_rate_hz, estop_shm_name, estop_heartbeat_hz,
        #               max_linear_x/y, max_angular_z, proximity_min_dist_m,
        #               proximity_depth_pixel_thresh, comms_timeout_s,
        #               joint_rate_of_change_limit, joint_limits)
        # TODO(REQ-35): open / create POSIX shared-memory segment for the E-STOP byte flag
        # TODO(REQ-35): subscribe inputs — cmd_vel (Twist), cmd/arm (JointCmd),
        #               depth (Image), joint_states (JointState)
        # TODO(REQ-35): create publishers — validated_twist (Twist),
        #               validated_joint (JointCmd), estop (EstopFlag)
        # TODO(REQ-35): main timer at loop_rate_hz running the validation pipeline
        # TODO(REQ-35): heartbeat timer at estop_heartbeat_hz publishing EstopFlag
        #               (active flag + current reason) even when state has not changed
        # TODO(REQ-35): comms watchdog — emit REASON_COMMS_TIMEOUT if any subscribed
        #               command stream has been silent for longer than comms_timeout_s
        # TODO(REQ-35): call gc.disable() once steady-state warm-up is done

        self.get_logger().info('safety_monitor_node started (TBD)')

    # TODO(REQ-35): def _check_joint_limits(self, cmd) -> tuple[bool, str]
    # TODO(REQ-35): def _check_velocity_limits(self, twist) -> tuple[bool, str]
    # TODO(REQ-35): def _check_proximity(self, depth_image) -> tuple[bool, str]   # RealSense Depth only
    # TODO(REQ-35): def _check_self_collision(self, target_q, current_q) -> tuple[bool, str]
    # TODO(REQ-35): def _check_rate_of_change(self, target_q, current_q) -> tuple[bool, str]
    # TODO(REQ-35): def _trigger_estop(self, reason: int, detail: str) -> None
    #               # reason uses kist_drl_g1_msgs.msg.EstopFlag.REASON_* constants;
    #               # sets the SHM byte AND publishes EstopFlag on DDS.


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
