"""
Ankle IMU streaming (G1 left/right ankle).

2026-05-22 added — KIST mail (Yim,Sehyuk). GearSonic (VLA balance correction)
expects ankle IMU as a key input alongside the base IMU emitted by
joint_state_node.

Publications:
- /onboard/sensors/imu/ankle_left   (sensor_msgs/Imu)  frame_id = ankle_left_link
- /onboard/sensors/imu/ankle_right  (sensor_msgs/Imu)  frame_id = ankle_right_link

Open question: G1 SDK ankle IMU exposure is unconfirmed. While the source is
TBD this node publishes a placeholder (zero quat, zero vectors) at
publish_rate_hz so downstream wiring (comm_bridge relay + PC subscriber) can
be exercised end-to-end. Replace with the real source once the SDK channel
is confirmed.

TODO(REQ-42) [TASK-999]: declare params (publish_rate_hz + per-side frame_id).
TODO(REQ-42) [TASK-999]: confirm G1 SDK ankle IMU source; swap placeholder for real read.
TODO(REQ-42) [TASK-999]: paired timestamp from same lowstate sample if both ankles share a frame.
"""
import rclpy
from rclpy.node import Node


class ImuAnkleNode(Node):
    def __init__(self) -> None:
        super().__init__('imu_ankle_node')
        # TODO(REQ-42) [TASK-999]: wire publishers + timer (placeholder publish
        #                          until G1 SDK ankle IMU source is confirmed).
        self.get_logger().info(
            'imu_ankle_node started (TBD — publishes ankle_left/right Imu, currently placeholder)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuAnkleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
