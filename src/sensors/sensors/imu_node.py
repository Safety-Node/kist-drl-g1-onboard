"""
G1 IMU streaming — base + left/right ankle.

Owns all IMU outputs as a single concern. 2026-05-23 refactor reverses the
2026-05-15 fan-out decision (joint_state_node previously published base IMU
as a side-effect of its lowstate subscription). Now:
- joint_state_node owns JointState only.
- imu_node owns the full IMU surface (base + ankles).
Both nodes subscribe to G1 SDK lowstate independently; DDS multi-subscriber
cost is negligible at the 100 Hz lowstate rate, and ownership becomes
symmetric with the topic prefix (node name ↔ /onboard/sensors/imu/*).

Publications:
- /onboard/sensors/imu/data         (sensor_msgs/Imu)  base IMU,
                                                       frame_id = base_link
- /onboard/sensors/imu/ankle_left   (sensor_msgs/Imu)  frame_id = ankle_left_link
- /onboard/sensors/imu/ankle_right  (sensor_msgs/Imu)  frame_id = ankle_right_link

Open question: G1 SDK ankle IMU exposure is unconfirmed. Base IMU rides on
the existing lowstate channel; ankle IMUs may come from a separate SDK
endpoint (TBD — KIST or SDK doc check). Until confirmed, ankle topics
publish a placeholder so downstream wiring (comm_bridge relay + PC
subscriber) can be exercised end-to-end.

TODO(REQ-42) [TASK-999]: declare params (publish_rate_hz + 3 frame_ids).
TODO(REQ-42) [TASK-999]: base IMU from G1 SDK lowstate (own subscription).
TODO(REQ-42) [TASK-999]: confirm G1 SDK ankle IMU source; swap placeholder for real read.
TODO(REQ-42) [TASK-999]: paired timestamp if both ankles share a frame from the same SDK sample.
"""
import rclpy
from rclpy.node import Node


class ImuNode(Node):
    def __init__(self) -> None:
        super().__init__('imu_node')
        # TODO(REQ-42) [TASK-999]: wire 3 publishers + timer.
        #   Base IMU: real (from G1 SDK lowstate, independent subscription
        #             from joint_state_node).
        #   Ankle L/R: placeholder publish until SDK source confirmed.
        self.get_logger().info(
            'imu_node started (TBD — publishes base + ankle_left/right Imu)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
