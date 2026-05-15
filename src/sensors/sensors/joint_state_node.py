"""joint_state_node — publishes JointState + Imu read from the G1 SDK lowstate stream.

The G1 SDK exposes a single channel (`rt/lf/lowstate`) that carries both joint
encoder values *and* the body IMU. We unpack both here so we only subscribe
to the SDK channel once — saves CPU and avoids two consumers racing on the
same DDS reader.

Outputs:
  /onboard/sensors/joint_states (sensor_msgs/JointState) — q, dq, tau_est
  /onboard/sensors/imu/data     (sensor_msgs/Imu)        — orientation, gyro, accel

TODO(REQ-42): subscribe to G1 SDK lowstate via unitree_sdk2_python.
TODO(REQ-42): publish JointState at publish_rate_hz (default 100 Hz).
TODO(REQ-42): publish Imu at the same cadence (lowstate already carries IMU).
TODO(REQ-42): tag JointState.header.frame_id default; tag Imu.header.frame_id
              with `imu_frame_id` param (default 'base_link').
"""
import rclpy
from rclpy.node import Node


class JointStateNode(Node):
    def __init__(self) -> None:
        super().__init__('joint_state_node')
        # TODO(REQ-42): declare params (publish_rate_hz, network_interface, domain_id, imu_frame_id)
        # TODO(REQ-42): initialise unitree_sdk2_python channel subscriber for lowstate
        # TODO(REQ-42): create publisher /onboard/sensors/joint_states (sensor_msgs/JointState)
        # TODO(REQ-42): create publisher /onboard/sensors/imu/data (sensor_msgs/Imu)
        # TODO(REQ-42): create timer that unpacks the most recent lowstate frame
        #               into a JointState + Imu pair and publishes both
        self.get_logger().info('joint_state_node started (TBD — publishes JointState + Imu)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
