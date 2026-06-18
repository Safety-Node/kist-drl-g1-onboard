"""
Unitree G1 built-in LiDAR point cloud relay.

G1의 Livox Mid-360 라이다는 /utlidar/cloud_livox_mid360 (sensor_msgs/PointCloud2,
Reliable QoS)으로 발행된다. 이 노드는 해당 토픽을 구독해 표준 onboard 네임스페이스로
재발행한다. odom_node와 동일한 relay 패턴.

Publications:
    /onboard/sensors/lidar/points  sensor_msgs/PointCloud2  BestEffort
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import PointCloud2


_BE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# /utlidar/cloud_livox_mid360 is published by Unitree internal driver with RELIABLE
_SUB_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1,
)


class LidarNode(Node):

    def __init__(self) -> None:
        super().__init__('lidar_node')

        self.declare_parameter('source_topic', '/utlidar/cloud_livox_mid360')
        self.declare_parameter('frame_id', 'utlidar_lidar')

        source = self.get_parameter('source_topic').value
        self._frame_id: str = self.get_parameter('frame_id').value

        self._pub = self.create_publisher(
            PointCloud2, '/onboard/sensors/lidar/points', _BE_QOS)

        self._sub = self.create_subscription(
            PointCloud2, source, self._on_cloud, _SUB_QOS)

        self.get_logger().info(
            f'lidar_node ready — relay {source} → /onboard/sensors/lidar/points')

    def _on_cloud(self, msg: PointCloud2) -> None:
        msg.header.frame_id = self._frame_id
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
