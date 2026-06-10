"""
Unitree G1 built-in LiDAR point cloud publisher.

Subscribes to the robot's DDS topic rt/utlidar/cloud and republishes on
/onboard/sensors/lidar/points as sensor_msgs/PointCloud2.

Publications:
    /onboard/sensors/lidar/points  sensor_msgs/PointCloud2  BestEffort  ~10 Hz

SDK / ROS 2 CycloneDDS coexistence
------------------------------------
unitree_sdk2py and rmw_cyclonedds_cpp share libddsc.so, so only one
process can own domain 0.  _patch_channel_factory() replaces
ChannelFactory.Init() with a version that skips Domain() creation and
creates a DomainParticipant directly on the already-active domain.
(Same approach as imu_node.)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField

import unitree_sdk2py.core.channel as _sdk_ch
from unitree_sdk2py.core.channel import ChannelFactory, ChannelSubscriber
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_


def _patch_channel_factory() -> None:
    """Replace ChannelFactory.Init to skip Domain() creation.

    rmw_cyclonedds_cpp and unitree_sdk2py share libddsc.so.  Calling
    Domain(0, config) after rclpy.init() raises DDSException because
    domain 0 already exists.  We skip straight to DomainParticipant(),
    which joins the already-active domain.
    """
    from cyclonedds.domain import DomainParticipant as _DDP

    def _init(self, id: int, networkInterface=None, qos=None) -> bool:
        if self.__class__._ChannelFactory__initialized:
            return True
        with self.__class__._ChannelFactory__init_lock:
            if self.__class__._ChannelFactory__initialized:
                return True
            try:
                self.__class__._ChannelFactory__participant = _DDP(id)
            except Exception:
                return False
            self.__class__._ChannelFactory__qos = qos
            self.__class__._ChannelFactory__initialized = True
            return True

    _sdk_ch.ChannelFactory.Init = _init


_patch_channel_factory()

_BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class LidarNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_node')

        self.declare_parameter('frame_id', 'utlidar_lidar')
        self.declare_parameter('network_interface', 'eth0')
        self.declare_parameter('domain_id', 0)

        self._frame_id: str = self.get_parameter('frame_id').value
        network_iface: str = self.get_parameter('network_interface').value
        domain_id: int = self.get_parameter('domain_id').value

        ChannelFactory().Init(domain_id, network_iface)

        self._pub = self.create_publisher(
            PointCloud2, '/onboard/sensors/lidar/points', _BEST_EFFORT_QOS)

        self._sub = ChannelSubscriber('rt/utlidar/cloud', PointCloud2_)
        self._sub.Init(self._on_cloud, 10)

        self.get_logger().info(
            f'lidar_node ready — iface={network_iface}, domain={domain_id}, '
            f'frame_id={self._frame_id}')

    def _on_cloud(self, msg: PointCloud2_) -> None:
        out = PointCloud2()
        out.header.stamp.sec = msg.header.stamp.sec
        out.header.stamp.nanosec = msg.header.stamp.nanosec
        out.header.frame_id = self._frame_id
        out.height = msg.height
        out.width = msg.width
        out.fields = [
            PointField(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count)
            for f in msg.fields
        ]
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.data = bytes(msg.data)
        out.is_dense = msg.is_dense
        self._pub.publish(out)


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
