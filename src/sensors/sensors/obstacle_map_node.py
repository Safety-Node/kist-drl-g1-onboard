"""
Obstacle map node — LiDAR point cloud → 2D OccupancyGrid (TASK-53).

고정 직사각형 공간(x_min~x_max, y_min~y_max)을 균일 격자로 분할하고,
매 LiDAR 프레임마다 각 격자점 근방에 포인트가 있으면 OBSTACLE(100),
없으면 FREE(0)로 실시간 발행한다.

LiDAR 포인트는 utlidar_lidar 프레임(로봇 로컬) 기준이므로,
/onboard/sensors/location 의 로봇 pose를 이용해 map 프레임으로 변환한다.

    p_map = R(θ) * p_lidar + [x_robot + dx_lidar, y_robot + dy_lidar]

Subscriptions:
    /onboard/sensors/lidar/points   sensor_msgs/PointCloud2   BestEffort
    /onboard/sensors/location       geometry_msgs/PoseStamped BestEffort

Publications:
    /onboard/sensors/lidar/occupancy  nav_msgs/OccupancyGrid  BestEffort
"""
from __future__ import annotations

import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import PointCloud2, PointField


_BE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_FIELD_DTYPE = {
    PointField.INT8:    np.int8,
    PointField.UINT8:   np.uint8,
    PointField.INT16:   np.int16,
    PointField.UINT16:  np.uint16,
    PointField.INT32:   np.int32,
    PointField.UINT32:  np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
}


def _quat_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _parse_xyz(msg: PointCloud2) -> np.ndarray | None:
    """PointCloud2 binary data → (N, 3) float32 [x, y, z]. Returns None if empty."""
    if msg.width == 0 or not msg.data:
        return None
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ('x', 'y', 'z')):
        return None
    dtype = np.dtype([
        (f.name, _FIELD_DTYPE.get(f.datatype, np.float32))
        for f in sorted(msg.fields, key=lambda f: f.offset)
    ])
    try:
        arr = np.frombuffer(bytes(msg.data), dtype=dtype)
    except Exception:
        return None
    xyz = np.stack([
        arr['x'].astype(np.float32),
        arr['y'].astype(np.float32),
        arr['z'].astype(np.float32),
    ], axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)]


class ObstacleMapNode(Node):

    def __init__(self) -> None:
        super().__init__('obstacle_map_node')

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter('x_min', 0.0)
        self.declare_parameter('x_max', 8.18)
        self.declare_parameter('y_min', 0.0)
        self.declare_parameter('y_max', 6.69)
        self.declare_parameter('resolution', 0.1)       # m
        self.declare_parameter('z_min', 0.1)            # m — ignore ground
        self.declare_parameter('z_max', 1.8)            # m — ignore ceiling
        self.declare_parameter('obstacle_radius', 0.15) # m — proximity threshold
        self.declare_parameter('robot_radius', 0.5)    # m — 로봇 근접 포인트 제외 반경
        self.declare_parameter('lidar_offset_x', 0.0)  # m — LiDAR mount offset (robot center)
        self.declare_parameter('lidar_offset_y', 0.0)  # m — G1 head 정중앙 → (0, 0)
        self.declare_parameter('mirror_y', False)       # LiDAR 상하 반전 장착 시 Y축 반전
        self.declare_parameter('frame_id', 'map')

        x_min               = self.get_parameter('x_min').value
        x_max               = self.get_parameter('x_max').value
        y_min               = self.get_parameter('y_min').value
        y_max               = self.get_parameter('y_max').value
        self._res           = self.get_parameter('resolution').value
        self._z_min         = self.get_parameter('z_min').value
        self._z_max         = self.get_parameter('z_max').value
        self.get_parameter('obstacle_radius')          # yaml 호환 유지
        self._robot_r       = self.get_parameter('robot_radius').value
        self._lidar_dx      = self.get_parameter('lidar_offset_x').value
        self._lidar_dy      = self.get_parameter('lidar_offset_y').value
        self._mirror_y      = self.get_parameter('mirror_y').value
        self._frame_id      = self.get_parameter('frame_id').value

        # ── Robot pose (location_node에서 수신, 스레드 안전) ──────
        self._pose_lock = threading.Lock()
        self._robot_x: float | None = None
        self._robot_y: float | None = None
        self._robot_yaw: float | None = None

        # ── Grid dimensions ───────────────────────────────────────
        self._nx = int(np.ceil((x_max - x_min) / self._res))
        self._ny = int(np.ceil((y_max - y_min) / self._res))

        # ── OccupancyGrid 메타 ────────────────────────────────────
        self._map_meta = MapMetaData()
        self._map_meta.resolution        = self._res
        self._map_meta.width             = self._nx
        self._map_meta.height            = self._ny
        self._map_meta.origin.position.x = float(x_min)
        self._map_meta.origin.position.y = float(y_min)
        self._map_meta.origin.orientation.w = 1.0

        # ── Pub / Sub ─────────────────────────────────────────────
        self._pub = self.create_publisher(
            OccupancyGrid, '/onboard/sensors/lidar/occupancy', _BE_QOS)
        self._pub_filtered = self.create_publisher(
            PointCloud2, '/onboard/sensors/lidar/filtered_points', _BE_QOS)

        self.create_subscription(
            PoseStamped, '/onboard/sensors/location',
            self._on_location, _BE_QOS)

        self.create_subscription(
            PointCloud2, '/onboard/sensors/lidar/points',
            self._on_cloud, _BE_QOS)

        self.get_logger().info(
            f'obstacle_map_node ready — '
            f'grid {self._nx}×{self._ny} ({self._nx * self._ny} cells), '
            f'res={self._res}m, z=[{self._z_min},{self._z_max}]m, '
            f'lidar_offset=({self._lidar_dx},{self._lidar_dy})m'
        )

    # ── Location callback ─────────────────────────────────────────

    def _on_location(self, msg: PoseStamped) -> None:
        yaw = _quat_to_yaw(msg.pose.orientation)
        with self._pose_lock:
            self._robot_x   = msg.pose.position.x
            self._robot_y   = msg.pose.position.y
            self._robot_yaw = yaw

    # ── LiDAR callback ───────────────────────────────────────────

    def _on_cloud(self, msg: PointCloud2) -> None:
        with self._pose_lock:
            rx, ry, ryaw = self._robot_x, self._robot_y, self._robot_yaw

        if rx is None:
            self.get_logger().warn('obstacle_map_node: waiting for robot pose...', once=True)
            self._publish_empty(msg.header)
            return

        xyz = _parse_xyz(msg)
        if xyz is None or len(xyz) == 0:
            self._publish_empty(msg.header)
            return

        # Z 범위 필터 — 바닥·천장 제거
        mask = (xyz[:, 2] >= self._z_min) & (xyz[:, 2] <= self._z_max)
        pts = xyz[mask, :2]   # (M, 2) — LiDAR 로컬 XY

        # 로봇 근접 포인트 제거 — LiDAR 로컬 원점 기준 robot_radius 이내 제외
        if self._robot_r > 0:
            dist2 = pts[:, 0] ** 2 + pts[:, 1] ** 2
            pts = pts[dist2 >= self._robot_r ** 2]

        if len(pts) == 0:
            self._publish_empty(msg.header)
            return

        # LiDAR 상하 반전 장착 보정
        if self._mirror_y:
            pts[:, 1] = -pts[:, 1]

        # LiDAR 로컬 → map 프레임 변환
        # p_map = R(yaw) * p_lidar + [rx + dx, ry + dy]
        cos_y, sin_y = math.cos(ryaw), math.sin(ryaw)
        pts_map = np.empty_like(pts)
        pts_map[:, 0] = cos_y * pts[:, 0] - sin_y * pts[:, 1] + rx + self._lidar_dx
        pts_map[:, 1] = sin_y * pts[:, 0] + cos_y * pts[:, 1] + ry + self._lidar_dy

        # map 범위 밖 점 제거
        meta = self._map_meta
        in_bounds = (
            (pts_map[:, 0] >= meta.origin.position.x) &
            (pts_map[:, 0] <  meta.origin.position.x + meta.width  * self._res) &
            (pts_map[:, 1] >= meta.origin.position.y) &
            (pts_map[:, 1] <  meta.origin.position.y + meta.height * self._res)
        )
        pts_map = pts_map[in_bounds]

        if len(pts_map) == 0:
            self._publish_empty(msg.header)
            return

        # LiDAR 점 → 격자 인덱스 변환
        x_origin = self._map_meta.origin.position.x
        y_origin = self._map_meta.origin.position.y
        cols = ((pts_map[:, 0] - x_origin) / self._res).astype(np.int32)
        rows = ((pts_map[:, 1] - y_origin) / self._res).astype(np.int32)

        valid = (cols >= 0) & (cols < self._nx) & (rows >= 0) & (rows < self._ny)
        cols = cols[valid]
        rows = rows[valid]

        grid = np.zeros((self._ny, self._nx), dtype=np.int8)
        if len(cols) > 0:
            grid[rows, cols] = 100
        self._publish(msg.header, grid.ravel())
        self._publish_filtered(msg.header, pts_map)

    # ── Publish helpers ───────────────────────────────────────────

    def _publish(self, src_header, cells: np.ndarray) -> None:
        grid = OccupancyGrid()
        grid.header.stamp    = src_header.stamp
        grid.header.frame_id = self._frame_id
        grid.info            = self._map_meta
        grid.data            = cells.tolist()
        self._pub.publish(grid)

    def _publish_empty(self, src_header) -> None:
        self._publish(src_header, np.zeros(self._nx * self._ny, dtype=np.int8))

    def _publish_filtered(self, src_header, pts_xy: np.ndarray) -> None:
        n = len(pts_xy)
        xyz = np.zeros((n, 3), dtype=np.float32)
        xyz[:, :2] = pts_xy
        msg = PointCloud2()
        msg.header.stamp    = src_header.stamp
        msg.header.frame_id = self._frame_id
        msg.height     = 1
        msg.width      = n
        msg.fields     = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step   = 12
        msg.row_step     = 12 * n
        msg.data         = xyz.tobytes()
        msg.is_dense     = True
        self._pub_filtered.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
