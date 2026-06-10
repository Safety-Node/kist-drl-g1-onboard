"""
Obstacle map node — LiDAR point cloud → 2D OccupancyGrid (TASK-53).

고정 직사각형 공간(x_min~x_max, y_min~y_max)을 균일 격자로 분할하고,
매 LiDAR 프레임마다 각 격자점 근방에 포인트가 있으면 OBSTACLE(100),
없으면 FREE(0)로 실시간 발행한다.

Subscriptions:
    /onboard/sensors/lidar/points  sensor_msgs/PointCloud2  BestEffort

Publications:
    /onboard/sensors/lidar/occupancy  nav_msgs/OccupancyGrid  BestEffort
"""
from __future__ import annotations

import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from scipy.spatial import cKDTree

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
        self.declare_parameter('frame_id', 'map')

        x_min           = self.get_parameter('x_min').value
        x_max           = self.get_parameter('x_max').value
        y_min           = self.get_parameter('y_min').value
        y_max           = self.get_parameter('y_max').value
        self._res       = self.get_parameter('resolution').value
        self._z_min     = self.get_parameter('z_min').value
        self._z_max     = self.get_parameter('z_max').value
        self._obs_r     = self.get_parameter('obstacle_radius').value
        self._frame_id  = self.get_parameter('frame_id').value

        # ── Static grid (x-major, row = y, col = x) ──────────────
        xs = np.arange(x_min, x_max, self._res, dtype=np.float32)
        ys = np.arange(y_min, y_max, self._res, dtype=np.float32)
        gx, gy = np.meshgrid(xs, ys)          # shape: (ny, nx)
        self._nx = len(xs)
        self._ny = len(ys)
        self._grid_xy = np.stack(
            [gx.ravel(), gy.ravel()], axis=1)  # (ny*nx, 2)

        # ── OccupancyGrid template ────────────────────────────────
        self._map_meta = MapMetaData()
        self._map_meta.resolution       = self._res
        self._map_meta.width            = self._nx
        self._map_meta.height           = self._ny
        self._map_meta.origin.position.x = float(x_min)
        self._map_meta.origin.position.y = float(y_min)
        self._map_meta.origin.orientation.w = 1.0

        # ── Pub / Sub ─────────────────────────────────────────────
        self._pub = self.create_publisher(
            OccupancyGrid, '/onboard/sensors/lidar/occupancy', _BE_QOS)

        self.create_subscription(
            PointCloud2, '/onboard/sensors/lidar/points',
            self._on_cloud, _BE_QOS)

        self.get_logger().info(
            f'obstacle_map_node ready — '
            f'grid {self._nx}×{self._ny} ({len(self._grid_xy)} cells), '
            f'res={self._res}m, z=[{self._z_min},{self._z_max}]m, '
            f'r={self._obs_r}m'
        )

    # ── Callback ──────────────────────────────────────────────────

    def _on_cloud(self, msg: PointCloud2) -> None:
        xyz = _parse_xyz(msg)

        if xyz is None or len(xyz) == 0:
            self._publish_empty(msg.header)
            return

        # Z 범위 필터 — 바닥·천장 제거
        mask = (xyz[:, 2] >= self._z_min) & (xyz[:, 2] <= self._z_max)
        pts2d = xyz[mask, :2]   # (M, 2) — XY only

        if len(pts2d) == 0:
            self._publish_empty(msg.header)
            return

        # KD-tree: 각 격자점의 최근접 LiDAR 점까지 거리 계산
        tree = cKDTree(pts2d)
        distances, _ = tree.query(
            self._grid_xy, k=1,
            distance_upper_bound=self._obs_r,
            workers=-1,
        )

        # distance < obs_r → OBSTACLE(100), else FREE(0)
        cells = np.where(distances < self._obs_r, 100, 0).astype(np.int8)

        self._publish(msg.header, cells)

    # ── Publish helpers ───────────────────────────────────────────

    def _publish(self, src_header, cells: np.ndarray) -> None:
        grid = OccupancyGrid()
        grid.header.stamp    = src_header.stamp
        grid.header.frame_id = self._frame_id
        grid.info            = self._map_meta
        grid.data            = cells.tolist()
        self._pub.publish(grid)

    def _publish_empty(self, src_header) -> None:
        cells = np.zeros(len(self._grid_xy), dtype=np.int8)
        self._publish(src_header, cells)


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
