"""
UWB + odometry EKF 융합 — /onboard/sensors/location 발행.

/onboard/sensors/odom (nav_msgs/Odometry, ~50 Hz) 를 predict 입력으로,
/onboard/sensors/uwb/pose (geometry_msgs/PoseStamped, ~10 Hz) 를 update 입력으로
사용해 UwbOdomAEKF 로 융합한 위치를 /onboard/sensors/location 으로 발행한다.

발행 조건
---------
- EKF initialized (첫 UWB 수신 이후)
- yaw_calibrated (b_θ 불확실성 < 5°) — heading 신뢰 전에는 발행 안 함

Publications
------------
  /onboard/sensors/location  geometry_msgs/PoseStamped  BestEffort  ~20 Hz
    - position.x/y  : EKF 추정 위치 (m, UWB 로컬 프레임)
    - position.z    : 0.0
    - orientation   : global_yaw_rad → quaternion (z 축 회전)
    - frame_id      : 파라미터 frame_id (기본 "map")

Related: REQ-42 (센서 수집/전달), REQ-45 (자율 보행 측위 입력), TASK-51
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from sensors.utils.uwb_odom_aekf import UwbOdomAEKF


_BE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

_SUB_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10,
)


def _quat_to_yaw(q) -> float:
    """orientation quaternion → yaw (rad, CCW positive)."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    """yaw (rad) → quaternion (x, y, z, w) — 순수 z 축 회전."""
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class LocationNode(Node):
    """UWB + odom EKF 융합 위치 발행 노드."""

    def __init__(self) -> None:
        super().__init__('location_node')

        self.declare_parameter('odom_topic', '/onboard/sensors/odom')
        self.declare_parameter('uwb_topic', '/onboard/sensors/uwb/pose')
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('frame_id', 'map')

        odom_topic = self.get_parameter('odom_topic').value
        uwb_topic = self.get_parameter('uwb_topic').value
        publish_hz = float(self.get_parameter('publish_hz').value)
        self._frame_id: str = self.get_parameter('frame_id').value

        self._ekf = UwbOdomAEKF()
        self._latest_odom_yaw: float = 0.0
        self._uwb_samples: int = 0

        self._pub = self.create_publisher(PoseStamped, '/onboard/sensors/location', _BE_QOS)
        self._pub_diag = self.create_publisher(
            DiagnosticStatus, '/onboard/sensors/location/diagnostics', _BE_QOS
        )

        self._sub_odom = self.create_subscription(Odometry, odom_topic, self._on_odom, _SUB_QOS)
        self._sub_uwb = self.create_subscription(PoseStamped, uwb_topic, self._on_uwb, _SUB_QOS)

        self._timer = self.create_timer(1.0 / publish_hz, self._on_timer)

        self.get_logger().info(
            f'location_node ready — {odom_topic} + {uwb_topic} '
            f'→ /onboard/sensors/location @ {publish_hz:.0f} Hz'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = _quat_to_yaw(msg.pose.pose.orientation)
        self._latest_odom_yaw = yaw
        self._ekf.predict(x, y, yaw)

    def _on_uwb(self, msg: PoseStamped) -> None:
        uwb_x = msg.pose.position.x
        uwb_y = msg.pose.position.y

        if not self._ekf.initialized:
            self._ekf.initialize(uwb_x, uwb_y, self._latest_odom_yaw)
            self.get_logger().info(
                f'EKF initialized at UWB ({uwb_x:.2f}, {uwb_y:.2f})'
            )
            return

        self._ekf.update(uwb_x, uwb_y)
        self._uwb_samples += 1

    def _on_timer(self) -> None:
        if not self._ekf.initialized:
            return

        now = self.get_clock().now().to_msg()
        calibrated = self._ekf.yaw_calibrated
        yaw_deg = math.degrees(self._ekf.global_yaw_rad)

        # diagnostics — always published after EKF init
        diag = DiagnosticStatus()
        diag.level = DiagnosticStatus.OK if calibrated else DiagnosticStatus.WARN
        diag.name = 'location_node'
        diag.message = 'yaw calibrated' if calibrated else 'calibrating — move robot'
        diag.values = [
            KeyValue(key='std_bias_deg',   value=f'{self._ekf.std_bias_deg:.3f}'),
            KeyValue(key='yaw_calibrated', value=str(calibrated)),
            KeyValue(key='ekf_x',          value=f'{self._ekf.x_m:.4f}'),
            KeyValue(key='ekf_y',          value=f'{self._ekf.y_m:.4f}'),
            KeyValue(key='ekf_yaw_deg',    value=f'{yaw_deg:.2f}'),
            KeyValue(key='uwb_samples',    value=str(self._uwb_samples)),
        ]
        self._pub_diag.publish(diag)

        # location — only when calibrated
        if not calibrated:
            return

        qx, qy, qz, qw = _yaw_to_quat(self._ekf.global_yaw_rad)

        msg = PoseStamped()
        msg.header.stamp = now
        msg.header.frame_id = self._frame_id
        msg.pose.position.x = self._ekf.x_m
        msg.pose.position.y = self._ekf.y_m
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
