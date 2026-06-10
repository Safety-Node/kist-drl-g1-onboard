"""
RealSense D435i camera node — pyrealsense2 SDK 직접 사용.

realsense2_camera C++ 드라이버를 대체. 프레임 캡처는 background thread에서
독립적으로 실행하므로, outbound_relay의 DDS 콜백이 느려도 capture pipeline이
막히지 않는다.

Publications (BEST_EFFORT, depth=1):
  /onboard/sensors/camera/color/image_raw/compressed   CompressedImage (JPEG)
  /onboard/sensors/camera/aligned_depth_to_color/image_raw  Image (16UC1)
"""
import threading
from typing import Optional

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header

_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_WARMUP_FRAMES = 10


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__('camera_node')

        self.declare_parameter('color_width', 1280)
        self.declare_parameter('color_height', 720)
        self.declare_parameter('color_fps', 30)
        self.declare_parameter('depth_width', 1280)
        self.declare_parameter('depth_height', 720)
        self.declare_parameter('depth_fps', 30)
        self.declare_parameter('align_depth', True)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('serial_no', '')

        self._color_w   = int(self.get_parameter('color_width').value)
        self._color_h   = int(self.get_parameter('color_height').value)
        self._color_fps = int(self.get_parameter('color_fps').value)
        self._depth_w   = int(self.get_parameter('depth_width').value)
        self._depth_h   = int(self.get_parameter('depth_height').value)
        self._depth_fps = int(self.get_parameter('depth_fps').value)
        self._do_align  = bool(self.get_parameter('align_depth').value)
        self._jpeg_q    = int(self.get_parameter('jpeg_quality').value)
        self._serial    = str(self.get_parameter('serial_no').value)

        self._pub_color = self.create_publisher(
            CompressedImage,
            '/onboard/sensors/camera/color/image_raw/compressed',
            _QOS,
        )
        self._pub_depth = self.create_publisher(
            Image,
            '/onboard/sensors/camera/aligned_depth_to_color/image_raw',
            _QOS,
        )

        # 최신 프레임을 background thread가 쓰고, 타이머가 읽는다
        self._latest_bgr:   Optional[np.ndarray] = None  # (H, W, 3) uint8
        self._latest_depth: Optional[np.ndarray] = None  # (H, W) uint16 raw
        self._frame_lock = threading.Lock()

        self._running = False
        self._capture_thread: Optional[threading.Thread] = None
        self._pipeline: Optional[rs.pipeline] = None
        self._aligner:  Optional[rs.align]    = None

        self._start_pipeline()

        self._timer = self.create_timer(1.0 / self._color_fps, self._publish)
        self.get_logger().info(
            f'camera_node ready — '
            f'{self._color_w}×{self._color_h} @ {self._color_fps} Hz '
            f'(align_depth={self._do_align})')

    # ------------------------------------------------------------------ #

    def _start_pipeline(self) -> None:
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) == 0:
            raise RuntimeError('camera_node: RealSense device not found')

        cfg = rs.config()
        if self._serial:
            cfg.enable_device(self._serial)
        cfg.enable_stream(
            rs.stream.color, self._color_w, self._color_h,
            rs.format.bgr8, self._color_fps)
        cfg.enable_stream(
            rs.stream.depth, self._depth_w, self._depth_h,
            rs.format.z16, self._depth_fps)

        self._pipeline = rs.pipeline()
        self._pipeline.start(cfg)

        if self._do_align:
            self._aligner = rs.align(rs.stream.color)

        # auto-exposure warm-up
        for _ in range(_WARMUP_FRAMES):
            self._pipeline.wait_for_frames()

        self._running = True
        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True, name='cam_capture')
        self._capture_thread.start()

    def _capture_loop(self) -> None:
        while self._running:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=150)
                if self._aligner:
                    frames = self._aligner.process(frames)
                color_f = frames.get_color_frame()
                depth_f = frames.get_depth_frame()
                if color_f and depth_f:
                    bgr   = np.asanyarray(color_f.get_data()).copy()
                    depth = np.asanyarray(depth_f.get_data()).copy()  # uint16
                    with self._frame_lock:
                        self._latest_bgr   = bgr
                        self._latest_depth = depth
            except Exception as e:
                if self._running:
                    self.get_logger().warn(f'camera capture: {e}')

    def _publish(self) -> None:
        with self._frame_lock:
            bgr   = self._latest_bgr
            depth = self._latest_depth
        if bgr is None or depth is None:
            return

        stamp = self.get_clock().now().to_msg()

        # Color → JPEG CompressedImage
        ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_q])
        if ok:
            msg = CompressedImage()
            msg.header = Header(stamp=stamp, frame_id='camera_color_optical_frame')
            msg.format = 'jpeg'
            msg.data   = buf.tobytes()
            self._pub_color.publish(msg)

        # Depth → 16UC1 Image (raw z16 값, mm 단위)
        msg = Image()
        msg.header       = Header(stamp=stamp, frame_id='camera_depth_optical_frame')
        msg.height       = depth.shape[0]
        msg.width        = depth.shape[1]
        msg.encoding     = '16UC1'
        msg.is_bigendian = False
        msg.step         = depth.shape[1] * 2
        msg.data         = depth.tobytes()
        self._pub_depth.publish(msg)

    # ------------------------------------------------------------------ #

    def destroy_node(self) -> None:
        self._running = False
        if self._capture_thread:
            self._capture_thread.join(timeout=2.0)
        if self._pipeline:
            try:
                self._pipeline.stop()
            except Exception:
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
