"""
RealSense D435i camera node — pyrealsense2 SDK 직접 사용.

realsense2_camera C++ 드라이버를 대체. 프레임 캡처/인코딩/발행을 모두
background thread에서 처리하므로 ROS2 타이머 스케줄링 지연 없이 하드웨어
프레임 레이트 그대로 발행한다.

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

        self._running = False
        self._capture_thread: Optional[threading.Thread] = None
        self._pipeline: Optional[rs.pipeline] = None
        self._aligner:  Optional[rs.align]    = None

        self._start_pipeline()

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

        for _ in range(_WARMUP_FRAMES):
            self._pipeline.wait_for_frames()

        self._running = True
        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True, name='cam_capture')
        self._capture_thread.start()

    def _capture_loop(self) -> None:
        while self._running:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
                if self._aligner:
                    frames = self._aligner.process(frames)
                color_f = frames.get_color_frame()
                depth_f = frames.get_depth_frame()
                if not color_f or not depth_f:
                    continue

                bgr   = np.asanyarray(color_f.get_data()).copy()
                depth = np.asanyarray(depth_f.get_data()).copy()
                stamp = self.get_clock().now().to_msg()

                ok, buf = cv2.imencode(
                    '.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_q])
                if ok:
                    cmsg = CompressedImage()
                    cmsg.header = Header(
                        stamp=stamp, frame_id='camera_color_optical_frame')
                    cmsg.format = 'jpeg'
                    cmsg.data   = buf.tobytes()
                    self._pub_color.publish(cmsg)

                dmsg = Image()
                dmsg.header       = Header(
                    stamp=stamp, frame_id='camera_depth_optical_frame')
                dmsg.height       = depth.shape[0]
                dmsg.width        = depth.shape[1]
                dmsg.encoding     = '16UC1'
                dmsg.is_bigendian = False
                dmsg.step         = depth.shape[1] * 2
                dmsg.data         = depth.tobytes()
                self._pub_depth.publish(dmsg)

            except Exception as e:
                if self._running:
                    self.get_logger().warn(f'camera capture: {e}')

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
