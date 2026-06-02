"""
UWB beacon receiver → /onboard/sensors/uwb/pose (geometry_msgs/PoseStamped).

Replaces SLAM-based localisation (REQ-37).

Hardware
--------
Decawave DWM series (DWM1001 / DWM3001CDK etc.) connected via USB-serial.
Device node is /dev/uwb — set up the udev symlink on the target machine:
  SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="uwb"
  (DWM1001: VID=0483, PID=5740; verify with ``lsusb`` if using a different module)

Protocol (DWM UART shell)
-------------------------
1. Send ``\\r\\r`` → wait for ``dwm>`` prompt.
2. Send ``lec\\r`` → location engine continuous mode.
3. Parse lines: ``POS,<x>,<y>,<z>,<quality>\\r\\n``
   (DWM does trilateration internally; anchor table in sensors_params.yaml
    is informational / for site-survey records only.)

Publish policy
--------------
- Publishes **only** when a valid fix is available (quality > 0).
- No placeholder publish — fake zero-pose can cause false task-success
  detection downstream.
Topics
------
  /onboard/sensors/uwb/pose  geometry_msgs/PoseStamped  BestEffort  ~20 Hz
"""

from __future__ import annotations

import abc
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header


# ---------------------------------------------------------------------------
# Transport layer
# ---------------------------------------------------------------------------

@dataclass
class UwbSample:
    """Single UWB position fix."""
    x_m: float
    y_m: float
    z_m: float
    quality: int        # 0-100; DWM quality factor
    received_at: float  # time.monotonic()
    is_new: bool        # False after the sample has been published once


class UwbTransport(abc.ABC):
    """Strategy interface for UWB data sources."""

    @abc.abstractmethod
    def start(self) -> None: ...

    @abc.abstractmethod
    def stop(self) -> None: ...

    @abc.abstractmethod
    def latest_sample(self) -> Optional[UwbSample]: ...


class StubTransport(UwbTransport):
    """Returns no fix (simulates device with no anchors in range)."""

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def latest_sample(self) -> Optional[UwbSample]:
        return None


class SerialTransport(UwbTransport):
    """DWM UART shell — lec streaming, POS line parser.

    Runs a background reader thread that reconnects on serial errors
    with exponential backoff (max 16 s).
    """

    _RECONNECT_BASE_S = 1.0
    _RECONNECT_MAX_S = 16.0

    def __init__(self, port: str, baud: int) -> None:
        self._port = port
        self._baud = baud
        self._lock = threading.Lock()
        self._sample: Optional[UwbSample] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._reader_loop, daemon=True, name="UwbSerial"
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

    def latest_sample(self) -> Optional[UwbSample]:
        with self._lock:
            return self._sample

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _reader_loop(self) -> None:
        """Outer reconnect loop — retries with backoff on any error."""
        import serial  # import here so stub/udp paths don't require pyserial

        backoff = self._RECONNECT_BASE_S
        while self._running:
            ser = None
            try:
                ser = serial.Serial(self._port, self._baud, timeout=0.2)
                self._init_streaming(ser)
                backoff = self._RECONNECT_BASE_S  # reset on success
                self._read_loop(ser)
            except Exception as exc:
                if self._running:
                    import logging
                    logging.warning(
                        "UwbSerial: error (%s), reconnecting in %.1fs", exc, backoff
                    )
                    time.sleep(backoff)
                    backoff = min(backoff * 2, self._RECONNECT_MAX_S)
            finally:
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass

    def _read_loop(self, ser) -> None:
        """Inner read loop — exits on serial error or stop()."""
        buf = bytearray()
        while self._running:
            n = int(getattr(ser, "in_waiting", 0) or 0)
            if n > 0:
                buf.extend(ser.read(n))
                for line in self._extract_lines(buf):
                    sample = self._parse_pos(line)
                    if sample is not None:
                        with self._lock:
                            self._sample = sample
            else:
                time.sleep(0.005)

    def _init_streaming(self, ser) -> None:
        """Enter DWM shell and start lec mode."""
        self._enter_shell(ser)
        self._start_lec(ser)

    def _enter_shell(self, ser, timeout: float = 4.0) -> None:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        # Two CR presses to wake shell
        ser.write(b"\r\r")
        ser.flush()

        buf = bytearray()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            n = int(getattr(ser, "in_waiting", 0) or 0)
            if n > 0:
                buf.extend(ser.read(n))
                if b"dwm>" in buf:
                    return
                # Already in lec streaming — toggle off first
                if b"DIST" in buf or b"POS" in buf:
                    buf.clear()
                    ser.write(b"lec\r")
                    ser.flush()
                    time.sleep(0.15)
                    ser.reset_input_buffer()
                    buf.clear()
                    # Now wait for dwm> again
                    deadline = time.monotonic() + timeout
            else:
                time.sleep(0.01)
        raise RuntimeError(f"UwbSerial: failed to enter DWM shell on {self._port}")

    def _start_lec(self, ser, timeout: float = 2.0) -> None:
        ser.reset_input_buffer()
        ser.write(b"lec\r")
        ser.flush()

        buf = bytearray()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            n = int(getattr(ser, "in_waiting", 0) or 0)
            if n > 0:
                buf.extend(ser.read(n))
                if b"DIST" in buf or b"POS" in buf or b"dwm>" in buf:
                    return  # accepted (no anchors = dwm> is ok)
            else:
                time.sleep(0.01)
        raise RuntimeError("UwbSerial: lec command not acknowledged")

    @staticmethod
    def _extract_lines(buf: bytearray) -> list[bytes]:
        out: list[bytes] = []
        start = 0
        while True:
            idx = buf.find(b"\r\n", start)
            if idx < 0:
                break
            line = bytes(buf[start:idx])
            if line:
                out.append(line)
            start = idx + 2
        if start:
            del buf[:start]
        return out

    @staticmethod
    def _parse_pos(line: bytes) -> Optional[UwbSample]:
        idx = line.find(b"POS,")
        if idx < 0:
            return None
        parts = line[idx:].split(b",")
        if len(parts) < 5:
            return None
        try:
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
            q = int(float(parts[4]))
        except (ValueError, IndexError):
            return None
        if q <= 0:
            return None  # quality=0 means no fix
        return UwbSample(x_m=x, y_m=y, z_m=z, quality=q,
                         received_at=time.monotonic(), is_new=True)


class UdpTransport(UwbTransport):
    """UDP receiver — placeholder (not yet implemented)."""

    def start(self) -> None:
        raise NotImplementedError("UdpTransport: not implemented [TASK-30]")

    def stop(self) -> None:
        pass

    def latest_sample(self) -> Optional[UwbSample]:
        return None


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

_BE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class UwbNode(Node):
    """UWB pose publisher — /onboard/sensors/uwb/pose @ publish_rate_hz."""

    def __init__(self) -> None:
        super().__init__('uwb_node')

        # ---- Parameters ----
        self.declare_parameter('transport',        'stub')
        self.declare_parameter('serial_port',      '/dev/uwb')
        self.declare_parameter('serial_baud',      115200)
        self.declare_parameter('udp_listen_port',  50000)
        self.declare_parameter('publish_rate_hz',  20.0)
        self.declare_parameter('frame_id',         'map')

        transport_name  = self.get_parameter('transport').value
        serial_port     = self.get_parameter('serial_port').value
        serial_baud     = self.get_parameter('serial_baud').value
        rate_hz         = float(self.get_parameter('publish_rate_hz').value)
        self._frame_id  = self.get_parameter('frame_id').value

        # Anchor table — informational (DWM does trilateration internally)
        try:
            anchors = self.get_parameters_by_prefix('anchors')
            if anchors:
                self.get_logger().info(
                    'uwb_node: %d anchor(s) in config (informational)', len(anchors)
                )
        except Exception:
            pass

        # ---- Transport ----
        if transport_name == 'serial':
            self._transport: UwbTransport = SerialTransport(serial_port, serial_baud)
        elif transport_name == 'udp':
            self._transport = UdpTransport()
        else:
            if transport_name != 'stub':
                self.get_logger().warn(
                    "uwb_node: unknown transport '%s', falling back to stub",
                    transport_name
                )
            self._transport = StubTransport()

        self._transport.start()

        # ---- Publisher ----
        self._pub = self.create_publisher(PoseStamped, '/onboard/sensors/uwb/pose', _BE_QOS)

        # ---- Timer ----
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(
            'uwb_node started (transport=%s, port=%s, rate=%.1fHz)',
            transport_name, serial_port, rate_hz
        )

    def _on_timer(self) -> None:
        sample = self._transport.latest_sample()
        if sample is None:
            return  # no fix — publish nothing

        # Dedup — skip if this sample was already published
        if not sample.is_new:
            return

        # Mark consumed
        sample.is_new = False  # type: ignore[misc]

        # Build and publish PoseStamped
        now = self.get_clock().now().to_msg()
        msg = PoseStamped()
        msg.header = Header(stamp=now, frame_id=self._frame_id)
        msg.pose.position = Point(x=sample.x_m, y=sample.y_m, z=sample.z_m)
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # no yaw source
        self._pub.publish(msg)

    def destroy_node(self) -> None:
        self._transport.stop()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UwbNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
