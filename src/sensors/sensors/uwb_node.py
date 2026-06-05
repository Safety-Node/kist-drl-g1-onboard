"""
UWB beacon receiver → /onboard/sensors/uwb/pose (geometry_msgs/PoseStamped).

Replaces SLAM-based localisation (REQ-37).

Hardware
--------
Decawave DWM series (DWM1001 / DWM3001CDK etc.) connected via USB-serial.
Device node is /dev/uwb — set up the udev symlink on the target machine:
  SUBSYSTEM=="tty", ATTRS{idVendor}=="1366", ATTRS{idProduct}=="0105", SYMLINK+="uwb"
  (DWM1001-DEV: VID=1366, PID=0105, SEGGER J-Link OB; verify with ``udevadm info /dev/ttyACM0 | grep ID_``)

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

    The port is opened **once** and kept open for the lifetime of the node.
    Closing and re-opening causes DTR to toggle, which resets the DWM MCU
    (J-Link OB behaviour). Only a true OSError (physical disconnect) triggers
    a full re-open after waiting for the device node to reappear.
    """

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
        """Outer loop — open port once, re-open only on physical disconnect."""
        import serial

        while self._running:
            ser = None
            try:
                ser = serial.Serial(
                    self._port, self._baud, timeout=0.2,
                    dsrdtr=False, rtscts=False,
                )
                print("UwbSerial: port opened, checking DWM state...", flush=True)
                self._init_streaming(ser)
                print("UwbSerial: lec streaming active", flush=True)
                self._read_loop(ser)
            except OSError as exc:
                # Physical disconnect or device not found — wait before retry
                print(f"UwbSerial: OSError ({exc}), retrying in 2s...", flush=True)
                time.sleep(2.0)
            except Exception as exc:
                print(f"UwbSerial: unexpected error ({exc}), retrying...", flush=True)
                time.sleep(1.0)
            finally:
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass

    def _read_loop(self, ser) -> None:
        """Inner read loop — exits on OSError (physical disconnect) or stop()."""
        buf = bytearray()
        while self._running:
            try:
                n = int(getattr(ser, "in_waiting", 0) or 0)
            except OSError:
                raise
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
        """Detect DWM state and skip already-done steps.

        Decision tree:
          DIST/POS visible → lec already running → go straight to read loop
          dwm> visible     → shell active, just send lec
          otherwise        → enter shell first, then send lec
        """
        time.sleep(0.3)  # let DWM output settle after port open
        buf = bytearray()
        n = int(getattr(ser, "in_waiting", 0) or 0)
        if n > 0:
            buf.extend(ser.read(n))

        if b"DIST," in buf or b"POS," in buf:
            print("UwbSerial: lec already streaming, skipping init", flush=True)
            return

        if b"dwm>" in buf:
            print("UwbSerial: shell active, starting lec", flush=True)
            self._start_lec(ser)
            return

        # Unknown/silent state — enter shell then start lec
        self._enter_shell(ser)
        self._start_lec(ser)

    def _enter_shell(self, ser) -> None:
        """Send ``\\r`` repeatedly until ``dwm>`` prompt appears."""
        buf = bytearray()
        last_nudge_t = time.monotonic() - 2.0  # nudge immediately on entry

        while self._running:
            n = int(getattr(ser, "in_waiting", 0) or 0)
            if n > 0:
                buf.extend(ser.read(n))
                if b"dwm>" in buf:
                    return
            now = time.monotonic()
            if now - last_nudge_t >= 2.0:
                ser.write(b"\r")
                ser.flush()
                last_nudge_t = now
            time.sleep(0.01)

    def _start_lec(self, ser, timeout: float = 3.0) -> None:
        """Send ``lec`` and wait for first data line or ``dwm>`` echo."""
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
                    return
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
                         received_at=time.monotonic())


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
        self.declare_parameter('publish_rate_hz',  10.0)
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
                    f'uwb_node: {len(anchors)} anchor(s) in config (informational)'
                )
        except Exception:
            pass

        # ---- Transport ----
        if transport_name == 'serial':
            self._transport: UwbTransport = SerialTransport(serial_port, serial_baud)
        else:
            if transport_name != 'stub':
                self.get_logger().warn(
                    f"uwb_node: unknown transport '{transport_name}', falling back to stub"
                )
            self._transport = StubTransport()

        self._transport.start()

        # ---- Publisher ----
        self._pub = self.create_publisher(PoseStamped, '/onboard/sensors/uwb/pose', _BE_QOS)

        # ---- Timer ----
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(
            f'uwb_node started (transport={transport_name}, port={serial_port}, rate={rate_hz:.1f}Hz)'
        )

    def _on_timer(self) -> None:
        sample = self._transport.latest_sample()
        if sample is None:
            return  # no fix — publish nothing

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
