"""
G1 mic (UDP multicast) capture → /onboard/sensors/audio/pcm
(g1_onboard_msgs/AudioPCM, 16k/16-bit/mono).

The G1 4-mic array is owned by the robot's audio service (PC1), which *broadcasts*
the mixed-down stream over UDP multicast — NOT a local ALSA device. So this node
joins the multicast group and re-publishes each chunk as AudioPCM (see TASK-31 /
g1_data_transport_차이: mic = UDP side-channel, speaker = SDK PlayStream RPC).

Trap: channels=1 mono is intentional (G1 4-mic mix-down — see TASK-31).
Trap: needs 'Wake-up Conversation Mode' ON in the Unitree app or the robot sends
      NO packets — recv just times out. Timeout is logged, NOT fatal (keep spinning).
      During timeouts silence frames are published to maintain the 50 Hz stream.
Trap: iface_ip is the *onboard NIC IP* on the 192.168.123.x net (multicast join),
      NOT a NIC name. (speaker_node uses a NIC *name* for the SDK — asymmetry.)
Trap: publishes /onboard/sensors/audio/pcm (NOT /onboard/audio/... — mic/speaker
      namespace asymmetry; speaker subscribes /onboard/audio/playback — see TASK-31).
"""
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

from g1_onboard_msgs.msg import AudioPCM


# Pipeline LOCK — single source of truth is AudioPCM.msg's constants
# (AudioPCM.SAMPLE_RATE / CHANNELS / BIT_DEPTH). speaker_node / STT Provider
# read the same constants, so no literals are re-declared here.
BYTES_PER_SAMPLE = AudioPCM.BIT_DEPTH // 8  # 2 (int16 LE)

# Live media → BEST_EFFORT (freshness wins). Matches comm_bridge's best_effort
# relay of /onboard/sensors/audio/pcm and the lossy UDP-multicast source — a
# reliable queue would just back up stale audio ahead of the STT Provider.
# Depth isn't part of QoS compatibility, so it may differ from the relay's.
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class MicNode(Node):
    def __init__(self) -> None:
        super().__init__('mic_node')

        # --- Parameters --------------------------------------------------
        # Multicast source (robot audio service / PC1).
        self.declare_parameter('group_ip', '239.168.123.161')
        self.declare_parameter('port', 5555)
        # Onboard NIC IP on the 192.168.123.x net (multicast join interface). TBD HW.
        self.declare_parameter('iface_ip', '192.168.123.99')
        self.declare_parameter('sample_rate', AudioPCM.SAMPLE_RATE)   # LOCKED
        self.declare_parameter('channels', AudioPCM.CHANNELS)         # LOCKED (mono)
        self.declare_parameter('chunk_ms', 20)               # re-packetise to fixed chunks
        self.declare_parameter('recv_buf_bytes', 65536)

        self._group_ip: str = self.get_parameter('group_ip').value
        self._port: int = int(self.get_parameter('port').value)
        self._iface_ip: str = self.get_parameter('iface_ip').value
        self._sample_rate: int = int(self.get_parameter('sample_rate').value)
        self._channels: int = int(self.get_parameter('channels').value)
        chunk_ms: int = int(self.get_parameter('chunk_ms').value)
        self._recv_buf: int = int(self.get_parameter('recv_buf_bytes').value)
        # Socket timeout = one chunk period so the silence-fill loop stays in sync.
        self._chunk_sec: float = chunk_ms / 1000.0

        if (self._sample_rate != AudioPCM.SAMPLE_RATE
                or self._channels != AudioPCM.CHANNELS):
            # Downstream (speaker_node, STT) assumes the locked format.
            raise ValueError(
                f"Pipeline LOCKED to {AudioPCM.SAMPLE_RATE}Hz/{AudioPCM.CHANNELS}ch; "
                f"got {self._sample_rate}/{self._channels}")

        # Re-packetise the (arbitrary-sized) UDP payloads into deterministic
        # chunk_ms frames so downstream sees uniform AudioPCM messages.
        self._chunk_bytes = (
            self._sample_rate * self._channels * BYTES_PER_SAMPLE * chunk_ms // 1000)

        # --- ROS I/O -----------------------------------------------------
        self._pub = self.create_publisher(
            AudioPCM, '/onboard/sensors/audio/pcm', SENSOR_QOS)

        # --- Socket (multicast join) ------------------------------------
        self._sock = self._open_multicast_socket()

        # --- Capture thread ---------------------------------------------
        self._running = True
        self._warned_timeout = False  # log the "no packets" hint only once
        self._thread = threading.Thread(
            target=self._capture_loop, name='mic_capture', daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"mic_node capturing multicast {self._group_ip}:{self._port} "
            f"via iface {self._iface_ip} → /onboard/sensors/audio/pcm "
            f"({self._sample_rate}Hz/{self._channels}ch, {self._chunk_bytes}B chunks)")

    # -------------------------------------------------------------------
    # Socket setup
    # -------------------------------------------------------------------
    def _open_multicast_socket(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except OSError:
            pass  # not on all platforms

        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF,
                        socket.inet_aton(self._iface_ip))
        sock.bind(('', self._port))

        mreq = struct.pack('=4s4s', socket.inet_aton(self._group_ip),
                           socket.inet_aton(self._iface_ip))
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock.settimeout(self._chunk_sec)
        return sock

    # -------------------------------------------------------------------
    # Capture thread: recv → re-packetise → publish
    # -------------------------------------------------------------------
    def _capture_loop(self) -> None:
        silence = bytes(self._chunk_bytes)   # zero-filled frame for gap fill
        buf = bytearray()
        next_emit = time.monotonic() + self._chunk_sec

        while self._running:
            try:
                data, _ = self._sock.recvfrom(self._recv_buf)
            except socket.timeout:
                # PC1 paused — fill silence to keep the 50 Hz stream alive.
                # Use a while loop to catch up if OS delayed the timeout.
                now = time.monotonic()
                while now >= next_emit:
                    if not self._warned_timeout:
                        self.get_logger().warn(
                            "No mic packets — enable 'Wake-up Conversation Mode' in the "
                            "Unitree app and check iface_ip / multicast join. Filling silence...")
                        self._warned_timeout = True
                    self._publish(silence)
                    next_emit += self._chunk_sec
                continue
            except OSError:
                # socket closed during shutdown → exit cleanly
                break

            if not data:
                continue
            self._warned_timeout = False  # packets are flowing again

            buf.extend(data)
            # Emit as many whole chunk_ms frames as we have buffered.
            while len(buf) >= self._chunk_bytes:
                chunk = bytes(buf[:self._chunk_bytes])
                del buf[:self._chunk_bytes]
                self._publish(chunk)
            next_emit = time.monotonic() + self._chunk_sec

    def _publish(self, pcm: bytes) -> None:
        msg = AudioPCM()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mic'
        msg.sample_rate = self._sample_rate
        msg.channels = self._channels
        msg.bit_depth = AudioPCM.BIT_DEPTH
        msg.data = pcm  # int16 LE bytes — pipeline assumes even length (chunk_bytes is even)
        self._pub.publish(msg)

    # -------------------------------------------------------------------
    # Shutdown: stop thread → drop membership → close socket
    # -------------------------------------------------------------------
    def destroy_node(self) -> None:
        self.get_logger().info('mic_node shutting down; leaving multicast group...')
        self._running = False
        try:
            # Closing the socket unblocks a recvfrom() blocked in the thread.
            mreq = struct.pack('=4s4s', socket.inet_aton(self._group_ip),
                               socket.inet_aton(self._iface_ip))
            try:
                self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, mreq)
            except OSError:
                pass
            self._sock.close()
        finally:
            self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
