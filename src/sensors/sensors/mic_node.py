"""
G1 mic (reSpeaker XVF3800 / ALSA) capture → /onboard/sensors/audio/pcm
(g1_onboard_msgs/AudioPCM, 16k/16-bit/mono).

Previously received audio via UDP multicast from PC1 (robot audio service).
PC1's vui_service has internal VAD that cuts the stream after speech is
detected, causing intermittent audio dropouts after every utterance. This
version bypasses PC1 entirely and captures directly from the reSpeaker
XVF3800 4-mic array via ALSA/sounddevice (NX ALSA card 2: Array).

Trap: device="Array" is the reSpeaker XVF3800 substring match (ALSA card 2).
      Run `arecord -l` on NX to confirm. Override with the `device` param.
Trap: channels=1 mono is intentional (LOCKED pipeline format — see TASK-31).
Trap: reSpeaker XVF3800 natively supports 16kHz so no resampling is expected.
      scipy resample_poly is used automatically if native rate differs.
Trap: publishes /onboard/sensors/audio/pcm (NOT /onboard/audio/... — asymmetry
      with speaker_node which subscribes /onboard/audio/playback — see TASK-31).
Trap: BEST_EFFORT depth=10 ("stream" QoS) — matches comm_bridge outbound_relay
      qos: "stream" subscription for /onboard/sensors/audio/pcm.
"""
import math

import numpy as np
import sounddevice as sd
from scipy.signal import resample_poly

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

from g1_onboard_msgs.msg import AudioPCM


# Pipeline LOCK — single source of truth is AudioPCM.msg's constants.
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def _resample_gcd(src_rate: int, dst_rate: int) -> tuple[int, int]:
    g = math.gcd(src_rate, dst_rate)
    return dst_rate // g, src_rate // g


class MicNode(Node):
    def __init__(self) -> None:
        super().__init__('mic_node')

        # --- Parameters --------------------------------------------------
        # ALSA device name substring or index (reSpeaker XVF3800 = "Array").
        self.declare_parameter('device', 'Array')
        self.declare_parameter('sample_rate', AudioPCM.SAMPLE_RATE)   # LOCKED
        self.declare_parameter('channels', AudioPCM.CHANNELS)         # LOCKED (mono)
        self.declare_parameter('chunk_ms', 20)

        device_param: str = str(self.get_parameter('device').value)
        self._sample_rate: int = int(self.get_parameter('sample_rate').value)
        self._channels: int = int(self.get_parameter('channels').value)
        chunk_ms: int = int(self.get_parameter('chunk_ms').value)

        if (self._sample_rate != AudioPCM.SAMPLE_RATE
                or self._channels != AudioPCM.CHANNELS):
            raise ValueError(
                f"Pipeline LOCKED to {AudioPCM.SAMPLE_RATE}Hz/{AudioPCM.CHANNELS}ch; "
                f"got {self._sample_rate}/{self._channels}")

        # Accept integer device index or substring name.
        device = int(device_param) if device_param.isdigit() else device_param

        # Detect native sample rate so we can resample if needed.
        dev_info = sd.query_devices(device, 'input')
        native_sr = int(dev_info['default_samplerate'])
        self._resample = native_sr != AudioPCM.SAMPLE_RATE
        if self._resample:
            self._up, self._down = _resample_gcd(native_sr, AudioPCM.SAMPLE_RATE)
            self.get_logger().info(
                f"Device native rate={native_sr}Hz; "
                f"resampling ×{self._up}/{self._down} → {AudioPCM.SAMPLE_RATE}Hz")

        # blocksize in native samples → chunk_ms output at TARGET_RATE.
        self._blocksize = native_sr * chunk_ms // 1000
        self._chunks = 0

        # --- ROS I/O -----------------------------------------------------
        self._pub = self.create_publisher(
            AudioPCM, '/onboard/sensors/audio/pcm', SENSOR_QOS)

        # --- Audio stream ------------------------------------------------
        self._stream = sd.InputStream(
            device=device,
            samplerate=native_sr,
            channels=1,
            dtype='int16',
            blocksize=self._blocksize,
            callback=self._on_audio,
        )
        self._stream.start()

        self.get_logger().info(
            f"mic_node capturing {dev_info['name']!r} (device={device!r}) "
            f"{native_sr}Hz mono int16 → /onboard/sensors/audio/pcm "
            f"({AudioPCM.SAMPLE_RATE}Hz/{AudioPCM.CHANNELS}ch, chunk={chunk_ms}ms)")

    # -------------------------------------------------------------------
    # sounddevice callback — called from sounddevice's internal thread
    # -------------------------------------------------------------------
    def _on_audio(self, indata: np.ndarray, frames: int, time_info, status) -> None:
        if status:
            self.get_logger().warning(f"sounddevice status: {status}")

        samples = indata[:, 0]  # mono (shape: frames,)

        if self._resample:
            resampled = resample_poly(samples.astype(np.float32), self._up, self._down)
            samples = np.clip(np.rint(resampled), -32768, 32767).astype(np.int16)

        msg = AudioPCM()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mic'
        msg.sample_rate = AudioPCM.SAMPLE_RATE
        msg.channels = AudioPCM.CHANNELS
        msg.bit_depth = AudioPCM.BIT_DEPTH
        msg.data = samples.tobytes()
        self._pub.publish(msg)

        self._chunks += 1
        if self._chunks % 250 == 0:   # ~5s at 50Hz
            self.get_logger().info(f"published {self._chunks} chunks")

    # -------------------------------------------------------------------
    # Shutdown: stop stream
    # -------------------------------------------------------------------
    def destroy_node(self) -> None:
        self.get_logger().info('mic_node shutting down; stopping audio stream...')
        try:
            self._stream.stop()
            self._stream.close()
        except Exception:
            pass
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
