#!/usr/bin/env python3
"""
speaker_node 단일 기능 격리 테스트 (NX 온보드에서 실행).

comm_bridge / PC / TTS / Clova 전부 빼고, speaker_node 하나만 검증한다.
이 스크립트가 직접 /onboard/audio/playback 에 테스트 오디오(기본: 사인파 톤)를
publish 하고, speaker_node 가 발행하는 /onboard/audio/speaker_state 를 구독해
playing 전이를 출력한다.

검증 경로:
    [이 스크립트] sine tone → publish /onboard/audio/playback (AudioPCM)
       → speaker_node._on_pcm → 큐 → writer → AudioClient.PlayStream → 🔊 로봇 스피커
       → speaker_node publish /onboard/audio/speaker_state
       → [이 스크립트] 구독 → playing=True→False 전이 로그

전제 (둘 다 NX 온보드에서):
    1) 빌드:   cd ~/.../kist-drl-g1-onboard && colcon build && source install/setup.bash
    2) 스피커: ros2 run sensors speaker_node      (AudioClient init — 로봇 오디오 서비스 필요)
    3) 테스트: python3 scripts/exercise_speaker_live.py
실행 예:
    python3 scripts/exercise_speaker_live.py                  # 440Hz, 1.5s 톤
    python3 scripts/exercise_speaker_live.py --freq 880 --seconds 2
    python3 scripts/exercise_speaker_live.py --chunk-ms 200   # 200ms 청크로 쪼개 pace
    python3 scripts/exercise_speaker_live.py --wav hello.wav  # 16k/mono/16bit WAV 재생
"""

import argparse
import math
import sys
import time
import wave
from array import array

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from g1_onboard_msgs.msg import AudioPCM, SpeakerState

RATE = AudioPCM.SAMPLE_RATE     # 16000
CH = AudioPCM.CHANNELS          # 1
DEPTH = AudioPCM.BIT_DEPTH      # 16
BPS = DEPTH // 8                # 2 bytes/sample


def make_tone(freq: float, seconds: float, amp: float) -> bytes:
    """16kHz mono int16 사인파 PCM 생성 (numpy 없이 표준 라이브러리만)."""
    n = int(RATE * seconds)
    peak = int(max(0.0, min(1.0, amp)) * 32767)
    buf = array('h')  # signed int16
    two_pi_f = 2.0 * math.pi * freq
    for i in range(n):
        buf.append(int(peak * math.sin(two_pi_f * i / RATE)))
    return buf.tobytes()


def load_wav(path: str) -> bytes:
    """WAV 로드. 16k/mono/16bit 아니면 경고(그대로 보내 speaker_node가 drop하는지도 확인 가능)."""
    with wave.open(path, "rb") as wf:
        sr, ch, sw = wf.getframerate(), wf.getnchannels(), wf.getsampwidth()
        frames = wf.readframes(wf.getnframes())
    if (sr, ch, sw) != (RATE, CH, BPS):
        print(f"[warn] WAV {sr}Hz/{ch}ch/{sw*8}bit != locked {RATE}/{CH}/{DEPTH} "
              f"— speaker_node 가 format mismatch 로 drop 할 수 있음", file=sys.stderr)
    return frames


class SpeakerProbe(Node):
    def __init__(self, chunk_ms: int):
        super().__init__("exercise_speaker_live")
        self._chunk_ms = chunk_ms
        # speaker_node 와 동일한 기본 QoS(RELIABLE/depth10) → int 10 으로 매칭.
        self._pub = self.create_publisher(AudioPCM, "/onboard/audio/playback", 10)
        self._states = 0
        self._saw_playing = False
        self.create_subscription(
            SpeakerState, "/onboard/audio/speaker_state", self._on_state, 10)
        self.get_logger().info(
            "publisher: /onboard/audio/playback  |  "
            "subscriber: /onboard/audio/speaker_state")

    def _on_state(self, msg: SpeakerState) -> None:
        self._states += 1
        if msg.playing:
            self._saw_playing = True
        self.get_logger().info(
            f"SPEAKER_STATE  playing={msg.playing}  "
            f"chunk_id={msg.current_chunk_id}  queue_depth={msg.queue_depth}")

    def wait_for_speaker(self, timeout_s: float = 5.0) -> bool:
        """speaker_node 가 playback 구독을 붙일 때까지 대기."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._pub.get_subscription_count() > 0

    def _msg(self, pcm: bytes) -> AudioPCM:
        m = AudioPCM()
        m.header = Header()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "test"
        m.sample_rate = RATE
        m.channels = CH
        m.bit_depth = DEPTH
        m.data = array('B', pcm)   # uint8[]
        return m

    def play(self, pcm: bytes) -> None:
        if self._chunk_ms <= 0:
            self._pub.publish(self._msg(pcm))
            self.get_logger().info(f"published 1 msg, {len(pcm)} bytes "
                                   f"({len(pcm)/(RATE*BPS):.2f}s)")
            return
        step = RATE * BPS * self._chunk_ms // 1000
        step -= step % BPS
        n = 0
        for i in range(0, len(pcm), step):
            self._pub.publish(self._msg(pcm[i:i + step]))
            n += 1
            time.sleep(self._chunk_ms / 1000.0 * 0.9)   # 실시간보다 약간 빠르게 (큐 오버플로우 방지)
        self.get_logger().info(f"published {n} chunks, {len(pcm)} bytes total")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--freq", type=float, default=440.0, help="톤 주파수(Hz)")
    ap.add_argument("--seconds", type=float, default=1.5, help="톤 길이(초)")
    ap.add_argument("--amp", type=float, default=0.3, help="진폭 0~1 (0.3 권장, 과도하면 클리핑)")
    ap.add_argument("--wav", default=None, help="톤 대신 재생할 WAV 파일(16k/mono/16bit)")
    ap.add_argument("--chunk-ms", type=int, default=0, help="0=한 메시지 / >0=쪼개 pace")
    ap.add_argument("--listen-sec", type=float, default=6.0, help="발행 후 상태 대기(초)")
    args = ap.parse_args()

    pcm = load_wav(args.wav) if args.wav else make_tone(args.freq, args.seconds, args.amp)
    if len(pcm) % 2 != 0:
        pcm = pcm[:-1]  # int16 정렬 보장 (speaker_node 가 홀수 길이 drop)

    rclpy.init()
    node = SpeakerProbe(chunk_ms=args.chunk_ms)

    if not node.wait_for_speaker(5.0):
        node.get_logger().warn(
            "구독자 없음 — speaker_node 가 떠 있는지 확인하세요 "
            "(ros2 run sensors speaker_node). 그래도 발행은 시도함.")

    src = f"WAV {args.wav}" if args.wav else f"tone {args.freq:.0f}Hz {args.seconds}s"
    node.get_logger().info(f"play: {src}")
    node.play(pcm)

    node.get_logger().info(f"listening {args.listen_sec:.1f}s for speaker_state ...")
    end = time.monotonic() + max(1.0, args.listen_sec)
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    states, saw = node._states, node._saw_playing
    node.destroy_node()
    rclpy.shutdown()

    if saw:
        print(f"[PASS] speaker_state {states}건, playing=True 관측 — speaker_node 재생 확인")
        return 0
    print(f"[FAIL] speaker_state {states}건, playing=True 미관측. "
          f"speaker_node 로그(PlayStream/AudioClient) 와 ros2 topic echo "
          f"/onboard/audio/speaker_state 확인.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
