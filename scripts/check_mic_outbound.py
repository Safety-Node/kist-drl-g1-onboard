#!/usr/bin/env python3
"""
check_mic_outbound.py — integration probe for the OUTBOUND audio relay.

Mirror of comm_bridge_audio_check.py (which checks inbound PC→speaker) for the
mic→desktop direction:

    mic topic (domain 0)  --/onboard/sensors/audio/pcm-->  comm_bridge
                          --/bridge/sensors/audio_pcm-->  desktop STT (domain 1)

This process plays BOTH ends in separate rclpy contexts (exactly how comm_bridge
separates domains). comm_bridge must already be running.

QoS: BEST_EFFORT both ends — comm_bridge relays this topic best_effort, so a
RELIABLE subscriber would NOT match the relay's best_effort publisher and receive
nothing. (This is the #1 desktop-STT gotcha; the probe pins it correctly.)

Two modes:
  (default) synthetic byte-for-byte probe — publish N known AudioPCM chunks on
            domain 0, assert they arrive intact on domain 1. Exit 0=PASS, 1=FAIL.
  --record <wav> [--seconds S] — instead, SUBSCRIBE to /bridge/sensors/audio_pcm
            on domain 1 and reassemble whatever arrives into a WAV. Use together
            with replay_mic_multicast.py (or the real mic_node) to validate the
            full chain end-to-end by ear / by diff against the source WAV.
"""
import argparse
import sys
import threading
import time
import wave

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from g1_onboard_msgs.msg import AudioPCM

# Match comm_bridge's best_effort relay of /onboard/sensors/audio/pcm.
QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=10)

SRC = "/onboard/sensors/audio/pcm"   # domain 0 (mic side)
DST = "/bridge/sensors/audio_pcm"    # domain 1 (desktop side)

N = 10
CHUNK = 640  # 20 ms @ 16 kHz mono 16-bit


def _spin_bg(ctx: Context, node: Node) -> SingleThreadedExecutor:
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    return ex


def run_record(wav_path: str, seconds: float) -> int:
    """Subscribe to the desktop topic and dump reassembled PCM to a WAV."""
    ctx = Context()
    rclpy.init(context=ctx, domain_id=1)
    node = Node("mic_outbound_recorder", context=ctx)

    chunks: list = []
    node.create_subscription(AudioPCM, DST, lambda m: chunks.append(bytes(m.data)), QOS)
    ex = _spin_bg(ctx, node)

    print(f"[INFO] recording {DST} (domain 1) for {seconds}s → {wav_path}")
    deadline = time.time() + seconds
    while time.time() < deadline:
        time.sleep(0.1)

    pcm = b"".join(chunks)
    with wave.open(wav_path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(16000)
        w.writeframes(pcm)

    ex.shutdown()
    secs = len(pcm) / (16000 * 2)
    print(f"[INFO] got {len(chunks)} chunks, {len(pcm)} bytes ({secs:.2f}s) → {wav_path}")
    print("RESULT   : " + ("PASS ✅ (audio arrived)" if chunks else "FAIL ❌ (nothing received)"))
    return 0 if chunks else 1


def run_probe() -> int:
    """Synthetic byte-for-byte relay check (no robot, no mic_node)."""
    ctx_onboard = Context()   # mic side
    ctx_bridge = Context()    # desktop side
    rclpy.init(context=ctx_onboard, domain_id=0)
    rclpy.init(context=ctx_bridge, domain_id=1)

    onboard = Node("mic_tx", context=ctx_onboard)
    desktop = Node("desktop_rx", context=ctx_bridge)

    received: list = []
    desktop.create_subscription(AudioPCM, DST, lambda m: received.append(m), QOS)
    pub = onboard.create_publisher(AudioPCM, SRC, QOS)

    ex = _spin_bg(ctx_bridge, desktop)
    time.sleep(2.0)  # domain-0 ↔ comm_bridge ↔ domain-1 discovery

    sent = []
    for i in range(N):
        m = AudioPCM()
        m.sample_rate, m.channels, m.bit_depth = 16000, 1, 16
        m.data = bytes([i]) * CHUNK
        pub.publish(m)
        sent.append(bytes(m.data))
        time.sleep(0.05)

    deadline = time.time() + 5.0
    while time.time() < deadline and len(received) < N:
        time.sleep(0.05)

    ok = len(received) >= N
    intact = ok and all(bytes(received[i].data) == sent[i] for i in range(N))
    fmt_ok = ok and received[0].sample_rate == 16000 and received[0].channels == 1

    print("\n--- comm_bridge OUTBOUND audio relay ---")
    print(f"sent     : {N} chunks to {SRC} (domain 0)")
    print(f"received : {len(received)} on {DST} (domain 1)")
    print(f"payload  : {'byte-for-byte intact' if intact else 'MISMATCH'}")
    print(f"format   : {'16k/mono/16bit preserved' if fmt_ok else 'WRONG'}")
    print(f"RESULT   : {'PASS ✅' if (ok and intact and fmt_ok) else 'FAIL ❌'}\n")

    ex.shutdown()
    return 0 if (ok and intact and fmt_ok) else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="Outbound mic→desktop relay probe.")
    ap.add_argument("--record", metavar="WAV", help="record DST topic to a WAV instead of the byte probe")
    ap.add_argument("--seconds", type=float, default=5.0, help="record duration (with --record)")
    args = ap.parse_args()

    if args.record:
        return run_record(args.record, args.seconds)
    return run_probe()


if __name__ == "__main__":
    sys.exit(main())
