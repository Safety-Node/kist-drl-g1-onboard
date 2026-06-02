#!/usr/bin/env python3
"""
replay_mic_multicast.py — drive mic_node WITHOUT the robot.

Streams a 16 kHz / mono / 16-bit WAV over UDP multicast to the same group the
robot audio service uses (239.168.123.161:5555), at real-time pace. To mic_node
this is indistinguishable from the live robot mic, so the whole pipeline
(mic_node → comm_bridge → desktop STT) can be exercised on the bench.

No ROS / SDK needed — pure socket. Pairs with check_mic_outbound.py.

Usage:
  python3 scripts/replay_mic_multicast.py <wav> [--iface IP] [--loop]
    e.g. python3 scripts/replay_mic_multicast.py /tmp/g1_record_test100.wav \
             --iface 192.168.123.99 --loop

Notes:
  - --iface is the LOCAL NIC IP used to send multicast (match mic_node's
    iface_ip). On a single host, kernel loopback delivers it back to mic_node.
  - WAV must be 16 kHz / mono / 16-bit (the locked AudioPCM pipeline format).
"""
import argparse
import socket
import struct
import sys
import time
import wave

GROUP_IP = "239.168.123.161"   # robot PC1 multicast group
PORT = 5555
SAMPLE_RATE = 16000
CHANNELS = 1
SAMPWIDTH = 2                  # 16-bit


def load_pcm(path: str) -> bytes:
    with wave.open(path, "rb") as w:
        rate, ch, width = w.getframerate(), w.getnchannels(), w.getsampwidth()
        if (rate, ch, width) != (SAMPLE_RATE, CHANNELS, SAMPWIDTH):
            print(f"[ERROR] {path}: got {rate}Hz/{ch}ch/{width*8}bit, "
                  f"need {SAMPLE_RATE}/{CHANNELS}/16. Resample first.")
            sys.exit(1)
        return w.readframes(w.getnframes())


def main() -> int:
    ap = argparse.ArgumentParser(description="Replay a WAV over the robot mic multicast group.")
    ap.add_argument("wav")
    ap.add_argument("--iface", default="192.168.123.99", help="local NIC IP for multicast out")
    ap.add_argument("--group", default=GROUP_IP)
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--chunk-ms", type=int, default=20, help="packet size in ms")
    ap.add_argument("--ttl", type=int, default=1)
    ap.add_argument("--loop", action="store_true", help="repeat forever")
    args = ap.parse_args()

    pcm = load_pcm(args.wav)
    chunk_bytes = SAMPLE_RATE * CHANNELS * SAMPWIDTH * args.chunk_ms // 1000
    chunk_sec = args.chunk_ms / 1000.0
    n_chunks = (len(pcm) + chunk_bytes - 1) // chunk_bytes

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(args.iface))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, args.ttl)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)  # deliver to local mic_node

    print(f"[INFO] replay {args.wav}: {len(pcm)} bytes, {n_chunks} chunks "
          f"({args.chunk_ms}ms/{chunk_bytes}B) → {args.group}:{args.port} via {args.iface}"
          f"{' [loop]' if args.loop else ''}")

    try:
        while True:
            t0 = time.time()
            for i in range(n_chunks):
                chunk = pcm[i * chunk_bytes:(i + 1) * chunk_bytes]
                sock.sendto(chunk, (args.group, args.port))
                # real-time pace: keep wall-clock aligned to the audio timeline
                target = t0 + (i + 1) * chunk_sec
                slack = target - time.time()
                if slack > 0:
                    time.sleep(slack)
            print(f"[INFO] sent {n_chunks} chunks ({len(pcm) / (SAMPLE_RATE * SAMPWIDTH):.2f}s audio)")
            if not args.loop:
                break
    except KeyboardInterrupt:
        print("\n[INFO] stopped")
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
