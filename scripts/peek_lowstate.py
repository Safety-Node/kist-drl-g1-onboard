#!/usr/bin/env python3
"""
peek_lowstate.py — G1 SDK rt/lowstate를 N개 프레임만 받아 화면에 출력.

빌린 로봇과 같은 LAN에 물려있는 PC에서 실행하면, 그 로봇이 발사하는
실제 lowstate를 가로채서 볼 수 있음. 출력 텍스트를 그대로 복사해서
다른 곳(replay 파일 만들기, 정상 범위 파악 등)에 활용 가능.

사용법:
    python3 scripts/peek_lowstate.py            # DDS 기본 인터페이스
    python3 scripts/peek_lowstate.py eth0       # 인터페이스 명시
    python3 scripts/peek_lowstate.py wlan0      # 무선이면

인터페이스 모를 때: `ip -br link` 또는 `ifconfig`로 로봇과 같은 LAN의 NIC 찾기.

종료: N_FRAMES만큼 받으면 자동 종료. 또는 Ctrl-C.
"""
import sys
import time

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


N_FRAMES = 3                # 받을 프레임 수 (lowstate가 보통 500 Hz라 0.006초 분량)
PRINT_MOTOR_COUNT = 29      # G1 활성 모터 수
_received = 0


def handler(msg: LowState_) -> None:
    global _received
    _received += 1
    print(f'\n===== frame {_received}  '
          f'(tick={msg.tick}, mode_machine={msg.mode_machine}, mode_pr={msg.mode_pr}) =====')

    print(f'imu_state.quaternion    = {list(msg.imu_state.quaternion)}')
    print(f'imu_state.gyroscope     = {list(msg.imu_state.gyroscope)}')
    print(f'imu_state.accelerometer = {list(msg.imu_state.accelerometer)}')
    print(f'imu_state.rpy           = {list(msg.imu_state.rpy)}')
    print(f'imu_state.temperature   = {msg.imu_state.temperature}')

    qs   = [round(msg.motor_state[i].q,       4) for i in range(PRINT_MOTOR_COUNT)]
    dqs  = [round(msg.motor_state[i].dq,      4) for i in range(PRINT_MOTOR_COUNT)]
    taus = [round(msg.motor_state[i].tau_est, 4) for i in range(PRINT_MOTOR_COUNT)]
    print(f'q   (rad)   = {qs}')
    print(f'dq  (rad/s) = {dqs}')
    print(f'tau (Nm)    = {taus}')


def main() -> None:
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    sub = ChannelSubscriber('rt/lowstate', LowState_)
    sub.Init(handler, 10)

    print(f'waiting for {N_FRAMES} lowstate frames…')
    timeout_s = 10.0
    t0 = time.time()
    while _received < N_FRAMES:
        if time.time() - t0 > timeout_s:
            print(f'TIMEOUT: no frame in {timeout_s:.0f}s. '
                  f'네트워크/도메인 ID/방화벽 확인.')
            sys.exit(1)
        time.sleep(0.02)
    print('\ndone.')


if __name__ == '__main__':
    main()
