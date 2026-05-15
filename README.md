# kist-drl-g1-onboard

**KIST DRL — Unitree G1 NX Onboard Software Stack**

ROS 2 Humble workspace running on the Jetson Orin NX onboard the Unitree G1 humanoid.
Handles sensor publishing, UWB-based point-to-point navigation, NX↔PC bridging,
real-time safety, and motor execution.

> Target HW: Unitree G1 + Jetson Orin NX (Ubuntu 22.04, JetPack 6.x)
> Companion repo: `kist-drl-g1-ws` (PC / RTX 4090 side — VLA / Cortex / Providers)

---

## Packages

| Package | Build | Role |
|---|---|---|
| `kist_drl_g1_msgs` | `ament_cmake` | Custom interfaces (AudioPCM, …) |
| `sensors` | `ament_python` | RealSense Camera / IMU / Mic / JointState / **UWB** publishers |
| `comm_bridge` | `ament_python` | `/onboard/` ↔ `/bridge/` topic relay + QoS conversion |
| `navigation` | `ament_python` | UWB-based `goto_node` (P-controller, no Nav2 stack) |
| `safety_monitor` | `ament_python` | 200ms E-STOP, joint/velocity/proximity validation (systemd) |
| `motor_controller` | `ament_python` | 20Hz control loop, Ring Buffer + VLA chunk crossfade (systemd) |

See [docs/architecture.md](docs/architecture.md) for the data flow and topic naming convention.

---

## Design Notes (2026-05-14 spec change)

- **Localisation: UWB only.** SLAM / AMCL / LiDAR removed. `uwb_node` publishes absolute
  `PoseStamped` in the `map` frame.
- **Navigation: P-controller.** Nav2 stack (map_server, planner_server, controller_server,
  bt_navigator) removed. The demo environment is fixed and obstacle-free, so a single
  `goto_node` drives the robot directly to named goals.
- **Camera: kept** — RealSense D435i still feeds (a) safety_monitor proximity E-STOP via
  Depth, and (b) PC-side VLA inference via `comm_bridge`.
- **LiDAR: removed.** Livox MID-360 is no longer in the loop.
- **IMU: published, not consumed.** Available for future safety / RT monitoring use.

## Design Notes (2026-05-15 package reviews)

- **Audio split.** `audio_node` → `mic_node` (capture) + `speaker_node` (playback + state).
  Pipeline format locked to 16 kHz / 16-bit / mono; PC-side TTS (CLOVA at 22/24 kHz)
  resamples before publishing.
- **IMU fan-out.** `joint_state_node` subscribes G1 SDK lowstate once and publishes both
  `sensor_msgs/JointState` and `sensor_msgs/Imu`. No standalone `imu_node`.
- **Motor routing (4 modes).** `motor_controller` dispatches by mode:
  `VELOCITY_CMD → LocoClient.Move()`, `JOINT_CMD → rt/arm_sdk` (with weight blending,
  G1's `motor_cmd[29].q`), `LOCO_CMD → LocoClient.{Damp,StandUp,...}()`,
  `ESTOP → LocoClient.Damp() + weight ramp 1.0→0.0 + buffer flush`. `rt/lowcmd` is a
  debug-only optional path.
- **Safety output split.** `safety_monitor` publishes two validated topics
  (`validated_twist` + `validated_joint`) instead of a single `validated_cmd`, plus
  `EstopFlag` on DDS and a POSIX shared-memory byte for the zero-latency stop path.
- **Named goals: PC owns.** `goto_node` accepts `PoseStamped` only; PC resolves the
  name → coordinates before publishing on `/bridge/cmd/nav_goal`.

---

## Quick start for a new developer

1. **현재 위치 잡기** — `docs/architecture.md`를 1분 안에 훑으면 NX 데이터 흐름이 손에 잡힘.
2. **본인 작업 찾기** — REQ ID로 grep:
   ```bash
   grep -rn "TODO(REQ-35)" src/   # 예: 안전 관련 작업
   grep -rn "TODO(REQ-34)" src/   # 예: 모터 제어
   grep -rn "TODO(infra)"  .      # 빌드/배포 등 spec 무관 항목
   ```
3. **단일 패키지 빌드** — 전체 build 안 돌리고:
   ```bash
   ./scripts/build.sh sensors        # 하나만
   ```
4. **드라이버 없이 dev 빌드** — RealSense / G1 SDK 미설치 상태에서도 launch 통과:
   - 기본값: 경고만 출력 후 진행
   - 실배포 보드: `SENSORS_REQUIRE_CAMERA=1` 로 하드 fail 권장
5. **노드 단독 실행** — 전체 스택 안 띄우고 한 노드만 테스트:
   ```bash
   source install/setup.bash
   ros2 run sensors mic_node --ros-args --params-file install/sensors/share/sensors/config/sensors_params.yaml
   ```

---

## Build (host, native colcon)

```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Install dependencies (first time)
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source the overlay
source install/setup.bash
```

Convenience wrapper: `./scripts/build.sh`.

---

## Run

```bash
# All onboard nodes
./scripts/run_onboard.sh

# Or individually
ros2 launch sensors           sensors.launch.py
ros2 launch comm_bridge       comm_bridge.launch.py
ros2 launch navigation        navigation.launch.py
ros2 launch safety_monitor    safety_monitor.launch.py
ros2 launch motor_controller  motor_controller.launch.py
```

`safety_monitor` and `motor_controller` are intended to run as **systemd services** on the
deployed Orin NX. Unit files live under each package's `systemd/` directory — copy to
`/etc/systemd/system/` and `systemctl enable --now`.

---

## Topic Naming Convention

- `/onboard/*` — NX-internal only. DDS partition filter (`config/cyclonedds.xml`) blocks
  them from the Ethernet wire.
- `/bridge/*`  — Shared with the PC workstation (same DDS Domain ID).

Command channels (2026-05-15 routing convention):

| Topic | Type | Path |
|---|---|---|
| `/onboard/cmd/arm`  | `kist_drl_g1_msgs/JointCmd`    | upper-body via `rt/arm_sdk` (weight blending) |
| `/onboard/cmd/loco` | `kist_drl_g1_msgs/LocoCommand` | discrete LocoClient actions (Damp / StandUp / ...) |
| `/onboard/cmd/nav_goal` | `geometry_msgs/PoseStamped` | navigation target (PC resolves named goal first) |
| `/onboard/safety/validated_twist` | `geometry_msgs/Twist` | safety-gated walking command |
| `/onboard/safety/validated_joint` | `kist_drl_g1_msgs/JointCmd` | safety-gated arm command |
| `/onboard/safety/estop` | `kist_drl_g1_msgs/EstopFlag` | structured E-STOP event (DDS, PC-facing) |
| POSIX shm `safety_flag` | uint8 byte | zero-latency E-STOP (motor_controller polls every tick) |

---

## Status

🚧 **Scaffold only.** Every node body and config file currently contains `TODO(REQ-XX)`
markers tied to the SYS-REQ Notion database (or `TODO(infra)` for build/deploy items).
No business logic is implemented yet.

Grep your way to work:
```bash
grep -rn "TODO(REQ"   src/   # spec-tied work
grep -rn "TODO(infra" .      # build / deploy / CI items
```

REQ ID legend:
- REQ-27: voice command recognition (STT input)
- REQ-29: voice response output (TTS playback)
- REQ-30: named-goal routing
- REQ-32: NX↔PC bidirectional gateway
- REQ-33: NX↔PC wired transport (5 ms RTT)
- REQ-34: 20 Hz motor control
- REQ-35: motion command validation + 200 ms E-STOP (ISO 13482)
- REQ-37: indoor autonomous navigation (now UWB-based)
- REQ-38: 20 Hz pipeline timing NFR
- REQ-42: sensor data collection & PC delivery (HAL)
