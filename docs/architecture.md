# Architecture — G1 NX Onboard

Authoritative spec lives in Notion. This file is a developer-facing summary.

- C4 diagram: `KIST_DRL_G1_Arch.drawio.pdf` (note: still shows SLAM / LiDAR — to be updated)
- Container validation index: [Notion](https://www.notion.so/Container-Validation-355b39de7dd78017b419c16735040a3f)
- System requirements DB (SYS-REQ): [Notion](https://www.notion.so/d7d7c9b9943b4018a4bce2afb904d706)

---

## Spec change log

| Date | Change | Rationale |
|---|---|---|
| 2026-05-14 | SLAM/Nav2/LiDAR removed; UWB-only localisation; single `goto_node` replaces Nav2 stack; obstacle avoidance dropped (fixed demo environment); Camera Depth retained for safety proximity E-STOP only | Demo-driven simplification, time-saving (REQ-37 redefined) |
| 2026-05-16 | nav_goal channel reverts to `std_msgs/String` named goal (e.g. `"refrigerator"`); NX `goto_node` owns `named_goals.yaml` lookup again. Reverts 2026-05-15 decision 5a. | One-line CLI debugging without standing up the PC stack |
| 2026-05-22 | (1) NX `navigation` package removed — path planning moved to PC; `Nav Cmd Goal` + `Validated Twist` ICDs deprecated. (2) Walking integrated into low-level VLA (rt/lowcmd via `/onboard/cmd/low`); new `Joint Cmd Lower Body` ICD; `motor_controller` VELOCITY_CMD mode dropped. (3) Control loop 20 Hz → **100 Hz** (REQ-34/38 v2026-05-22). (4) Ankle IMU streaming added (`imu_ankle_node` → `/onboard/sensors/imu/ankle_{left,right}`) as GearSonic input. (5) GearSonic (whole-body balance correction) added to spec as 🚧 사양 합의 대기. | KIST mail (Yim,Sehyuk, 2026-05-22): high↔low transition's BalanceStand discontinuity, low-level minimum 100 Hz, GearSonic introduction |

---

## NX Container Map (post-2026-05-22)

```
G1 Onboard (Orin NX)                                  ↕ Ethernet/CycloneDDS ↕ PC (RTX 4090)

┌──────────────────────────────┐
│ sensors                      │
│  - camera_node (RealSense)   │ ─ Color/Depth ───► comm_bridge ─► PC (VLA)
│  - mic / speaker_node        │ ─ AudioPCM ──────► comm_bridge ─► PC (STT)
│  - joint_state_node          │ ─ JointState + IMU(base) ─► comm_bridge ─► PC (VLA)
│  - imu_ankle_node  (NEW 22)  │ ─ IMU(ankle L/R) ─► comm_bridge ─► PC (GearSonic)
│  - uwb_node                  │ ─ UWB Pose ─────► comm_bridge ─► PC (PC-side path planner)
└──────────────────────────────┘
                                                                ▲
        ┌──────────────────────────────────────────────┐        │
        │ safety_monitor                               │        │  /bridge/cmd/{arm, low, loco}
        │  - joint limit / velocity / proximity check  │ ◄──────┘  from PC (whole-body VLA + FSM)
        │  - 200 ms E-STOP path: DDS + shared memory   │
        │  - 100 Hz validation loop (REQ-34 v2026-05-22)│
        └──────────────────────────────────────────────┘
                                          │ Validated Joint / E-STOP
                                          ▼
                            ┌──────────────────────────────┐
                            │ motor_controller             │ ─ G1 SDK
                            │ 100 Hz loop + Ring Buffer    │   ├─ rt/arm_sdk    (from /cmd/arm)
                            │ + crossfade fallback         │   ├─ rt/lowcmd     (from /cmd/low, NEW)
                            │                              │   └─ LocoClient FSM (from /cmd/loco — demo entry/exit only)
                            └──────────────────────────────┘
                                          │ Buf State
                                          ▼
                                     comm_bridge ─► PC
```

Path planning lives on PC since 2026-05-22 (KIST mail) — walking is generated
by the whole-body VLA and arrives as `/bridge/cmd/low`. `goto_node` /
`named_goals.yaml` previously on NX have been retired together with the
`navigation` package.

## Topic Convention

| Prefix | Scope | Crosses Ethernet? |
|---|---|---|
| `/onboard/...` | NX internal | ❌ (DDS partition filter) |
| `/bridge/...`  | NX ↔ PC shared | ✅ |

`comm_bridge` is the only container that subscribes `/onboard/*` and publishes to `/bridge/*`
(and vice-versa).

### Audio sub-namespaces (2026-05-16)

Mic and speaker live under different prefixes on purpose:

| Topic | Role |
|---|---|
| `/onboard/sensors/audio/pcm` | mic capture (mic = sensor / input device) |
| `/onboard/audio/playback` | speaker input (speaker = actuator / output device) |
| `/onboard/audio/speaker_state` | speaker telemetry (STT echo-cancel hint) |

Only `mic_node` is a sensor; `speaker_node` is an actuator, so its inputs and
outputs sit directly under `/onboard/audio/` rather than under `/onboard/sensors/`.

## Real-time Constraints

| Path | Requirement | Notes |
|---|---|---|
| E-STOP detection → motor stop | ≤ 200 ms (REQ-35) | shared memory flag (Python `gc.disable()`, CPUAffinity=0/1) |
| Motor control loop | **100 Hz / 10 ms** (REQ-34 v2026-05-22) | busy-wait hybrid timer (rerated from 20 Hz; 20 ms ramp count → 100 steps for the same ~2.0 s envelope) |
| Low-level control loop end-to-end | 100 Hz / 99% (REQ-38 v2026-05-22) | VLA chunk emission (~15 Hz, 16-step chunks) replayed step-by-step at 100 Hz; bottleneck is wire + safety, not NX |
| NX↔PC RTT | < 5 ms (REQ-33) | wired LAN only |

## Process Isolation

- `safety_monitor` and `motor_controller` ship as **systemd units** (Nice=-20, CPUAffinity=0/1,
  Restart=always). Other ROS nodes run as ordinary `ros2 launch` processes.

## Build System Choice per Package

| Package | Build type | Reason |
|---|---|---|
| `g1_onboard_msgs` | `ament_cmake` | required for ROS IDL (.msg) code generation |
| all others (`sensors`, `comm_bridge`, `safety_monitor`, `motor_controller`) | `ament_python` | pure-Python nodes |
