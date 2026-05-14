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

---

## NX Container Map (post-2026-05-14)

```
G1 Onboard (Orin NX)                                  ↕ Ethernet/CycloneDDS ↕ PC (RTX 4090)

┌──────────────────────────────┐
│ sensors                      │
│  - camera_node (RealSense)   │ ─ Color/Depth ───► comm_bridge ─► PC (VLA)
│  - audio_node  (ALSA)        │ ─ AudioPCM ──────► comm_bridge ─► PC (STT)
│  - joint_state_node          │ ─ JointState ────► comm_bridge ─► PC (VLA)
│  - uwb_node    (NEW)         │ ─ /onboard/sensors/uwb/pose (PoseStamped, map frame)
└──────────────────────────────┘                           │
                                                           ▼
                            ┌──────────────────────────────┐
                            │ navigation / goto_node       │ ◄── /onboard/cmd/nav_goal (from PC)
                            │ (P-controller, no Nav2)      │
                            └──────────────────────────────┘
                                          │ /onboard/navigation/cmd_vel
                                          ▼
        ┌──────────────────────────────────────────────┐
        │ safety_monitor                               │ ◄── Depth (RealSense)  (proximity E-STOP)
        │  - joint limit / velocity / proximity check  │ ◄── /onboard/cmd/joint (from PC)
        │  - 200 ms E-STOP path: DDS + shared memory   │
        └──────────────────────────────────────────────┘
                                          │ Validated Cmd / E-STOP
                                          ▼
                            ┌──────────────────────────────┐
                            │ motor_controller             │ ─ G1 SDK (LocoClient / lowcmd)
                            │ Ring Buffer 20 Hz + crossfade│
                            └──────────────────────────────┘
                                          │ Buf State
                                          ▼
                                     comm_bridge ─► PC
```

## Topic Convention

| Prefix | Scope | Crosses Ethernet? |
|---|---|---|
| `/onboard/...` | NX internal | ❌ (DDS partition filter) |
| `/bridge/...`  | NX ↔ PC shared | ✅ |

`comm_bridge` is the only container that subscribes `/onboard/*` and publishes to `/bridge/*`
(and vice-versa).

## Real-time Constraints

| Path | Requirement | Notes |
|---|---|---|
| E-STOP detection → motor stop | ≤ 200 ms (REQ-35) | shared memory flag (Python `gc.disable()`, CPUAffinity=0/1) |
| Motor control loop | 20 Hz / 50 ms (REQ-34) | busy-wait hybrid timer |
| Full pipeline VLA→motor | 20 Hz / 99% (REQ-38) | bottleneck is VLA inference, not NX |
| NX↔PC RTT | < 5 ms (REQ-33) | wired LAN only |

## Process Isolation

- `safety_monitor` and `motor_controller` ship as **systemd units** (Nice=-20, CPUAffinity=0/1,
  Restart=always). Other ROS nodes run as ordinary `ros2 launch` processes.

## Build System Choice per Package

| Package | Build type | Reason |
|---|---|---|
| `kist_drl_g1_msgs` | `ament_cmake` | required for ROS IDL (.msg) code generation |
| all others | `ament_python` | pure-Python nodes |
