# kist-drl-g1-onboard

**KIST DRL — Unitree G1 NX Onboard Software Stack**

ROS 2 Humble workspace for the Jetson Orin NX onboard the Unitree G1.
Sensor publishing, UWB pose streaming, NX↔PC bridging, real-time safety, motor execution.

> Target HW: Unitree G1 + Jetson Orin NX (Ubuntu 22.04, JetPack 6.x)
> Companion repo: `kist-drl-g1-ws` (PC side — VLA / Cortex / Providers)

> 🚧 **Scaffold only.** Node bodies carry `TODO(REQ-XX) [TASK-XX]` markers that
> link back to the Notion Task DB.

---

## Packages

| Package | Build | Role |
|---|---|---|
| `g1_onboard_msgs` | ament_cmake | Custom interfaces (AudioPCM, JointCmd, …) |
| `sensors` | ament_python | RealSense / mic / speaker / joint_state / IMU (base + ankle L/R) / UWB |
| `comm_bridge` | ament_python | `/onboard/` ↔ `/bridge/` relay + QoS conversion |
| `safety_monitor` | ament_python | 200 ms E-STOP + command validation (systemd) |
| `motor_controller` | ament_python | 100 Hz G1 SDK dispatch (systemd) |

---

## Build & Run

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

./scripts/run_onboard.sh                       # all nodes
ros2 launch <pkg> <pkg>.launch.py              # one package
```

`safety_monitor` and `motor_controller` run as **systemd services** on the deployed
NX — copy each package's `systemd/*.service` to `/etc/systemd/system/` and enable.

---

## Topic Convention

- `/onboard/*` — NX-internal (blocked from the LAN by `config/cyclonedds.xml` partition)
- `/bridge/*` — NX ↔ PC shared

Full interface contract lives in the ICD database (link below).

### Sensors detail

| Topic | Producer | Note |
|---|---|---|
| `/onboard/sensors/camera/color`, `/depth` | `camera_node` (RealSense) | → PC VLA via `comm_bridge` |
| `/onboard/sensors/audio/pcm` | `mic_node` | → PC STT via `comm_bridge` |
| `/onboard/sensors/joint_states` | `joint_state_node` | → PC VLA via `comm_bridge` |
| `/onboard/sensors/imu/base` | `imu_node` | base IMU from lowstate |
| `/onboard/sensors/imu/ankle_left`, `/ankle_right` | `imu_node` | for GearSonic balance correction |
| `/onboard/sensors/uwb_pose` | `uwb_node` | consumed by PC `TaskSrvProvider` for sub-task success detection (no onboard navigation) |

`imu_node` (added 2026-05-22, ownership unified 2026-05-23) owns all IMU
streams; `joint_state_node` publishes joint states only. See the
[Notion **CONV** page](https://app.notion.com/p/377b39de7dd780b391f3ceec30226a0e) for the rationale.

---

## Engineering rules

Code-level architectural decisions (topic namespace, audio
sub-namespaces, IMU ownership, systemd isolation, build system,
real-time budgets) are documented in the [Notion **CONV** page](https://app.notion.com/p/377b39de7dd780b391f3ceec30226a0e).
The C4 container diagram lives in the shared `KIST_DRL_G1_Arch.drawio`
file (linked from Notion), not duplicated in this repo.

---

## Where the spec lives

| Layer | Notion |
|---|---|
| Requirements | [SYS-REQ DB](https://www.notion.so/d7d7c9b9943b4018a4bce2afb904d706) |
| Interface contracts | [ICD DB](https://www.notion.so/b319b5cec8f2429389fb5fac8c042503) |
| Work items | [Tasks DB](https://www.notion.so/cd779d7a54b343b6a9e5449f4620a44c) |
| Verification | [Tests DB](https://www.notion.so/a67e62ef1cfc4f85be29a340107846b6) |

Each `TODO([TASK-XX])` in code links to the matching Task page.

---

## Contributing

PRs are squash-merged to `main`. Conventions enforced in CI:

- Branch name: `TASK-{number}` (Notion-linked work) or `chore/{description}` (non-task housekeeping — e.g. `chore/fix-typo-in-readme`, `chore/bump-cyclonedds-dep`)
- PR title: `[TASK-{number}] <type>(<scope>)?: <subject>` or `[chore] <type>(<scope>)?: <subject>` ([Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/))

`chore/...` branches are for housekeeping that doesn't warrant a Notion Task (typo fixes, dep bumps, comment cleanup). Notion tracking is by-pass for these.

**PR title casing rules** (enforced by the regex):

| Part | Rule | Example |
|---|---|---|
| `type` | lowercase, must be in the allowed set (`feat`, `fix`, `docs`, `style`, `refactor`, `perf`, `test`, `build`, `ci`, `chore`, `revert`) | `feat` |
| `scope` (optional) | lowercase letters / digits / `_` / `-` only | `(sensors)`, `(comm_bridge)` |
| `subject` first char | must be lowercase | `a` in `add UWB driver` |
| `subject` after first char | anything — acronyms like `UWB`, `ROS`, `NX` are OK | `add UWB driver` |

Examples: `[TASK-42] feat(sensors): add UWB driver` ✓ &nbsp;&nbsp; `[TASK-42] feat(sensors): Add UWB driver` ✗

See `.github/workflows/pr-meta.yml`.
