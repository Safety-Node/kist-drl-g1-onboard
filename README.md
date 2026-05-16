# kist-drl-g1-onboard

**KIST DRL — Unitree G1 NX Onboard Software Stack**

ROS 2 Humble workspace for the Jetson Orin NX onboard the Unitree G1.
Sensor publishing, UWB navigation, NX↔PC bridging, real-time safety, motor execution.

> Target HW: Unitree G1 + Jetson Orin NX (Ubuntu 22.04, JetPack 6.x)
> Companion repo: `kist-drl-g1-ws` (PC side — VLA / Cortex / Providers)

> 🚧 **Scaffold only.** Node bodies carry `TODO(REQ-XX) [TASK-XX]` markers that
> link back to the Notion Task DB.

---

## Packages

| Package | Build | Role |
|---|---|---|
| `g1_onboard_msgs` | ament_cmake | Custom interfaces (AudioPCM, JointCmd, …) |
| `sensors` | ament_python | RealSense / mic / speaker / joint_state / UWB |
| `comm_bridge` | ament_python | `/onboard/` ↔ `/bridge/` relay + QoS conversion |
| `navigation` | ament_python | UWB `goto_node` (P-controller) |
| `safety_monitor` | ament_python | 200 ms E-STOP + command validation (systemd) |
| `motor_controller` | ament_python | 20 Hz G1 SDK dispatch (systemd) |

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

- Branch name: `TASK-{number}[-kebab-description]`
- PR title: [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/)

See `.github/workflows/pr-meta.yml`.

