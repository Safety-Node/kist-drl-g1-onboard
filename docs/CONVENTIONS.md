# Conventions — kist-drl-g1-onboard

Code-level architectural decisions for the NX onboard (Jetson Orin NX) side.
The system spec (requirements, ICDs, container diagram, change log) lives in
Notion + the shared drawio file; this file captures **engineering rules**
that contributors should follow when writing or reviewing onboard code.

If you make a new decision that affects multiple files or future tasks,
append it here in the same format (see *Pattern* at the bottom).

---

## CONV-001 — Topic namespace: `/onboard/*` (internal) vs `/bridge/*` (PC-shared)

**Status**: Accepted · **Date**: 2026-05-14

### Context
NX runs ROS 2 nodes that produce both internal data (sensor raw, safety
intermediates) and data the PC stack needs (camera, audio, joint state,
UWB pose, IMU). Letting everything onto the wire wastes bandwidth and
makes the ICD ambiguous about what's a contract vs an implementation
detail.

### Decision
Two top-level namespaces, separated by intent:

| Prefix         | Scope          | Crosses Ethernet?                                  |
|----------------|----------------|----------------------------------------------------|
| `/onboard/...` | NX internal    | ❌ (blocked by `config/cyclonedds.xml` partition)  |
| `/bridge/...`  | NX ↔ PC shared | ✅                                                 |

`comm_bridge` is the **only** package that subscribes `/onboard/*` and
publishes `/bridge/*` (and vice-versa for commands). Other NX nodes never
publish or subscribe `/bridge/*` directly.

### Consequences
- ✅ One choke point (`comm_bridge`) for QoS conversion + partition policy.
- ✅ ICD entries map 1:1 to `/bridge/*` topics → contract is unambiguous.
- ⚠️ Adding a PC-facing stream means editing `comm_bridge` even if the
  producer node is otherwise self-contained. Acceptable cost.

### Affected
- `config/cyclonedds.xml` (partition filter)
- `src/comm_bridge/comm_bridge/bridge_node.py`

---

## CONV-002 — Audio sub-namespaces: mic under `/sensors/`, speaker under `/audio/`

**Status**: Accepted · **Date**: 2026-05-16

### Context
Mic and speaker are both "audio" but play opposite roles. Lumping both
under `/onboard/sensors/audio/*` muddles the sensor-vs-actuator
distinction the rest of the topic tree relies on.

### Decision
Mic is a **sensor** (input device), speaker is an **actuator** (output
device). Their topics live in different subtrees:

| Topic                            | Role                                      |
|----------------------------------|-------------------------------------------|
| `/onboard/sensors/audio/pcm`     | mic capture                               |
| `/onboard/audio/playback`        | speaker input (TTS samples in)            |
| `/onboard/audio/speaker_state`   | speaker telemetry (STT echo-cancel hint)  |

`/onboard/sensors/*` = "world data coming in." Anything an actuator
emits or consumes lives outside `/sensors/`.

### Consequences
- ✅ Sensor/actuator separation stays consistent across all node types.
- ✅ STT echo-cancel hint topic (`/audio/speaker_state`) is co-located
  with the actuator that produces it, not buried under `/sensors/`.

---

## CONV-003 — IMU ownership: one `imu_node` for all IMU sources

**Status**: Accepted · **Date**: 2026-05-23 (supersedes 2026-05-15 lowstate fan-out)

### Context
G1 has multiple IMU sources: base IMU (from lowstate) and ankle L/R IMUs
(needed for GearSonic, added 2026-05-22). Originally base IMU was
fan-out by `joint_state_node` because both came from the same lowstate
packet. With ankle IMUs added, that mixed naming (`joint_state_node`
publishing IMU) became inconsistent.

### Decision
A single `imu_node` owns all IMU streams:

```
/onboard/sensors/imu/base
/onboard/sensors/imu/ankle_left
/onboard/sensors/imu/ankle_right
```

`joint_state_node` publishes only `/onboard/sensors/joint_states`.

DDS multi-subscriber cost at 100 Hz lowstate is negligible — both nodes
can subscribe the same upstream packet independently.

### Consequences
- ✅ Node name ↔ topic prefix symmetry (`imu_node` ↔ `/onboard/sensors/imu/*`).
- ✅ Future IMU sources (wrist, head, …) drop in under the same node /
  prefix without renaming.
- ⚠️ Two nodes subscribe the same lowstate stream. Cost is small but
  non-zero; revisit if NX CPU becomes tight.

### Affected
- `src/sensors/sensors/imu_node.py` (new)
- `src/sensors/sensors/joint_state_node.py` (IMU publish removed)

---

## CONV-004 — Process isolation: `safety_monitor` + `motor_controller` as systemd units

**Status**: Accepted · **Date**: 2026-05-14

### Context
E-STOP path budget is 200 ms (REQ-35) and motor control loop is 100 Hz
(REQ-34 v2026-05-22). Running these under `ros2 launch` alongside
sensor / bridge nodes means scheduling jitter from unrelated ROS
processes can blow the budget.

### Decision
`safety_monitor` and `motor_controller` ship as **systemd units**, not
`ros2 launch` children:

- `Nice=-20` (highest priority)
- `CPUAffinity=0/1` (pinned to dedicated cores)
- `Restart=always`
- Python `gc.disable()` in the hot path
- Shared-memory flag for the E-STOP fast path (DDS in parallel for redundancy)

Other ROS nodes (`sensors`, `comm_bridge`) run as ordinary
`ros2 launch` processes — they tolerate scheduling jitter.

### Consequences
- ✅ Critical loops get OS-level priority + CPU isolation.
- ✅ systemd handles restart-on-crash without manual supervision.
- ⚠️ Deployment now has two surfaces (systemd + ros2 launch); the
  `systemd/*.service` files under each package must be copied to
  `/etc/systemd/system/` on the NX. Documented in README.

### Affected
- `src/safety_monitor/systemd/*.service`
- `src/motor_controller/systemd/*.service`

---

## CONV-005 — Build system: `ament_cmake` only where required, else `ament_python`

**Status**: Accepted · **Date**: 2026-05-14

### Context
ROS 2 supports both `ament_cmake` and `ament_python`. CMake is heavier
to maintain but required for IDL (`.msg` / `.srv`) code generation;
pure-Python nodes have no reason to pay that cost.

### Decision

| Package             | Build         | Reason                                             |
|---------------------|---------------|----------------------------------------------------|
| `g1_onboard_msgs`   | `ament_cmake` | required for ROS IDL (.msg) code generation       |
| `sensors`           | `ament_python`| pure-Python nodes                                  |
| `comm_bridge`       | `ament_python`| pure-Python nodes                                  |
| `safety_monitor`    | `ament_python`| pure-Python nodes                                  |
| `motor_controller`  | `ament_python`| pure-Python nodes                                  |

New packages default to `ament_python` unless they declare a `.msg` /
`.srv` / `.action` interface.

### Consequences
- ✅ Less CMake plumbing to maintain per package.
- ⚠️ If a Python package later needs to bundle C++ extensions, it has
  to migrate to `ament_cmake_python` — accept the migration cost when
  it happens rather than pre-paying it.

---

## CONV-006 — Real-time budgets (single source of truth: Notion SYS-REQ)

**Status**: Accepted · **Date**: 2026-05-22

### Context
Real-time numbers (E-STOP 200 ms, motor loop 100 Hz, NX↔PC RTT 5 ms)
are referenced from many places (node docstrings, README, this file).
Multiple sources of truth = stale numbers when they change (e.g.
20 Hz → 100 Hz move on 2026-05-22).

### Decision
The **Notion SYS-REQ DB** is the single source of truth for numeric
budgets. Onboard code references them by REQ-ID, not by inline number:

| Path                              | Authoritative REQ                  |
|-----------------------------------|------------------------------------|
| E-STOP detection → motor stop     | REQ-35 (≤ 200 ms)                  |
| Motor control loop                | REQ-34 v2026-05-22 (100 Hz / 10 ms)|
| Low-level control end-to-end      | REQ-38 v2026-05-22 (100 Hz / 99%)  |
| NX↔PC RTT                         | REQ-33 (< 5 ms)                    |

Node-level docstrings cite the REQ-ID; the README quotes the number for
convenience but always with the REQ-ID alongside.

### Consequences
- ✅ When a budget changes, update Notion + the README — code keeps the
  REQ-ID citation and stays correct.
- ⚠️ Reviewers can't grep a number in code to know if it's still
  current; they have to look up the REQ-ID in Notion. Tooling could
  inline this later (CI check against a YAML mirror of REQ-IDs).

---

## CONV-007 — Walking lives in PC-side whole-body VLA (no onboard path planner)

**Status**: Accepted · **Date**: 2026-05-22 (KIST mail)

### Context
Original plan split locomotion (high-level `LocoClient.Move`) from
manipulation (low-level VLA joint cmd). KIST observed visible
BalanceStand discontinuities at high↔low transitions and required
≥ 100 Hz low-level control.

### Decision
Walking is generated **directly by the PC-side whole-body VLA** — there
is no separate path planner anywhere. The NX:

- has no `navigation` package (deleted 2026-05-22)
- has no `goto_node` (deleted with navigation)
- has no `named_goals.yaml`
- receives `/bridge/cmd/arm` (rt/arm_sdk) + `/bridge/cmd/low` (rt/lowcmd)
  from PC and dispatches them through the G1 SDK
- still publishes `/bridge/sensors/uwb_pose` but only because the
  PC-side `TaskSrvProvider` uses it for sub-task success detection
  (e.g. "reached fridge?"). NX does not consume UWB.

`LocoClient` FSM (rt/loco) is retained for demo entry/exit + posture
commands; full usage scope is TBD.

### Consequences
- ✅ Continuous whole-body motion at the VLA's chosen rate, no FSM
  transition discontinuity.
- ✅ NX stays sensor + safety + motor dispatch only — simpler scope.
- ⚠️ Onboard repo lost ~one whole package (`navigation`); historical
  ICDs (`Nav Cmd Goal`, `Validated Twist`) are marked `[DEPRECATED]`
  in Notion.

### Affected
- (deleted) `src/navigation/` package
- `src/motor_controller/` — `/bridge/cmd/low` route added, VELOCITY_CMD
  mode dropped (부활 2026-05-26: workstation CONV-012 로 PC
  NavigationProvider 가 다시 `/bridge/cmd/vel` 발행. velocity_buf /
  VELOCITY_CMD / LocoClient.Move 의미 복원. EstopFlag.REASON_VELOCITY 도
  부활.)
- `src/safety_monitor/` — validates `/bridge/cmd/{arm,low}` instead of
  `Validated Twist` (2026-05-26: cmd_vel watchdog 도 다시 active —
  per-stream comms watchdog 만, validated_twist publication 은 여전히
  dropped.)

---

## Pattern for new conventions

When a decision affects multiple tasks or future code review, add a new
section with:

```
## CONV-NNN — Title
**Status**: Accepted | Open | Superseded by CONV-MMM · **Date**: YYYY-MM-DD

### Context
What problem are we solving? Constraints, alternatives considered briefly.

### Decision
What we chose. Concrete, code-pointing if possible.

### Consequences
Trade-offs, follow-ups, what it constrains in the future.

### Affected (optional)
Files / Notion SYS-REQ / other CONVs touched.
```

Reference: the per-decision history is also reflected in the Notion
**Meta Data → Spec Change Log** row dated when it was first agreed,
and the C4 container diagram lives in the shared `KIST_DRL_G1_Arch.drawio`
file (not duplicated here).
