#!/usr/bin/env python3
"""
Manual verifier for safety_monitor (robot connected, NO motion). [TASK-33]

safety_monitor never drives the robot — it only validates and raises E-STOP —
so this is safe to run with the robot powered/connected as long as
motor_controller is OFF or dry_run=true. Each scenario launches its own
safety_monitor (clean latch state), injects one input, prints the E-STOP reason,
and tears the node down.

VLA is assumed NOT publishing chunks: validated_joint_chunk stays silent and the
arm/low comms watchdog never arms on its own — that is expected, not a failure.

Run (ROS + workspace overlay sourced):
    python3 src/safety_monitor/test/verify_safety_monitor.py

Do NOT have another safety_monitor running (it shares topics + the SHM byte).
"""
import os
import pathlib
import signal
import subprocess
import sys
import time
from multiprocessing import shared_memory

import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from g1_onboard_msgs.msg import EstopFlag, JointCmd, JointCmdChunk
from sensor_msgs.msg import JointState

REPO = pathlib.Path(__file__).resolve().parents[3]
PARAMS = REPO / 'install/safety_monitor/share/safety_monitor/config/safety_params.yaml'
SHM_NAME = 'safety_flag'

_RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=10)
_BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=1)

_REASON = {0: 'NONE', 1: 'JOINT_LIMIT', 2: 'VELOCITY', 3: 'PROXIMITY',
           4: 'COMMS_TIMEOUT', 5: 'WATCHDOG', 6: 'MANUAL', 7: 'SELF_COLLISION',
           8: 'MALFORMED_CMD'}


def step(names=('left_knee',), q=(0.0,), dq=None, mode=0, weight=0.0):
    n = len(names)
    s = JointCmd()
    s.joint_names = list(names)
    s.q = [float(x) for x in q]
    s.dq = [0.0] * n if dq is None else [float(x) for x in dq]
    s.kp = [0.0] * n
    s.kd = [0.0] * n
    s.tau_ff = [0.0] * n
    s.mode = mode
    s.weight = float(weight)
    return s


def chunk(steps, chunk_id=1):
    c = JointCmdChunk()
    c.chunk_id = chunk_id
    c.steps = list(steps)
    return c


class Node:
    """One safety_monitor process + a test pub/sub node."""

    def __init__(self, idx):
        cmd = ['ros2', 'run', 'safety_monitor', 'safety_monitor_node',
               '--ros-args', '--params-file', str(PARAMS)]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL, start_new_session=True)
        self.node = rclpy.create_node(f'sm_verify_{idx}')
        self.arm = self.node.create_publisher(JointCmdChunk, '/onboard/cmd/arm', _RELIABLE)
        self.js = self.node.create_publisher(JointState, '/onboard/sensors/joint_states', _BEST_EFFORT)
        self._estops = []
        self.node.create_subscription(EstopFlag, '/onboard/safety/estop', self._estops.append, _RELIABLE)
        self._shm = None

    def wait_ready(self, timeout=15):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self._estops and self.arm.get_subscription_count() > 0:
                return True
        raise TimeoutError('safety_monitor did not come up')

    def spin(self, dt):
        end = time.time() + dt
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_estop(self, timeout=3.0):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            for m in self._estops:
                if m.active:
                    return m
        return None

    def heartbeat_count(self, dt=3.2):
        self._estops.clear()
        self.spin(dt)
        return len(self._estops)

    def shm_byte(self):
        if self._shm is None:
            self._shm = shared_memory.SharedMemory(name=SHM_NAME, create=False)
        return self._shm.buf[0]

    def close(self):
        if self._shm is not None:
            self._shm.close()
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            self.proc.wait(timeout=5)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        self.node.destroy_node()


# --- scenarios: each returns (estop_msg_or_None, expected_reason) ---

def sc_malformed(n):
    n.arm.publish(chunk([step(names=('left_knee',), q=(0.0, 0.0))]))  # q len 2 != names 1
    return n.wait_estop(), EstopFlag.REASON_MALFORMED_CMD


def sc_joint_limit(n):
    n.arm.publish(chunk([step(names=('left_knee',), q=(10.0,))]))
    return n.wait_estop(), EstopFlag.REASON_JOINT_LIMIT


def sc_rate(n):
    n.arm.publish(chunk([step(names=('left_knee',), q=(0.0,)),
                         step(names=('left_knee',), q=(0.5,))]))  # Δq 0.5 > 0.20
    return n.wait_estop(), EstopFlag.REASON_JOINT_LIMIT


def sc_comms(n):
    n.arm.publish(chunk([step(names=('left_knee',), q=(0.0,), weight=0.0)]))  # arm, weight 0 = no motion
    return n.wait_estop(timeout=2.0), EstopFlag.REASON_COMMS_TIMEOUT  # then silence > 0.5s


def sc_staleness(n):
    js = JointState()
    js.name = ['left_knee']
    js.position = [0.0]
    for _ in range(3):
        n.js.publish(js)
        n.spin(0.1)
    return n.wait_estop(timeout=2.0), EstopFlag.REASON_COMMS_TIMEOUT  # silence > 0.5s


SCENARIOS = [
    ('malformed (array length)', sc_malformed),
    ('joint_limit (q out of range)', sc_joint_limit),
    ('rate-of-change (delta q)', sc_rate),
    ('comms watchdog (arm silent)', sc_comms),
    ('state staleness (joint_states)', sc_staleness),
]


def main():
    if subprocess.run(['pgrep', '-f', 'lib/safety_monitor/safety[_]monitor_node'],
                      stdout=subprocess.DEVNULL).returncode == 0:
        print('!! another safety_monitor is already running — kill it first '
              '(shares topics + SHM). Aborting.')
        sys.exit(1)

    rclpy.init()
    results = []
    try:
        for i, (name, fn) in enumerate(SCENARIOS):
            n = Node(i)
            try:
                n.wait_ready()
                m, expect = fn(n)
                got = m.reason if m else None
                shm = n.shm_byte() if m else 0
                ok = (got == expect) and (shm == 1)
                detail = m.detail if m else '(no E-STOP)'
                results.append(ok)
                tag = 'PASS' if ok else 'FAIL'
                print(f'[{tag}] {name:34s} expect={_REASON[expect]:<13} '
                      f'got={_REASON.get(got, "None"):<13} shm={shm}  "{detail}"')
            finally:
                n.close()

        # heartbeat (fresh node, no estop expected)
        n = Node(99)
        try:
            n.wait_ready()
            hb = n.heartbeat_count(3.2)
            ok = hb >= 3
            results.append(ok)
            print(f'[{"PASS" if ok else "FAIL"}] {"heartbeat 1Hz":34s} '
                  f'expect=>=3 over 3.2s  got={hb}')
        finally:
            n.close()
    finally:
        rclpy.shutdown()

    passed = sum(results)
    print(f'\n=== {passed}/{len(results)} passed ===')
    sys.exit(0 if passed == len(results) else 1)


if __name__ == '__main__':
    main()
