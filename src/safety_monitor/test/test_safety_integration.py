"""
HW-free integration harness for safety_monitor (REQ-35). [TASK-33]

safety_monitor is pure validation + E-STOP emission, so its whole pipeline is
testable without the robot: launch the real node, inject ROS messages, assert
on /onboard/safety/estop, /onboard/safety/validated_joint_chunk, and the POSIX
SHM 'safety_flag' byte. The robot is only needed for the *physical* stop
(motor_controller acting on SHM) — not exercised here.

Run (ROS + workspace overlay sourced):
    python -m pytest src/safety_monitor/test/test_safety_integration.py -v -s

Each test launches its own node process (clean latch state) and tears it down.
"""
import itertools
import os
import pathlib
import signal
import subprocess
import time
from multiprocessing import shared_memory

import numpy as np
import pytest
import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from g1_onboard_msgs.msg import EstopFlag, JointCmd, JointCmdChunk
from sensor_msgs.msg import Image, JointState

REPO = pathlib.Path(__file__).resolve().parents[3]
PARAMS = REPO / 'install/safety_monitor/share/safety_monitor/config/safety_params.yaml'
DEPTH_TOPIC = '/onboard/sensors/depth/image_raw'
SHM_NAME = 'safety_flag'

_RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=10)
_BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
_counter = itertools.count()


def make_step(names=('left_knee',), q=(0.0,), dq=None, kp=None, kd=None,
              tau=None, mode=0, weight=0.0):
    n = len(names)
    s = JointCmd()
    s.joint_names = list(names)
    s.q = [float(x) for x in q]
    s.dq = [0.0] * n if dq is None else [float(x) for x in dq]
    s.kp = [0.0] * n if kp is None else [float(x) for x in kp]
    s.kd = [0.0] * n if kd is None else [float(x) for x in kd]
    s.tau_ff = [0.0] * n if tau is None else [float(x) for x in tau]
    s.mode = mode
    s.weight = float(weight)
    return s


def make_chunk(steps, chunk_id=1):
    c = JointCmdChunk()
    c.chunk_id = chunk_id
    c.steps = list(steps)
    return c


def make_depth(width=64, height=64, fill_mm=5000, close=False, encoding='16UC1'):
    frame = np.full((height, width), fill_mm, dtype=np.uint16)
    if close:
        cy, cx = height // 2, width // 2
        frame[cy - 15:cy + 15, cx - 15:cx + 15] = 100  # 100 mm < 300 mm → close
    img = Image()
    img.height, img.width, img.encoding = height, width, encoding
    img.is_bigendian, img.step = 0, width * 2
    img.data = frame.tobytes()
    return img


class Harness:
    def __init__(self, **overrides):
        cmd = ['ros2', 'run', 'safety_monitor', 'safety_monitor_node',
               '--ros-args', '--params-file', str(PARAMS)]
        for k, v in overrides.items():
            cmd += ['-p', f'{k}:={v}']
        # start_new_session: `ros2 run` forks the node child; signal the whole
        # group on teardown so the node isn't orphaned onto shared topics.
        self.proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL, start_new_session=True)
        self.node = rclpy.create_node(f'sm_test_{next(_counter)}')
        self.arm = self.node.create_publisher(JointCmdChunk, '/onboard/cmd/arm', _RELIABLE)
        self.low = self.node.create_publisher(JointCmdChunk, '/onboard/cmd/low', _RELIABLE)
        self.js = self.node.create_publisher(JointState, '/onboard/sensors/joint_states', _BEST_EFFORT)
        self.depth = self.node.create_publisher(Image, DEPTH_TOPIC, _BEST_EFFORT)
        self._estops = []
        self._validated = []
        self.node.create_subscription(EstopFlag, '/onboard/safety/estop', self._estops.append, _RELIABLE)
        self.node.create_subscription(JointCmdChunk, '/onboard/safety/validated_joint_chunk',
                                      self._validated.append, _RELIABLE)
        self._shm = None

    # --- lifecycle ---
    def wait_ready(self, timeout=15):
        """Node up + discovery done: proven by first heartbeat AND cmd match."""
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self._estops and self.arm.get_subscription_count() > 0:
                return True
        raise TimeoutError('safety_monitor did not come up')

    def close(self):
        if self._shm is not None:
            self._shm.close()
        pgid = os.getpgid(self.proc.pid)
        try:
            os.killpg(pgid, signal.SIGINT)
            self.proc.wait(timeout=5)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        self.node.destroy_node()

    # --- helpers ---
    def spin(self, dt):
        end = time.time() + dt
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def reset(self):
        self._estops.clear()
        self._validated.clear()

    def wait_estop(self, timeout=3.0):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            for m in self._estops:
                if m.active:
                    return m
        return None

    def wait_validated(self, timeout=2.0):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self._validated:
                return self._validated[0]
        return None

    def shm_byte(self):
        if self._shm is None:
            self._shm = shared_memory.SharedMemory(name=SHM_NAME, create=False)
        return self._shm.buf[0]


@pytest.fixture(scope='session', autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def launch():
    made = []

    def _make(**overrides):
        h = Harness(**overrides)
        made.append(h)
        h.wait_ready()
        h.reset()
        return h

    yield _make
    for h in made:
        h.close()


# ===== A. validation → E-STOP =====

def test_malformed_array_length(launch):
    h = launch()
    bad = make_step(names=('left_knee',), q=(0.0, 0.0))  # q len 2 != names 1
    h.arm.publish(make_chunk([bad]))
    m = h.wait_estop()
    assert m and m.reason == EstopFlag.REASON_MALFORMED_CMD, m
    assert 'length' in m.detail


def test_malformed_weight(launch):
    h = launch()
    h.arm.publish(make_chunk([make_step(weight=1.5)]))
    m = h.wait_estop()
    assert m and m.reason == EstopFlag.REASON_MALFORMED_CMD
    assert 'weight' in m.detail


def test_malformed_mode(launch):
    h = launch()
    h.arm.publish(make_chunk([make_step(mode=7)]))
    m = h.wait_estop()
    assert m and m.reason == EstopFlag.REASON_MALFORMED_CMD
    assert 'mode' in m.detail


def test_malformed_unknown_joint(launch):
    h = launch()
    h.arm.publish(make_chunk([make_step(names=('foo_joint',))]))
    m = h.wait_estop()
    assert m and m.reason == EstopFlag.REASON_MALFORMED_CMD
    assert 'unknown joint' in m.detail


def test_joint_limit_q(launch):
    h = launch()
    h.arm.publish(make_chunk([make_step(names=('left_knee',), q=(10.0,))]))
    m = h.wait_estop()
    assert m and m.reason == EstopFlag.REASON_JOINT_LIMIT
    assert 'q' in m.detail


def test_joint_limit_dq(launch):
    h = launch()
    h.arm.publish(make_chunk([make_step(names=('left_knee',), q=(0.0,), dq=(999.0,))]))
    m = h.wait_estop()
    assert m and m.reason == EstopFlag.REASON_JOINT_LIMIT
    assert 'dq' in m.detail


def test_joint_limit_tau(launch):
    h = launch()
    h.arm.publish(make_chunk([make_step(names=('left_knee',), q=(0.0,), tau=(999.0,))]))
    m = h.wait_estop()
    assert m and m.reason == EstopFlag.REASON_JOINT_LIMIT
    assert 'tau' in m.detail


def test_rate_of_change(launch):
    h = launch()
    s0 = make_step(names=('left_knee',), q=(0.0,))
    s1 = make_step(names=('left_knee',), q=(0.5,))  # Δq 0.5 > 0.20
    h.arm.publish(make_chunk([s0, s1]))
    m = h.wait_estop()
    assert m and m.reason == EstopFlag.REASON_JOINT_LIMIT
    assert 'rate' in m.detail


def test_bad_step_drops_whole_chunk(launch):
    h = launch()
    good = make_step(names=('left_knee',), q=(0.0,))
    bad = make_step(names=('left_knee',), q=(10.0,))
    h.arm.publish(make_chunk([good, bad]))
    assert h.wait_estop() is not None
    assert h.wait_validated(timeout=1.0) is None  # nothing forwarded


# ===== B. clamp mode + passthrough =====

def test_clamp_mode_clamps_value(launch):
    h = launch(joint_limit_estop='false')
    # left_knee q range ~[-0.087, 2.880]; 10.0 should clamp to q_max, no estop
    h.arm.publish(make_chunk([make_step(names=('left_knee',), q=(10.0,))]))
    out = h.wait_validated()
    assert out is not None
    assert out.steps[0].q[0] <= 2.8799
    assert h.wait_estop(timeout=0.5) is None


def test_passthrough_preserves_chunk(launch):
    h = launch()
    h.arm.publish(make_chunk([make_step(names=('left_knee',), q=(0.5,))], chunk_id=42))
    out = h.wait_validated()
    assert out is not None and out.chunk_id == 42
    assert abs(out.steps[0].q[0] - 0.5) < 1e-6


# ===== C. watchdogs =====

def test_comms_watchdog_arm(launch):
    h = launch()
    h.arm.publish(make_chunk([make_step(names=('left_knee',), q=(0.0,))]))  # arm the stream
    m = h.wait_estop(timeout=2.0)  # then silence > 0.5s
    assert m and m.reason == EstopFlag.REASON_COMMS_TIMEOUT
    assert 'arm' in m.detail


def test_state_staleness_joint_states(launch):
    h = launch()
    js = JointState()
    js.name = ['left_knee']
    js.position = [0.0]
    for _ in range(3):
        h.js.publish(js)
        h.spin(0.1)
    m = h.wait_estop(timeout=2.0)  # silence > 0.5s
    assert m and m.reason == EstopFlag.REASON_COMMS_TIMEOUT
    assert 'joint_states' in m.detail


def test_self_watchdog_via_sigstop(launch):
    h = launch()
    gid = os.getpgid(h.proc.pid)  # group: reaches the node child, not just `ros2 run`
    os.killpg(gid, signal.SIGSTOP)  # freeze the loop
    time.sleep(0.6)
    os.killpg(gid, signal.SIGCONT)  # resume → dt >> overrun factor
    m = h.wait_estop(timeout=2.0)
    assert m and m.reason == EstopFlag.REASON_WATCHDOG
    assert 'overrun' in m.detail


# ===== D. E-STOP path / SHM / heartbeat / latch =====

def test_shm_byte_flips(launch):
    h = launch()
    assert h.shm_byte() == 0
    h.arm.publish(make_chunk([make_step(names=('left_knee',), q=(10.0,))]))
    h.wait_estop()
    assert h.shm_byte() == 1


def test_heartbeat_1hz(launch):
    h = launch()
    h.reset()
    h.spin(3.2)
    assert len(h._estops) >= 3  # ~1 Hz


def test_latch_holds(launch):
    h = launch()  # estop_latch defaults true
    h.arm.publish(make_chunk([make_step(names=('left_knee',), q=(10.0,))]))
    assert h.wait_estop() is not None
    h.reset()
    h.arm.publish(make_chunk([make_step(names=('left_knee',), q=(0.0,))]))  # clean
    assert h.wait_validated(timeout=1.0) is None  # latched: still dropped
    assert h.shm_byte() == 1


def test_nonlatch_clears(launch):
    h = launch(estop_latch='false')
    h.arm.publish(make_chunk([make_step(names=('left_knee',), q=(10.0,))]))
    assert h.wait_estop() is not None
    h.arm.publish(make_chunk([make_step(names=('left_knee',), q=(0.0,))]))  # clean clears
    assert h.wait_validated(timeout=1.5) is not None
    assert h.shm_byte() == 0


# ===== D2. SW E-STOP latency (inject → SHM=1), no robot needed =====

def test_estop_latency_under_200ms(launch):
    h = launch(estop_latch='false')  # so each trial self-clears
    bad = make_chunk([make_step(names=('left_knee',), q=(10.0,))])
    good = make_chunk([make_step(names=('left_knee',), q=(0.0,))])
    n, worst = 50, 0.0
    for _ in range(n):
        while h.shm_byte() != 0:           # ensure cleared
            h.arm.publish(good); h.spin(0.02)
        t0 = time.time()
        h.arm.publish(bad)
        while h.shm_byte() != 1 and time.time() - t0 < 0.5:
            pass
        worst = max(worst, time.time() - t0)
        assert h.shm_byte() == 1, 'estop did not fire'
    print(f'\n[latency] worst over {n} trials = {worst * 1e3:.1f} ms')
    assert worst < 0.200


# ===== E. proximity (synthetic depth, no camera) =====

def test_proximity_triggers(launch):
    h = launch(proximity_enable='true')
    for _ in range(3):
        h.depth.publish(make_depth(close=True))
        h.spin(0.1)
    m = h.wait_estop(timeout=2.0)
    assert m and m.reason == EstopFlag.REASON_PROXIMITY
    assert 'within' in m.detail


def test_proximity_clear_distance(launch):
    h = launch(proximity_enable='true')
    # stream continuously so depth-staleness can't fire; isolate the proximity check
    end = time.time() + 1.0
    while time.time() < end:
        h.depth.publish(make_depth(close=False))
        h.spin(0.1)
    assert not any(e.active for e in h._estops)


def test_proximity_fail_open_bad_encoding(launch):
    h = launch(proximity_enable='true')
    end = time.time() + 1.0
    while time.time() < end:
        h.depth.publish(make_depth(close=True, encoding='rgb8'))  # skipped, fail-open
        h.spin(0.1)
    assert not any(e.active for e in h._estops)
