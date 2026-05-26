"""
to_joint_state() 순수 함수 단위테스트.

ROS 노드 / rclpy.init 없이 동작. 테스트 로컬 fake로 LowState_ 모양을 흉내
→ to_joint_state 호출 → 필드 매핑 확인. IMU 매핑은 imu_node 책임이므로
별도 테스트 (TASK-38).

실행:
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    pytest src/sensors/test/test_to_joint_state.py -v
"""
from builtin_interfaces.msg import Time

from sensors.joint_state_node import to_joint_state


# ---------------------------------------------------------------------------
# 테스트 로컬 fake — G1 LowState_의 motor_state[i].q/dq/tau_est 만 흉내.
# 실제 LowState_ 대신 duck-typing으로 to_joint_state에 넘김.
# ---------------------------------------------------------------------------
class _FakeMotor:
    __slots__ = ('q', 'dq', 'tau_est')

    def __init__(self) -> None:
        self.q = 0.0
        self.dq = 0.0
        self.tau_est = 0.0


class _FakeLowState:
    def __init__(self, n_motor: int = 35) -> None:
        self.motor_state = [_FakeMotor() for _ in range(n_motor)]


def _stamp() -> Time:
    return Time(sec=1234, nanosec=5678)


# ---------------------------------------------------------------------------
# JointState 매핑
# ---------------------------------------------------------------------------
def test_joint_state_field_mapping():
    """motor_state[i].q/dq/tau_est → JointState.position/velocity/effort[i]."""
    state = _FakeLowState(n_motor=35)
    state.motor_state[0].q, state.motor_state[0].dq, state.motor_state[0].tau_est = 0.5, 0.1, 1.2
    state.motor_state[1].q = -0.3
    state.motor_state[2].q = 1.5

    joint_names = ['left_hip_pitch', 'left_hip_roll', 'left_hip_yaw']
    js = to_joint_state(state, joint_names, _stamp())

    # ROS float64[] 필드는 array.array('d')로 노출 — 비교 전 list로 변환.
    assert js.name == joint_names
    assert list(js.position) == [0.5, -0.3, 1.5]
    assert js.velocity[0] == 0.1
    assert js.effort[0] == 1.2


def test_joint_count_matches_names_length():
    """joint_names가 N개면 position/velocity/effort 모두 N개."""
    state = _FakeLowState(n_motor=35)
    joint_names = [f'joint_{i}' for i in range(29)]

    js = to_joint_state(state, joint_names, _stamp())

    assert len(js.position) == 29
    assert len(js.velocity) == 29
    assert len(js.effort) == 29


# ---------------------------------------------------------------------------
# Header
# ---------------------------------------------------------------------------
def test_header_stamp_passthrough():
    state = _FakeLowState(n_motor=35)
    stamp = Time(sec=42, nanosec=1234)

    js = to_joint_state(state, ['joint_0'], stamp)

    assert js.header.stamp == stamp
