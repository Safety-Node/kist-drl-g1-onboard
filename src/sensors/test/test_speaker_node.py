"""
L0 unit tests for speaker_node — no robot, no Unitree SDK required.

The Unitree SDK (`unitree_sdk2py`) is faked in sys.modules before importing the
node, so these run on any machine with rclpy + g1_onboard_msgs built (e.g. CI).
They exercise the pure logic: format validation, chunk-id allocation, queue
drop-OLDEST overflow, SpeakerState transitions, and that the writer thread hands
PCM to AudioClient.PlayStream.

Skipped automatically where rclpy or g1_onboard_msgs is unavailable.
"""
import sys
import time
import types

import pytest

rclpy = pytest.importorskip('rclpy')


# --- Fake unitree_sdk2py so `import sensors.speaker_node` succeeds anywhere ---
class _FakeAudioClient:
    def __init__(self):
        self.play_calls = []
        self.stop_calls = []

    def SetTimeout(self, _t):
        pass

    def Init(self):
        pass

    def PlayStream(self, app_name, stream_id, pcm_bytes):
        self.play_calls.append((app_name, stream_id, bytes(pcm_bytes)))
        return 0, b''

    def PlayStop(self, app_name):
        self.stop_calls.append(app_name)
        return 0, b''


def _install_fake_unitree():
    for name in ('unitree_sdk2py', 'unitree_sdk2py.core', 'unitree_sdk2py.g1',
                 'unitree_sdk2py.g1.audio'):
        sys.modules.setdefault(name, types.ModuleType(name))
    core = types.ModuleType('unitree_sdk2py.core.channel')
    core.ChannelFactoryInitialize = lambda *a, **k: None
    sys.modules['unitree_sdk2py.core.channel'] = core
    audio = types.ModuleType('unitree_sdk2py.g1.audio.g1_audio_client')
    audio.AudioClient = _FakeAudioClient
    sys.modules['unitree_sdk2py.g1.audio.g1_audio_client'] = audio


_install_fake_unitree()

try:
    from g1_onboard_msgs.msg import AudioPCM
    from sensors.speaker_node import SpeakerNode, IDLE_CHUNK_ID, MAX_CHUNK_ID
except Exception as e:  # noqa: BLE001
    pytest.skip(f"sensors / g1_onboard_msgs not built: {e}",
                allow_module_level=True)


# --------------------------------------------------------------------------
# Fixtures / helpers
# --------------------------------------------------------------------------
@pytest.fixture(scope='module', autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    """A SpeakerNode with its writer thread stopped, for deterministic logic tests."""
    n = SpeakerNode()
    n._running = False
    n._wake.set()
    n._thread.join(timeout=2.0)
    # Capture published SpeakerState messages instead of going on the wire.
    n._states = []
    n._state_pub.publish = lambda m: n._states.append(m)  # type: ignore[assignment]
    yield n
    n.destroy_node()


def make_pcm(data: bytes, rate=16000, channels=1, bit_depth=16) -> AudioPCM:
    msg = AudioPCM()
    msg.sample_rate = rate
    msg.channels = channels
    msg.bit_depth = bit_depth
    msg.data = data
    return msg


# --------------------------------------------------------------------------
# Format validation
# --------------------------------------------------------------------------
def test_validate_accepts_locked_format(node):
    assert node._validate(make_pcm(b'\x00\x00' * 160)) is True


def test_validate_rejects_wrong_rate(node):
    assert node._validate(make_pcm(b'\x00\x00' * 160, rate=24000)) is False


def test_validate_rejects_wrong_channels(node):
    assert node._validate(make_pcm(b'\x00\x00' * 160, channels=2)) is False


def test_validate_rejects_empty(node):
    assert node._validate(make_pcm(b'')) is False


def test_validate_rejects_odd_length(node):
    assert node._validate(make_pcm(b'\x00\x00\x00')) is False  # 3 bytes, not int16-aligned


# --------------------------------------------------------------------------
# Chunk-id allocation (uint32, 0 reserved for idle)
# --------------------------------------------------------------------------
def test_alloc_chunk_id_increments(node):
    a = node._alloc_chunk_id()
    b = node._alloc_chunk_id()
    assert (a, b) == (1, 2)


def test_alloc_chunk_id_wraps_skipping_zero(node):
    node._next_chunk_id = MAX_CHUNK_ID
    assert node._alloc_chunk_id() == MAX_CHUNK_ID
    assert node._next_chunk_id == 1  # wrapped to 1, not 0 (idle reserved)


# --------------------------------------------------------------------------
# Queue overflow → drop OLDEST, depth capped at max_queue_depth (< 256)
# --------------------------------------------------------------------------
def test_queue_drops_oldest_on_overflow(node):
    cap = node._max_q
    for _ in range(cap + 10):
        node._on_pcm(make_pcm(b'\x01\x02' * 80))
    assert len(node._queue) == cap  # deque(maxlen) dropped the oldest
    # queue_depth must stay representable as uint8
    assert node._states[-1].queue_depth == cap
    assert cap < 256


# --------------------------------------------------------------------------
# SpeakerState transitions
# --------------------------------------------------------------------------
def test_idle_state_representation(node):
    # __init__ already emitted the first idle state via the real publisher
    # (before this fixture swapped in the capture hook), so publish once more
    # now that nothing is queued and the node is still idle.
    node._publish_state()
    assert node._states[-1].playing is False
    assert node._states[-1].current_chunk_id == IDLE_CHUNK_ID
    assert node._states[-1].queue_depth == 0


def test_push_publishes_state(node):
    before = len(node._states)
    node._on_pcm(make_pcm(b'\x00\x00' * 80))
    assert len(node._states) == before + 1
    assert node._states[-1].queue_depth == 1


# --------------------------------------------------------------------------
# Writer thread actually plays via AudioClient.PlayStream
# --------------------------------------------------------------------------
def test_writer_calls_playstream():
    n = SpeakerNode()  # writer running
    try:
        n._on_pcm(make_pcm(b'\x10\x20' * 80))
        deadline = time.time() + 2.0
        while time.time() < deadline and not n._client.play_calls:
            time.sleep(0.02)
        assert n._client.play_calls, 'PlayStream was never called'
        app_name, stream_id, pcm = n._client.play_calls[0]
        assert app_name == n._app_name
        assert stream_id  # non-empty session id
        assert pcm == b'\x10\x20' * 80
    finally:
        n.destroy_node()
