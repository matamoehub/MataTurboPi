import importlib.util
import math
import sys
import types
from pathlib import Path


def _load_buzzer_lib_with_stubs():
    # Stub ROS modules so we can unit-test pure music logic without ROS runtime.
    rclpy_mod = types.ModuleType("rclpy")
    rclpy_mod.ok = lambda: True
    rclpy_mod.init = lambda args=None: None
    sys.modules["rclpy"] = rclpy_mod

    node_mod = types.ModuleType("rclpy.node")

    class _Node:
        def __init__(self, *_a, **_k):
            pass

        def create_publisher(self, *_a, **_k):
            class _Pub:
                def publish(self, _msg):
                    pass

            return _Pub()

    node_mod.Node = _Node
    sys.modules["rclpy.node"] = node_mod

    exec_mod = types.ModuleType("rclpy.executors")

    class _Exec:
        def add_node(self, _node):
            pass

        def spin_once(self, timeout_sec=0.0):
            pass

    exec_mod.SingleThreadedExecutor = _Exec
    sys.modules["rclpy.executors"] = exec_mod

    qos_mod = types.ModuleType("rclpy.qos")

    class _QoSProfile:
        def __init__(self, **_kwargs):
            pass

    class _QoSRel:
        RELIABLE = "RELIABLE"

    class _QoSHist:
        KEEP_LAST = "KEEP_LAST"

    qos_mod.QoSProfile = _QoSProfile
    qos_mod.QoSReliabilityPolicy = _QoSRel
    qos_mod.QoSHistoryPolicy = _QoSHist
    sys.modules["rclpy.qos"] = qos_mod

    msg_mod = types.ModuleType("ros_robot_controller_msgs.msg")

    class _BuzzerState:
        def __init__(self):
            self.freq = 0
            self.on_time = 0.0
            self.off_time = 0.0
            self.repeat = 1

        def get_fields_and_field_types(self):
            return {
                "freq": "uint16",
                "on_time": "float32",
                "off_time": "float32",
                "repeat": "uint16",
            }

    msg_mod.BuzzerState = _BuzzerState
    sys.modules["ros_robot_controller_msgs.msg"] = msg_mod

    buzzer_path = Path(__file__).resolve().parents[1] / "lib" / "buzzer_lib.py"
    spec = importlib.util.spec_from_file_location("buzzer_lib_for_test", str(buzzer_path))
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_note_to_freq_a4():
    buzzer_lib = _load_buzzer_lib_with_stubs()
    assert buzzer_lib.note_to_freq("A4") == 440


def test_note_to_freq_sharp_and_flat_match():
    buzzer_lib = _load_buzzer_lib_with_stubs()
    assert buzzer_lib.note_to_freq("C#4") == buzzer_lib.note_to_freq("Db4")


def test_note_to_freq_rest_is_silence():
    buzzer_lib = _load_buzzer_lib_with_stubs()
    assert buzzer_lib.note_to_freq("R") == 0


def test_note_to_freq_invalid_note_raises():
    import pytest

    buzzer_lib = _load_buzzer_lib_with_stubs()
    with pytest.raises(ValueError):
        buzzer_lib.note_to_freq("H2")


def test_play_notes_parses_score_tokens():
    buzzer_lib = _load_buzzer_lib_with_stubs()
    calls = []

    class _Fake:
        def play_note(self, note, beats=1.0, bpm=120):
            calls.append((note, beats, bpm))

    buzzer_lib.Buzzer.play_notes(_Fake(), "C4:1 D4:0.5 R:2 E4", bpm=90)
    assert calls == [("C4", 1.0, 90), ("D4", 0.5, 90), ("R", 2.0, 90), ("E4", 1.0, 90)]


def test_note_to_freq_c4_is_close_to_reference():
    buzzer_lib = _load_buzzer_lib_with_stubs()
    # C4 is 261.63 Hz; implementation rounds to integer.
    assert math.isclose(buzzer_lib.note_to_freq("C4"), 262, rel_tol=0, abs_tol=1)

