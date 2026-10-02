"""Tests for arm_lib high-level sequences (grab/place/ready/self_test).

These exercise the REAL arm_lib.Arm logic against a fake servo board, so they
verify the actual servo command order the robot would send — not a stub. Plus
a namespace-level check that the sequences degrade safely on a robot with no
arm (like this dev machine, which has no ros_robot_controller_sdk).
"""
import importlib
import importlib.util
import sys
import types
from pathlib import Path

_LIB = Path(__file__).resolve().parents[1] / "lib"


def _load_arm_lib():
    if str(_LIB) not in sys.path:
        sys.path.insert(0, str(_LIB))
    sys.modules.pop("arm_lib", None)
    return importlib.import_module("arm_lib")


class _FakeBoard:
    """Records pwm_servo_set_position(duration, [[servo_id, position]]) calls."""
    def __init__(self):
        self.calls = []

    def pwm_servo_set_position(self, duration, pairs):
        # pairs is like [[servo_id, position]]
        self.calls.append((pairs[0][0], pairs[0][1]))


def _armed(arm_lib):
    """A real Arm with a fake board injected so available is True."""
    arm = arm_lib.Arm()           # construction never raises; board is None here
    board = _FakeBoard()
    arm._board = board            # inject
    arm._error = None
    assert arm.available is True
    return arm, board


def test_grab_sends_open_lower_close_raise_in_order():
    arm_lib = _load_arm_lib()
    arm, board = _armed(arm_lib)
    assert arm.grab(settle=0) is True
    # gripper open (6,1500) -> lift down (5,1200) -> gripper close (6,2500) -> lift up (5,2000)
    assert board.calls == [(6, 1500), (5, 1200), (6, 2500), (5, 2000)]


def test_place_sends_lower_open_raise_in_order():
    arm_lib = _load_arm_lib()
    arm, board = _armed(arm_lib)
    assert arm.place(settle=0) is True
    assert board.calls == [(5, 1200), (6, 1500), (5, 2000)]


def test_ready_opens_gripper_and_sets_mid_lift():
    arm_lib = _load_arm_lib()
    arm, board = _armed(arm_lib)
    assert arm.ready() is True
    assert board.calls == [(6, 1500), (5, 1600)]   # _LIFT_CARRY = (1200+2000)//2


def test_self_test_reports_ok_and_exercises_every_servo():
    arm_lib = _load_arm_lib()
    arm, board = _armed(arm_lib)
    result = arm.self_test(pause=0)
    assert result["available"] is True
    assert result["ok"] is True
    assert set(result["steps"]) == {
        "gripper_open", "gripper_close", "gripper_open_2",
        "lift_up", "lift_down", "lift_mid",
    }
    assert all(result["steps"].values())
    # both servos were driven
    assert any(sid == 6 for sid, _ in board.calls)
    assert any(sid == 5 for sid, _ in board.calls)


def test_grab_and_place_do_nothing_when_unavailable():
    arm_lib = _load_arm_lib()
    arm = arm_lib.Arm()           # no board on this machine -> unavailable
    assert arm.available is False
    assert arm.grab(settle=0) is False
    assert arm.place(settle=0) is False
    assert arm.self_test(pause=0)["available"] is False


def test_namespace_sequences_degrade_safely_without_hardware():
    # Minimal stubs so student_robot_v2 imports on a dev machine.
    for name in ("ros_service_client",):
        mod = types.ModuleType(name)
        mod.clear_process_singleton = lambda *_a, **_k: None
        mod.get_process_singleton = lambda *_a, **_k: None
        mod.set_process_singleton = lambda *_a, **_k: None
        sys.modules[name] = mod
    sys.modules["robot_moves"] = types.ModuleType("robot_moves")
    sys.modules.pop("arm_lib", None)   # use the real arm_lib (unavailable here)
    if str(_LIB) not in sys.path:
        sys.path.insert(0, str(_LIB))

    spec = importlib.util.spec_from_file_location(
        "student_robot_v2_arm_seq", str(_LIB / "student_robot_v2.py")
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    robot = mod.RobotV2(verbose=False)

    assert robot.arm.available is False
    assert robot.arm.grab() is False
    assert robot.arm.place() is False
    assert robot.arm.ready() is False
    assert robot.arm.test()["available"] is False
