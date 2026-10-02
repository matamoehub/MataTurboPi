import importlib.util
import sys
import types
from pathlib import Path


def _install_support_modules(arm_available: bool = True):
    svc_mod = types.ModuleType("ros_service_client")
    svc_mod.clear_process_singleton = lambda *_a, **_k: None
    svc_mod.get_process_singleton = lambda *_a, **_k: None
    svc_mod.set_process_singleton = lambda *_a, **_k: None
    sys.modules["ros_service_client"] = svc_mod

    moves_mod = types.ModuleType("robot_moves")
    moves_mod.forward = lambda **_k: None
    sys.modules["robot_moves"] = moves_mod

    arm_mod = types.ModuleType("arm_lib")

    class _FakeArm:
        def __init__(self):
            self.available = arm_available
            self.calls = []

        def open_gripper(self, duration=0.5):
            self.calls.append(("open_gripper", duration))
            return self.available

        def close_gripper(self, duration=0.5):
            self.calls.append(("close_gripper", duration))
            return self.available

        def set_gripper(self, position, duration=0.5):
            self.calls.append(("set_gripper", position, duration))
            return self.available

        def lift_up(self, duration=0.5):
            self.calls.append(("lift_up", duration))
            return self.available

        def lift_down(self, duration=0.5):
            self.calls.append(("lift_down", duration))
            return self.available

        def set_lift(self, position, duration=0.5):
            self.calls.append(("set_lift", position, duration))
            return self.available

        def set_position(self, servo_id, position, duration=0.5):
            self.calls.append(("set_position", servo_id, position, duration))
            return self.available

    arm_mod._backend = None

    def _get_arm():
        if arm_mod._backend is None:
            arm_mod._backend = _FakeArm()
        return arm_mod._backend

    arm_mod.get_arm = _get_arm
    sys.modules["arm_lib"] = arm_mod
    return arm_mod


def _load_student_robot_v2():
    path = Path(__file__).resolve().parents[1] / "lib" / "student_robot_v2.py"
    spec = importlib.util.spec_from_file_location("student_robot_v2_for_test_arm", str(path))
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_arm_available_dispatches_to_backend():
    arm_mod = _install_support_modules(arm_available=True)
    student_robot_v2 = _load_student_robot_v2()

    robot = student_robot_v2.RobotV2(verbose=False)

    assert robot.arm.available is True
    assert robot.arm.open_gripper() is True
    assert robot.arm.close_gripper() is True
    assert robot.arm.set_gripper(2000) is True
    assert robot.arm.lift_up() is True
    assert robot.arm.lift_down() is True
    assert robot.arm.set_lift(1600) is True

    backend = arm_mod._backend
    assert backend.calls == [
        ("open_gripper", 0.5),
        ("close_gripper", 0.5),
        ("set_gripper", 2000, 0.5),
        ("lift_up", 0.5),
        ("lift_down", 0.5),
        ("set_lift", 1600, 0.5),
    ]


def test_gripper_alias_is_same_namespace_as_arm():
    _install_support_modules(arm_available=True)
    student_robot_v2 = _load_student_robot_v2()
    robot = student_robot_v2.RobotV2(verbose=False)
    assert robot.gripper is robot.arm


def test_arm_unavailable_returns_false_without_raising():
    _install_support_modules(arm_available=False)
    student_robot_v2 = _load_student_robot_v2()
    robot = student_robot_v2.RobotV2(verbose=False)

    assert robot.arm.available is False
    # Every command must return False, not raise — safe to call unconditionally
    # whether or not this particular robot has the arm attachment.
    assert robot.arm.open_gripper() is False
    assert robot.arm.close_gripper() is False
    assert robot.arm.set_gripper(2000) is False
    assert robot.arm.lift_up() is False
    assert robot.arm.lift_down() is False
    assert robot.arm.set_lift(1600) is False
    assert robot.arm.set_position(5, 1500) is False


def test_real_arm_lib_degrades_safely_without_hiwonder_sdk():
    """End-to-end with the REAL arm_lib.py (not stubbed): on a machine with
    no ros_robot_controller_sdk / no arm hardware (like this dev machine),
    RobotV2 construction must still succeed and every arm command must
    return False rather than raising."""
    svc_mod = types.ModuleType("ros_service_client")
    svc_mod.clear_process_singleton = lambda *_a, **_k: None
    svc_mod.get_process_singleton = lambda *_a, **_k: None
    svc_mod.set_process_singleton = lambda *_a, **_k: None
    sys.modules["ros_service_client"] = svc_mod
    moves_mod = types.ModuleType("robot_moves")
    sys.modules["robot_moves"] = moves_mod
    sys.modules.pop("arm_lib", None)

    lib_dir = str(Path(__file__).resolve().parents[1] / "lib")
    if lib_dir not in sys.path:
        sys.path.insert(0, lib_dir)

    student_robot_v2 = _load_student_robot_v2()
    robot = student_robot_v2.RobotV2(verbose=False)

    assert robot.arm.available is False
    assert robot.arm.open_gripper() is False
