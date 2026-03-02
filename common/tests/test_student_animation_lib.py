import importlib.util
from pathlib import Path


def _load_student_animation_lib():
    p = Path(__file__).resolve().parents[2] / "lessons" / "lib" / "student_animation_lib.py"
    spec = importlib.util.spec_from_file_location("student_animation_lib_for_test", str(p))
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_fidget_plan_deterministic():
    al = _load_student_animation_lib()
    a = al.build_fidget_plan(1.0, step_s=0.05, seed=9)
    b = al.build_fidget_plan(1.0, step_s=0.05, seed=9)
    assert a == b
    assert len(a) == 20


def test_singleton_and_reset():
    al = _load_student_animation_lib()

    class FakeRobot:
        def forward(self, seconds=0.0, speed=None, **kwargs):
            return (seconds, speed, kwargs)

    x = al.get_animation_lib(robot=FakeRobot(), eyes=None, camera=None, tts=None, base_speed=300)
    y = al.get_animation_lib(base_speed=200)
    assert id(x) == id(y)
    assert y.base_speed == 200

    al.reset_animation_lib()
    z = al.get_animation_lib(robot=FakeRobot(), eyes=None, camera=None, tts=None, base_speed=300)
    assert id(z) != id(y)
