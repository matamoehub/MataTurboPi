import importlib.util
import sys
import types
from pathlib import Path


def _load_line_follower_lib():
    # Stub infrared_lib
    infrared_mod = types.ModuleType("infrared_lib")

    class _Infrared:
        def __init__(self, states=None):
            self._states = states or [False, True, True, False]

        def read(self):
            return list(self._states)

    def _get_infrared():
        return _Infrared()

    infrared_mod.Infrared = _Infrared
    infrared_mod.get_infrared = _get_infrared
    sys.modules["infrared_lib"] = infrared_mod

    # Stub robot_moves
    rm_mod = types.ModuleType("robot_moves")
    rm_mod.calls = []

    def _drive_for(vx, vy, seconds, speed):
        rm_mod.calls.append(("drive_for", vx, vy, seconds, speed))

    def _stop():
        rm_mod.calls.append(("stop",))

    rm_mod.drive_for = _drive_for
    rm_mod.stop = _stop
    sys.modules["robot_moves"] = rm_mod

    p = Path(__file__).resolve().parents[1] / "lib" / "line_follower_lib.py"
    spec = importlib.util.spec_from_file_location("line_follower_lib_for_test", str(p))
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module, rm_mod


def test_calc_error_centered():
    line_follower_lib, _rm = _load_line_follower_lib()
    lf = line_follower_lib.LineFollower()
    # sensors 1 and 2 active -> average of weights -1 and +1 = 0
    assert lf._calc_error([False, True, True, False]) == 0.0


def test_step_drives_forward_with_turn():
    line_follower_lib, rm = _load_line_follower_lib()
    # Right-most sensor active -> positive error -> positive turn.
    ir = line_follower_lib.Infrared(states=[False, False, False, True])
    lf = line_follower_lib.LineFollower(infrared=ir, base_speed=200)
    out = lf.step(seconds=0.05)
    assert out["turn"] > 0
    assert rm.calls and rm.calls[0][0] == "drive_for"
    _, vx, vy, seconds, speed = rm.calls[0]
    assert vx == 1.0
    assert seconds == 0.05
    assert speed == 200
    assert vy == out["turn"]


def test_follow_for_stops_robot():
    line_follower_lib, rm = _load_line_follower_lib()
    lf = line_follower_lib.LineFollower()
    lf.follow_for(duration_s=0.01, step_s=0.01)
    assert any(call[0] == "stop" for call in rm.calls)

