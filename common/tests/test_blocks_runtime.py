"""Tests for blocks_runtime.py -- the Python side of generated block programs.

Uses a fake robot object (no hardware, no student_robot_v2 import needed for
most tests) so these run on any dev machine. Run manually (no pytest in this
environment):

    python3 -c "
import importlib.util, sys
spec = importlib.util.spec_from_file_location('t', 'common/tests/test_blocks_runtime.py')
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
fns = [f for f in dir(m) if f.startswith('test_')]
passed = failed = 0
for name in fns:
    try:
        getattr(m, name)(); passed += 1; print('ok  ', name)
    except Exception as e:
        failed += 1; print('FAIL', name, '-', e)
print(f'{passed} passed, {failed} failed')
sys.exit(1 if failed else 0)
"
"""
import io
import os
import sys
import contextlib
from pathlib import Path

_LIB = Path(__file__).resolve().parents[1] / "lib"


def _load_blocks_runtime():
    if str(_LIB) not in sys.path:
        sys.path.insert(0, str(_LIB))
    sys.modules.pop("blocks_runtime", None)
    import importlib
    return importlib.import_module("blocks_runtime")


class _FakeRobot:
    """Minimal stand-in for the real myRobot object."""
    def __init__(self, sonar_value=None, direction="center", arm_raises=False):
        self.stopped = False
        self.stop_calls = 0
        self._sonar_value = sonar_value
        self._direction = direction
        self._arm_raises = arm_raises
        self.sonar = self
        self.vision = self
        self.arm = self

    def stop(self):
        self.stopped = True
        self.stop_calls += 1

    # sonar.distance_cm()
    def distance_cm(self, filtered=True):
        return self._sonar_value

    # vision.target_position(color, show=False)
    def target_position(self, color, show=True, **kwargs):
        return {"direction": self._direction, "color": color}

    # arm.grab()
    def grab(self):
        if self._arm_raises:
            raise RuntimeError("arm not available on this robot")
        return True


def _capture_stdout(fn):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        fn()
    return buf.getvalue()


def test_blk_prints_a_marker_with_the_block_id():
    # Note: str.strip() also strips \x1e (it's treated as whitespace-like),
    # so compare with rstrip("\n") only -- stripping the marker byte itself
    # would hide a real mismatch.
    br = _load_blocks_runtime()
    out = _capture_stdout(lambda: br.blk("a1b2"))
    assert out.rstrip("\n") == "\x1eBLK a1b2", repr(out)


def test_finish_stops_the_robot_and_prints_done_marker():
    br = _load_blocks_runtime()
    robot = _FakeRobot()
    out = _capture_stdout(lambda: br.finish(robot))
    assert robot.stopped is True
    assert "\x1eDONE" in out


def test_finish_stops_the_robot_before_the_settle_delay_not_after():
    """Motors must stop with no delay (safety); the settle pause -- added
    after a real pilot-robot bug where a quick eye-colour/sound command
    appeared to do nothing because the process exited before the write
    reached hardware -- comes AFTER stop(), not before it. A sequence
    check, not just a timing check: if stop() ever moved after the sleep,
    this would catch it even on a fast machine where timing alone might not."""
    br = _load_blocks_runtime()
    robot = _FakeRobot()
    call_order = []

    original_sleep = br.time.sleep
    def recording_sleep(seconds):
        call_order.append(("sleep", seconds))

    original_stop = robot.stop
    def recording_stop():
        call_order.append(("stop", None))
        original_stop()

    robot.stop = recording_stop
    br.time.sleep = recording_sleep
    try:
        br.finish(robot)
    finally:
        br.time.sleep = original_sleep

    assert [c[0] for c in call_order] == ["stop", "sleep"]


def test_finish_settle_delay_is_configurable_via_env_var():
    # This file is run with plain function calls (no pytest fixtures), so
    # the env var is set/restored by hand rather than via monkeypatch.
    os.environ["BLOCKS_EXIT_SETTLE_S"] = "0"
    try:
        br = _load_blocks_runtime()  # re-imports, so the env var is read fresh
        robot = _FakeRobot()
        slept = []
        br.time.sleep = lambda s: slept.append(s)
        br.finish(robot)
        assert slept == []  # 0 disables the delay entirely -- no sleep() call at all
    finally:
        os.environ.pop("BLOCKS_EXIT_SETTLE_S", None)


def test_finish_settle_delay_defaults_to_a_short_positive_pause():
    os.environ.pop("BLOCKS_EXIT_SETTLE_S", None)
    br = _load_blocks_runtime()
    robot = _FakeRobot()

    slept = []
    br.time.sleep = lambda s: slept.append(s)
    br.finish(robot)
    assert slept == [br._EXIT_SETTLE_S]
    assert 0 < br._EXIT_SETTLE_S <= 1.0  # short enough not to make Stop feel sluggish


def test_finish_is_safe_with_no_robot():
    br = _load_blocks_runtime()
    out = _capture_stdout(lambda: br.finish(None))
    assert "\x1eDONE" in out


def test_finish_is_safe_when_robot_stop_raises():
    br = _load_blocks_runtime()

    class _BrokenRobot:
        def stop(self):
            raise RuntimeError("serial port gone")

    out = _capture_stdout(lambda: br.finish(_BrokenRobot()))
    assert "\x1eDONE" in out  # still printed -- finish() never raises


def test_guard_runs_body_and_stops_robot_on_success():
    br = _load_blocks_runtime()
    robot = _FakeRobot()
    ran = []

    def body():
        with br.guard(robot):
            br.blk("a1")
            ran.append("a1")
            br.blk("a2")
            ran.append("a2")

    out = _capture_stdout(body)
    assert ran == ["a1", "a2"]
    assert robot.stopped is True
    assert "\x1eDONE" in out
    assert "\x1eERR" not in out


def test_guard_catches_exception_reports_last_block_and_still_stops():
    br = _load_blocks_runtime()
    robot = _FakeRobot(arm_raises=True)

    def body():
        with br.guard(robot):
            br.blk("before")
            br.blk("the_grab")
            robot.arm.grab()  # raises
            br.blk("never_reached")

    out = _capture_stdout(body)
    assert robot.stopped is True  # motors always stopped even on error
    assert "\x1eDONE" in out
    assert "\x1eERR the_grab|" in out  # tagged with the failing block, not "before"
    assert "This robot has no claw." in out


def test_guard_with_no_blk_calls_reports_empty_block_id():
    br = _load_blocks_runtime()
    robot = _FakeRobot()

    def body():
        with br.guard(robot):
            raise ValueError("boom, no blk() was ever called")

    out = _capture_stdout(body)
    assert "\x1eERR |" in out  # block id is empty, not a stale id from a previous run
    assert robot.stopped is True


def test_distance_cm_or_returns_default_when_sonar_sees_nothing():
    br = _load_blocks_runtime()
    robot = _FakeRobot(sonar_value=None)
    assert br.distance_cm_or(robot, default=999) == 999


def test_distance_cm_or_returns_real_reading_when_present():
    br = _load_blocks_runtime()
    robot = _FakeRobot(sonar_value=15.5)
    assert br.distance_cm_or(robot, default=999) == 15.5


def test_distance_cm_or_returns_default_on_sensor_exception():
    br = _load_blocks_runtime()

    class _Angry:
        def distance_cm(self, filtered=True):
            raise RuntimeError("i2c bus error")

    robot = _FakeRobot()
    robot.sonar = _Angry()
    assert br.distance_cm_or(robot, default=777) == 777


def test_ball_direction_is_true_when_direction_matches():
    br = _load_blocks_runtime()
    robot = _FakeRobot(direction="left")
    assert br.ball_direction_is(robot, "red", "left") is True
    assert br.ball_direction_is(robot, "red", "right") is False


def test_ball_direction_is_treats_vision_failure_as_lost():
    br = _load_blocks_runtime()

    class _Angry:
        def target_position(self, color, show=True, **kwargs):
            raise RuntimeError("camera not available")

    robot = _FakeRobot()
    robot.vision = _Angry()
    assert br.ball_direction_is(robot, "red", "lost") is True
    assert br.ball_direction_is(robot, "red", "left") is False


def test_friendly_message_maps_known_failure_patterns():
    br = _load_blocks_runtime()
    assert br._friendly_message(RuntimeError("arm not available")) == "This robot has no claw."
    assert "lost connection" in br._friendly_message(TimeoutError("timed out"))
    assert "see well enough" in br._friendly_message(RuntimeError("vision backend down"))
    # unrecognised error still produces readable, non-empty text
    msg = br._friendly_message(ValueError("some other problem"))
    assert "some other problem" in msg


def test_last_block_id_resets_correctly_across_two_guard_calls_in_one_process():
    # A regression guard for a real bug class: module-level state from one
    # run leaking into a later run in the same process (shouldn't happen --
    # each run is its own subprocess -- but the module state itself must
    # still behave correctly if that assumption is ever violated).
    br = _load_blocks_runtime()
    robot = _FakeRobot()

    def body1():
        with br.guard(robot):
            br.blk("run1_block")
            raise RuntimeError("first run fails")

    out1 = _capture_stdout(body1)
    assert "\x1eERR run1_block|" in out1

    def body2():
        with br.guard(robot):
            br.blk("run2_block")
            raise RuntimeError("second run fails")

    out2 = _capture_stdout(body2)
    assert "\x1eERR run2_block|" in out2  # not run1_block
