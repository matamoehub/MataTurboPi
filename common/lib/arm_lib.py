#!/usr/bin/env python3
"""arm_lib.py
Direct (non-ROS) control for the optional TurboPi robotic-arm attachment.

Some TurboPi units ship with a robotic-arm + gripper attachment (lift servo
on PWM channel 5, gripper servo on channel 6 — see Hiwonder's install docs:
wiki.hiwonder.com/projects/TurboPi -> 3.7/3.8 Robotic Arm Installation/Control).
Most don't. This module auto-detects whether the arm's control SDK is
actually importable and usable on THIS robot, and degrades to a safe no-op
otherwise — every public function returns False instead of raising when the
arm isn't present, so calling them is always safe regardless of hardware.

Talks directly to Hiwonder's ros_robot_controller_sdk.Board — the same
direct-serial path (no ROS2 involved) already used for the arm on SpiderPi.
The ROS2 /servo_controller topic path was tried first on real hardware and
silently did nothing; this direct path is field-confirmed working.

Known per-servo safe ranges (from the vendor's servo.yaml on tested hardware):
    lift (channel 5):    1200-2000
    gripper (channel 6):  1500-2500  (1500=open, 2500=closed)
Override via ARM_LIFT_SERVO_ID / ARM_GRIPPER_SERVO_ID env vars if a given
robot's arm is wired to different channels.

Examples:
    import arm_lib
    arm = arm_lib.get_arm()
    if arm.available:
        arm.open_gripper()
        arm.close_gripper()
        arm.lift_up()
"""
__version__ = "1.0.0"

import os
import sys
import threading
from typing import Optional

_LIFT_ID = int(os.environ.get("ARM_LIFT_SERVO_ID", "5"))
_GRIPPER_ID = int(os.environ.get("ARM_GRIPPER_SERVO_ID", "6"))

_LIFT_MIN, _LIFT_MAX = 1200, 2000
_GRIPPER_OPEN, _GRIPPER_CLOSED = 1500, 2500
_GRIPPER_MIN, _GRIPPER_MAX = 1500, 2500
_DEFAULT_DURATION_S = float(os.environ.get("ARM_MOVE_DURATION_S", "0.5"))

# Known locations for the vendor's direct-serial SDK across different TurboPi
# image versions — searched in order, first importable one wins. Varies by
# image because this hasn't been vendored into common/lib (yet) the way
# sonar_lib/eyes_lib are; it's whatever ships with the robot's own image.
_SDK_SEARCH_PATHS = [
    "/home/ubuntu/TurboPi/HiwonderSDK",
    "/home/ubuntu/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller",
    "/home/ubuntu/RRCLite_demo",
    "/home/ubuntu/software/pwm_servo_control",
    "/home/pi/TurboPi/HiwonderSDK",
]


class Arm:
    """Lazy-connecting wrapper around ros_robot_controller_sdk.Board.

    Construction never raises — connection is attempted once and any
    failure (SDK not found, board not reachable, serial port busy) just
    leaves `available` False. Call sites don't need their own try/except.
    """

    def __init__(self):
        self._board = None
        self._lock = threading.Lock()
        self._error: Optional[str] = None
        self._connect()

    def _connect(self):
        for path in _SDK_SEARCH_PATHS:
            if os.path.isdir(path) and path not in sys.path:
                sys.path.append(path)
        try:
            import ros_robot_controller_sdk as rrc  # type: ignore
            self._board = rrc.Board()
        except Exception as e:
            self._board = None
            self._error = str(e)

    @property
    def available(self) -> bool:
        return self._board is not None

    def error(self) -> Optional[str]:
        """Why the arm is unavailable, or None if it's working."""
        return self._error

    def _set(self, servo_id: int, position: int, duration: float) -> bool:
        if not self.available:
            return False
        try:
            with self._lock:
                self._board.pwm_servo_set_position(duration, [[servo_id, int(position)]])
            return True
        except Exception as e:
            self._error = str(e)
            self._board = None  # stop retrying a board that just failed
            return False

    # ── Gripper (channel 6) ──────────────────────────────────────────────
    def open_gripper(self, duration: float = _DEFAULT_DURATION_S) -> bool:
        return self._set(_GRIPPER_ID, _GRIPPER_OPEN, duration)

    def close_gripper(self, duration: float = _DEFAULT_DURATION_S) -> bool:
        return self._set(_GRIPPER_ID, _GRIPPER_CLOSED, duration)

    def set_gripper(self, position: int, duration: float = _DEFAULT_DURATION_S) -> bool:
        """position: raw pulse width, clamped to the gripper's safe range (1500-2500)."""
        position = max(_GRIPPER_MIN, min(_GRIPPER_MAX, int(position)))
        return self._set(_GRIPPER_ID, position, duration)

    # ── Lift (channel 5) ─────────────────────────────────────────────────
    def lift_up(self, duration: float = _DEFAULT_DURATION_S) -> bool:
        return self._set(_LIFT_ID, _LIFT_MAX, duration)

    def lift_down(self, duration: float = _DEFAULT_DURATION_S) -> bool:
        return self._set(_LIFT_ID, _LIFT_MIN, duration)

    def set_lift(self, position: int, duration: float = _DEFAULT_DURATION_S) -> bool:
        """position: raw pulse width, clamped to the lift's safe range (1200-2000)."""
        position = max(_LIFT_MIN, min(_LIFT_MAX, int(position)))
        return self._set(_LIFT_ID, position, duration)

    # ── Raw escape hatch ─────────────────────────────────────────────────
    def set_position(self, servo_id: int, position: int, duration: float = _DEFAULT_DURATION_S) -> bool:
        """Move any arm servo channel directly, no range clamping."""
        return self._set(servo_id, position, duration)


_arm_singleton: Optional[Arm] = None
_singleton_lock = threading.Lock()


def get_arm() -> Arm:
    """Return the shared Arm instance, connecting on first call."""
    global _arm_singleton
    if _arm_singleton is None:
        with _singleton_lock:
            if _arm_singleton is None:
                _arm_singleton = Arm()
    return _arm_singleton


def is_available() -> bool:
    """True if this robot has a working arm attachment."""
    return get_arm().available
