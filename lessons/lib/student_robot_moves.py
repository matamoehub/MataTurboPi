"""
student_robot_moves.py

Students complete the missing moves here.
This gets reused in later lessons: you add new helper functions over time.
"""

import robot_moves as base

# Students should use these settings (teachers can tune)
BASE_SPEED = 220
RATE_HZ = 30

def setup():
    base.set_base_speed(BASE_SPEED)
    base.set_rate(RATE_HZ)

# ✅ Example functions already done (students can look at these)
def forward(seconds: float):
    setup()
    base.forward(seconds)

def turn_left(seconds: float):
    setup()
    base.turn_left(seconds)

def drift_left(seconds: float, turn_blend: float = 0.55):
    setup()
    base.drift_left(seconds=seconds, turn_blend=turn_blend)

# ---------------------------
# TODO (students must complete)
# ---------------------------

def backward(seconds: float):
    """
    Move backward for `seconds`.
    """
    raise NotImplementedError("TODO: implement backward(seconds)")

def move_right(seconds: float):
    """
    Strafe right for `seconds`.
    """
    raise NotImplementedError("TODO: implement move_right(seconds)")

def spin_right(seconds: float):
    """
    Spin/turn right for `seconds`.
    """
    raise NotImplementedError("TODO: implement spin_right(seconds)")
