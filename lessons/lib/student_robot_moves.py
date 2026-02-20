"""
student_robot_moves.py

Level 2 task:
- You are building your own movement library that is reused in later lessons.
- Keep the provided functions as examples.
- Implement the TODO functions using wheel-level patterns.
"""

import robot_moves as base

# Students should use these settings (teachers can tune)
BASE_SPEED = 220
RATE_HZ = 30

def setup():
    base.set_base_speed(BASE_SPEED)
    base.set_rate(RATE_HZ)

# Wheel order used by _spam(fl, fr, rl, rr, seconds):
#   fl = front-left, fr = front-right, rl = rear-left, rr = rear-right

# ✅ Provided examples (students can look at these)
def forward(seconds: float = 0.5, speed: float = None):
    setup()
    s = float(BASE_SPEED if speed is None else speed)
    base._spam(+s, +s, +s, +s, seconds)

def move_left(seconds: float = 0.5, speed: float = None):
    setup()
    s = float(BASE_SPEED if speed is None else speed)
    base._spam(-s, +s, +s, -s, seconds)

def turn_left(seconds: float = 0.5, speed: float = None):
    setup()
    s = float(BASE_SPEED if speed is None else speed)
    base._spam(-s, +s, -s, +s, seconds)

# ---------------------------
# TODO (students must complete)
# ---------------------------

def backward(seconds: float = 0.5, speed: float = None):
    """
    Move backward for `seconds`.
    Replace the pseudo-code with your wheel pattern.
    """
    # setup()
    # s = float(BASE_SPEED if speed is None else speed)
    # base._spam(?, ?, ?, ?, seconds)
    raise NotImplementedError("TODO: implement backward(seconds, speed)")

def move_right(seconds: float = 0.5, speed: float = None):
    """
    Strafe right for `seconds`.
    Replace the pseudo-code with your wheel pattern.
    """
    # setup()
    # s = float(BASE_SPEED if speed is None else speed)
    # base._spam(?, ?, ?, ?, seconds)
    raise NotImplementedError("TODO: implement move_right(seconds, speed)")

def turn_right(seconds: float = 0.5, speed: float = None):
    """
    Spin/turn right for `seconds`.
    Replace the pseudo-code with your wheel pattern.
    """
    # setup()
    # s = float(BASE_SPEED if speed is None else speed)
    # base._spam(?, ?, ?, ?, seconds)
    raise NotImplementedError("TODO: implement turn_right(seconds, speed)")

def drift_right(seconds: float = 0.9, speed: float = None, turn_blend: float = 0.55):
    """
    Challenge extension:
    Drift right in an arc using two ideas:
    1) strafe-right pattern
    2) turn-right pattern
    Mix them with turn_blend (0.0..1.0), then send one _spam command.
    """
    # setup()
    # s = float(BASE_SPEED if speed is None else speed)
    # k = max(0.0, min(1.0, float(turn_blend)))
    #
    # Step 1: write strafe-right wheel values (fl, fr, rl, rr)
    # Step 2: write turn-right wheel values  (fl, fr, rl, rr)
    # Step 3: combine each wheel: wheel = strafe + k * turn
    # Step 4: base._spam(fl, fr, rl, rr, seconds)
    raise NotImplementedError("Challenge: implement drift_right(seconds, speed, turn_blend)")

# Backward compatibility aliases used by some older notebooks.
def left(seconds: float = 0.5, speed: float = None):
    return move_left(seconds=seconds, speed=speed)

def right(seconds: float = 0.5, speed: float = None):
    return move_right(seconds=seconds, speed=speed)

def spin_right(seconds: float = 0.5, speed: float = None):
    return turn_right(seconds=seconds, speed=speed)
