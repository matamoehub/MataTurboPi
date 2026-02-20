"""
solutions/lib/student_robot_moves.py

Teacher reference implementation for Level 2 student library.
"""

import robot_moves as base

BASE_SPEED = 220
RATE_HZ = 30

def setup():
    base.set_base_speed(BASE_SPEED)
    base.set_rate(RATE_HZ)

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

def drift_left(seconds: float = 0.9, speed: float = None, turn_blend: float = 0.55):
    setup()
    s = float(BASE_SPEED if speed is None else speed)
    k = max(0.0, min(1.0, float(turn_blend)))

    # strafe-left
    s_fl, s_fr, s_rl, s_rr = (-1.0 * s, +1.0 * s, +1.0 * s, -1.0 * s)
    # turn-left
    t_fl, t_fr, t_rl, t_rr = (-1.0 * s, +1.0 * s, -1.0 * s, +1.0 * s)

    fl = s_fl + k * t_fl
    fr = s_fr + k * t_fr
    rl = s_rl + k * t_rl
    rr = s_rr + k * t_rr
    base._spam(fl, fr, rl, rr, seconds)

def backward(seconds: float = 0.5, speed: float = None):
    setup()
    s = float(BASE_SPEED if speed is None else speed)
    base._spam(-s, -s, -s, -s, seconds)

def move_right(seconds: float = 0.5, speed: float = None):
    setup()
    s = float(BASE_SPEED if speed is None else speed)
    base._spam(+s, -s, -s, +s, seconds)

def turn_right(seconds: float = 0.5, speed: float = None):
    setup()
    s = float(BASE_SPEED if speed is None else speed)
    base._spam(+s, -s, +s, -s, seconds)

def drift_right(seconds: float = 0.9, speed: float = None, turn_blend: float = 0.55):
    setup()
    s = float(BASE_SPEED if speed is None else speed)
    k = max(0.0, min(1.0, float(turn_blend)))

    # strafe-right
    s_fl, s_fr, s_rl, s_rr = (+1.0 * s, -1.0 * s, -1.0 * s, +1.0 * s)
    # turn-right
    t_fl, t_fr, t_rl, t_rr = (+1.0 * s, -1.0 * s, +1.0 * s, -1.0 * s)

    fl = s_fl + k * t_fl
    fr = s_fr + k * t_fr
    rl = s_rl + k * t_rl
    rr = s_rr + k * t_rr
    base._spam(fl, fr, rl, rr, seconds)

def diagonal_left(seconds: float = 0.8, speed: float = None):
    setup()
    s = float(BASE_SPEED if speed is None else speed)
    base._spam(+s, 0.0, 0.0, +s, seconds)

def diagonal_right(seconds: float = 0.8, speed: float = None):
    setup()
    s = float(BASE_SPEED if speed is None else speed)
    base._spam(0.0, +s, +s, 0.0, seconds)

# Backward compatibility aliases used by some older notebooks.
def left(seconds: float = 0.5, speed: float = None):
    return move_left(seconds=seconds, speed=speed)

def right(seconds: float = 0.5, speed: float = None):
    return move_right(seconds=seconds, speed=speed)

def spin_right(seconds: float = 0.5, speed: float = None):
    return turn_right(seconds=seconds, speed=speed)
