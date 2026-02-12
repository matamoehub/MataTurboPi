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

def forward(seconds: float):
    setup()
    base.forward(seconds)

def turn_left(seconds: float):
    setup()
    base.turn_left(seconds)

def drift_left(seconds: float):
    setup()
    base.drift_left(seconds)

def backward(seconds: float):
    setup()
    base.backward(seconds)

def move_right(seconds: float):
    setup()
    base.right(seconds)

def spin_right(seconds: float):
    setup()
    base.turn_right(seconds)
