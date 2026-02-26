"""Lesson 1 Level 1 notebook header.
Use in any cell: from lesson_header import *
"""

import time

from lesson_loader import setup as _setup

_setup(verbose=False)

import robot_moves as rm

if "bot" not in globals():
    bot = rm.RobotMoves(base_speed=220, rate_hz=30)
    try:
        bot.stop()
        time.sleep(0.1)
    except Exception:
        pass
