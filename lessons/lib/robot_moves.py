# lessons/lib/robot_moves.py
# Student-owned movement library (edit this file).
# Uses the proven base motor code from common/lib/robot_moves.py

from __future__ import annotations
import time

import robot_moves as base  # <-- this will resolve to common/lib/robot_moves.py once sys.path is set

class RobotMoves:
    PAUSE_S = 0.20

    def __init__(self, base_speed: float = 220.0, rate_hz: float = 30.0):
        base.set_base_speed(base_speed)
        base.set_rate(rate_hz)

    def stop(self):
        base.stop()

    def horn(self):
        return base.horn(block=True)

    # --- examples they can copy ---
    def forward(self, seconds: float = 0.6):
        base.forward(seconds); base.stop(); time.sleep(self.PAUSE_S)

    def move_left(self, seconds: float = 0.6):
        base.left(seconds); base.stop(); time.sleep(self.PAUSE_S)

    def spin_left(self, seconds: float = 0.5):
        base.turn_left(seconds); base.stop(); time.sleep(self.PAUSE_S)

    # --- TODO: students complete these ---
    def backward(self, seconds: float = 0.6):
        """TODO: drive backwards for seconds (hint: base.backward(seconds))"""
        raise NotImplementedError("TODO: implement backward() in lessons/lib/robot_moves.py")

    def move_right(self, seconds: float = 0.6):
        """TODO: strafe right (hint: base.right(seconds))"""
        raise NotImplementedError("TODO: implement move_right() in lessons/lib/robot_moves.py")

    def spin_right(self, seconds: float = 0.5):
        """TODO: spin right (hint: base.turn_right(seconds))"""
        raise NotImplementedError("TODO: implement spin_right() in lessons/lib/robot_moves.py")
