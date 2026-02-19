#!/usr/bin/env python3
"""
line_follower_lib.py
Simple PID line following built on Infrared + robot_moves.
"""

import time
from dataclasses import dataclass
from typing import List, Optional

from infrared_lib import Infrared, get_infrared


@dataclass
class PIDConfig:
    kp: float = 25.0
    ki: float = 0.0
    kd: float = 4.0
    integral_limit: float = 100.0


class LineFollower:
    """
    Reads 4 IR sensors and commands robot movement with a PID steering correction.

    Expected sensor bit order: [s0, s1, s2, s3]
    Default weights assume center is between sensors 1 and 2.
    """

    def __init__(
        self,
        infrared: Optional[Infrared] = None,
        base_speed: float = 220.0,
        weights: Optional[List[float]] = None,
        pid: Optional[PIDConfig] = None,
        max_turn: float = 0.8,
    ):
        self.ir = infrared if infrared is not None else get_infrared()
        self.base_speed = float(base_speed)
        self.weights = weights if weights is not None else [-3.0, -1.0, 1.0, 3.0]
        self.pid = pid if pid is not None else PIDConfig()
        self.max_turn = float(max_turn)

        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = None

        import robot_moves as rm

        self.rm = rm

    def _calc_error(self, states: List[bool]) -> float:
        if len(states) != len(self.weights):
            raise ValueError(f"Expected {len(self.weights)} sensor states, got {len(states)}")
        active = [w for w, s in zip(self.weights, states) if bool(s)]
        if not active:
            return self._prev_error
        return float(sum(active) / len(active))

    def _pid_turn(self, error: float) -> float:
        now = time.time()
        dt = 0.02 if self._last_time is None else max(1e-3, now - self._last_time)
        self._last_time = now

        self._integral += error * dt
        lim = abs(float(self.pid.integral_limit))
        self._integral = max(-lim, min(lim, self._integral))

        deriv = (error - self._prev_error) / dt
        self._prev_error = error

        u = self.pid.kp * error + self.pid.ki * self._integral + self.pid.kd * deriv
        # Normalize to [-max_turn, +max_turn]
        # Using a conservative divisor makes tuning predictable in notebooks.
        turn = u / 200.0
        return max(-self.max_turn, min(self.max_turn, turn))

    def step(self, seconds: float = 0.05, speed: Optional[float] = None) -> dict:
        """
        One control step:
        - read sensors
        - compute PID turn
        - command movement
        Returns debug info for logging/notebooks.
        """
        states = self.ir.read()
        error = self._calc_error(states)
        turn = self._pid_turn(error)
        v = self.base_speed if speed is None else float(speed)

        # drive_for(vx, vy, seconds, speed): vx forward component, vy strafe component.
        self.rm.drive_for(vx=1.0, vy=turn, seconds=float(seconds), speed=v)
        return {"states": states, "error": error, "turn": turn, "speed": v}

    def follow_for(self, duration_s: float = 3.0, step_s: float = 0.05, speed: Optional[float] = None) -> None:
        end = time.time() + float(duration_s)
        try:
            while time.time() < end:
                self.step(seconds=step_s, speed=speed)
        finally:
            self.rm.stop()

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = None

