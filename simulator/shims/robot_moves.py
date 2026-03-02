#!/usr/bin/env python3
"""Drop-in simulator shim for common/lib/robot_moves.py."""

from __future__ import annotations

import os
from typing import Optional

from simulator.core.sim_state import apply_robot_motion, load_state, reset_state, save_state

BASE_SPEED: float = float(os.environ.get("BASE_SPEED", "300"))
RATE_HZ: float = float(os.environ.get("RATE_HZ", "20"))

SPEED_TO_M_S = 1.0 / 380.0
TURN_SPEED_TO_DEG_S = 95.0 / 300.0


def _vel(speed: Optional[float]) -> float:
    s = float(BASE_SPEED if speed is None else speed)
    return s * SPEED_TO_M_S


def set_base_speed(speed: float):
    global BASE_SPEED
    BASE_SPEED = float(speed)


def set_rate(hz: float):
    global RATE_HZ
    RATE_HZ = float(hz)


def _move(vx: float, vy: float, omega: float, seconds: float, label: str):
    st = load_state()
    apply_robot_motion(st, vx, vy, omega, seconds, label)
    save_state(st)


def stop():
    st = load_state()
    st["robot"]["vx"] = 0.0
    st["robot"]["vy"] = 0.0
    st["robot"]["omega_deg_s"] = 0.0
    st["last_command"] = "stop"
    save_state(st)


def emergency_stop():
    stop()


def forward(seconds: float = 0.5, speed: float = None):
    v = _vel(speed)
    _move(vx=v, vy=0.0, omega=0.0, seconds=seconds, label="forward")


def backward(seconds: float = 0.5, speed: float = None):
    v = _vel(speed)
    _move(vx=-v, vy=0.0, omega=0.0, seconds=seconds, label="backward")


def left(seconds: float = 0.5, speed: float = None):
    v = _vel(speed)
    _move(vx=0.0, vy=-v, omega=0.0, seconds=seconds, label="left")


def right(seconds: float = 0.5, speed: float = None):
    v = _vel(speed)
    _move(vx=0.0, vy=v, omega=0.0, seconds=seconds, label="right")


def turn_left(seconds: float = 0.5, speed: float = None):
    s = float(BASE_SPEED if speed is None else speed)
    _move(vx=0.0, vy=0.0, omega=s * TURN_SPEED_TO_DEG_S, seconds=seconds, label="turn_left")


def turn_right(seconds: float = 0.5, speed: float = None):
    s = float(BASE_SPEED if speed is None else speed)
    _move(vx=0.0, vy=0.0, omega=-s * TURN_SPEED_TO_DEG_S, seconds=seconds, label="turn_right")


def diagonal_left(seconds: float = 0.8, speed: float = None):
    v = _vel(speed)
    _move(vx=v * 0.7, vy=-v * 0.7, omega=0.0, seconds=seconds, label="diagonal_left")


def diagonal_right(seconds: float = 0.8, speed: float = None):
    v = _vel(speed)
    _move(vx=v * 0.7, vy=v * 0.7, omega=0.0, seconds=seconds, label="diagonal_right")


def drive_for(vx: float, vy: float, seconds: float, speed: float = None):
    s = float(BASE_SPEED if speed is None else speed)
    scale = s * SPEED_TO_M_S
    _move(vx=float(vx) * scale, vy=float(vy) * scale, omega=0.0, seconds=seconds, label="drive_for")


def drift_left(seconds: float = None, speed: float = None, turn_blend: float = None):
    dur = 1.0 if seconds is None else float(seconds)
    s = float(BASE_SPEED if speed is None else speed)
    k = 0.55 if turn_blend is None else max(0.0, min(1.0, float(turn_blend)))
    v = s * SPEED_TO_M_S
    _move(vx=0.05 * v, vy=-v, omega=s * TURN_SPEED_TO_DEG_S * k * 0.8, seconds=dur, label="drift_left")


def drift_right(seconds: float = None, speed: float = None, turn_blend: float = None):
    dur = 1.0 if seconds is None else float(seconds)
    s = float(BASE_SPEED if speed is None else speed)
    k = 0.55 if turn_blend is None else max(0.0, min(1.0, float(turn_blend)))
    v = s * SPEED_TO_M_S
    _move(vx=0.05 * v, vy=v, omega=-s * TURN_SPEED_TO_DEG_S * k * 0.8, seconds=dur, label="drift_right")


def horn(*_args, **_kwargs) -> bool:
    st = load_state()
    st["last_command"] = "horn"
    save_state(st)
    return True


def Horn():
    return horn()


def reset_simulator_state():
    reset_state()


def _spam(fl: float, fr: float, rl: float, rr: float, seconds: float):
    avg = (float(fl) + float(fr) + float(rl) + float(rr)) / 4.0
    strafe = (float(fl) - float(fr) - float(rl) + float(rr)) / 4.0
    turn = (-float(fl) + float(fr) - float(rl) + float(rr)) / 4.0

    v_forward = avg * SPEED_TO_M_S
    v_strafe = strafe * SPEED_TO_M_S
    omega = turn * TURN_SPEED_TO_DEG_S * 1.1

    _move(vx=v_forward, vy=v_strafe, omega=omega, seconds=seconds, label="wheel_mix")


class RobotMoves:
    def __init__(self, base_speed: float = None, rate_hz: float = None):
        if base_speed is not None:
            set_base_speed(base_speed)
        if rate_hz is not None:
            set_rate(rate_hz)

    def stop(self):
        stop()

    def emergency_stop(self):
        emergency_stop()

    def forward(self, seconds: float = 0.5, speed: float = None):
        forward(seconds, speed)

    def backward(self, seconds: float = 0.5, speed: float = None):
        backward(seconds, speed)

    def left(self, seconds: float = 0.5, speed: float = None):
        left(seconds, speed)

    def right(self, seconds: float = 0.5, speed: float = None):
        right(seconds, speed)

    def turn_left(self, seconds: float = 0.5, speed: float = None):
        turn_left(seconds, speed)

    def turn_right(self, seconds: float = 0.5, speed: float = None):
        turn_right(seconds, speed)

    def diagonal_left(self, seconds: float = 0.8, speed: float = None):
        diagonal_left(seconds, speed)

    def diagonal_right(self, seconds: float = 0.8, speed: float = None):
        diagonal_right(seconds, speed)

    def drift_left(self, seconds: float = None, speed: float = None, turn_blend: float = None):
        drift_left(seconds=seconds, speed=speed, turn_blend=turn_blend)

    def drift_right(self, seconds: float = None, speed: float = None, turn_blend: float = None):
        drift_right(seconds=seconds, speed=speed, turn_blend=turn_blend)

    def drive_for(self, vx: float, vy: float, seconds: float, speed: float = None):
        drive_for(vx=vx, vy=vy, seconds=seconds, speed=speed)

    def horn(self, *a, **k):
        return horn(*a, **k)
