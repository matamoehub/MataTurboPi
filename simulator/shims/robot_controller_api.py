"""Drop-in simulator shim for common/lib/robot_controller_api.py."""

from __future__ import annotations

from simulator.core.sim_state import load_state, save_state


def enable_motors(on: bool = True, settle_s: float = 0.05):
    st = load_state()
    st["motors_enabled"] = bool(on)
    st["last_command"] = "enable_motors" if on else "disable_motors"
    save_state(st)


def send_speeds(pairs):
    st = load_state()
    st["last_motor_pairs"] = [(int(i), float(s)) for (i, s) in pairs]
    st["last_command"] = "send_speeds"
    save_state(st)


def stream_speeds(pairs, seconds: float = 0.5, rate_hz: float = 20.0):
    send_speeds(pairs)


def all_stop(ids=(1, 2, 3, 4)):
    st = load_state()
    st["last_motor_pairs"] = [(int(i), 0.0) for i in ids]
    st["last_command"] = "all_stop"
    st["robot"]["vx"] = 0.0
    st["robot"]["vy"] = 0.0
    st["robot"]["omega_deg_s"] = 0.0
    save_state(st)
