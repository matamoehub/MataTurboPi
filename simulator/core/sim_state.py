"""Shared state store for simulator GUI and lesson shim modules."""

from __future__ import annotations

import json
import math
import os
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, List

STATE_ENV = "MATA_SIM_STATE"
DEFAULT_STATE_FILE = "/tmp/mataturbopi_sim_state.json"


def state_path() -> Path:
    return Path(os.environ.get(STATE_ENV, DEFAULT_STATE_FILE)).expanduser().resolve()


def _default_state() -> Dict[str, Any]:
    return {
        "version": 1,
        "updated_at": time.time(),
        "robot": {
            "x": 0.0,
            "y": 0.0,
            "heading_deg": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "omega_deg_s": 0.0,
        },
        "eyes": {
            "left": [0, 0, 0],
            "right": [0, 0, 0],
        },
        "camera": {
            "yaw": 1500,
            "pitch": 1500,
            "center": 1500,
            "min_pos": 1000,
            "max_pos": 2000,
        },
        "trace": [],
        "last_command": "",
    }


def _atomic_write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix="sim_state_", suffix=".json", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=True, indent=2)
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def load_state() -> Dict[str, Any]:
    path = state_path()
    if not path.exists():
        st = _default_state()
        _atomic_write_json(path, st)
        return st

    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        data = _default_state()
        _atomic_write_json(path, data)

    merged = _default_state()
    merged.update({k: v for k, v in data.items() if k in merged and not isinstance(merged[k], dict)})
    for key in ("robot", "eyes", "camera"):
        if isinstance(data.get(key), dict):
            merged[key].update(data[key])
    if isinstance(data.get("trace"), list):
        merged["trace"] = data["trace"]
    if isinstance(data.get("last_command"), str):
        merged["last_command"] = data["last_command"]
    merged["updated_at"] = float(data.get("updated_at", merged["updated_at"]))
    return merged


def save_state(state: Dict[str, Any]) -> Dict[str, Any]:
    state["updated_at"] = time.time()
    _atomic_write_json(state_path(), state)
    return state


def reset_state() -> Dict[str, Any]:
    st = _default_state()
    save_state(st)
    return st


def append_trace(state: Dict[str, Any], label: str) -> None:
    r = state["robot"]
    item = {
        "x": float(r["x"]),
        "y": float(r["y"]),
        "heading_deg": float(r["heading_deg"]),
        "t": time.time(),
        "label": label,
    }
    trace: List[Dict[str, Any]] = state.setdefault("trace", [])
    trace.append(item)
    if len(trace) > 600:
        del trace[:-600]


def apply_robot_motion(
    state: Dict[str, Any],
    vx_m_s: float,
    vy_m_s: float,
    omega_deg_s: float,
    seconds: float,
    label: str,
) -> Dict[str, Any]:
    dt = max(0.0, float(seconds))
    r = state["robot"]

    heading = math.radians(float(r["heading_deg"]))
    c = math.cos(heading)
    s = math.sin(heading)

    # Simulator convention:
    # - vx = forward/back
    # - vy = right/left (positive = right)
    # - heading_deg = 0 means facing up on screen (+Y)
    #
    # Render math uses local X=right and local Y=forward,
    # so map (vy, vx) into (local_x, local_y) before rotating.
    wx = float(vy_m_s) * c - float(vx_m_s) * s
    wy = float(vy_m_s) * s + float(vx_m_s) * c

    r["x"] = float(r["x"]) + wx * dt
    r["y"] = float(r["y"]) + wy * dt
    r["heading_deg"] = (float(r["heading_deg"]) + float(omega_deg_s) * dt) % 360.0
    r["vx"] = float(vx_m_s)
    r["vy"] = float(vy_m_s)
    r["omega_deg_s"] = float(omega_deg_s)

    state["last_command"] = str(label)
    append_trace(state, label)
    return state
