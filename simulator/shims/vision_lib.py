#!/usr/bin/env python3
"""Simulator vision shim for notebook colour-cup workflows."""

from __future__ import annotations

import copy
import math
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from simulator.core.sim_state import load_state

__version__ = "1.0.0-sim"

HSVRange = Tuple[Tuple[int, int, int], Tuple[int, int, int]]

DEFAULT_COLOR_PROFILES: Dict[str, List[HSVRange]] = {
    "red": [((0, 120, 50), (10, 255, 255)), ((170, 120, 50), (179, 255, 255))],
    "green": [((35, 80, 40), (85, 255, 255))],
    "blue": [((90, 80, 40), (135, 255, 255))],
}

SIM_COLOR_HEX = {
    "red": "#d94a4a",
    "green": "#3ba55d",
    "blue": "#3f7dd9",
    "unknown": "#8a94a6",
}


def _normalize_color_name(color: str) -> str:
    value = str(color or "").strip().lower()
    aliases = {"r": "red", "g": "green", "b": "blue"}
    return aliases.get(value, value)


def _display_svg(svg_text: str) -> bool:
    try:  # pragma: no cover - notebook-only behavior
        from IPython.display import SVG, display
    except Exception:
        return False
    display(SVG(data=svg_text))
    return True


def _write_svg(svg_text: str, save_path: Optional[str] = None) -> str:
    target = Path(save_path) if save_path else Path(tempfile.gettempdir()) / f"vision_sim_{int(time.time() * 1000)}.svg"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(svg_text, encoding="utf-8")
    return str(target)


def _obstacle_color_name(obs: Dict[str, Any]) -> str:
    return _normalize_color_name(obs.get("color_name") or obs.get("color") or "blue")


def _camera_yaw_offset_deg(state: Dict[str, Any]) -> float:
    camera = state.get("camera", {})
    yaw = float(camera.get("yaw", 1500))
    center = float(camera.get("center", 1500))
    return ((yaw - center) / 500.0) * 45.0


def _visible_obstacles(state: Dict[str, Any], fov_deg: float = 70.0, max_range_m: float = 1.6) -> List[Dict[str, Any]]:
    robot = state.get("robot", {})
    rx = float(robot.get("x", 0.0))
    ry = float(robot.get("y", 0.0))
    heading_deg = float(robot.get("heading_deg", 0.0)) + _camera_yaw_offset_deg(state)
    heading = math.radians(heading_deg)
    forward_x = -math.sin(heading)
    forward_y = math.cos(heading)
    right_x = math.cos(heading)
    right_y = math.sin(heading)
    half_fov = float(fov_deg) / 2.0

    visible: List[Dict[str, Any]] = []
    for obs in state.get("course", {}).get("obstacles", []):
        dx = float(obs.get("x", 0.0)) - rx
        dy = float(obs.get("y", 0.0)) - ry
        forward_dist = dx * forward_x + dy * forward_y
        if forward_dist <= 0.0 or forward_dist > max_range_m:
            continue
        lateral = dx * right_x + dy * right_y
        angle_deg = math.degrees(math.atan2(lateral, forward_dist))
        if abs(angle_deg) > half_fov:
            continue
        item = dict(obs)
        item["distance_m"] = round(forward_dist, 3)
        item["angle_deg"] = round(angle_deg, 1)
        item["lateral_m"] = round(lateral, 3)
        item["color_name"] = _obstacle_color_name(obs)
        visible.append(item)

    visible.sort(key=lambda item: float(item["angle_deg"]))
    for idx, item in enumerate(visible, start=1):
        item["index"] = idx
    return visible


def _render_svg(
    state: Dict[str, Any],
    visible: Sequence[Dict[str, Any]],
    *,
    highlight_color: Optional[str] = None,
    calibration: Optional[Dict[str, Any]] = None,
    title: str = "Simulator Vision",
) -> str:
    width = 320
    height = 240
    sky = "#d9eefc"
    floor = "#d8d0bd"
    horizon_y = 96
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        f'<rect x="0" y="0" width="{width}" height="{height}" fill="{sky}"/>',
        f'<rect x="0" y="{horizon_y}" width="{width}" height="{height - horizon_y}" fill="{floor}"/>',
        f'<text x="12" y="20" font-size="14" fill="#243447">{title}</text>',
        f'<text x="12" y="38" font-size="11" fill="#4b5563">Course: {state.get("course", {}).get("title", "default")}</text>',
        f'<line x1="0" y1="{horizon_y}" x2="{width}" y2="{horizon_y}" stroke="#9aa5b1" stroke-width="2"/>',
    ]

    for obs in sorted(visible, key=lambda item: float(item["distance_m"]), reverse=True):
        angle = float(obs["angle_deg"])
        dist = max(0.18, float(obs["distance_m"]))
        x = width * 0.5 + (angle / 35.0) * (width * 0.38)
        y = 185 - min(110, dist * 70)
        r = max(10, 34 - dist * 12)
        color_name = obs.get("color_name", "unknown")
        fill = SIM_COLOR_HEX.get(color_name, SIM_COLOR_HEX["unknown"])
        stroke = "#111827"
        stroke_width = 4 if highlight_color and color_name == highlight_color else 2
        parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{r:.1f}" fill="{fill}" stroke="{stroke}" stroke-width="{stroke_width}"/>')
        parts.append(f'<circle cx="{x:.1f}" cy="{y - r * 0.15:.1f}" r="{r * 0.45:.1f}" fill="#ffffff" fill-opacity="0.45"/>')
        parts.append(f'<text x="{x:.1f}" y="{y + r + 14:.1f}" text-anchor="middle" font-size="11" fill="#1f2937">{obs.get("id", "cup")} ({color_name})</text>')
        parts.append(f'<text x="{x:.1f}" y="{y - r - 8:.1f}" text-anchor="middle" font-size="10" fill="#111827">#{obs.get("index", 0)}</text>')

    if calibration:
        box = calibration.get("box_size", 80)
        left = (width - box) / 2.0
        top = (height - box) / 2.0
        parts.append(f'<rect x="{left:.1f}" y="{top:.1f}" width="{box}" height="{box}" fill="none" stroke="#ffffff" stroke-width="2" stroke-dasharray="6 4"/>')
        parts.append(f'<text x="{width/2:.1f}" y="{top - 10:.1f}" text-anchor="middle" font-size="11" fill="#ffffff">Aim {calibration.get("color","target")} cup here</text>')

    if not visible:
        parts.append('<text x="160" y="150" text-anchor="middle" font-size="14" fill="#374151">No cups visible in current camera view</text>')

    parts.append("</svg>")
    return "".join(parts)


class Vision:
    def __init__(self, camera_index: Optional[int] = None, width: int = 320, height: int = 240, warmup_s: float = 0.0, min_area: int = 350):
        self.camera_index = int(camera_index or 0)
        self.width = int(width)
        self.height = int(height)
        self.warmup_s = float(warmup_s)
        self.min_area = int(min_area)
        self._profiles = copy.deepcopy(DEFAULT_COLOR_PROFILES)

    def set_color_profile(self, color: str, lower_hsv, upper_hsv=None):
        name = _normalize_color_name(color)
        if upper_hsv is None:
            ranges = [tuple(pair) for pair in lower_hsv]
        else:
            ranges = [(tuple(int(v) for v in lower_hsv), tuple(int(v) for v in upper_hsv))]
        self._profiles[name] = ranges
        return {"color": name, "ranges": ranges, "simulated": True}

    def get_color_profile(self, color: str):
        name = _normalize_color_name(color)
        return copy.deepcopy(self._profiles[name])

    def show_profiles(self):
        for name in sorted(self._profiles):
            print(f"{name}: {self._profiles[name]}")
        return copy.deepcopy(self._profiles)

    def capture(self, show: bool = True, save_path: Optional[str] = None, title: str = "Simulator Camera Capture"):
        state = load_state()
        visible = _visible_obstacles(state)
        svg = _render_svg(state, visible, title=title)
        displayed = _display_svg(svg) if show else False
        path = _write_svg(svg, save_path=save_path)
        if not displayed:
            print(f"Image saved: {path}")
        return {"frame_bgr": None, "displayed": displayed, "path": path, "count": len(visible), "objects": visible}

    def show_color(self, color: str, show: bool = True, save_path: Optional[str] = None, min_area: Optional[int] = None):
        return self.find_color_objects(color=color, show=show, save_path=save_path, min_area=min_area)

    def find_color_objects(self, color: str, show: bool = True, save_path: Optional[str] = None, min_area: Optional[int] = None):
        state = load_state()
        name = _normalize_color_name(color)
        visible = _visible_obstacles(state)
        matches = [dict(item) for item in visible if _normalize_color_name(item.get("color_name", "")) == name]
        for idx, item in enumerate(matches, start=1):
            item["index"] = idx
        svg = _render_svg(state, visible, highlight_color=name, title=f"Detected {name} cups: {len(matches)}")
        displayed = _display_svg(svg) if show else False
        path = _write_svg(svg, save_path=save_path)
        if not displayed:
            print(f"Image saved: {path}")
        return {"color": name, "found": bool(matches), "count": len(matches), "objects": matches, "path": path, "ranges": copy.deepcopy(self._profiles.get(name, [])), "simulated": True}

    def which_object(self, color: str, show: bool = True, save_path: Optional[str] = None, min_area: Optional[int] = None) -> int:
        result = self.find_color_objects(color=color, show=show, save_path=save_path, min_area=min_area)
        if not result["objects"]:
            print(f"No visible {result['color']} cup found")
            return 0
        index = int(result["objects"][0]["index"])
        print(f"{result['color']} cup index: {index}")
        return index

    def calibrate_color(self, color: str, box_size: int = 80, hue_pad: int = 12, sat_pad: int = 70, val_pad: int = 70, show: bool = True, save_path: Optional[str] = None, persist: bool = True):
        state = load_state()
        name = _normalize_color_name(color)
        if persist and name not in self._profiles:
            self._profiles[name] = copy.deepcopy(DEFAULT_COLOR_PROFILES.get(name, []))
        visible = _visible_obstacles(state)
        svg = _render_svg(state, visible, highlight_color=name, calibration={"color": name, "box_size": box_size}, title=f"Calibrate {name} cup")
        displayed = _display_svg(svg) if show else False
        path = _write_svg(svg, save_path=save_path)
        if not displayed:
            print(f"Image saved: {path}")
        result = {
            "color": name,
            "sample_hsv": None,
            "ranges": copy.deepcopy(self._profiles.get(name, [])),
            "path": path,
            "persisted": bool(persist),
            "simulated": True,
        }
        print(result)
        return result


_VISION_SINGLETON: Optional[Vision] = None


def get_vision(camera_index: Optional[int] = None, width: int = 320, height: int = 240, warmup_s: float = 0.0, min_area: int = 350) -> Vision:
    global _VISION_SINGLETON
    if _VISION_SINGLETON is None:
        _VISION_SINGLETON = Vision(camera_index=camera_index, width=width, height=height, warmup_s=warmup_s, min_area=min_area)
    return _VISION_SINGLETON


def reset_vision() -> None:
    global _VISION_SINGLETON
    _VISION_SINGLETON = None


def capture(*args, **kwargs):
    return get_vision().capture(*args, **kwargs)


def show_color(*args, **kwargs):
    return get_vision().show_color(*args, **kwargs)


def find_color_objects(*args, **kwargs):
    return get_vision().find_color_objects(*args, **kwargs)


def which_object(*args, **kwargs):
    return get_vision().which_object(*args, **kwargs)


def calibrate_color(*args, **kwargs):
    return get_vision().calibrate_color(*args, **kwargs)


def set_color_profile(*args, **kwargs):
    return get_vision().set_color_profile(*args, **kwargs)


def get_color_profile(*args, **kwargs):
    return get_vision().get_color_profile(*args, **kwargs)


def show_profiles():
    return get_vision().show_profiles()
