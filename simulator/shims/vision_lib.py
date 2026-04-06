#!/usr/bin/env python3
"""Simulator vision shim for notebook colour and MediaPipe workflows."""

from __future__ import annotations

import copy
import math
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from simulator.core.sim_state import load_state

__version__ = "1.1.0-sim"

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

GESTURE_STYLES = {
    "open_palm": ("Open Palm", "#ffffff"),
    "thumbs_up": ("Thumbs Up", "#facc15"),
    "peace": ("Peace", "#a78bfa"),
    "point": ("Point", "#38bdf8"),
    "fist": ("Fist", "#f97316"),
    "unknown": ("Unknown", "#cbd5e1"),
}

POSE_STYLES = {
    "hands_up": ("Hands Up", "#22c55e"),
    "t_pose": ("T Pose", "#60a5fa"),
    "left_hand_up": ("Left Hand Up", "#e879f9"),
    "right_hand_up": ("Right Hand Up", "#f472b6"),
    "neutral": ("Neutral", "#cbd5e1"),
}


def _normalize_color_name(color: str) -> str:
    value = str(color or "").strip().lower()
    aliases = {"r": "red", "g": "green", "b": "blue"}
    return aliases.get(value, value)


def _gesture_to_game_move(gesture: str) -> Optional[str]:
    value = str(gesture or "").strip().lower()
    mapping = {
        "fist": "rock",
        "open_palm": "paper",
        "peace": "scissors",
        "rock": "rock",
        "paper": "paper",
        "scissors": "scissors",
    }
    return mapping.get(value)


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


def _viewer_vectors(state: Dict[str, Any]) -> Tuple[float, float, float, float, float]:
    robot = state.get("robot", {})
    heading_deg = float(robot.get("heading_deg", 0.0)) + _camera_yaw_offset_deg(state)
    heading = math.radians(heading_deg)
    forward_x = -math.sin(heading)
    forward_y = math.cos(heading)
    right_x = math.cos(heading)
    right_y = math.sin(heading)
    return heading_deg, forward_x, forward_y, right_x, right_y


def _visible_obstacles(state: Dict[str, Any], fov_deg: float = 70.0, max_range_m: float = 1.6) -> List[Dict[str, Any]]:
    robot = state.get("robot", {})
    rx = float(robot.get("x", 0.0))
    ry = float(robot.get("y", 0.0))
    _heading_deg, forward_x, forward_y, right_x, right_y = _viewer_vectors(state)
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


def _visible_people(state: Dict[str, Any], fov_deg: float = 78.0, max_range_m: float = 2.4) -> List[Dict[str, Any]]:
    robot = state.get("robot", {})
    rx = float(robot.get("x", 0.0))
    ry = float(robot.get("y", 0.0))
    _heading_deg, forward_x, forward_y, right_x, right_y = _viewer_vectors(state)
    half_fov = float(fov_deg) / 2.0
    visible: List[Dict[str, Any]] = []
    for person in state.get("course", {}).get("people", []):
        dx = float(person.get("x", 0.0)) - rx
        dy = float(person.get("y", 0.0)) - ry
        forward_dist = dx * forward_x + dy * forward_y
        if forward_dist <= 0.0 or forward_dist > max_range_m:
            continue
        lateral = dx * right_x + dy * right_y
        angle_deg = math.degrees(math.atan2(lateral, forward_dist))
        if abs(angle_deg) > half_fov:
            continue
        item = dict(person)
        item["distance_m"] = round(forward_dist, 3)
        item["angle_deg"] = round(angle_deg, 1)
        item["lateral_m"] = round(lateral, 3)
        item["face_visible"] = bool(person.get("face_visible", True))
        item["hand_gesture"] = str(person.get("hand_gesture", "unknown") or "unknown")
        item["pose_label"] = str(person.get("pose_label", "neutral") or "neutral")
        visible.append(item)
    visible.sort(key=lambda item: float(item["angle_deg"]))
    for idx, item in enumerate(visible, start=1):
        item["index"] = idx
    return visible


def _screen_x_from_angle(angle_deg: float, width: int = 320) -> float:
    return width * 0.5 + (float(angle_deg) / 35.0) * (width * 0.38)


def _screen_y_from_distance(distance_m: float) -> float:
    return 188 - min(118, max(0.18, float(distance_m)) * 66)


def _face_box(person: Dict[str, Any], width: int = 320, height: int = 240) -> Dict[str, int]:
    dist = max(0.28, float(person.get("distance_m", 1.2)))
    cx = _screen_x_from_angle(float(person.get("angle_deg", 0.0)), width)
    face_w = max(34, 110 - dist * 30)
    face_h = max(44, face_w * 1.2)
    x = int(round(max(0, min(width - face_w, cx - face_w / 2))))
    y = int(round(max(14, min(height - face_h - 12, 64 + dist * 10))))
    return {
        "x": x,
        "y": y,
        "w": int(round(face_w)),
        "h": int(round(face_h)),
        "cx": int(round(x + face_w / 2)),
        "cy": int(round(y + face_h / 2)),
    }


def _render_svg(
    state: Dict[str, Any],
    visible: Sequence[Dict[str, Any]],
    *,
    people: Optional[Sequence[Dict[str, Any]]] = None,
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

    for person in sorted(list(people or []), key=lambda item: float(item.get("distance_m", 1.0)), reverse=True):
        angle = float(person.get("angle_deg", 0.0))
        dist = max(0.28, float(person.get("distance_m", 1.0)))
        x = _screen_x_from_angle(angle, width)
        y = _screen_y_from_distance(dist) - 20
        body_h = max(48, 112 - dist * 20)
        body_w = max(20, body_h * 0.34)
        pose_label = str(person.get("pose_label", "neutral"))
        pose_text, pose_color = POSE_STYLES.get(pose_label, POSE_STYLES["neutral"])
        gesture_label = str(person.get("hand_gesture", "unknown"))
        gesture_text, gesture_color = GESTURE_STYLES.get(gesture_label, GESTURE_STYLES["unknown"])
        head_r = max(9, body_h * 0.14)
        parts.append(f'<circle cx="{x:.1f}" cy="{y - body_h * 0.36:.1f}" r="{head_r:.1f}" fill="#f2d0b1" stroke="#8b5e3c" stroke-width="2"/>')
        parts.append(f'<line x1="{x:.1f}" y1="{y - body_h * 0.22:.1f}" x2="{x:.1f}" y2="{y + body_h * 0.22:.1f}" stroke="{pose_color}" stroke-width="4" stroke-linecap="round"/>')
        if pose_label == "hands_up":
            arms = [(-body_w * 0.9, -body_h * 0.1, -body_w * 1.3, -body_h * 0.55), (body_w * 0.9, -body_h * 0.1, body_w * 1.3, -body_h * 0.55)]
        elif pose_label == "t_pose":
            arms = [(-body_w * 0.25, -body_h * 0.08, -body_w * 1.5, -body_h * 0.08), (body_w * 0.25, -body_h * 0.08, body_w * 1.5, -body_h * 0.08)]
        elif pose_label == "left_hand_up":
            arms = [(-body_w * 0.9, -body_h * 0.08, -body_w * 1.3, -body_h * 0.55), (body_w * 0.25, -body_h * 0.02, body_w * 1.1, body_h * 0.1)]
        elif pose_label == "right_hand_up":
            arms = [(-body_w * 0.25, -body_h * 0.02, -body_w * 1.1, body_h * 0.1), (body_w * 0.9, -body_h * 0.08, body_w * 1.3, -body_h * 0.55)]
        else:
            arms = [(-body_w * 0.3, 0, -body_w * 1.0, body_h * 0.1), (body_w * 0.3, 0, body_w * 1.0, body_h * 0.1)]
        for ax1, ay1, ax2, ay2 in arms:
            parts.append(f'<line x1="{x + ax1:.1f}" y1="{y + ay1:.1f}" x2="{x + ax2:.1f}" y2="{y + ay2:.1f}" stroke="{pose_color}" stroke-width="4" stroke-linecap="round"/>')
        parts.append(f'<line x1="{x:.1f}" y1="{y + body_h * 0.22:.1f}" x2="{x - body_w * 0.7:.1f}" y2="{y + body_h * 0.65:.1f}" stroke="{pose_color}" stroke-width="4" stroke-linecap="round"/>')
        parts.append(f'<line x1="{x:.1f}" y1="{y + body_h * 0.22:.1f}" x2="{x + body_w * 0.7:.1f}" y2="{y + body_h * 0.65:.1f}" stroke="{pose_color}" stroke-width="4" stroke-linecap="round"/>')
        if person.get("face_visible", True):
            fb = _face_box(person, width, height)
            parts.append(f'<rect x="{fb["x"]}" y="{fb["y"]}" width="{fb["w"]}" height="{fb["h"]}" fill="none" stroke="#22c55e" stroke-width="2" rx="6"/>')
        parts.append(f'<text x="{x:.1f}" y="{y - body_h * 0.78:.1f}" text-anchor="middle" font-size="10" fill="#111827">face {person.get("index", 0)}</text>')
        parts.append(f'<text x="{x:.1f}" y="{y + body_h * 0.86:.1f}" text-anchor="middle" font-size="10" fill="{gesture_color}">{gesture_text}</text>')
        parts.append(f'<text x="{x:.1f}" y="{y + body_h * 1.02:.1f}" text-anchor="middle" font-size="10" fill="{pose_color}">{pose_text}</text>')

    for obs in sorted(visible, key=lambda item: float(item["distance_m"]), reverse=True):
        angle = float(obs["angle_deg"])
        dist = max(0.18, float(obs["distance_m"]))
        x = _screen_x_from_angle(angle, width)
        y = _screen_y_from_distance(dist)
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
        parts.append(f'<text x="{width/2:.1f}" y="{top - 10:.1f}" text-anchor="middle" font-size="11" fill="#ffffff">Aim {calibration.get("color","target")} object here</text>')

    if not visible and not people:
        parts.append('<text x="160" y="150" text-anchor="middle" font-size="14" fill="#374151">Nothing visible in current camera view</text>')

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
        people = _visible_people(state)
        svg = _render_svg(state, visible, people=people, title=title)
        displayed = _display_svg(svg) if show else False
        path = _write_svg(svg, save_path=save_path)
        if not displayed:
            print(f"Image saved: {path}")
        return {"frame_bgr": None, "displayed": displayed, "path": path, "count": len(visible), "objects": visible, "people": people, "simulated": True}

    def show_color(self, color: str, show: bool = True, save_path: Optional[str] = None, min_area: Optional[int] = None):
        return self.find_color_objects(color=color, show=show, save_path=save_path, min_area=min_area)

    def find_color_objects(self, color: str, show: bool = True, save_path: Optional[str] = None, min_area: Optional[int] = None):
        state = load_state()
        name = _normalize_color_name(color)
        visible = _visible_obstacles(state)
        people = _visible_people(state)
        matches = [dict(item) for item in visible if _normalize_color_name(item.get("color_name", "")) == name]
        for idx, item in enumerate(matches, start=1):
            item["index"] = idx
        svg = _render_svg(state, visible, people=people, highlight_color=name, title=f"Detected {name} cups: {len(matches)}")
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
        people = _visible_people(state)
        svg = _render_svg(state, visible, people=people, highlight_color=name, calibration={"color": name, "box_size": box_size}, title=f"Calibrate {name} object")
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

    def detect_faces(self, show: bool = True, save_path: Optional[str] = None, min_confidence: float = 0.5):
        state = load_state()
        visible = _visible_obstacles(state)
        people = _visible_people(state)
        faces = []
        for person in people:
            if not person.get("face_visible", True):
                continue
            face = _face_box(person, self.width, self.height)
            face["index"] = int(person.get("index", len(faces) + 1))
            face["score"] = max(float(min_confidence), 0.91)
            face["person_id"] = person.get("id", f"person_{face['index']}")
            faces.append(face)
        svg = _render_svg(state, visible, people=people, title=f"Detected faces: {len(faces)}")
        displayed = _display_svg(svg) if show else False
        path = _write_svg(svg, save_path=save_path)
        if not displayed:
            print(f"Image saved: {path}")
        return {"found": bool(faces), "count": len(faces), "faces": faces, "path": path, "simulated": True}

    def show_faces(self, show: bool = True, save_path: Optional[str] = None, min_confidence: float = 0.5):
        return self.detect_faces(show=show, save_path=save_path, min_confidence=min_confidence)

    def recognize_faces(self, show: bool = True, save_path: Optional[str] = None, min_confidence: float = 0.5):
        return self.detect_faces(show=show, save_path=save_path, min_confidence=min_confidence)

    def recognize_hands(self, show: bool = True, save_path: Optional[str] = None, max_hands: int = 2, min_detection_confidence: float = 0.5, min_tracking_confidence: float = 0.5):
        state = load_state()
        visible = _visible_obstacles(state)
        people = _visible_people(state)
        hands = []
        for person in people[: max(1, int(max_hands))]:
            gesture = str(person.get("hand_gesture", "unknown") or "unknown")
            hands.append({
                "index": len(hands) + 1,
                "handedness": str(person.get("handedness", "right") or "right").title(),
                "gesture": gesture,
                "game_move": _gesture_to_game_move(gesture),
                "fingers": {
                    "thumb": gesture in {"open_palm", "thumbs_up"},
                    "index": gesture in {"open_palm", "peace", "point"},
                    "middle": gesture in {"open_palm", "peace"},
                    "ring": gesture == "open_palm",
                    "pinky": gesture == "open_palm",
                },
                "bbox": _face_box(person, self.width, self.height),
                "person_id": person.get("id", f"person_{len(hands)+1}"),
            })
        svg = _render_svg(state, visible, people=people, title=f"Detected hands: {len(hands)}")
        displayed = _display_svg(svg) if show else False
        path = _write_svg(svg, save_path=save_path)
        if not displayed:
            print(f"Image saved: {path}")
        return {
            "found": bool(hands),
            "count": len(hands),
            "hands": hands,
            "game_moves": [hand["game_move"] for hand in hands if hand.get("game_move")],
            "path": path,
            "simulated": True,
        }

    def show_hands(self, *args, **kwargs):
        return self.recognize_hands(*args, **kwargs)

    def detect_pose(self, show: bool = True, save_path: Optional[str] = None, min_detection_confidence: float = 0.5, min_tracking_confidence: float = 0.5):
        state = load_state()
        visible = _visible_obstacles(state)
        people = _visible_people(state)
        person = people[0] if people else None
        if person is None:
            pose = {"found": False, "label": "none", "landmarks": {}, "path": None, "simulated": True}
        else:
            pose = {
                "found": True,
                "label": str(person.get("pose_label", "neutral") or "neutral"),
                "landmarks": {
                    "nose": {"x": 0.5, "y": 0.18, "z": 0.0, "visibility": 0.99},
                    "left_shoulder": {"x": 0.42, "y": 0.36, "z": 0.0, "visibility": 0.99},
                    "right_shoulder": {"x": 0.58, "y": 0.36, "z": 0.0, "visibility": 0.99},
                    "left_wrist": {"x": 0.34, "y": 0.16 if person.get("pose_label") in {"hands_up", "left_hand_up"} else 0.46, "z": 0.0, "visibility": 0.98},
                    "right_wrist": {"x": 0.66, "y": 0.16 if person.get("pose_label") in {"hands_up", "right_hand_up"} else 0.46, "z": 0.0, "visibility": 0.98},
                },
                "person_id": person.get("id", "person_1"),
                "simulated": True,
            }
        svg = _render_svg(state, visible, people=people, title=f"Pose: {pose['label']}")
        displayed = _display_svg(svg) if show else False
        path = _write_svg(svg, save_path=save_path)
        if not displayed:
            print(f"Image saved: {path}")
        pose["path"] = path
        return pose

    def show_pose(self, *args, **kwargs):
        return self.detect_pose(*args, **kwargs)

    def recognize_pose(self, *args, **kwargs):
        return self.detect_pose(*args, **kwargs)


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


def detect_faces(*args, **kwargs):
    return get_vision().detect_faces(*args, **kwargs)


def show_faces(*args, **kwargs):
    return get_vision().show_faces(*args, **kwargs)


def recognize_faces(*args, **kwargs):
    return get_vision().recognize_faces(*args, **kwargs)


def recognize_hands(*args, **kwargs):
    return get_vision().recognize_hands(*args, **kwargs)


def show_hands(*args, **kwargs):
    return get_vision().show_hands(*args, **kwargs)


def detect_pose(*args, **kwargs):
    return get_vision().detect_pose(*args, **kwargs)


def show_pose(*args, **kwargs):
    return get_vision().show_pose(*args, **kwargs)


def recognize_pose(*args, **kwargs):
    return get_vision().recognize_pose(*args, **kwargs)
