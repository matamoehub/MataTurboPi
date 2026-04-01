#!/usr/bin/env python3
from __future__ import annotations

"""
Notebook-friendly camera and vision helper.

Uses the OpenCV pattern already taught in Lesson 3 and wraps it in a
singleton-safe library that can:
- capture a camera image
- detect red / green / blue objects
- run MediaPipe face / hand / pose analysis
- display annotated images inline in Jupyter
- calibrate colour HSV ranges from the notebook
"""
__version__ = "1.1.0"

import copy
import os
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from ros_service_client import clear_process_singleton, get_process_singleton, set_process_singleton


HSVRange = Tuple[Tuple[int, int, int], Tuple[int, int, int]]


DEFAULT_COLOR_PROFILES: Dict[str, List[HSVRange]] = {
    "red": [
        ((0, 120, 50), (10, 255, 255)),
        ((170, 120, 50), (179, 255, 255)),
    ],
    "green": [
        ((35, 80, 40), (85, 255, 255)),
    ],
    "blue": [
        ((90, 80, 40), (135, 255, 255)),
    ],
}


def _normalize_color_name(color: str) -> str:
    value = str(color or "").strip().lower()
    aliases = {
        "r": "red",
        "g": "green",
        "b": "blue",
    }
    return aliases.get(value, value)


def _require_runtime():
    try:
        import cv2  # type: ignore
        import numpy as np  # type: ignore
    except Exception as e:  # pragma: no cover - depends on robot runtime
        raise RuntimeError(
            "vision_lib requires OpenCV and NumPy on the robot image. "
            f"Import failed: {e}"
        ) from e
    return cv2, np


def _require_mediapipe_runtime():
    try:
        import mediapipe as mp  # type: ignore
    except Exception as e:  # pragma: no cover - depends on robot runtime
        raise RuntimeError(
            "vision_lib MediaPipe features require the mediapipe package on the robot image. "
            f"Import failed: {e}"
        ) from e
    return mp


def _display_png_bytes(png_bytes: bytes) -> bool:
    try:  # pragma: no cover - notebook-only behavior
        from IPython.display import Image, display
    except Exception:
        return False
    display(Image(data=png_bytes))
    return True


def _coerce_range(lower: Sequence[int], upper: Sequence[int]) -> HSVRange:
    low = tuple(int(v) for v in lower)
    high = tuple(int(v) for v in upper)
    if len(low) != 3 or len(high) != 3:
        raise ValueError("HSV ranges must contain exactly 3 values")
    return low, high


def _expand_hue_wrap(lower: Tuple[int, int, int], upper: Tuple[int, int, int]) -> List[HSVRange]:
    lh, ls, lv = lower
    uh, us, uv = upper
    if lh < 0:
        return [
            ((0, ls, lv), (uh, us, uv)),
            ((180 + lh, ls, lv), (179, us, uv)),
        ]
    if uh > 179:
        return [
            ((lh, ls, lv), (179, us, uv)),
            ((0, ls, lv), (uh - 180, us, uv)),
        ]
    return [((max(0, lh), ls, lv), (min(179, uh), us, uv))]


def _clamp_pixel(value: float, maximum: int) -> int:
    return max(0, min(int(round(value)), maximum))


def _hand_landmark_xy(landmarks, index: int) -> Tuple[float, float]:
    point = landmarks[index]
    return float(point.x), float(point.y)


def _classify_hand_gesture(landmarks, handedness: str) -> Tuple[str, Dict[str, bool]]:
    wrist_x, wrist_y = _hand_landmark_xy(landmarks, 0)
    thumb_tip_x, thumb_tip_y = _hand_landmark_xy(landmarks, 4)
    thumb_ip_x, thumb_ip_y = _hand_landmark_xy(landmarks, 3)
    index_tip_y = _hand_landmark_xy(landmarks, 8)[1]
    index_pip_y = _hand_landmark_xy(landmarks, 6)[1]
    middle_tip_y = _hand_landmark_xy(landmarks, 12)[1]
    middle_pip_y = _hand_landmark_xy(landmarks, 10)[1]
    ring_tip_y = _hand_landmark_xy(landmarks, 16)[1]
    ring_pip_y = _hand_landmark_xy(landmarks, 14)[1]
    pinky_tip_y = _hand_landmark_xy(landmarks, 20)[1]
    pinky_pip_y = _hand_landmark_xy(landmarks, 18)[1]

    fingers = {
        "thumb": (thumb_tip_x < thumb_ip_x) if handedness.lower().startswith("right") else (thumb_tip_x > thumb_ip_x),
        "index": index_tip_y < index_pip_y,
        "middle": middle_tip_y < middle_pip_y,
        "ring": ring_tip_y < ring_pip_y,
        "pinky": pinky_tip_y < pinky_pip_y,
    }

    if all(fingers.values()):
        return "open_palm", fingers
    if not any(fingers.values()):
        return "fist", fingers
    if fingers["thumb"] and not fingers["index"] and not fingers["middle"] and not fingers["ring"] and not fingers["pinky"]:
        if thumb_tip_y < wrist_y and thumb_ip_y < wrist_y:
            return "thumbs_up", fingers
        return "thumb_out", fingers
    if fingers["index"] and not fingers["middle"] and not fingers["ring"] and not fingers["pinky"]:
        return "point", fingers
    if fingers["index"] and fingers["middle"] and not fingers["ring"] and not fingers["pinky"]:
        return "peace", fingers
    if fingers["index"] and fingers["pinky"] and not fingers["middle"] and not fingers["ring"]:
        return "rock", fingers
    return "unknown", fingers


def _classify_pose(landmarks) -> str:
    left_shoulder = landmarks[11]
    right_shoulder = landmarks[12]
    left_wrist = landmarks[15]
    right_wrist = landmarks[16]
    left_elbow = landmarks[13]
    right_elbow = landmarks[14]
    nose = landmarks[0]

    wrists_above_shoulders = left_wrist.y < left_shoulder.y and right_wrist.y < right_shoulder.y
    wrists_out_wide = abs(left_wrist.y - left_shoulder.y) < 0.10 and abs(right_wrist.y - right_shoulder.y) < 0.10
    elbows_out_wide = abs(left_elbow.y - left_shoulder.y) < 0.12 and abs(right_elbow.y - right_shoulder.y) < 0.12

    if wrists_above_shoulders:
        return "hands_up"
    if wrists_out_wide and elbows_out_wide:
        return "t_pose"
    if left_wrist.y < nose.y and right_wrist.y >= right_shoulder.y:
        return "left_hand_up"
    if right_wrist.y < nose.y and left_wrist.y >= left_shoulder.y:
        return "right_hand_up"
    return "neutral"


class Vision:
    def __init__(
        self,
        camera_index: Optional[int] = None,
        width: int = 320,
        height: int = 240,
        warmup_s: float = 0.15,
        min_area: int = 350,
    ):
        self.camera_index = int(os.environ.get("CAM_INDEX", camera_index if camera_index is not None else 0))
        self.width = int(width)
        self.height = int(height)
        self.warmup_s = float(warmup_s)
        self.min_area = int(min_area)
        self._profiles: Dict[str, List[HSVRange]] = copy.deepcopy(DEFAULT_COLOR_PROFILES)

    def set_color_profile(
        self,
        color: str,
        lower_hsv: Sequence[int] | Sequence[Sequence[int]],
        upper_hsv: Optional[Sequence[int]] = None,
    ) -> Dict[str, Any]:
        name = _normalize_color_name(color)
        if upper_hsv is None:
            ranges = []
            for pair in lower_hsv:  # type: ignore[assignment]
                if len(pair) != 2:
                    raise ValueError("Expected [(lower, upper), ...] when upper_hsv is omitted")
                ranges.append(_coerce_range(pair[0], pair[1]))  # type: ignore[index]
        else:
            ranges = [_coerce_range(lower_hsv, upper_hsv)]  # type: ignore[arg-type]
        self._profiles[name] = ranges
        return {"color": name, "ranges": ranges}

    def get_color_profile(self, color: str) -> List[HSVRange]:
        name = _normalize_color_name(color)
        if name not in self._profiles:
            raise KeyError(f"Unknown colour profile: {color}")
        return copy.deepcopy(self._profiles[name])

    def show_profiles(self) -> Dict[str, List[HSVRange]]:
        for name in sorted(self._profiles):
            print(f"{name}: {self._profiles[name]}")
        return copy.deepcopy(self._profiles)

    def _open_capture(self):
        cv2, _np = _require_runtime()
        cap = cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            raise RuntimeError(
                f"Camera failed to open on index {self.camera_index}. "
                "Try setting CAM_INDEX=1 on this robot."
            )
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.warmup_s > 0:
            time.sleep(self.warmup_s)
        return cap

    def capture_frame(self):
        cap = self._open_capture()
        try:
            ok, frame = cap.read()
        finally:
            cap.release()
        if not ok or frame is None:
            raise RuntimeError("Camera opened, but no image frame was captured")
        return frame

    def _capture_rgb_frame(self):
        cv2, _np = _require_runtime()
        frame_bgr = self.capture_frame()
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        return frame_bgr, frame_rgb

    def _write_image(self, frame_bgr, save_path: Optional[str] = None) -> str:
        cv2, _np = _require_runtime()
        target = Path(save_path) if save_path else Path(tempfile.gettempdir()) / f"vision_capture_{int(time.time() * 1000)}.png"
        target.parent.mkdir(parents=True, exist_ok=True)
        ok = cv2.imwrite(str(target), frame_bgr)
        if not ok:
            raise RuntimeError(f"Failed to write image to {target}")
        return str(target)

    def show_image(self, frame_bgr, save_path: Optional[str] = None, title: Optional[str] = None) -> Dict[str, Any]:
        cv2, _np = _require_runtime()
        ok, encoded = cv2.imencode(".png", frame_bgr)
        if not ok:
            raise RuntimeError("Failed to encode image for notebook display")
        displayed = _display_png_bytes(encoded.tobytes())
        path = self._write_image(frame_bgr, save_path=save_path)
        if title:
            print(title)
        if not displayed:
            print(f"Image saved: {path}")
        return {"displayed": displayed, "path": path}

    def capture(self, show: bool = True, save_path: Optional[str] = None, title: str = "Camera Capture") -> Dict[str, Any]:
        frame = self.capture_frame()
        info = self.show_image(frame, save_path=save_path, title=title) if show else {"displayed": False, "path": self._write_image(frame, save_path=save_path)}
        return {"frame_bgr": frame, **info}

    def _combined_mask(self, hsv_frame, ranges: Sequence[HSVRange]):
        cv2, np = _require_runtime()
        mask = None
        for lower, upper in ranges:
            part = cv2.inRange(hsv_frame, np.array(lower), np.array(upper))
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=2)
        return mask

    def find_color_objects(
        self,
        color: str,
        show: bool = True,
        save_path: Optional[str] = None,
        min_area: Optional[int] = None,
    ) -> Dict[str, Any]:
        cv2, _np = _require_runtime()
        name = _normalize_color_name(color)
        ranges = self.get_color_profile(name)
        frame = self.capture_frame()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self._combined_mask(hsv, ranges)

        contours, _hier = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        threshold = int(self.min_area if min_area is None else min_area)
        objects = []
        annotated = frame.copy()

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < threshold:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            m = cv2.moments(contour)
            if m["m00"] == 0:
                continue
            cx = int(m["m10"] / m["m00"])
            cy = int(m["m01"] / m["m00"])
            objects.append({
                "x": int(x),
                "y": int(y),
                "w": int(w),
                "h": int(h),
                "cx": cx,
                "cy": cy,
                "area": area,
            })

        objects.sort(key=lambda item: item["cx"])
        for idx, item in enumerate(objects, start=1):
            item["index"] = idx
            cv2.rectangle(
                annotated,
                (item["x"], item["y"]),
                (item["x"] + item["w"], item["y"] + item["h"]),
                (0, 255, 255),
                2,
            )
            cv2.putText(
                annotated,
                f"{name} #{idx}",
                (item["x"], max(18, item["y"] - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
            )

        path = None
        if show:
            info = self.show_image(annotated, save_path=save_path, title=f"Detected {name} objects: {len(objects)}")
            path = info["path"]
        elif save_path:
            path = self._write_image(annotated, save_path=save_path)

        return {
            "color": name,
            "found": bool(objects),
            "count": len(objects),
            "objects": objects,
            "path": path,
            "ranges": ranges,
        }

    def detect_faces(
        self,
        show: bool = True,
        save_path: Optional[str] = None,
        min_confidence: float = 0.5,
        model_selection: int = 0,
    ) -> Dict[str, Any]:
        cv2, _np = _require_runtime()
        mp = _require_mediapipe_runtime()
        frame_bgr, frame_rgb = self._capture_rgb_frame()
        annotated = frame_bgr.copy()
        h, w = annotated.shape[:2]
        faces: List[Dict[str, Any]] = []

        with mp.solutions.face_detection.FaceDetection(
            model_selection=int(model_selection),
            min_detection_confidence=float(min_confidence),
        ) as detector:
            result = detector.process(frame_rgb)

        detections = getattr(result, "detections", None) or []
        for idx, detection in enumerate(detections, start=1):
            bbox = detection.location_data.relative_bounding_box
            x = _clamp_pixel(bbox.xmin * w, w - 1)
            y = _clamp_pixel(bbox.ymin * h, h - 1)
            bw = max(1, _clamp_pixel(bbox.width * w, w))
            bh = max(1, _clamp_pixel(bbox.height * h, h))
            face = {
                "index": idx,
                "x": x,
                "y": y,
                "w": bw,
                "h": bh,
                "cx": int(x + bw / 2),
                "cy": int(y + bh / 2),
                "score": float(detection.score[0]) if getattr(detection, "score", None) else 0.0,
            }
            faces.append(face)
            cv2.rectangle(annotated, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.putText(
                annotated,
                f"face #{idx}",
                (x, max(18, y - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
            )

        path = None
        if show:
            info = self.show_image(annotated, save_path=save_path, title=f"Detected faces: {len(faces)}")
            path = info["path"]
        elif save_path:
            path = self._write_image(annotated, save_path=save_path)

        return {
            "found": bool(faces),
            "count": len(faces),
            "faces": faces,
            "path": path,
        }

    def show_faces(self, show: bool = True, save_path: Optional[str] = None, min_confidence: float = 0.5):
        return self.detect_faces(show=show, save_path=save_path, min_confidence=min_confidence)

    def recognize_faces(self, show: bool = True, save_path: Optional[str] = None, min_confidence: float = 0.5):
        return self.detect_faces(show=show, save_path=save_path, min_confidence=min_confidence)

    def recognize_hands(
        self,
        show: bool = True,
        save_path: Optional[str] = None,
        max_hands: int = 2,
        min_detection_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
    ) -> Dict[str, Any]:
        cv2, _np = _require_runtime()
        mp = _require_mediapipe_runtime()
        frame_bgr, frame_rgb = self._capture_rgb_frame()
        annotated = frame_bgr.copy()
        h, w = annotated.shape[:2]
        hands_found: List[Dict[str, Any]] = []

        with mp.solutions.hands.Hands(
            static_image_mode=True,
            max_num_hands=int(max_hands),
            min_detection_confidence=float(min_detection_confidence),
            min_tracking_confidence=float(min_tracking_confidence),
        ) as detector:
            result = detector.process(frame_rgb)

        multi_landmarks = getattr(result, "multi_hand_landmarks", None) or []
        multi_handedness = getattr(result, "multi_handedness", None) or []
        for idx, landmarks in enumerate(multi_landmarks, start=1):
            handedness_label = "unknown"
            if idx - 1 < len(multi_handedness):
                try:
                    handedness_label = multi_handedness[idx - 1].classification[0].label
                except Exception:
                    handedness_label = "unknown"

            xs = [_clamp_pixel(pt.x * w, w - 1) for pt in landmarks.landmark]
            ys = [_clamp_pixel(pt.y * h, h - 1) for pt in landmarks.landmark]
            gesture, fingers = _classify_hand_gesture(landmarks.landmark, handedness_label)
            hand = {
                "index": idx,
                "handedness": handedness_label,
                "gesture": gesture,
                "fingers": fingers,
                "bbox": {
                    "x": min(xs),
                    "y": min(ys),
                    "w": max(xs) - min(xs),
                    "h": max(ys) - min(ys),
                },
            }
            hands_found.append(hand)

            mp.solutions.drawing_utils.draw_landmarks(
                annotated,
                landmarks,
                mp.solutions.hands.HAND_CONNECTIONS,
            )
            cv2.putText(
                annotated,
                f"{handedness_label} {gesture}",
                (hand["bbox"]["x"], max(18, hand["bbox"]["y"] - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 0),
                2,
            )

        path = None
        if show:
            info = self.show_image(annotated, save_path=save_path, title=f"Detected hands: {len(hands_found)}")
            path = info["path"]
        elif save_path:
            path = self._write_image(annotated, save_path=save_path)

        return {
            "found": bool(hands_found),
            "count": len(hands_found),
            "hands": hands_found,
            "path": path,
        }

    def show_hands(
        self,
        show: bool = True,
        save_path: Optional[str] = None,
        max_hands: int = 2,
        min_detection_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
    ):
        return self.recognize_hands(
            show=show,
            save_path=save_path,
            max_hands=max_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )

    def detect_pose(
        self,
        show: bool = True,
        save_path: Optional[str] = None,
        min_detection_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
    ) -> Dict[str, Any]:
        cv2, _np = _require_runtime()
        mp = _require_mediapipe_runtime()
        frame_bgr, frame_rgb = self._capture_rgb_frame()
        annotated = frame_bgr.copy()

        with mp.solutions.pose.Pose(
            static_image_mode=True,
            min_detection_confidence=float(min_detection_confidence),
            min_tracking_confidence=float(min_tracking_confidence),
        ) as detector:
            result = detector.process(frame_rgb)

        pose_landmarks = getattr(result, "pose_landmarks", None)
        pose = {
            "found": bool(pose_landmarks),
            "label": "none",
            "landmarks": {},
        }
        if pose_landmarks is not None:
            mp.solutions.drawing_utils.draw_landmarks(
                annotated,
                pose_landmarks,
                mp.solutions.pose.POSE_CONNECTIONS,
            )
            key_names = {
                "nose": 0,
                "left_shoulder": 11,
                "right_shoulder": 12,
                "left_elbow": 13,
                "right_elbow": 14,
                "left_wrist": 15,
                "right_wrist": 16,
                "left_hip": 23,
                "right_hip": 24,
            }
            for name, idx in key_names.items():
                point = pose_landmarks.landmark[idx]
                pose["landmarks"][name] = {
                    "x": float(point.x),
                    "y": float(point.y),
                    "z": float(point.z),
                    "visibility": float(point.visibility),
                }
            pose["label"] = _classify_pose(pose_landmarks.landmark)
            cv2.putText(
                annotated,
                f"pose: {pose['label']}",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
            )

        path = None
        if show:
            info = self.show_image(annotated, save_path=save_path, title=f"Pose: {pose['label']}")
            path = info["path"]
        elif save_path:
            path = self._write_image(annotated, save_path=save_path)
        pose["path"] = path
        return pose

    def show_pose(
        self,
        show: bool = True,
        save_path: Optional[str] = None,
        min_detection_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
    ):
        return self.detect_pose(
            show=show,
            save_path=save_path,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )

    def recognize_pose(
        self,
        show: bool = True,
        save_path: Optional[str] = None,
        min_detection_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
    ):
        return self.detect_pose(
            show=show,
            save_path=save_path,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )

    def show_color(self, color: str, show: bool = True, save_path: Optional[str] = None, min_area: Optional[int] = None) -> Dict[str, Any]:
        return self.find_color_objects(color=color, show=show, save_path=save_path, min_area=min_area)

    def which_object(self, color: str, show: bool = True, save_path: Optional[str] = None, min_area: Optional[int] = None) -> int:
        result = self.find_color_objects(color=color, show=show, save_path=save_path, min_area=min_area)
        if not result["objects"]:
            return 0
        largest = max(result["objects"], key=lambda item: item["area"])
        print(f"{result['color']} object index: {largest['index']}")
        return int(largest["index"])

    def calibrate_color(
        self,
        color: str,
        box_size: int = 80,
        hue_pad: int = 12,
        sat_pad: int = 70,
        val_pad: int = 70,
        show: bool = True,
        save_path: Optional[str] = None,
        persist: bool = True,
    ) -> Dict[str, Any]:
        cv2, np = _require_runtime()
        name = _normalize_color_name(color)
        frame = self.capture_frame()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h, w = frame.shape[:2]
        half = max(10, int(box_size) // 2)
        cx = w // 2
        cy = h // 2
        x0 = max(0, cx - half)
        y0 = max(0, cy - half)
        x1 = min(w, cx + half)
        y1 = min(h, cy + half)
        roi = hsv[y0:y1, x0:x1]
        if roi.size == 0:
            raise RuntimeError("Calibration ROI was empty")

        median = np.median(roi.reshape(-1, 3), axis=0)
        mh, ms, mv = [int(round(v)) for v in median]
        lower = (
            mh - int(hue_pad),
            max(0, ms - int(sat_pad)),
            max(0, mv - int(val_pad)),
        )
        upper = (
            mh + int(hue_pad),
            min(255, ms + int(sat_pad)),
            min(255, mv + int(val_pad)),
        )
        ranges = _expand_hue_wrap(lower, upper)
        if persist:
            self._profiles[name] = ranges

        annotated = frame.copy()
        cv2.rectangle(annotated, (x0, y0), (x1, y1), (255, 255, 255), 2)
        cv2.putText(
            annotated,
            f"{name} HSV~{(mh, ms, mv)}",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )
        path = None
        if show:
            info = self.show_image(annotated, save_path=save_path, title=f"Calibrated {name}: {ranges}")
            path = info["path"]
        elif save_path:
            path = self._write_image(annotated, save_path=save_path)

        result = {
            "color": name,
            "sample_hsv": (mh, ms, mv),
            "ranges": ranges,
            "path": path,
            "persisted": bool(persist),
        }
        print(result)
        return result


def get_vision(
    camera_index: Optional[int] = None,
    width: int = 320,
    height: int = 240,
    warmup_s: float = 0.15,
    min_area: int = 350,
) -> Vision:
    key = "vision_lib:vision"
    inst = get_process_singleton(key)
    if inst is None:
        inst = set_process_singleton(
            key,
            Vision(
                camera_index=camera_index,
                width=width,
                height=height,
                warmup_s=warmup_s,
                min_area=min_area,
            ),
        )
    return inst


def reset_vision() -> None:
    clear_process_singleton("vision_lib:vision")


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
