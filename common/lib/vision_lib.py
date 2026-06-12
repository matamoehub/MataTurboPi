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
- undistort frames using camera calibration (calibration_param.npz)
- report angular offset and lateral cm in target_position()
- run YOLOv8 nano object detection
"""
__version__ = "1.3.4"

import copy
import base64
import json
import math
import os
import tempfile
import time
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

# Default calibration search paths (checked in order)
_CALIBRATION_SEARCH_PATHS = [
    Path("/opt/robot/calibration/camera_calibration.npz"),
    Path(__file__).resolve().parent.parent / "calibration" / "camera_calibration.npz",
    Path.home() / "camera_calibration.npz",
]

# YOLO model — yolov8n nano, pre-installed on the robot at a fixed path.
# Students never need to choose or specify a model.
# Override with YOLO_MODEL env var for advanced use.
_YOLO_MODEL_NAME = "yolov8n.pt"
_YOLO_MODEL_SEARCH_PATHS = [
    Path("/opt/robot/models/yolov8n.pt"),          # pre-installed by ops
    Path(__file__).resolve().parent.parent / "models" / "yolov8n.pt",  # repo copy
    Path.home() / ".config" / "Ultralytics" / "yolov8n.pt",  # ultralytics cache
]
_DEFAULT_YOLO_MODEL = os.environ.get("YOLO_MODEL", _YOLO_MODEL_NAME)

from ros_service_client import clear_process_singleton, get_process_singleton, set_process_singleton


HSVRange = Tuple[Tuple[int, int, int], Tuple[int, int, int]]
OPS_WEB_ENABLED_VALUES = {"1", "true", "yes", "on", "auto"}


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


def _ops_web_enabled() -> bool:
    value = str(os.environ.get("MATA_OPS_WEB_CAMERA", "auto")).strip().lower()
    return value in OPS_WEB_ENABLED_VALUES


def _ops_web_base() -> str:
    return os.environ.get("OPS_WEB_BASE", "http://127.0.0.1")


def _ops_web_snapshot_path() -> str:
    return os.environ.get("OPS_WEB_SNAPSHOT_PATH", "/api/vision/snapshot")


def _ops_web_timeout_s() -> float:
    try:
        return float(os.environ.get("OPS_WEB_TIMEOUT_S", "5.0"))
    except Exception:
        return 5.0


def _ops_web_snapshot_frame(
    camera_index: int = 0,
    width: int = 640,
    height: int = 480,
    mirror: bool = False,
):
    cv2, np = _require_runtime()
    url = _ops_web_base().rstrip("/") + _ops_web_snapshot_path()
    payload = {
        "camera_index": int(camera_index),
        "width": int(width),
        "height": int(height),
        "mirror": bool(mirror),
    }
    data = json.dumps(payload).encode("utf-8")
    headers = {
        "Content-Type": "application/json",
        "Accept": "application/json",
    }
    token = str(os.environ.get("ROBOT_TOKEN", "")).strip()
    if token:
        headers["Authorization"] = f"Bearer {token}"
    req = urllib.request.Request(url, data=data, headers=headers, method="POST")
    with urllib.request.urlopen(req, timeout=_ops_web_timeout_s()) as response:
        body = json.loads(response.read().decode("utf-8"))
    if not body.get("ok"):
        raise RuntimeError(f"robot_ops vision snapshot failed: {body}")
    encoded = body.get("image_jpeg_b64")
    if not encoded:
        raise RuntimeError("robot_ops vision snapshot did not include image_jpeg_b64")
    raw = base64.b64decode(encoded)
    arr = np.frombuffer(raw, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError("robot_ops vision snapshot JPEG decode failed")
    return frame


class _OpsWebVideoCapture:
    def __init__(self, source=0, api_preference=None, *args, **kwargs):
        cv2, _np = _require_runtime()
        self._cv2 = cv2
        self._source = source
        self._api_preference = api_preference
        self._width = 640
        self._height = 480
        self._direct = None
        self._use_ops_web = False
        self._opened = False

        original = getattr(cv2, "_mata_original_VideoCapture", None)
        if original is None:
            original = getattr(cv2, "VideoCapture")

        try:
            if api_preference is None:
                self._direct = original(source, *args, **kwargs)
            else:
                self._direct = original(source, api_preference, *args, **kwargs)
            self._opened = bool(self._direct.isOpened())
        except Exception:
            self._direct = None
            self._opened = False

        if not self._opened and _ops_web_enabled() and self._is_supported_source(source):
            self._use_ops_web = True
            self._opened = True

    @staticmethod
    def _is_supported_source(source) -> bool:
        if isinstance(source, int):
            return source >= 0
        if isinstance(source, str) and source.startswith("/dev/video"):
            return source.removeprefix("/dev/video").isdigit()
        return False

    def _camera_index(self) -> int:
        if isinstance(self._source, int):
            return int(self._source)
        if isinstance(self._source, str) and self._source.startswith("/dev/video"):
            try:
                return int(self._source.removeprefix("/dev/video"))
            except Exception:
                return 0
        return 0

    def isOpened(self):
        return bool(self._opened)

    def read(self):
        if self._use_ops_web:
            try:
                frame = _ops_web_snapshot_frame(
                    camera_index=self._camera_index(),
                    width=int(self._width),
                    height=int(self._height),
                    mirror=False,
                )
                return True, frame
            except Exception:
                return False, None
        if self._direct is None:
            return False, None
        return self._direct.read()

    def release(self):
        if self._direct is not None:
            try:
                return self._direct.release()
            except Exception:
                return None
        return None

    def set(self, prop_id, value):
        if int(prop_id) == int(getattr(self._cv2, "CAP_PROP_FRAME_WIDTH", -1)):
            self._width = int(value)
        elif int(prop_id) == int(getattr(self._cv2, "CAP_PROP_FRAME_HEIGHT", -1)):
            self._height = int(value)
        if self._direct is not None:
            try:
                return self._direct.set(prop_id, value)
            except Exception:
                return False
        return True

    def get(self, prop_id):
        if int(prop_id) == int(getattr(self._cv2, "CAP_PROP_FRAME_WIDTH", -1)):
            return float(self._width)
        if int(prop_id) == int(getattr(self._cv2, "CAP_PROP_FRAME_HEIGHT", -1)):
            return float(self._height)
        if self._direct is not None:
            try:
                return self._direct.get(prop_id)
            except Exception:
                return 0.0
        return 0.0

    def __getattr__(self, name: str):
        if self._direct is None:
            raise AttributeError(name)
        return getattr(self._direct, name)


class _OpsWebFrameCapture:
    def __init__(self, camera_index: int = 0, width: int = 640, height: int = 480):
        self.camera_index = int(camera_index)
        self.width = int(width)
        self.height = int(height)

    def isOpened(self):
        return True

    def read(self):
        try:
            frame = _ops_web_snapshot_frame(
                camera_index=self.camera_index,
                width=self.width,
                height=self.height,
                mirror=False,
            )
            return True, frame
        except Exception:
            return False, None

    def release(self):
        return None

    def set(self, prop_id, value):
        cv2, _np = _require_runtime()
        if int(prop_id) == int(getattr(cv2, "CAP_PROP_FRAME_WIDTH", -1)):
            self.width = int(value)
        elif int(prop_id) == int(getattr(cv2, "CAP_PROP_FRAME_HEIGHT", -1)):
            self.height = int(value)
        return True


def install_opencv_capture_fallback() -> bool:
    if not _ops_web_enabled():
        return False
    try:
        import cv2  # type: ignore
    except Exception:
        return False
    if getattr(cv2, "_mata_ops_web_capture_installed", False):
        return True
    cv2._mata_original_VideoCapture = cv2.VideoCapture
    cv2.VideoCapture = _OpsWebVideoCapture
    cv2._mata_ops_web_capture_installed = True
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
        skip_frames: int = 3,
    ):
        self.camera_index = int(os.environ.get("CAM_INDEX", camera_index if camera_index is not None else 0))
        self.width = int(width)
        self.height = int(height)
        self.warmup_s = float(warmup_s)
        self.min_area = int(min_area)
        # Number of frames to read and discard after opening the camera.
        # V4L2 buffers stale frames from the moment VideoCapture is created —
        # the first read() returns an old frame, not the current scene.
        # 3 discards clears the buffer reliably on all tested USB cameras.
        # Override with CAM_SKIP_FRAMES env var if your camera needs more/fewer.
        self.skip_frames = int(os.environ.get("CAM_SKIP_FRAMES", skip_frames))
        self._profiles: Dict[str, List[HSVRange]] = copy.deepcopy(DEFAULT_COLOR_PROFILES)
        # Camera calibration — loaded lazily on first use or explicit call.
        self._cal_K: Optional[Any] = None    # 3x3 camera matrix (numpy)
        self._cal_D: Optional[Any] = None    # distortion coefficients
        self._cal_dim: Optional[Tuple[int,int]] = None  # (w, h) calibration was made at
        self._cal_map1: Optional[Any] = None  # precomputed undistort map1
        self._cal_map2: Optional[Any] = None  # precomputed undistort map2
        self._yolo_model: Optional[Any] = None  # loaded YOLO model (lazy)

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
        direct_error = None
        cap = None
        try:
            cap = self._open_capture()
            # Drain stale V4L2 buffer frames — the first N reads after open
            # return buffered-up old frames, not the current scene. This is
            # why the camera fails 1-in-4 calls from Jupyter: the buffer is
            # stale and cap.read() returns an empty or black frame.
            for _ in range(self.skip_frames):
                cap.read()
            ok, frame = cap.read()
            if ok and frame is not None:
                return frame
            direct_error = "Camera opened, but no image frame was captured"
        except Exception as e:
            direct_error = str(e)
        finally:
            try:
                if cap is not None:
                    cap.release()
            except Exception:
                pass

        if _ops_web_enabled():
            try:
                return _ops_web_snapshot_frame(
                    camera_index=self.camera_index,
                    width=max(160, int(self.width)),
                    height=max(120, int(self.height)),
                    mirror=False,
                )
            except Exception as e:
                raise RuntimeError(
                    "Camera capture failed through OpenCV and robot_ops web. "
                    f"OpenCV error: {direct_error}. robot_ops error: {e}"
                ) from e

        raise RuntimeError(direct_error or "Camera capture failed")

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
        frame_h, frame_w = frame.shape[:2]

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
            "width": int(frame_w),
            "height": int(frame_h),
            "center_x": int(frame_w // 2),
        }

    # ── Camera calibration ────────────────────────────────────────────────────

    def load_calibration(self, path: Optional[str] = None) -> bool:
        """Load camera calibration from an npz file (calibration_param.npz).

        Searches default paths if path is not given:
          /opt/robot/calibration/camera_calibration.npz
          common/calibration/camera_calibration.npz  (repo)
          ~/camera_calibration.npz

        Returns True if calibration was loaded successfully.
        """
        cv2, np = _require_runtime()
        candidates = [Path(path)] if path else _CALIBRATION_SEARCH_PATHS
        for p in candidates:
            if not p.exists():
                continue
            try:
                data = np.load(str(p))
                K = data["k_array"]           # 3×3 camera matrix
                D = data["d_array"].flatten() # distortion coefficients
                dim = tuple(int(v) for v in data["dim_array"])  # (w, h)
                w, h = dim
                # Precompute undistort maps for fast per-frame use.
                new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
                map1, map2 = cv2.initUndistortRectifyMap(
                    K, D, None, new_K, (w, h), cv2.CV_16SC2
                )
                self._cal_K   = new_K
                self._cal_D   = D
                self._cal_dim = (w, h)
                self._cal_map1 = map1
                self._cal_map2 = map2
                print(f"[vision_lib] calibration loaded: {p.name}  "
                      f"fx={new_K[0,0]:.1f} fy={new_K[1,1]:.1f} "
                      f"cx={new_K[0,2]:.1f} cy={new_K[1,2]:.1f}")
                return True
            except Exception as e:
                print(f"[vision_lib] calibration load failed ({p}): {e}")
        print("[vision_lib] no calibration file found — angular offset will use frame-centre estimate")
        return False

    def _ensure_calibration(self) -> bool:
        """Load calibration from default paths if not already loaded."""
        if self._cal_K is not None:
            return True
        return self.load_calibration()

    def undistort_frame(self, frame: Any) -> Any:
        """Undistort a frame using the loaded camera calibration.

        Loads calibration automatically from default paths if needed.
        Returns the original frame unchanged if calibration is unavailable.
        """
        if not self._ensure_calibration():
            return frame
        cv2, _np = _require_runtime()
        return cv2.remap(frame, self._cal_map1, self._cal_map2, cv2.INTER_LINEAR)

    def pixel_to_angle(self, pixel_x: float, pixel_y: Optional[float] = None) -> Dict[str, float]:
        """Convert pixel coordinates to angular offset from camera centre.

        Uses camera calibration when available, otherwise falls back to a
        reasonable estimate based on frame size.

        Returns {"angle_x_deg": float, "angle_y_deg": float}
          Positive angle_x = object is to the RIGHT of centre.
          Positive angle_y = object is BELOW centre.
        """
        if self._ensure_calibration() and self._cal_K is not None:
            fx = float(self._cal_K[0, 0])
            fy = float(self._cal_K[1, 1])
            cx = float(self._cal_K[0, 2])
            cy = float(self._cal_K[1, 2])
        else:
            # Fallback: assume ~60° FOV for a typical USB webcam.
            fx = fy = self.width / (2 * math.tan(math.radians(30)))
            cx = self.width / 2.0
            cy = self.height / 2.0

        angle_x = math.degrees(math.atan2(float(pixel_x) - cx, fx))
        angle_y = math.degrees(math.atan2(float(pixel_y) - cy, fy)) if pixel_y is not None else 0.0
        return {"angle_x_deg": round(angle_x, 2), "angle_y_deg": round(angle_y, 2)}

    def estimate_lateral_cm(
        self,
        pixel_cx: float,
        pixel_width: float,
        object_diameter_cm: float = 6.5,
    ) -> Optional[float]:
        """Estimate lateral distance (cm) of an object from camera centre.

        Uses the known real-world diameter of the object and its pixel width to
        estimate depth, then converts the pixel offset to cm.

        pixel_cx         — object centre x in the frame
        pixel_width      — object bounding-box width in pixels
        object_diameter_cm — real diameter in cm (default 6.5 cm for a standard football)

        Returns lateral cm (positive = right, negative = left), or None if
        calibration is unavailable or pixel_width is zero.
        """
        if pixel_width <= 0:
            return None
        if not self._ensure_calibration() or self._cal_K is None:
            return None
        fx = float(self._cal_K[0, 0])
        cx = float(self._cal_K[0, 2])
        # Depth from apparent size: Z = fx * D_real / D_pixels
        depth_cm = fx * float(object_diameter_cm) / float(pixel_width)
        # Lateral distance: X = (px - cx) / fx * Z
        lateral_cm = (float(pixel_cx) - cx) / fx * depth_cm
        return round(lateral_cm, 1)

    # ── YOLO object detection ─────────────────────────────────────────────────

    def _ensure_yolo(self) -> Any:
        """Load YOLOv8 nano model lazily.

        Checks pre-installed paths first (/opt/robot/models/yolov8n.pt),
        then falls back to ultralytics auto-download.
        Students never need to specify a model — it is always yolov8n.
        """
        if self._yolo_model is not None:
            return self._yolo_model
        # ultralytics imports matplotlib.pyplot on load which triggers the
        # Jupyter inline backend and crashes with a version mismatch:
        #   AttributeError: 'RcParams' object has no attribute '_get'
        # Force a non-interactive backend before the import to avoid this.
        try:
            import matplotlib
            matplotlib.use("Agg")
        except Exception:
            pass
        try:
            from ultralytics import YOLO  # type: ignore
        except ImportError:
            raise RuntimeError(
                "YOLO requires the ultralytics package. "
                "Ask ops to run: pip install ultralytics"
            )
        # Use pre-installed model file if available — avoids internet dependency
        for p in _YOLO_MODEL_SEARCH_PATHS:
            if p.exists():
                self._yolo_model = YOLO(str(p))
                print(f"[vision_lib] YOLO nano loaded from {p}")
                return self._yolo_model
        # Not pre-installed — download (requires internet, first run only)
        print("[vision_lib] downloading yolov8n.pt (first use only)...")
        self._yolo_model = YOLO(_YOLO_MODEL_NAME)
        print("[vision_lib] YOLO nano ready")
        return self._yolo_model

    def detect_objects_yolo(
        self,
        conf: float = 0.5,
        show: bool = True,
        save_path: Optional[str] = None,
        classes: Optional[List[int]] = None,
        object_diameter_cm: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Run YOLOv8 nano object detection on a captured frame.

        Always uses the pre-installed yolov8n model — students do not
        need to choose or download a model.

        Args:
            conf:               Confidence threshold 0-1. Lower = more detections.
            show:               Display annotated frame in Jupyter.
            save_path:          Optional path to save annotated image.
            classes:            Filter to specific COCO class IDs (None = all).
                                e.g. classes=[32] for sports ball only.
            object_diameter_cm: If given, estimates lateral cm for each object.

        Returns dict with:
            found       bool
            count       int
            objects     list of {label, confidence, x, y, w, h, cx, cy,
                                  angle_x_deg, lateral_cm (if calibrated)}
            path        saved image path or None
        """
        cv2, np = _require_runtime()
        yolo = self._ensure_yolo()
        frame = self.capture_frame()

        results = yolo(frame, conf=conf, classes=classes, verbose=False)

        if self._ensure_calibration() and self._cal_K is not None:
            fx = float(self._cal_K[0, 0])
            cx_cam = float(self._cal_K[0, 2])
        else:
            fx = self.width / (2 * math.tan(math.radians(30)))
            cx_cam = self.width / 2.0

        objects = []
        annotated = frame.copy()
        for result in results:
            # Use YOLO's built-in plot() for professional annotated frame
            # (per-class colours, background labels, proper styling)
            annotated = result.plot()

            boxes = result.boxes
            names = result.names
            if boxes is None:
                continue
            for box in boxes:
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
                w = x2 - x1
                h_box = y2 - y1
                cx_obj = x1 + w // 2
                cy_obj = y1 + h_box // 2
                cls_id = int(box.cls[0])
                label  = names[cls_id] if names else str(cls_id)
                conf_v = float(box.conf[0])

                angle_x = math.degrees(math.atan2(cx_obj - cx_cam, fx))
                obj: Dict[str, Any] = {
                    "label":       label,
                    "class_id":    cls_id,
                    "confidence":  round(conf_v, 3),
                    "x": x1, "y": y1, "w": w, "h": h_box,
                    "cx": cx_obj, "cy": cy_obj,
                    "angle_x_deg": round(angle_x, 2),
                    "lateral_cm":  None,
                }
                if object_diameter_cm is not None and w > 0:
                    obj["lateral_cm"] = self.estimate_lateral_cm(
                        cx_obj, w, object_diameter_cm
                    )
                objects.append(obj)

        path = None
        if show:
            info = self.show_image(annotated, save_path=save_path,
                                   title=f"YOLO: {len(objects)} objects detected")
            path = info["path"]
        elif save_path:
            path = self._write_image(annotated, save_path=save_path)

        return {
            "found":   bool(objects),
            "count":   len(objects),
            "objects": objects,
            "path":    path,
        }

    def target_position(
        self,
        color: str,
        target_x: Optional[int] = None,
        deadzone: int = 50,
        show: bool = True,
        min_area: Optional[int] = None,
    ) -> Dict[str, Any]:
        """Find the largest colour object and return its direction from centre.

        Original API — unchanged for backward compatibility.
        For angular offset and lateral cm use locate_object() instead.

        Returns: found, direction ("left"|"center"|"right"|"lost"),
                 error (pixels from centre), target_x, deadzone, object, result.
        """
        result = self.find_color_objects(color=color, show=show, min_area=min_area)
        objects = result["objects"]
        centre_x = int(result["center_x"] if target_x is None else target_x)
        threshold = abs(int(deadzone))

        if not objects:
            return {
                "color":     result["color"],
                "found":     False,
                "direction": "lost",
                "error":     None,
                "target_x":  centre_x,
                "deadzone":  threshold,
                "object":    None,
                "result":    result,
            }

        target = max(objects, key=lambda item: item["area"])
        error = int(target["cx"] - centre_x)
        if abs(error) <= threshold:
            direction = "center"
        elif error < 0:
            direction = "left"
        else:
            direction = "right"

        return {
            "color":     result["color"],
            "found":     True,
            "direction": direction,
            "error":     error,
            "target_x":  centre_x,
            "deadzone":  threshold,
            "object":    target,
            "result":    result,
        }

    def locate_object(
        self,
        color: str,
        target_x: Optional[int] = None,
        deadzone: int = 50,
        show: bool = True,
        min_area: Optional[int] = None,
        object_diameter_cm: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Find the largest colour object with real-world position data.

        Enhanced version of target_position() — adds angular offset and
        optional lateral distance. Uses camera calibration automatically.

        Args:
            color:               Colour name to detect ("red", "green", "blue").
            deadzone:            Pixel half-width of the "center" zone.
            object_diameter_cm:  Real diameter of the object in cm.
                                 If given, lateral_cm is estimated from
                                 the object's pixel width + calibration.
                                 e.g. 6.5 for a standard football.

        Returns:
            direction     "left" | "center" | "right" | "lost"  (same as target_position)
            found         bool
            error         pixel offset from centre (+ = right, - = left)
            error_norm    normalised -1.0..1.0 (resolution-independent, no depth needed)
            angle_x_deg   lateral angle in degrees — positive = right of centre
                          Uses calibration when loaded; falls back to FOV estimate.
            lateral_cm    lateral distance in cm — only if object_diameter_cm given
                          AND calibration is loaded; otherwise None.
            object        largest detected object dict (x, y, w, h, cx, cy, area)
        """
        result = self.find_color_objects(color=color, show=show, min_area=min_area)
        objects = result["objects"]
        centre_x = int(result["center_x"] if target_x is None else target_x)
        threshold = abs(int(deadzone))

        if not objects:
            return {
                "color":       result["color"],
                "found":       False,
                "direction":   "lost",
                "error":       None,
                "error_norm":  None,
                "angle_x_deg": None,
                "lateral_cm":  None,
                "target_x":    centre_x,
                "deadzone":    threshold,
                "object":      None,
                "result":      result,
            }

        target = max(objects, key=lambda item: item["area"])
        error = int(target["cx"] - centre_x)

        # Normalised: -1.0 (far left) to +1.0 (far right)
        frame_w = result.get("width", self.width) or self.width
        error_norm = round(error / max(1, frame_w / 2), 3)

        # Angular offset — uses calibration if available
        angles = self.pixel_to_angle(target["cx"], target["cy"])
        angle_x_deg = angles["angle_x_deg"]

        # Lateral cm — only when real object size is known
        lateral_cm: Optional[float] = None
        if object_diameter_cm is not None:
            lateral_cm = self.estimate_lateral_cm(
                target["cx"],
                target.get("w", 0),
                object_diameter_cm,
            )

        if abs(error) <= threshold:
            direction = "center"
        elif error < 0:
            direction = "left"
        else:
            direction = "right"

        return {
            "color":       result["color"],
            "found":       True,
            "direction":   direction,
            "error":       error,
            "error_norm":  error_norm,
            "angle_x_deg": angle_x_deg,
            "lateral_cm":  lateral_cm,
            "target_x":    centre_x,
            "deadzone":    threshold,
            "object":      target,
            "result":      result,
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
                "game_move": _gesture_to_game_move(gesture),
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

        # Keep one entry per detected hand (None when the gesture didn't map to
        # a rock/paper/scissors move) so game_moves[0] is safe whenever a hand
        # was seen.  first_move is None-safe even when no hand was detected.
        game_moves = [hand.get("game_move") for hand in hands_found]
        return {
            "found": bool(hands_found),
            "count": len(hands_found),
            "hands": hands_found,
            "game_moves": game_moves,
            "first_move": game_moves[0] if game_moves else None,
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
