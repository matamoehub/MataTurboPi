"""Lesson 8 header. Use in any cell: from lesson_header import *"""

from lesson_loader import setup as _setup
lesson_info = _setup(verbose=False)

import time

_LOAD_ERRORS = {}

try:
    import vision_lib
    vision = vision_lib.get_vision()
except Exception as e:
    vision_lib = None
    vision = None
    _LOAD_ERRORS["vision_lib"] = str(e)

try:
    import camera_lib
    cam = camera_lib.get_camera()
except Exception as e:
    camera_lib = None
    cam = None
    _LOAD_ERRORS["camera_lib"] = str(e)

try:
    import tts_lib as tts
except Exception as e:
    tts = None
    _LOAD_ERRORS["tts_lib"] = str(e)

try:
    import eyes_lib
    eyes = eyes_lib.get_eyes()
except Exception as e:
    eyes_lib = None
    eyes = None
    _LOAD_ERRORS["eyes_lib"] = str(e)

try:
    import buzzer_lib
    bz = buzzer_lib.get_buzzer()
except Exception as e:
    buzzer_lib = None
    bz = None
    _LOAD_ERRORS["buzzer_lib"] = str(e)


def show_mediapipe_status():
    print("backend:", lesson_info.get("backend"))
    print("ros_domain_id:", lesson_info.get("ros_domain_id"))
    print("vision:", "ready" if vision is not None else f"unavailable -> {_LOAD_ERRORS.get('vision_lib')}")
    print("camera:", "ready" if cam is not None else f"unavailable -> {_LOAD_ERRORS.get('camera_lib')}")
    print("tts:", "ready" if tts is not None else f"unavailable -> {_LOAD_ERRORS.get('tts_lib')}")
    print("eyes:", "ready" if eyes is not None else f"unavailable -> {_LOAD_ERRORS.get('eyes_lib')}")
    print("buzzer:", "ready" if bz is not None else f"unavailable -> {_LOAD_ERRORS.get('buzzer_lib')}")
    if vision_lib is not None:
        print("vision_lib:", getattr(vision_lib, "__version__", "unknown"))
    if _LOAD_ERRORS:
        print("load_errors:")
        for name in sorted(_LOAD_ERRORS):
            print(f" - {name}: {_LOAD_ERRORS[name]}")
