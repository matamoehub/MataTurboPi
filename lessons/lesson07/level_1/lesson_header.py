"""Lesson 7 header. Use in any cell: from lesson_header import *"""

from lesson_loader import setup as _setup
lesson_info = _setup(verbose=False)

import time

_LOAD_ERRORS = {}

try:
    import student_robot_moves as moves
except Exception as e:
    moves = None
    _LOAD_ERRORS["student_robot_moves"] = str(e)

try:
    import eyes_lib
    eyes = eyes_lib.get_eyes()
except Exception as e:
    eyes_lib = None
    eyes = None
    _LOAD_ERRORS["eyes_lib"] = str(e)

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
    import buzzer_lib
    bz = buzzer_lib.get_buzzer()
except Exception as e:
    buzzer_lib = None
    bz = None
    _LOAD_ERRORS["buzzer_lib"] = str(e)

try:
    import sonar_lib
    sonar = sonar_lib.get_sonar()
except Exception as e:
    sonar_lib = None
    sonar = None
    _LOAD_ERRORS["sonar_lib"] = str(e)

try:
    import ultrasonic_lib
    ultra = ultrasonic_lib.get_ultrasonic()
except Exception as e:
    ultrasonic_lib = None
    ultra = None
    _LOAD_ERRORS["ultrasonic_lib"] = str(e)


def show_advanced_project_status():
    print("backend:", lesson_info.get("backend"))
    print("ros_domain_id:", lesson_info.get("ros_domain_id"))
    print("student_robot_moves:", "ready" if moves is not None else f"unavailable -> {_LOAD_ERRORS.get('student_robot_moves')}")
    print("eyes:", "ready" if eyes is not None else f"unavailable -> {_LOAD_ERRORS.get('eyes_lib')}")
    print("camera:", "ready" if cam is not None else f"unavailable -> {_LOAD_ERRORS.get('camera_lib')}")
    print("tts:", "ready" if tts is not None else f"unavailable -> {_LOAD_ERRORS.get('tts_lib')}")
    print("buzzer:", "ready" if bz is not None else f"unavailable -> {_LOAD_ERRORS.get('buzzer_lib')}")
    print("sonar:", "ready" if sonar is not None else f"unavailable -> {_LOAD_ERRORS.get('sonar_lib')}")
    print("ultrasonic alias:", "ready" if ultra is not None else f"unavailable -> {_LOAD_ERRORS.get('ultrasonic_lib')}")
    if _LOAD_ERRORS:
        print("load_errors:")
        for name in sorted(_LOAD_ERRORS):
            print(f" - {name}: {_LOAD_ERRORS[name]}")


def stop_project_robot():
    try:
        if moves is not None and hasattr(moves, "base") and hasattr(moves.base, "stop"):
            moves.base.stop()
    except Exception:
        pass
