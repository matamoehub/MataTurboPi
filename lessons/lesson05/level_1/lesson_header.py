"""Lesson 5 header. Use in any cell: from lesson_header import *"""

from lesson_loader import setup as _setup
lesson_info = _setup(verbose=False)

import importlib
import time

_LOAD_ERRORS = {}

import robot_moves as rm

if hasattr(rm, "RobotMoves"):
    bot = rm.RobotMoves(base_speed=300, rate_hz=20)
else:
    bot = rm

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

try:
    import infrared_lib
    ir = infrared_lib.get_infrared()
except Exception as e:
    infrared_lib = None
    ir = None
    _LOAD_ERRORS["infrared_lib"] = str(e)

try:
    import line_follower_lib as lfl
except Exception as e:
    lfl = None
    _LOAD_ERRORS["line_follower_lib"] = str(e)

try:
    line_pid = lfl.LineFollower(infrared=ir) if lfl is not None and ir is not None else None
except Exception as e:
    line_pid = None
    _LOAD_ERRORS["line_pid"] = str(e)

try:
    ros_line = lfl.get_ros_line_follower() if lfl is not None and hasattr(lfl, "get_ros_line_follower") else None
except Exception as e:
    ros_line = None
    _LOAD_ERRORS["ros_line"] = str(e)

try:
    import avoidance_lib
    avoid = avoidance_lib.get_avoidance()
except Exception as e:
    avoidance_lib = None
    avoid = None
    _LOAD_ERRORS["avoidance_lib"] = str(e)

try:
    import tracking_lib
    track = tracking_lib.get_tracking()
except Exception as e:
    tracking_lib = None
    track = None
    _LOAD_ERRORS["tracking_lib"] = str(e)

try:
    import qrcode_lib
    qr = qrcode_lib.get_qrcode()
except Exception as e:
    qrcode_lib = None
    qr = None
    _LOAD_ERRORS["qrcode_lib"] = str(e)

try:
    import student_animation_lib as al
    importlib.reload(al)
except Exception as e:
    al = None
    _LOAD_ERRORS["student_animation_lib"] = str(e)

try:
    anim = al.get_animation_lib(robot=bot, eyes=eyes, camera=cam, tts=tts, base_speed=300) if al else None
except Exception as e:
    anim = None
    _LOAD_ERRORS["animation"] = str(e)


def show_lesson_status():
    print("backend:", lesson_info.get("backend"))
    print("ros_domain_id:", lesson_info.get("ros_domain_id"))
    print("robot_moves:", getattr(rm, "__file__", "<built-in>"))
    print("bot type:", type(bot).__name__)
    print("eyes:", "ready" if eyes is not None else f"unavailable -> {_LOAD_ERRORS.get('eyes_lib')}")
    print("camera:", "ready" if cam is not None else f"unavailable -> {_LOAD_ERRORS.get('camera_lib')}")
    print("tts:", "ready" if tts is not None else f"unavailable -> {_LOAD_ERRORS.get('tts_lib')}")
    print("buzzer:", "ready" if bz is not None else f"unavailable -> {_LOAD_ERRORS.get('buzzer_lib')}")
    print("sonar:", "ready" if sonar is not None else f"unavailable -> {_LOAD_ERRORS.get('sonar_lib')}")
    print("ultrasonic alias:", "ready" if ultra is not None else f"unavailable -> {_LOAD_ERRORS.get('ultrasonic_lib')}")
    print("infrared:", "ready" if ir is not None else f"unavailable -> {_LOAD_ERRORS.get('infrared_lib')}")
    print("line follower pid:", "ready" if line_pid is not None else f"unavailable -> {_LOAD_ERRORS.get('line_pid')}")
    print("line follower ros:", "ready" if ros_line is not None else f"unavailable -> {_LOAD_ERRORS.get('ros_line')}")
    print("avoidance:", "ready" if avoid is not None else f"unavailable -> {_LOAD_ERRORS.get('avoidance_lib')}")
    print("tracking:", "ready" if track is not None else f"unavailable -> {_LOAD_ERRORS.get('tracking_lib')}")
    print("qrcode:", "ready" if qr is not None else f"unavailable -> {_LOAD_ERRORS.get('qrcode_lib')}")
    print("animation:", "ready" if anim is not None else f"unavailable -> {_LOAD_ERRORS.get('animation')}")
    if _LOAD_ERRORS:
        print("load_errors:")
        for name in sorted(_LOAD_ERRORS):
            print(f" - {name}: {_LOAD_ERRORS[name]}")


def stop_everything():
    try:
        if anim is not None:
            anim.stop()
    except Exception:
        pass
    try:
        if hasattr(bot, "stop"):
            bot.stop()
        elif hasattr(rm, "stop"):
            rm.stop()
    except Exception:
        pass
