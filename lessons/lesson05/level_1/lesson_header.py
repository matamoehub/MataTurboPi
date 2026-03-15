"""Lesson 5 header. Use in any cell: from lesson_header import *"""

from lesson_loader import setup as _setup
_setup(verbose=False)

import importlib
import time

import robot_moves as rm

if hasattr(rm, "RobotMoves"):
    bot = rm.RobotMoves(base_speed=300, rate_hz=20)
else:
    bot = rm

try:
    import eyes_lib
    eyes = eyes_lib.get_eyes()
except Exception:
    eyes_lib = None
    eyes = None

try:
    import camera_lib
    cam = camera_lib.get_camera()
except Exception:
    camera_lib = None
    cam = None

try:
    import tts_lib as tts
except Exception:
    tts = None

try:
    import buzzer_lib
    bz = buzzer_lib.get_buzzer()
except Exception:
    buzzer_lib = None
    bz = None

try:
    import sonar_lib
    sonar = sonar_lib.get_sonar()
except Exception:
    sonar_lib = None
    sonar = None

try:
    import ultrasonic_lib
    ultra = ultrasonic_lib.get_ultrasonic()
except Exception:
    ultrasonic_lib = None
    ultra = None

try:
    import infrared_lib
    ir = infrared_lib.get_infrared()
except Exception:
    infrared_lib = None
    ir = None

try:
    import line_follower_lib as lfl
except Exception:
    lfl = None

try:
    line_pid = lfl.LineFollower(infrared=ir) if lfl is not None and ir is not None else None
except Exception:
    line_pid = None

try:
    ros_line = lfl.get_ros_line_follower() if lfl is not None and hasattr(lfl, "get_ros_line_follower") else None
except Exception:
    ros_line = None

try:
    import avoidance_lib
    avoid = avoidance_lib.get_avoidance()
except Exception:
    avoidance_lib = None
    avoid = None

try:
    import tracking_lib
    track = tracking_lib.get_tracking()
except Exception:
    tracking_lib = None
    track = None

try:
    import qrcode_lib
    qr = qrcode_lib.get_qrcode()
except Exception:
    qrcode_lib = None
    qr = None

try:
    import student_animation_lib as al
    importlib.reload(al)
except Exception:
    al = None

anim = al.get_animation_lib(robot=bot, eyes=eyes, camera=cam, tts=tts, base_speed=300) if al else None


def show_lesson_status():
    print("robot_moves:", getattr(rm, "__file__", "<built-in>"))
    print("bot type:", type(bot).__name__)
    print("eyes:", "ready" if eyes is not None else "unavailable")
    print("camera:", "ready" if cam is not None else "unavailable")
    print("tts:", "ready" if tts is not None else "unavailable")
    print("buzzer:", "ready" if bz is not None else "unavailable")
    print("sonar:", "ready" if sonar is not None else "unavailable")
    print("ultrasonic alias:", "ready" if ultra is not None else "unavailable")
    print("infrared:", "ready" if ir is not None else "unavailable")
    print("line follower pid:", "ready" if line_pid is not None else "unavailable")
    print("line follower ros:", "ready" if ros_line is not None else "unavailable")
    print("avoidance:", "ready" if avoid is not None else "unavailable")
    print("tracking:", "ready" if track is not None else "unavailable")
    print("qrcode:", "ready" if qr is not None else "unavailable")
    print("animation:", "ready" if anim is not None else "unavailable")


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
