"""Lesson 2 header. Use in any cell: from lesson_header import *"""

from lesson_loader import setup as _setup
_setup(verbose=False)

import time
import importlib

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
    if al is not None:
        print("student_animation_lib:", getattr(al, "__file__", "<built-in>"))
        if anim is not None:
            print("anim.eyes:", "ready" if getattr(anim, "eyes", None) is not None else "unavailable")
            print("anim.camera:", "ready" if getattr(anim, "camera", None) is not None else "unavailable")
            print("anim.tts:", "ready" if getattr(anim, "tts", None) is not None else "unavailable")
    else:
        print("student_animation_lib: unavailable")
