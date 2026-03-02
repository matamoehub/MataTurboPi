"""Lesson 2 header. Use in any cell: from lesson_header import *"""

from lesson_loader import setup as _setup
_setup(verbose=False)

import time
import importlib

try:
    from fast_hi_wonder import InfraredSensors
except Exception:
    InfraredSensors = None

try:
    from lesson01.level_1 import moves
except Exception:
    moves = None

try:
    import student_animation_lib as al
    importlib.reload(al)
except Exception:
    al = None

anim = al.get_animation_lib(base_speed=300) if al else None
