"""Lesson 2 header. Use in any cell: from lesson_header import *"""

from lesson_loader import setup as _setup
_setup(verbose=False)

import time
import importlib

try:
    import student_animation_lib
except Exception:
    student_animation_lib = None

anim = student_animation_lib.get_animation_lib() if student_animation_lib else None
