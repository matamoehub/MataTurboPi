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
