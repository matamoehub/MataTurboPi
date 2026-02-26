"""Lesson 4 header. Use in any cell: from lesson_header import *"""

from lesson_loader import setup as _setup
_setup(verbose=False)

import time
import random
import importlib

try:
    import tts_lib as tts
except Exception:
    tts = None
