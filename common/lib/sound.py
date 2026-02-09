"""
common/lib/sound.py
Play mp3 sound effects from common/sounds/.

Notes:
- 'aplay' does NOT play mp3.
- We try mpg123, then ffplay as a fallback.
"""

import os
import shutil
import subprocess

SOUND_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "sounds"))

def _cmd_exists(name: str) -> bool:
    return shutil.which(name) is not None

def play(filename: str) -> None:
    path = os.path.join(SOUND_DIR, filename)
    if not os.path.exists(path):
        raise FileNotFoundError(path)

    # Prefer mpg123 for mp3
    if _cmd_exists("mpg123"):
        subprocess.Popen(["mpg123", "-q", path])
        return

    # Fallback: ffplay (often present with ffmpeg)
    if _cmd_exists("ffplay"):
        subprocess.Popen(["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", path])
        return

    raise RuntimeError("No mp3 player found. Install 'mpg123' (recommended) or 'ffmpeg' (ffplay).")

def horn() -> None:
    play("horn.mp3")

def dixie() -> None:
    play("dixie.mp3")

def meepmeep() -> None:
    play("meepmeep.mp3")

