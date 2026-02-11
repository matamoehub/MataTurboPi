"""
Teacher Solution Library
Lesson 2 – Completed RobotMoves wrapper

Uses base library in common/lib
"""

from pathlib import Path
import importlib.util


# -----------------------------
# Load base robot_moves library
# -----------------------------

def _load_base():
    here = Path(__file__).resolve()

    root = None
    for p in [here] + list(here.parents):
        if (p / "common" / "lib").is_dir():
            root = p
            break

    if root is None:
        raise FileNotFoundError("Could not find common/lib")

    base_path = root / "common" / "lib" / "robot_moves.py"

    spec = importlib.util.spec_from_file_location(
        "base_robot_moves", str(base_path)
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_base = _load_base()


# -----------------------------
# Completed Student API
# -----------------------------

class RobotMoves:

    def __init__(self):
        _base.set_base_speed(220)
        _base.set_rate(30)

    def forward(self, seconds):
        _base.forward(seconds)

    def backward(self, seconds):
        _base.backward(seconds)

    def left(self, seconds):
        _base.left(seconds)

    def right(self, seconds):
        _base.right(seconds)

    def spin_right(self, seconds):
        _base.turn_right(seconds)

    def drift_left(self, seconds):
        _base.drift_left(seconds)

    def drift_right(self, seconds):
        _base.drift_right(seconds)

    def stop(self):
        _base.stop()

    def horn(self):
        _base.horn(block=True)
