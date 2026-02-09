"""
common/lib/robot.py
Only place that imports turbopi-fast-sdk.
"""

import time
from turbopi_fast_sdk import TurboPi

_tp = TurboPi()

def drive(lf: int, rf: int, lb: int, rb: int, secs: float = 0.2) -> None:
    _tp.set_motor_speed(int(lf), int(rf), int(lb), int(rb))
    time.sleep(float(secs))
    _tp.stop()

def stop() -> None:
    _tp.stop()

def beep(secs: float = 0.1) -> None:
    _tp.buzzer.on()
    time.sleep(float(secs))
    _tp.buzzer.off()

def say(msg: str) -> None:
    print(f"🗣️ {msg}")

