#!/usr/bin/env python3
# robot_moves.py — cmd_vel moves (mecanum) + drift helpers + horn (mpg123)

from __future__ import annotations
from typing import Optional
import os, sys, time, atexit, subprocess
from pathlib import Path

from robot_controller_api import RobotController

# ---------------------------- Settings ----------------------------

DEFAULT_LINEAR  = float(os.environ.get("RM_LINEAR", "0.18"))   # m/s-ish
DEFAULT_STRAFE  = float(os.environ.get("RM_STRAFE", "0.18"))
DEFAULT_TURN    = float(os.environ.get("RM_TURN",   "0.90"))  # rad/s-ish
DEFAULT_RATE_HZ = float(os.environ.get("RM_RATE_HZ", "20"))

_CMD_VEL_TOPIC  = os.environ.get("RM_CMD_VEL_TOPIC", "/cmd_vel")

# drift tuning (still cmd_vel)
_DRIFT_RATE_HZ  = float(os.environ.get("DRIFT_RATE_HZ", "20"))
_DRIFT_GAP      = float(os.environ.get("DRIFT_GAP", "0.15"))
_DRIFT_VY_LEFT  = float(os.environ.get("DRIFT_VY_LEFT",  "-0.20"))
_DRIFT_WZ_LEFT  = float(os.environ.get("DRIFT_WZ_LEFT",   "0.50"))
_DRIFT_VY_RIGHT = float(os.environ.get("DRIFT_VY_RIGHT",   "0.20"))
_DRIFT_WZ_RIGHT = float(os.environ.get("DRIFT_WZ_RIGHT",  "-0.50"))
_DRIFT_VX_BIAS  = float(os.environ.get("DRIFT_VX_BIAS", "0.00"))

# ---------------------------- Logging ----------------------------

def _log(*a):
    print("[robot_moves]", *a, file=sys.stderr, flush=True)

# ---------------------------- Singleton controller ----------------------------

_CTRL: Optional[RobotController] = None

def get_controller() -> RobotController:
    global _CTRL
    if _CTRL is None:
        _CTRL = RobotController(cmd_vel_topic=_CMD_VEL_TOPIC, flush_spin=False, flush_sleep_s=0.02)
        atexit.register(stop)
        _log("ready -> cmd_vel:", _CMD_VEL_TOPIC)
    return _CTRL

# ---------------------------- Core publish/hold ----------------------------

def _require_subscriber(ctrl: RobotController):
    # publisher object lives inside the Node; rclpy publisher has get_subscription_count()
    subs = ctrl._cmd_pub.get_subscription_count()
    if subs < 1:
        raise RuntimeError(f"No subscriber on {ctrl.cmd_vel_topic}. Is mecanum_chassis_node running?")

def drive(vx: float = 0.0, vy: float = 0.0, wz: float = 0.0):
    ctrl = get_controller()
    _require_subscriber(ctrl)
    ctrl.publish_cmd_vel(vx, vy, wz)

def drive_for(seconds: float, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0, rate_hz: float = DEFAULT_RATE_HZ):
    ctrl = get_controller()
    _require_subscriber(ctrl)
    end = time.time() + float(seconds)
    dt = 1.0 / float(rate_hz)
    while time.time() < end:
        ctrl.publish_cmd_vel(vx, vy, wz)
        time.sleep(dt)
    ctrl.stop()

def stop():
    global _CTRL
    if _CTRL is not None:
        try:
            _CTRL.stop()
        except Exception:
            pass

def emergency_stop():
    stop()

# ---------------------------- Student-friendly moves ----------------------------

def forward(seconds: float = 0.5, speed: float = DEFAULT_LINEAR):
    drive_for(seconds, vx=+float(speed), vy=0.0, wz=0.0)

def backward(seconds: float = 0.5, speed: float = DEFAULT_LINEAR):
    drive_for(seconds, vx=-float(speed), vy=0.0, wz=0.0)

def left(seconds: float = 0.5, speed: float = DEFAULT_STRAFE):
    # left strafe is negative y in many ROS setups; if yours is reversed, flip sign here
    drive_for(seconds, vx=0.0, vy=-float(speed), wz=0.0)

def right(seconds: float = 0.5, speed: float = DEFAULT_STRAFE):
    drive_for(seconds, vx=0.0, vy=+float(speed), wz=0.0)

def turn_left(seconds: float = 0.5, speed: float = DEFAULT_TURN):
    drive_for(seconds, vx=0.0, vy=0.0, wz=+float(speed))

def turn_right(seconds: float = 0.5, speed: float = DEFAULT_TURN):
    drive_for(seconds, vx=0.0, vy=0.0, wz=-float(speed))

def diagonal_left(seconds: float = 0.8, speed: float = DEFAULT_LINEAR):
    s = float(speed)
    drive_for(seconds, vx=+s, vy=-s, wz=0.0)

def diagonal_right(seconds: float = 0.8, speed: float = DEFAULT_LINEAR):
    s = float(speed)
    drive_for(seconds, vx=+s, vy=+s, wz=0.0)

# ---------------------------- Drift helpers ----------------------------

def drift_left(seconds: float = 1.2):
    drive_for(seconds, vx=_DRIFT_VX_BIAS, vy=_DRIFT_VY_LEFT, wz=_DRIFT_WZ_LEFT, rate_hz=_DRIFT_RATE_HZ)
    if _DRIFT_GAP > 0:
        time.sleep(_DRIFT_GAP)

def drift_right(seconds: float = 1.2):
    drive_for(seconds, vx=_DRIFT_VX_BIAS, vy=_DRIFT_VY_RIGHT, wz=_DRIFT_WZ_RIGHT, rate_hz=_DRIFT_RATE_HZ)
    if _DRIFT_GAP > 0:
        time.sleep(_DRIFT_GAP)

# ---------------------------- Horn (mpg123) ----------------------------

HORN_FILE   = os.environ.get("HORN_FILE", "meepmeep.mp3")
HORN_CMD    = os.environ.get("HORN_CMD", "mpg123")
HORN_DEVICE = os.environ.get("HORN_DEVICE")
HORN_DEFAULT_VOLUME = int(os.environ.get("HORN_DEFAULT_VOLUME", "22"))  # 0..100

_SEARCH_DIRS = [
    Path.cwd(),
    Path(__file__).resolve().parent,
    Path("/opt/sounds"),
    Path.home() / "Music",
    Path.home() / "Downloads",
]

def _find_horn() -> Optional[Path]:
    p = Path(HORN_FILE).expanduser()
    if p.is_file():
        return p
    for d in _SEARCH_DIRS:
        q = d / HORN_FILE
        if q.is_file():
            return q
    return None

def _mpg123_scale_from_percent(pct: int) -> int:
    try:
        p = int(pct)
    except Exception:
        p = HORN_DEFAULT_VOLUME
    p = max(0, min(100, p))
    return max(1, int(round(32768 * (p / 100.0))))

def horn(path: Optional[str] = None, device: Optional[str] = HORN_DEVICE,
         volume: Optional[int] = None, block: bool = True) -> bool:
    mp3 = Path(path).expanduser() if path else _find_horn()
    if not mp3 or not mp3.exists():
        print("[horn] can't find", HORN_FILE, "-- set HORN_FILE or pass a path")
        return False
    vol_pct = HORN_DEFAULT_VOLUME if volume is None else int(volume)
    scale = _mpg123_scale_from_percent(vol_pct)
    cmd = [HORN_CMD, "-q", "-f", str(scale)]
    if device:
        cmd += ["-a", device]
    cmd.append(str(mp3))
    try:
        subprocess.run(cmd, check=True) if block else subprocess.Popen(cmd)
        return True
    except FileNotFoundError:
        print("[horn] mpg123 not found. Install: sudo apt update && sudo apt install -y mpg123 alsa-utils")
        return False
    except Exception as e:
        print("[horn] play failed:", e)
        return False

def Horn():
    return horn()

# ---------------------------- Notebook-friendly class ----------------------------

class RobotMoves:
    """
    Stable class wrapper for student notebooks.
    """
    def __init__(self, defaults_linear: float = DEFAULT_LINEAR, defaults_strafe: float = DEFAULT_STRAFE, defaults_turn: float = DEFAULT_TURN, rate_hz: float = DEFAULT_RATE_HZ):
        self.linear = float(defaults_linear)
        self.strafe = float(defaults_strafe)
        self.turn = float(defaults_turn)
        self.rate_hz = float(rate_hz)
        get_controller()  # init

    def stop(self):
        stop()

    def forward(self, seconds: float = 0.5, speed: Optional[float] = None):
        forward(seconds, self.linear if speed is None else float(speed))

    def backward(self, seconds: float = 0.5, speed: Optional[float] = None):
        backward(seconds, self.linear if speed is None else float(speed))

    def left(self, seconds: float = 0.5, speed: Optional[float] = None):
        left(seconds, self.strafe if speed is None else float(speed))

    def right(self, seconds: float = 0.5, speed: Optional[float] = None):
        right(seconds, self.strafe if speed is None else float(speed))

    def turn_left(self, seconds: float = 0.5, speed: Optional[float] = None):
        turn_left(seconds, self.turn if speed is None else float(speed))

    def turn_right(self, seconds: float = 0.5, speed: Optional[float] = None):
        turn_right(seconds, self.turn if speed is None else float(speed))

    def diagonal_left(self, seconds: float = 0.8, speed: Optional[float] = None):
        diagonal_left(seconds, self.linear if speed is None else float(speed))

    def diagonal_right(self, seconds: float = 0.8, speed: Optional[float] = None):
        diagonal_right(seconds, self.linear if speed is None else float(speed))

    def drift_left(self, seconds: float = 1.2):
        drift_left(seconds)

    def drift_right(self, seconds: float = 1.2):
        drift_right(seconds)

    def drive_for(self, vx: float, vy: float, seconds: float, wz: float = 0.0):
        drive_for(seconds, vx=float(vx), vy=float(vy), wz=float(wz), rate_hz=self.rate_hz)
