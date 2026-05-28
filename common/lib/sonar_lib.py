#!/usr/bin/env python3
"""
sonar_lib — Ultrasonic / sonar distance reader.

Primary:  direct I2C  (smbus2, bus 1, addr 0x77, Hiwonder trigger-then-read protocol)
Fallback: ROS2 topic  (/sonar_controller/get_distance)

The I2C path reads in ~1 ms with no ROS dependency, so the sonar works even
when sonar_controller is restarting or the ROS graph is degraded.

Hiwonder sonar I2C protocol (confirmed 2026-05-29):
  - Write trigger byte [0x00] to address 0x77  (starts measurement)
  - Read 2 bytes from address 0x77
  - Interpret as little-endian unsigned mm
  - Values > 5000 mm are clamped to 5000 mm by the hardware
  NOTE: the old read_i2c_block_data(0x77, 0x01, 2) big-endian method reads
  stale register bytes and does NOT trigger a measurement — always wrong.

Quick start (new API):
    from sonar_lib import get_distance_cm
    print(get_distance_cm())           # → 45.0, or None if unavailable

Backward-compatible API still works:
    from sonar_lib import get_sonar
    sonar = get_sonar()
    print(sonar.get_distance_cm())
"""
__version__ = "2.2.0"

import os
import threading
import time
from collections import deque
from statistics import mean, stdev
from typing import Deque, List, Optional

from ros_service_client import (
    clear_process_singleton,
    get_process_singleton,
    set_process_singleton,
)

# ── I2C configuration ─────────────────────────────────────────────────────────
# On TurboPi the Hiwonder sonar+RGB module is at I2C address 0x77 on bus 1.
# Protocol: write trigger byte [0x00], read 2 bytes little-endian mm.
# Set SONAR_I2C_ENABLED=0 to disable and fall back to the ROS topic.
_I2C_ENABLED = os.environ.get("SONAR_I2C_ENABLED", "1").strip() != "0"
_I2C_BUS  = int(os.environ.get("SONAR_I2C_BUS",  "1"))
_I2C_ADDR = int(os.environ.get("SONAR_I2C_ADDR", "0x77"), 16)
_I2C_MIN_MM = 20    # sensor floor — below this the reading is noise
_I2C_MAX_MM = 5000  # hardware ceiling (Hiwonder SDK clamps at 5000 mm)

# Rate-limit cache: avoids hammering the I2C bus on rapid successive calls.
# 50 ms → ~20 reads/s max.  Set SONAR_I2C_CACHE_TTL_S=0 to disable.
_I2C_CACHE_TTL_S = float(os.environ.get("SONAR_I2C_CACHE_TTL_S", "0.05"))

_smbus2_available: Optional[bool] = None   # None = not yet probed


def _smbus2_ok() -> bool:
    global _smbus2_available
    if _smbus2_available is None:
        try:
            import smbus2  # noqa: F401
            _smbus2_available = True
        except ImportError:
            _smbus2_available = False
    return bool(_smbus2_available)


# ── ROS imports — optional, used only as fallback ─────────────────────────────
try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Int32
except Exception as _e:
    rclpy = None                    # type: ignore[assignment]
    SingleThreadedExecutor = None   # type: ignore[assignment]
    Node = object                   # type: ignore[assignment,misc]
    Int32 = None                    # type: ignore[assignment]
    _SONAR_IMPORT_ERROR: Optional[Exception] = _e
else:
    _SONAR_IMPORT_ERROR = None

DEFAULT_TOPIC = (
    os.environ.get("SONAR_TOPIC", "/sonar_controller/get_distance").strip()
    or "/sonar_controller/get_distance"
)

_OUTLIER_SIGMA = float(os.environ.get("SONAR_OUTLIER_SIGMA", "1.5"))


# ── I2C cache (module-level, thread-safe) ─────────────────────────────────────
_i2c_lock = threading.Lock()
_i2c_cache_mm: Optional[int] = None
_i2c_cache_ts: float = 0.0
_last_source: str = "none"   # "i2c" | "ros" | "none"


# ── Helpers ───────────────────────────────────────────────────────────────────

def _filtered_mean(samples: List[int]) -> float:
    """Outlier-rejecting mean used by the ROS window filter."""
    if len(samples) <= 2:
        return mean(samples)
    mu = mean(samples)
    sigma = stdev(samples)
    if sigma == 0.0:
        return mu
    survivors = [v for v in samples if abs(v - mu) <= _OUTLIER_SIGMA * sigma]
    return mean(survivors) if survivors else mu


def _read_i2c_raw() -> Optional[int]:
    """One synchronous Hiwonder-protocol I2C read.  Returns mm, or None on error.

    Protocol: write trigger byte [0x00] to start measurement, wait 3 ms for the
    MCU to complete the ultrasonic cycle, then read 2 bytes as little-endian mm.
    This matches sdk/sonar.py but adds the settle delay to avoid the "26 cm /
    25.5 cm" sentinel (0x00FF) the MCU outputs while mid-measurement.
    """
    if not _I2C_ENABLED:
        return None
    if not _smbus2_ok():
        return None
    try:
        from smbus2 import SMBus, i2c_msg
        with SMBus(_I2C_BUS) as bus:
            # Trigger measurement.  The sensor MCU resets its output register to
            # an initialising sentinel (~255 mm = 0x00FF) while computing the new
            # ultrasonic measurement.  Reading immediately would return that sentinel.
            # A 3 ms settle time lets the sensor complete the measurement for targets
            # up to ~50 cm.  For farther targets the sensor will have finished its
            # *previous* cycle (triggered 50 ms ago by the cache TTL), so we still
            # get a fresh real value.
            bus.i2c_rdwr(i2c_msg.write(_I2C_ADDR, [0x00]))
            time.sleep(0.003)   # 3 ms: avoids reading during MCU reset window
            # Read 2-byte little-endian result
            read = i2c_msg.read(_I2C_ADDR, 2)
            bus.i2c_rdwr(read)
            mm = int.from_bytes(bytes(list(read)), byteorder='little', signed=False)
            if mm > 5000:
                mm = 5000
            if _I2C_MIN_MM <= mm <= _I2C_MAX_MM:
                return mm
    except Exception:
        pass
    return None


# ── Primary API ───────────────────────────────────────────────────────────────

def get_distance_mm(use_cache: bool = True) -> Optional[int]:
    """
    Return the current sonar distance in mm, or None if unavailable.

    Uses ROS subscriber by default.  If SONAR_I2C_ENABLED=1, tries I2C first
    (< 1 ms, no ROS dependency) then falls back to the ROS subscriber.

    Results are cached for _I2C_CACHE_TTL_S (default 50 ms) to avoid
    hammering the I2C bus on rapid successive calls.
    Pass use_cache=False to always do a fresh hardware read.
    """
    global _i2c_cache_mm, _i2c_cache_ts, _last_source
    now = time.monotonic()

    if use_cache and _I2C_CACHE_TTL_S > 0:
        with _i2c_lock:
            if _i2c_cache_mm is not None and (now - _i2c_cache_ts) < _I2C_CACHE_TTL_S:
                return _i2c_cache_mm

    # Primary: I2C
    mm = _read_i2c_raw()
    if mm is not None:
        with _i2c_lock:
            _i2c_cache_mm = mm
            _i2c_cache_ts = time.monotonic()
            _last_source = "i2c"
        return mm

    # Fallback: ROS subscriber
    try:
        sonar = _get_ros_sonar()
        ros_mm = sonar.wait_for_reading(timeout_s=2.0)
        if ros_mm is not None:
            with _i2c_lock:
                _i2c_cache_mm = int(ros_mm)
                _i2c_cache_ts = time.monotonic()
                _last_source = "ros"
            return int(ros_mm)
    except Exception:
        pass

    return None


def get_distance_cm(use_cache: bool = True) -> Optional[float]:
    """Return the current sonar distance in cm, or None if unavailable."""
    mm = get_distance_mm(use_cache=use_cache)
    if mm is None:
        return None
    return round(mm / 10.0, 1)


def last_source() -> str:
    """Return 'i2c', 'ros', or 'none' — how the last reading was obtained."""
    with _i2c_lock:
        return _last_source


# ── ROS Sonar node — fallback subscriber ─────────────────────────────────────

def _require_runtime() -> None:
    if _SONAR_IMPORT_ERROR is not None or rclpy is None or Int32 is None:
        raise RuntimeError(
            f"ROS sonar dependencies are not available: {_SONAR_IMPORT_ERROR}"
        )


def _rclpy_init_once() -> None:
    _require_runtime()
    if not rclpy.ok():
        rclpy.init(args=None)


class Sonar(Node):  # type: ignore[misc]
    """
    ROS2-based sonar subscriber.

    Used as a fallback when I2C is unavailable.  New code should call
    get_distance_mm() / get_distance_cm() instead, which use I2C as primary.
    """

    def __init__(
        self,
        topic: str = DEFAULT_TOPIC,
        window: int = 5,
        node_name: str = "sonar_distance_client",
    ):
        _rclpy_init_once()
        uniq = f"{node_name}_{os.getpid()}_{int(time.time() * 1000) % 100000}"
        super().__init__(uniq)
        self.topic = str(topic)
        self.window = max(1, int(window))
        self._last_mm: Optional[int] = None
        self._last_at: Optional[float] = None
        self._samples: Deque[int] = deque(maxlen=self.window)

        self.create_subscription(Int32, self.topic, self._distance_callback, 10)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_stop = threading.Event()
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

    def _spin_loop(self) -> None:
        while not self._spin_stop.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception:
                time.sleep(0.05)

    def close(self) -> None:
        self._spin_stop.set()
        try:
            self._executor.remove_node(self)
        except Exception:
            pass
        try:
            self.destroy_node()
        except Exception:
            pass

    def _distance_callback(self, msg: Int32) -> None:
        value = int(msg.data)
        self._last_mm = value
        self._last_at = time.time()
        self._samples.append(value)

    def has_reading(self) -> bool:
        return self._last_mm is not None

    def wait_for_reading(
        self, timeout_s: float = 2.0, max_age_s: float = 5.0
    ) -> Optional[int]:
        """Wait up to *timeout_s* for a reading no older than *max_age_s*.

        Returns the distance in mm, or None if no fresh reading arrives in time.

        The age guard prevents returning a stale cached value from a previous
        sonar_controller run — the common cause of the "stuck at 26 cm" symptom
        (sonar_controller publishes once, crashes, and _last_mm never updates).
        """
        end = time.time() + float(timeout_s)
        while time.time() < end:
            if self._last_mm is not None and self._last_at is not None:
                if (time.time() - self._last_at) <= max_age_s:
                    return int(self._last_mm)
            time.sleep(0.02)
        return None  # no fresh reading within timeout

    def get_distance_mm(self, filtered: bool = False) -> int:
        if filtered and self._samples:
            return int(round(_filtered_mean(list(self._samples))))
        if self._last_mm is None:
            return 0
        return int(self._last_mm)

    def get_distance_cm(self, filtered: bool = False) -> int:
        return int(round(self.get_distance_mm(filtered=filtered) / 10.0))

    def is_closer_than(self, threshold_cm: float, filtered: bool = True) -> bool:
        if not self.has_reading():
            return False
        return bool(self.get_distance_cm(filtered=filtered) <= float(threshold_cm))

    @property
    def last_update_age_s(self) -> Optional[float]:
        if self._last_at is None:
            return None
        return max(0.0, time.time() - self._last_at)


def _get_ros_sonar(topic: str = DEFAULT_TOPIC, window: int = 5) -> Sonar:
    """Get (or create) the module-level ROS Sonar singleton."""
    key = "sonar_lib:sonar"
    inst = get_process_singleton(key)
    if inst is None:
        resolved = str(topic or DEFAULT_TOPIC).strip() or DEFAULT_TOPIC
        if not resolved.startswith("/"):
            resolved = f"/{resolved}"
        inst = set_process_singleton(key, Sonar(topic=resolved, window=window))
    return inst  # type: ignore[return-value]


# ── Backward-compatible API ───────────────────────────────────────────────────

def get_sonar(topic: str = DEFAULT_TOPIC, window: int = 5) -> Sonar:
    """
    Return the module-level ROS Sonar singleton.

    Kept for backward compatibility.  Prefer get_distance_mm() / get_distance_cm()
    for new code — they use I2C as primary and don't require sonar_controller running.
    """
    return _get_ros_sonar(topic=topic, window=window)


def reset_sonar() -> None:
    """
    Clear all sonar state: ROS subscriber singleton and I2C reading cache.

    Call this when you want the next read to be guaranteed fresh
    (e.g. after the robot has moved to a new position).
    """
    global _i2c_cache_mm, _i2c_cache_ts, _last_source
    key = "sonar_lib:sonar"
    inst = get_process_singleton(key)
    if inst is not None:
        try:
            inst.close()
        except Exception:
            pass
    clear_process_singleton(key)
    with _i2c_lock:
        _i2c_cache_mm = None
        _i2c_cache_ts = 0.0
        _last_source = "none"
