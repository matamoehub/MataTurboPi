__version__ = "2.0.2"

# eyes_lib.py
"""RGB eye LED control for Hiwonder TurboPi.

Backend priority (auto mode):
  1. I2C direct  — smbus2 → addr 0x77 registers 3-8 (fastest, no ROS needed)
  2. HTTP        — local eyes_controller service at port 8766
  3. ROS         — publish RGBStates to /sonar_controller/set_rgb

The I2C path writes directly to the Hiwonder sonar+RGB module (same device as
the sonar, different registers):
  LED 0: reg 3 (R), reg 4 (G), reg 5 (B)
  LED 1: reg 6 (R), reg 7 (G), reg 8 (B)

Set EYES_I2C_ENABLED=0 to skip I2C and use HTTP/ROS instead.
Set EYES_BACKEND=ros  to force the ROS path (useful on non-Pi dev machines).
"""
import atexit
import json
import os
import threading
import time
import urllib.error
import urllib.request
from typing import List, Optional, Tuple

try:
    from ros_robot_controller_msgs.msg import RGBState, RGBStates
    _ROS_MSGS_OK = True
except Exception:
    RGBState = None   # type: ignore[assignment,misc]
    RGBStates = None  # type: ignore[assignment,misc]
    _ROS_MSGS_OK = False

ENV_TOPIC = os.getenv("EYES_TOPIC", "").strip()
EYES_BACKEND = os.getenv("EYES_BACKEND", "auto").strip().lower() or "auto"
EYES_CONTROLLER_URL = (os.getenv("EYES_CONTROLLER_URL", "").strip() or "http://127.0.0.1:8766").rstrip("/")
EYES_CONTROLLER_TIMEOUT_S = float(os.getenv("EYES_CONTROLLER_TIMEOUT_S", "0.35"))

CANDIDATE_TOPICS = [
    "/sonar_controller/set_rgb",
    "/ros_robot_controller/set_rgb",
]

DEFAULT_INDICES = (0, 1)

# How long to spin after each ROS publish so messages actually go out in notebooks.
FLUSH_SPIN_S = float(os.getenv("EYES_FLUSH_SPIN_S", "0.15"))
FLUSH_PAUSE_S = float(os.getenv("EYES_FLUSH_PAUSE_S", "0.00"))

# ── I2C configuration ─────────────────────────────────────────────────────────
# The Hiwonder sonar+RGB module at 0x77 has two RGB LEDs.
# Protocol: plain write_byte_data — no trigger needed, unlike sonar reads.
# Set EYES_I2C_ENABLED=0 to skip I2C and fall through to HTTP/ROS.
_I2C_ENABLED = os.environ.get("EYES_I2C_ENABLED", "1").strip() != "0"
_I2C_BUS     = int(os.environ.get("EYES_I2C_BUS",  "1"))
_I2C_ADDR    = int(os.environ.get("EYES_I2C_ADDR", "0x77"), 16)
# LED index → first R register; G = reg+1, B = reg+2
_I2C_LED_REG: dict = {0: 3, 1: 6}

_smbus2_available: Optional[bool] = None


def _smbus2_ok() -> bool:
    global _smbus2_available
    if _smbus2_available is None:
        try:
            import smbus2  # noqa: F401
            _smbus2_available = True
        except ImportError:
            _smbus2_available = False
    return bool(_smbus2_available)


def _i2c_write_pixel(index: int, r: int, g: int, b: int) -> bool:
    """Write one RGB LED via direct I2C.  Returns True on success.

    Uses write_byte_data (register-addressed) — three separate byte writes,
    matching the Hiwonder SDK's setPixelColor() exactly.
    """
    if not _I2C_ENABLED or not _smbus2_ok():
        return False
    reg = _I2C_LED_REG.get(index)
    if reg is None:
        return False
    try:
        from smbus2 import SMBus
        with SMBus(_I2C_BUS) as bus:
            bus.write_byte_data(_I2C_ADDR, reg,     r & 0xFF)
            bus.write_byte_data(_I2C_ADDR, reg + 1, g & 0xFF)
            bus.write_byte_data(_I2C_ADDR, reg + 2, b & 0xFF)
        return True
    except Exception:
        return False


def _i2c_available() -> bool:
    """Probe I2C: try writing black to LED 0.  Returns True if it works."""
    return _i2c_write_pixel(0, 0, 0, 0)


# ── HTTP controller helpers ───────────────────────────────────────────────────

def _qos_rel():
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    )


def _rclpy_init_once() -> None:
    import rclpy
    if not rclpy.ok():
        rclpy.init(args=None)


def _controller_health(url: str) -> tuple:
    try:
        req = urllib.request.Request(f"{url}/health", method="GET")
        with urllib.request.urlopen(req, timeout=EYES_CONTROLLER_TIMEOUT_S) as resp:
            payload = json.loads(resp.read().decode("utf-8") or "{}")
        if payload.get("ok"):
            return True, f"controller:{url}"
        return False, f"controller_not_ok:{payload}"
    except Exception as exc:
        return False, f"controller_unavailable:{exc}"


def _controller_post(url: str, payload: dict) -> None:
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        f"{url}/set",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=max(0.35, EYES_CONTROLLER_TIMEOUT_S)) as resp:
        body = json.loads(resp.read().decode("utf-8") or "{}")
    if not body.get("ok"):
        raise RuntimeError(f"controller_rejected:{body}")


# ── Eyes class ────────────────────────────────────────────────────────────────

class Eyes:
    """RGB eye helper.

    Backend selection (EYES_BACKEND=auto, default):
      1. I2C direct   — fastest, no ROS dependency
      2. HTTP service — local eyes_controller at port 8766
      3. ROS publish  — direct RGBStates publish (slowest, ROS required)

    Force a specific backend with EYES_BACKEND=i2c|controller|ros.
    """

    def __init__(
        self,
        topic: Optional[str] = None,
        indices: Tuple[int, int] = DEFAULT_INDICES,
        node_name: str = "eyes_rgb_client",
    ):
        self.left_i  = int(indices[0])
        self.right_i = int(indices[1])
        self.topic = topic or ENV_TOPIC or CANDIDATE_TOPICS[0]
        self.backend_reason = ""
        self.controller_url = EYES_CONTROLLER_URL
        self._ros_node = None
        self._exec = None
        self.pub = None
        self.backend = "ros"  # default, overwritten below

        forced = EYES_BACKEND  # "auto" | "i2c" | "controller" | "ros"

        if forced == "ros":
            self.backend_reason = "ros_forced"
            self._init_ros_backend(topic=topic, node_name=node_name)

        elif forced == "i2c":
            if not _i2c_available():
                raise RuntimeError("EYES_BACKEND=i2c but I2C probe failed")
            self.backend = "i2c"
            self.backend_reason = "i2c_forced"

        elif forced == "controller":
            ok, reason = _controller_health(self.controller_url)
            if not ok:
                raise RuntimeError(reason)
            self.backend = "controller"
            self.backend_reason = reason

        else:  # auto
            # 1. Try I2C direct
            if _I2C_ENABLED and _smbus2_ok() and _i2c_available():
                self.backend = "i2c"
                self.backend_reason = f"i2c_direct:bus{_I2C_BUS}:addr{hex(_I2C_ADDR)}"
            else:
                # 2. Try HTTP controller
                ok, reason = _controller_health(self.controller_url)
                if ok:
                    self.backend = "controller"
                    self.backend_reason = reason
                else:
                    # 3. Fall back to ROS
                    self.backend_reason = reason
                    self._init_ros_backend(topic=topic, node_name=node_name)

        self._blink_thread: Optional[threading.Thread] = None
        self._blink_stop = threading.Event()
        # Lock so blink thread and main thread can't interleave LED writes.
        # Without this, set_both() writes LED 0 then LED 1 separately — the
        # blink thread can slip in between and leave one eye on, one eye off.
        self._write_lock = threading.Lock()

        atexit.register(self.off)
        print(
            f"eyes_lib ready → backend={self.backend}"
            f" reason={self.backend_reason} topic={self.topic} indices={indices}"
        )

    def _init_ros_backend(self, topic: Optional[str], node_name: str) -> None:
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        _rclpy_init_once()

        uniq = f"{node_name}_{os.getpid()}_{int(time.time()*1000)%100000}"
        node = Node(uniq)
        self.topic = topic or ENV_TOPIC or self._auto_topic(node)
        self.pub = node.create_publisher(RGBStates, self.topic, _qos_rel())

        executor = SingleThreadedExecutor()
        executor.add_node(node)

        self._ros_node = node
        self._exec = executor
        self.backend = "ros"

    def _auto_topic(self, node) -> str:
        try:
            topics = {name for (name, _types) in node.get_topic_names_and_types()}
            for t in CANDIDATE_TOPICS:
                if t in topics:
                    return t
        except Exception:
            pass
        return CANDIDATE_TOPICS[0]

    def _flush(self, seconds: float = FLUSH_SPIN_S) -> None:
        if self._exec is None:
            return
        end = time.time() + float(seconds)
        while time.time() < end:
            self._exec.spin_once(timeout_sec=0.05)
        if FLUSH_PAUSE_S > 0:
            time.sleep(FLUSH_PAUSE_S)

    # ── Internal publish dispatch ─────────────────────────────────────────────

    def _set_pixels(self, states: List[Tuple[int, int, int, int]]) -> None:
        """Low-level dispatch.  states = list of (index, r, g, b) tuples.

        Held under _write_lock so multi-LED updates (e.g. set_both) are
        atomic — the blink thread cannot slip in between the two I2C writes
        and leave one eye on and one eye off.

        I2C: ALL LEDs are written inside a single SMBus context so a transient
        error on the first LED cannot leave the second LED in a different state
        (the "left eye off, right eye on" fault seen in the field).  Previously
        each LED opened its own SMBus context; a silent exception on LED 0
        skipped that write while LED 1 still succeeded.
        """
        with self._write_lock:
            if self.backend == "i2c":
                if _I2C_ENABLED and _smbus2_ok():
                    try:
                        from smbus2 import SMBus
                        with SMBus(_I2C_BUS) as bus:
                            for idx, r, g, b in states:
                                reg = _I2C_LED_REG.get(idx)
                                if reg is not None:
                                    bus.write_byte_data(_I2C_ADDR, reg,     r & 0xFF)
                                    bus.write_byte_data(_I2C_ADDR, reg + 1, g & 0xFF)
                                    bus.write_byte_data(_I2C_ADDR, reg + 2, b & 0xFF)
                    except Exception:
                        pass
                return

            if self.backend == "controller":
                _controller_post(
                    self.controller_url,
                    {
                        "states": [
                            {"index": idx, "red": r, "green": g, "blue": b}
                            for idx, r, g, b in states
                        ]
                    },
                )
                return

            # ROS path
            if RGBState is None or RGBStates is None:
                raise RuntimeError("ROS messages not available; cannot publish")
            msg = RGBStates()
            msg.states = [
                RGBState(index=idx, red=r, green=g, blue=b)
                for idx, r, g, b in states
            ]
            self.pub.publish(msg)
            self._flush()

    # Keep _publish_states for any internal callers that still pass RGBState objects
    def _publish_states(self, states) -> None:
        """Legacy internal path — accepts RGBState objects or (idx,r,g,b) tuples."""
        if states and hasattr(states[0], "index"):
            tuples = [(s.index, s.red, s.green, s.blue) for s in states]
        else:
            tuples = list(states)
        self._set_pixels(tuples)

    # ── Public API ────────────────────────────────────────────────────────────

    def set_both(self, r: int, g: int, b: int) -> None:
        self._set_pixels([
            (self.left_i,  int(r), int(g), int(b)),
            (self.right_i, int(r), int(g), int(b)),
        ])

    def set_left(self, r: int, g: int, b: int) -> None:
        self._set_pixels([(self.left_i, int(r), int(g), int(b))])

    def set_right(self, r: int, g: int, b: int) -> None:
        self._set_pixels([(self.right_i, int(r), int(g), int(b))])

    def set_index(self, idx: int, r: int, g: int, b: int) -> None:
        self._set_pixels([(int(idx), int(r), int(g), int(b))])

    def off(self) -> None:
        try:
            self.set_both(0, 0, 0)
        except Exception:
            pass

    def blink(self, color=(255, 255, 255), period_s=0.25, duration_s=1.0) -> None:
        end = time.time() + float(duration_s)
        on = True
        while time.time() < end:
            if on:
                self.set_both(*color)
            else:
                self.off()
            on = not on
            time.sleep(float(period_s))
        self.off()

    def scan_indices(
        self,
        start: int = 0,
        end: int = 16,
        color=(0, 0, 255),
        hold_s: float = 0.35,
    ) -> None:
        r, g, b = color
        for i in range(int(start), int(end) + 1):
            self.set_index(i, r, g, b)
            print("lit index", i)
            time.sleep(float(hold_s))
            self.set_index(i, 0, 0, 0)
            time.sleep(0.05)

    def diagnose(self) -> None:
        print("Backend:", self.backend)
        print("Backend detail:", self.backend_reason)
        if self.backend == "i2c":
            print(f"I2C bus={_I2C_BUS} addr={hex(_I2C_ADDR)}")
            print(f"  LED 0 regs: R={_I2C_LED_REG[0]} G={_I2C_LED_REG[0]+1} B={_I2C_LED_REG[0]+2}")
            print(f"  LED 1 regs: R={_I2C_LED_REG[1]} G={_I2C_LED_REG[1]+1} B={_I2C_LED_REG[1]+2}")
            return
        if self.backend == "controller":
            print("Controller URL:", self.controller_url)
            return
        print("Topic in use:", self.topic)
        if self._ros_node is None:
            return
        topics = self._ros_node.get_topic_names_and_types()
        names = sorted([t[0] for t in topics])
        print("Available RGB-like topics:")
        for n in names:
            if "rgb" in n.lower() or "led" in n.lower() or "sonar" in n.lower():
                print(" -", n)

    def close(self) -> None:
        try:
            self.off()
        except Exception:
            pass
        if self._ros_node is not None:
            try:
                self._ros_node.destroy_node()
            except Exception:
                pass
        self._ros_node = None
        self._exec = None
        self.pub = None


# ── Module-level singleton ────────────────────────────────────────────────────

_EYES_SINGLETON: Optional[Eyes] = None


def reset_eyes() -> None:
    global _EYES_SINGLETON
    inst = _EYES_SINGLETON
    _EYES_SINGLETON = None
    if inst is None:
        return
    try:
        inst.close()
    except Exception:
        pass


def get_eyes(
    topic: Optional[str] = None,
    indices: Tuple[int, int] = DEFAULT_INDICES,
    force_reset: bool = False,
) -> Eyes:
    """Singleton eyes instance.
    Use force_reset=True to recreate the backend after a controller or ROS failure.
    """
    global _EYES_SINGLETON
    if force_reset:
        reset_eyes()
    if _EYES_SINGLETON is None:
        _EYES_SINGLETON = Eyes(topic=topic, indices=indices)
    return _EYES_SINGLETON
