import importlib.util
import json
import sys
import types
from pathlib import Path


def _install_ros_stubs():
    rclpy_mod = types.ModuleType("rclpy")
    rclpy_mod.ok = lambda: True
    rclpy_mod.init = lambda args=None: None
    sys.modules["rclpy"] = rclpy_mod

    node_mod = types.ModuleType("rclpy.node")

    class _Pub:
        def __init__(self):
            self.published = []

        def publish(self, msg):
            self.published.append(msg)

    class _Node:
        def __init__(self, *_a, **_k):
            self.pub = _Pub()

        def create_publisher(self, *_a, **_k):
            return self.pub

        def get_topic_names_and_types(self):
            return [("/sonar_controller/set_rgb", ["ros_robot_controller_msgs/msg/RGBStates"])]

        def destroy_node(self):
            pass

    node_mod.Node = _Node
    sys.modules["rclpy.node"] = node_mod

    exec_mod = types.ModuleType("rclpy.executors")

    class _Exec:
        def add_node(self, _node):
            pass

        def spin_once(self, timeout_sec=0.0):
            pass

    exec_mod.SingleThreadedExecutor = _Exec
    sys.modules["rclpy.executors"] = exec_mod

    qos_mod = types.ModuleType("rclpy.qos")

    class _QoSProfile:
        def __init__(self, **_kwargs):
            pass

    class _QoSRel:
        RELIABLE = "RELIABLE"

    class _QoSHist:
        KEEP_LAST = "KEEP_LAST"

    qos_mod.QoSProfile = _QoSProfile
    qos_mod.QoSReliabilityPolicy = _QoSRel
    qos_mod.QoSHistoryPolicy = _QoSHist
    sys.modules["rclpy.qos"] = qos_mod

    msg_mod = types.ModuleType("ros_robot_controller_msgs.msg")

    class _RGBState:
        def __init__(self, index=0, red=0, green=0, blue=0):
            self.index = index
            self.red = red
            self.green = green
            self.blue = blue

    class _RGBStates:
        def __init__(self):
            self.states = []

    msg_mod.RGBState = _RGBState
    msg_mod.RGBStates = _RGBStates
    sys.modules["ros_robot_controller_msgs.msg"] = msg_mod


def _install_urlopen_stub(ok=True, calls=None):
    import urllib.request

    class _Resp:
        def __init__(self, payload):
            self._payload = payload

        def read(self):
            return json.dumps(self._payload).encode("utf-8")

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

    def _urlopen(req, timeout=0):
        method = getattr(req, "method", None) or req.get_method()
        full_url = req.full_url
        body = None
        if getattr(req, "data", None):
            body = json.loads(req.data.decode("utf-8"))
        if calls is not None:
            calls.append((method, full_url, body, timeout))
        if not ok:
            raise urllib.error.URLError("controller down")
        if full_url.endswith("/health"):
            return _Resp({"ok": True})
        return _Resp({"ok": True})

    urllib.request.urlopen = _urlopen


def _clear_optional_modules():
    for name in [
        "rclpy",
        "rclpy.node",
        "rclpy.executors",
        "rclpy.qos",
        "ros_robot_controller_msgs.msg",
    ]:
        sys.modules.pop(name, None)


def _install_smbus2_stub(calls: list) -> None:
    """Install a smbus2 stub that records (addr, reg, val) writes."""
    smbus2_mod = types.ModuleType("smbus2")

    class _SMBus:
        def __init__(self, bus):
            self._bus = bus

        def __enter__(self):
            return self

        def __exit__(self, *_):
            return False

        def write_byte_data(self, addr, reg, val):
            calls.append((addr, reg, val))

    smbus2_mod.SMBus = _SMBus
    sys.modules["smbus2"] = smbus2_mod


def _load_eyes_lib(
    monkeypatch,
    backend="auto",
    controller_ok=True,
    smbus2_available=False,
):
    _clear_optional_modules()
    sys.modules.pop("smbus2", None)
    _install_ros_stubs()
    monkeypatch.setenv("EYES_BACKEND", backend)
    monkeypatch.setenv("EYES_FLUSH_SPIN_S", "0")
    monkeypatch.setenv("EYES_FLUSH_PAUSE_S", "0")
    monkeypatch.setenv("EYES_CONTROLLER_URL", "http://127.0.0.1:8766")
    http_calls = []
    i2c_calls: list = []
    _install_urlopen_stub(ok=controller_ok, calls=http_calls)
    if smbus2_available:
        _install_smbus2_stub(i2c_calls)

    path = Path(__file__).resolve().parents[1] / "lib" / "eyes_lib.py"
    uid = f"{backend}_{'ctrl' if controller_ok else 'no_ctrl'}_{'i2c' if smbus2_available else 'no_i2c'}"
    module_name = f"eyes_lib_for_test_{uid}"
    spec = importlib.util.spec_from_file_location(module_name, str(path))
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module, http_calls, i2c_calls


# ── I2C backend tests ─────────────────────────────────────────────────────────

def test_prefers_i2c_when_smbus2_available(monkeypatch):
    eyes_lib, _http, i2c = _load_eyes_lib(monkeypatch, backend="auto", smbus2_available=True)
    eyes = eyes_lib.get_eyes(force_reset=True)

    assert eyes.backend == "i2c"
    # Probe write (off) happens in __init__; clear for assertion
    i2c.clear()

    eyes.set_both(10, 20, 30)
    # LED 0: regs 3,4,5 = R,G,B;  LED 1: regs 6,7,8
    assert (0x77, 3, 10) in i2c
    assert (0x77, 4, 20) in i2c
    assert (0x77, 5, 30) in i2c
    assert (0x77, 6, 10) in i2c
    assert (0x77, 7, 20) in i2c
    assert (0x77, 8, 30) in i2c


def test_i2c_set_left_writes_led0_only(monkeypatch):
    eyes_lib, _http, i2c = _load_eyes_lib(monkeypatch, backend="auto", smbus2_available=True)
    eyes = eyes_lib.get_eyes(force_reset=True)
    i2c.clear()

    eyes.set_left(100, 150, 200)
    # Only LED 0 registers should be written
    assert (0x77, 3, 100) in i2c
    assert (0x77, 4, 150) in i2c
    assert (0x77, 5, 200) in i2c
    # LED 1 registers must NOT be touched
    assert not any(reg in (6, 7, 8) for (_, reg, _) in i2c)


def test_i2c_set_right_writes_led1_only(monkeypatch):
    eyes_lib, _http, i2c = _load_eyes_lib(monkeypatch, backend="auto", smbus2_available=True)
    eyes = eyes_lib.get_eyes(force_reset=True)
    i2c.clear()

    eyes.set_right(50, 60, 70)
    assert (0x77, 6, 50) in i2c
    assert (0x77, 7, 60) in i2c
    assert (0x77, 8, 70) in i2c
    assert not any(reg in (3, 4, 5) for (_, reg, _) in i2c)


def test_i2c_off_writes_zeros(monkeypatch):
    eyes_lib, _http, i2c = _load_eyes_lib(monkeypatch, backend="auto", smbus2_available=True)
    eyes = eyes_lib.get_eyes(force_reset=True)
    i2c.clear()

    eyes.off()
    for reg in (3, 4, 5, 6, 7, 8):
        assert (0x77, reg, 0) in i2c


def test_i2c_does_not_use_http_or_ros(monkeypatch):
    eyes_lib, http_calls, _i2c = _load_eyes_lib(
        monkeypatch, backend="auto", controller_ok=True, smbus2_available=True
    )
    eyes = eyes_lib.get_eyes(force_reset=True)
    assert eyes.backend == "i2c"
    http_calls.clear()

    eyes.set_both(1, 2, 3)
    # No HTTP calls should have been made
    assert http_calls == []


# ── HTTP controller fallback tests ────────────────────────────────────────────

def test_prefers_local_controller_when_i2c_unavailable(monkeypatch):
    """Without smbus2, auto mode should fall back to the HTTP controller."""
    eyes_lib, http_calls, _i2c = _load_eyes_lib(
        monkeypatch, backend="auto", controller_ok=True, smbus2_available=False
    )
    eyes = eyes_lib.get_eyes(force_reset=True)

    assert eyes.backend == "controller"
    eyes.set_both(1, 2, 3)
    assert http_calls[0][0] == "GET"
    assert http_calls[-1][0] == "POST"
    assert http_calls[-1][1].endswith("/set")
    assert http_calls[-1][2]["states"] == [
        {"index": 0, "red": 1, "green": 2, "blue": 3},
        {"index": 1, "red": 1, "green": 2, "blue": 3},
    ]


def test_falls_back_to_ros_when_i2c_and_controller_unavailable(monkeypatch):
    eyes_lib, _http, _i2c = _load_eyes_lib(
        monkeypatch, backend="auto", controller_ok=False, smbus2_available=False
    )
    eyes = eyes_lib.get_eyes(force_reset=True)

    assert eyes.backend == "ros"
    eyes.set_left(10, 20, 30)
    pub = eyes.pub
    assert pub is not None
    assert len(pub.published) == 1
    msg = pub.published[0]
    assert [(s.index, s.red, s.green, s.blue) for s in msg.states] == [(0, 10, 20, 30)]


def test_forced_ros_backend_skips_i2c(monkeypatch):
    eyes_lib, _http, i2c = _load_eyes_lib(
        monkeypatch, backend="ros", smbus2_available=True
    )
    eyes = eyes_lib.get_eyes(force_reset=True)
    assert eyes.backend == "ros"
    i2c.clear()

    eyes.set_both(1, 2, 3)
    # I2C must not be used when backend is forced to ros
    assert i2c == []
