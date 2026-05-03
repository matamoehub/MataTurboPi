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


def _load_eyes_lib(monkeypatch, backend="auto", controller_ok=True):
    _clear_optional_modules()
    _install_ros_stubs()
    monkeypatch.setenv("EYES_BACKEND", backend)
    monkeypatch.setenv("EYES_FLUSH_SPIN_S", "0")
    monkeypatch.setenv("EYES_FLUSH_PAUSE_S", "0")
    monkeypatch.setenv("EYES_CONTROLLER_URL", "http://127.0.0.1:8766")
    calls = []
    _install_urlopen_stub(ok=controller_ok, calls=calls)

    path = Path(__file__).resolve().parents[1] / "lib" / "eyes_lib.py"
    module_name = f"eyes_lib_for_test_{backend}_{'controller' if controller_ok else 'ros'}"
    spec = importlib.util.spec_from_file_location(module_name, str(path))
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module, calls


def test_prefers_local_controller_when_available(monkeypatch):
    eyes_lib, calls = _load_eyes_lib(monkeypatch, backend="auto", controller_ok=True)
    eyes = eyes_lib.get_eyes(force_reset=True)

    assert eyes.backend == "controller"
    eyes.set_both(1, 2, 3)
    assert calls[0][0] == "GET"
    assert calls[-1][0] == "POST"
    assert calls[-1][1].endswith("/set")
    assert calls[-1][2]["states"] == [
        {"index": 0, "red": 1, "green": 2, "blue": 3},
        {"index": 1, "red": 1, "green": 2, "blue": 3},
    ]


def test_falls_back_to_ros_when_controller_unavailable(monkeypatch):
    eyes_lib, _calls = _load_eyes_lib(monkeypatch, backend="auto", controller_ok=False)
    eyes = eyes_lib.get_eyes(force_reset=True)

    assert eyes.backend == "ros"
    eyes.set_left(10, 20, 30)
    pub = eyes.pub
    assert pub is not None
    assert len(pub.published) == 1
    msg = pub.published[0]
    assert [(s.index, s.red, s.green, s.blue) for s in msg.states] == [(0, 10, 20, 30)]
