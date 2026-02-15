# eyes_lib.py
import os
import time
import threading
import atexit
from typing import Tuple, Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.executors import SingleThreadedExecutor

from ros_robot_controller_msgs.msg import RGBStates, RGBState

ENV_TOPIC = os.getenv("EYES_TOPIC", "").strip()

CANDIDATE_TOPICS = [
    "/sonar_controller/set_rgb",
    "/ros_robot_controller/set_rgb",
]

DEFAULT_INDICES = (0, 1)

# How long to spin after each publish. This is what makes notebooks reliable.
FLUSH_SPIN_S = float(os.getenv("EYES_FLUSH_SPIN_S", "0.15"))
# Optional small pause (usually not needed)
FLUSH_PAUSE_S = float(os.getenv("EYES_FLUSH_PAUSE_S", "0.00"))


def _qos_rel() -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    )


def _rclpy_init_once() -> None:
    if not rclpy.ok():
        rclpy.init(args=None)


class Eyes(Node):
    """
    Simple RGB eye helper.
    Publishes ros_robot_controller_msgs/RGBStates to the sonar_controller (or controller RGB topic).

    Key behaviour:
    - Spins a local executor after publish so messages actually go out in notebooks.
    - Uses a unique node name to avoid collisions when re-running cells.
    """

    def __init__(
        self,
        topic: Optional[str] = None,
        indices: Tuple[int, int] = DEFAULT_INDICES,
        node_name: str = "eyes_rgb_client",
    ):
        _rclpy_init_once()

        # unique node name avoids "Publisher already registered for provided node name"
        uniq = f"{node_name}_{os.getpid()}_{int(time.time()*1000)%100000}"
        super().__init__(uniq)

        self.topic = topic or ENV_TOPIC or self._auto_topic()
        self.left_i = int(indices[0])
        self.right_i = int(indices[1])

        self.pub = self.create_publisher(RGBStates, self.topic, _qos_rel())

        # Local executor so publish flush works reliably in notebooks
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self)

        self._blink_thread: Optional[threading.Thread] = None
        self._blink_stop = threading.Event()

        atexit.register(self.off)

        print(f"eyes_lib ready → topic={self.topic} indices={indices}")

    # ---------- ROS helpers ----------
    def _auto_topic(self) -> str:
        # Try to detect which topic exists in the graph (best-effort)
        try:
            topics = {name for (name, _types) in self.get_topic_names_and_types()}
            for t in CANDIDATE_TOPICS:
                if t in topics:
                    return t
        except Exception:
            pass
        return CANDIDATE_TOPICS[0]

    def _flush(self, seconds: float = FLUSH_SPIN_S) -> None:
        """
        Spin the local executor briefly so outgoing messages actually get sent.
        """
        end = time.time() + float(seconds)
        while time.time() < end:
            self._exec.spin_once(timeout_sec=0.05)
        if FLUSH_PAUSE_S > 0:
            time.sleep(FLUSH_PAUSE_S)

    def diagnose(self) -> None:
        topics = self.get_topic_names_and_types()
        names = sorted([t[0] for t in topics])
        print("Topic in use:", self.topic)
        print("Available RGB-like topics:")
        for n in names:
            if "rgb" in n.lower() or "led" in n.lower() or "sonar" in n.lower():
                print(" -", n)

    # ---------- Publish API ----------
    def _publish_states(self, states: List[RGBState]) -> None:
        msg = RGBStates()
        msg.states = states
        self.pub.publish(msg)
        self._flush()

    def set_both(self, r: int, g: int, b: int) -> None:
        self._publish_states(
            [
                RGBState(index=self.left_i, red=int(r), green=int(g), blue=int(b)),
                RGBState(index=self.right_i, red=int(r), green=int(g), blue=int(b)),
            ]
        )

    def set_left(self, r: int, g: int, b: int) -> None:
        self._publish_states([RGBState(index=self.left_i, red=int(r), green=int(g), blue=int(b))])

    def set_right(self, r: int, g: int, b: int) -> None:
        self._publish_states([RGBState(index=self.right_i, red=int(r), green=int(g), blue=int(b))])

    def set_index(self, idx: int, r: int, g: int, b: int) -> None:
        """
        Useful for scanning. Explicit index publish.
        """
        self._publish_states([RGBState(index=int(idx), red=int(r), green=int(g), blue=int(b))])

    def off(self) -> None:
        try:
            self.set_both(0, 0, 0)
        except Exception:
            # never crash on shutdown
            pass

    # ---------- Effects ----------
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
        """
        Cycles RGB indices to help you visually identify which indices are the eyes.
        Note: 'end' is inclusive.
        """
        r, g, b = color
        for i in range(int(start), int(end) + 1):
            self.set_index(i, r, g, b)
            print("lit index", i)
            time.sleep(float(hold_s))
            self.set_index(i, 0, 0, 0)
            time.sleep(0.05)


_EYES_SINGLETON: Optional[Eyes] = None


def get_eyes(topic: Optional[str] = None, indices: Tuple[int, int] = DEFAULT_INDICES) -> Eyes:
    """
    Singleton eyes instance (keeps one node/publisher alive per process).
    If you change topic/indices, restart the kernel/process to refresh.
    """
    global _EYES_SINGLETON
    if _EYES_SINGLETON is None:
        _EYES_SINGLETON = Eyes(topic=topic, indices=indices)
    return _EYES_SINGLETON
