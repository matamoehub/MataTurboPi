# eyes_lib.py
import os
import time
import threading
import atexit
from typing import Tuple, Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from ros_robot_controller_msgs.msg import RGBStates, RGBState

ENV_TOPIC = os.getenv("EYES_TOPIC", "").strip()

CANDIDATE_TOPICS = [
    "/sonar_controller/set_rgb",
    "/ros_robot_controller/set_rgb",
]

DEFAULT_INDICES = (0, 1)
FLUSH_PAUSE_S = 0.03

def _qos_rel():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )

def _rclpy_init_once():
    if not rclpy.ok():
        rclpy.init(args=None)

class Eyes(Node):
    def __init__(
        self,
        topic: Optional[str] = None,
        indices: Tuple[int, int] = DEFAULT_INDICES,
        node_name: str = "eyes_rgb_client"
    ):
        _rclpy_init_once()
        super().__init__(node_name)

        self.topic = topic or ENV_TOPIC or self._auto_topic()
        self.left_i = int(indices[0])
        self.right_i = int(indices[1])

        self.pub = self.create_publisher(RGBStates, self.topic, _qos_rel())

        self._blink_thread: Optional[threading.Thread] = None
        self._blink_stop = threading.Event()

        atexit.register(self.off)

        print(f"eyes_lib ready → topic={self.topic} indices={indices}")

    def _auto_topic(self) -> str:
        # Try to detect which topic exists in the graph
        topics = {name for (name, _types) in self.get_topic_names_and_types()}
        for t in CANDIDATE_TOPICS:
            if t in topics:
                return t
        # default to first candidate (even if not present)
        return CANDIDATE_TOPICS[0]

    def diagnose(self):
        topics = self.get_topic_names_and_types()
        names = sorted([t[0] for t in topics])
        print("Topic in use:", self.topic)
        print("Available RGB-like topics:")
        for n in names:
            if "rgb" in n.lower() or "led" in n.lower() or "sonar" in n.lower():
                print(" -", n)

    def _publish_states(self, states: List[RGBState]):
        msg = RGBStates()
        msg.states = states
        self.pub.publish(msg)
        time.sleep(FLUSH_PAUSE_S)

    def set_both(self, r: int, g: int, b: int):
        self._publish_states([
            RGBState(index=self.left_i,  red=int(r), green=int(g), blue=int(b)),
            RGBState(index=self.right_i, red=int(r), green=int(g), blue=int(b)),
        ])

    def set_left(self, r: int, g: int, b: int):
        self._publish_states([
            RGBState(index=self.left_i, red=int(r), green=int(g), blue=int(b)),
        ])

    def set_right(self, r: int, g: int, b: int):
        self._publish_states([
            RGBState(index=self.right_i, red=int(r), green=int(g), blue=int(b)),
        ])

    def off(self):
        self.set_both(0, 0, 0)

    def blink(self, color=(255,255,255), period_s=0.25, duration_s=1.0):
        end = time.time() + float(duration_s)
        on = True
        while time.time() < end:
            if on: self.set_both(*color)
            else:  self.off()
            on = not on
            time.sleep(period_s)
        self.off()

    def scan_indices(self, start: int = 0, end: int = 16, color=(0, 0, 255), hold_s: float = 0.35):
        """
        Cycles RGB indices to help you visually identify which indices are the eyes.
        """
        r, g, b = color
        for i in range(int(start), int(end)):
            self._publish_states([RGBState(index=i, red=r, green=g, blue=b)])
            print("lit index", i)
            time.sleep(float(hold_s))
            self._publish_states([RGBState(index=i, red=0, green=0, blue=0)])
            time.sleep(0.05)

_EYES_SINGLETON: Optional[Eyes] = None

def get_eyes(topic: Optional[str] = None, indices: Tuple[int,int] = DEFAULT_INDICES) -> Eyes:
    global _EYES_SINGLETON
    if _EYES_SINGLETON is None:
        _EYES_SINGLETON = Eyes(topic=topic, indices=indices)
    return _EYES_SINGLETON
