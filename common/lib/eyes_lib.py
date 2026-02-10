# eyes_lib.py
# Controls the ultrasonic "eyes" LEDs via /sonar_controller/set_rgb (indices 0 & 1)

import time
import threading
import atexit
from typing import Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from ros_robot_controller_msgs.msg import RGBStates, RGBState

DEFAULT_TOPIC   = "/sonar_controller/set_rgb"
DEFAULT_INDICES = (0, 1)   # left=0, right=1
FLUSH_PAUSE_S   = 0.03

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
        topic: str = DEFAULT_TOPIC,
        indices: Tuple[int, int] = DEFAULT_INDICES,
        node_name: str = "eyes_rgb_client"
    ):
        _rclpy_init_once()
        super().__init__(node_name)
        self.topic   = topic
        self.left_i  = int(indices[0])
        self.right_i = int(indices[1])
        self.pub = self.create_publisher(RGBStates, self.topic, _qos_rel())

        self._blink_thread: Optional[threading.Thread] = None
        self._blink_stop   = threading.Event()
        atexit.register(self.off)

    def _publish_states(self, states):
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
        self._publish_states([RGBState(index=self.left_i, red=int(r), green=int(g), blue=int(b))])

    def set_right(self, r: int, g: int, b: int):
        self._publish_states([RGBState(index=self.right_i, red=int(r), green=int(g), blue=int(b))])

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

    def wink(self, side="left", color=(255,255,255), on_s=0.15, off_s=0.10, repeats=2):
        s = side.lower().strip()
        for _ in range(int(repeats)):
            if s.startswith("l"):
                self.set_left(*color);  time.sleep(on_s); self.set_left(0,0,0)
            else:
                self.set_right(*color); time.sleep(on_s); self.set_right(0,0,0)
            time.sleep(off_s)

    def pulse(self, color, on_s=0.12, off_s=0.08, repeats=3):
        for _ in range(int(repeats)):
            self.set_both(*color); time.sleep(on_s)
            self.off();            time.sleep(off_s)

    def start_blinking_async(self, color=(255,255,255), period_s=0.35, jitter_s=0.10, duty=0.35):
        import random
        self.stop_blinking()
        self._blink_stop.clear()

        def _run():
            while not self._blink_stop.is_set():
                wait = max(0.05, period_s + random.uniform(-jitter_s, jitter_s))
                self.set_both(*color)
                t_end = time.time() + wait
                while time.time() < t_end and not self._blink_stop.is_set():
                    time.sleep(0.02)
                if self._blink_stop.is_set():
                    break

                blink_total = 0.15
                on_t  = blink_total * duty
                off_t = blink_total - on_t
                self.off();            time.sleep(off_t)
                self.set_both(*color); time.sleep(on_t)
                self.off();            time.sleep(off_t)
                self.set_both(*color)

            self.off()

        self._blink_thread = threading.Thread(target=_run, daemon=True)
        self._blink_thread.start()

    def stop_blinking(self):
        if self._blink_thread and self._blink_thread.is_alive():
            self._blink_stop.set()
            self._blink_thread.join(timeout=1.0)
        self._blink_thread = None
        self._blink_stop.clear()

_EYES_SINGLETON: Optional[Eyes] = None

def get_eyes(topic: str = DEFAULT_TOPIC, indices: Tuple[int,int] = DEFAULT_INDICES) -> Eyes:
    global _EYES_SINGLETON
    if _EYES_SINGLETON is None:
        _EYES_SINGLETON = Eyes(topic=topic, indices=indices)
        print(f"eyes_lib ready → topic={topic} indices={indices}")
    return _EYES_SINGLETON
