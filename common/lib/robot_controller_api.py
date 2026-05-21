# robot_controller_api.py — low-level TurboPi motor API (ROS2 direct motor control)
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool

# TurboPi / Hiwonder messages
from ros_robot_controller_msgs.msg import MotorsSpeedControl, MotorSpeedControl

TOPIC_SPEED  = "/ros_robot_controller/set_motor_speeds"
TOPIC_ENABLE = "/ros_robot_controller/enable_reception"

_node = None
_pub_speed = None
_pub_enable = None

# Set by all_stop() to interrupt any stream_speeds() running in another thread.
# Cleared at the start of each stream_speeds() so that a new move can proceed.
_stop_event = threading.Event()

def _qos_rel(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )

def _ensure():
    global _node, _pub_speed, _pub_enable
    if not rclpy.ok():
        rclpy.init(args=None)

    if _node is None:
        # keep node name stable (matches what you saw in ros2 node list)
        _node = Node("robot_controller_api")
        _pub_speed  = _node.create_publisher(MotorsSpeedControl, TOPIC_SPEED, _qos_rel())
        _pub_enable = _node.create_publisher(Bool, TOPIC_ENABLE, _qos_rel())

def enable_motors(on: bool = True, settle_s: float = 0.05):
    _ensure()
    _pub_enable.publish(Bool(data=bool(on)))
    if settle_s and settle_s > 0:
        time.sleep(float(settle_s))

def send_speeds(pairs):
    """
    pairs: [(motor_id, speed), ...]
    motor_id: 1..4 (FL, FR, RL, RR in your setup)
    speed: float (your calibrated scale)
    """
    _ensure()
    msg = MotorsSpeedControl()
    msg.data = [MotorSpeedControl(id=int(i), speed=float(s)) for (i, s) in pairs]
    _pub_speed.publish(msg)

def stream_speeds(pairs, seconds: float = 0.5, rate_hz: float = 20.0):
    """
    Re-publish 'pairs' for 'seconds' then stop the same motors.

    Interrupted cleanly in two scenarios:
      1. Jupyter Stop button / KeyboardInterrupt — the try/finally always
         sends zero speeds so the motors don't keep spinning.
      2. all_stop() called from another thread — sets _stop_event which
         causes the loop to exit early, then the finally block sends zeros.
    """
    _ensure()
    _stop_event.clear()          # allow this new movement to run
    end = time.time() + float(seconds)
    dt = 1.0 / float(rate_hz)
    ids = [int(i) for (i, _) in pairs]
    try:
        while time.time() < end:
            if _stop_event.is_set():
                break
            send_speeds(pairs)
            time.sleep(dt)
    finally:
        # Always zero the motors — catches KeyboardInterrupt and thread stop.
        send_speeds([(i, 0.0) for i in ids])

def all_stop(ids=(1, 2, 3, 4)):
    """
    Immediately stop all motors.

    Sets _stop_event so any stream_speeds() running in another thread exits
    its loop on the next iteration (within one rate_hz tick, typically 50 ms).
    """
    _ensure()
    _stop_event.set()
    send_speeds([(int(i), 0.0) for i in ids])
