# robot_controller_api.py — low-level TurboPi motor API (ROS2 direct motor control)
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
    This matches the behaviour you already know works.
    """
    _ensure()
    end = time.time() + float(seconds)
    dt = 1.0 / float(rate_hz)
    while time.time() < end:
        send_speeds(pairs)
        time.sleep(dt)

    ids = [int(i) for (i, _) in pairs]
    send_speeds([(i, 0.0) for i in ids])

def all_stop(ids=(1, 2, 3, 4)):
    _ensure()
    send_speeds([(int(i), 0.0) for i in ids])
