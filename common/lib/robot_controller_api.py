# robot_controller_api.py — low-level TurboPi motor API
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool
from ros_robot_controller_msgs.msg import MotorsSpeedControl, MotorSpeedControl

TOPIC_SPEED  = "/ros_robot_controller/set_motor_speeds"
TOPIC_ENABLE = "/ros_robot_controller/enable_reception"

_node = None
_pub_speed = None
_pub_enable = None

def _qos_rel():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )

def _ensure():
    global _node, _pub_speed, _pub_enable
    if not rclpy.ok():
        rclpy.init(args=None)
    if _node is None:
        _node = Node("turbopi_controller_api")
        _pub_speed  = _node.create_publisher(MotorsSpeedControl, TOPIC_SPEED, _qos_rel())
        _pub_enable = _node.create_publisher(Bool,               TOPIC_ENABLE, _qos_rel())

def enable_motors(on=True, settle=0.05):
    _ensure()
    _pub_enable.publish(Bool(data=bool(on)))
    if settle and settle > 0:
        time.sleep(settle)

def send_speeds(pairs):
    # pairs: [(id, speed), ...]
    _ensure()
    msg = MotorsSpeedControl()
    msg.data = [MotorSpeedControl(id=int(i), speed=float(s)) for (i, s) in pairs]
    _pub_speed.publish(msg)

def stream_speeds(pairs, seconds=0.5, rate_hz=20.0):
    # Re-publish for 'seconds' then stop the same motors
    _ensure()
    end = time.time() + float(seconds)
    dt = 1.0 / float(rate_hz)
    while time.time() < end:
        send_speeds(pairs)
        time.sleep(dt)
    ids = [int(i) for (i, _) in pairs]
    send_speeds([(i, 0.0) for i in ids])

def all_stop(ids=(1,2,3,4)):
    _ensure()
    send_speeds([(int(i), 0.0) for i in ids])
