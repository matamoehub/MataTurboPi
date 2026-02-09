# cmdvel_drifts.py — drift + tiny fidgets over /cmd_vel
import time, atexit, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

RATE_HZ = 20
GAP     = 0.25
VY_LEFT  = 0.200
WZ_LEFT  = 0.650
VY_RIGHT = -0.200
WZ_RIGHT = -0.845

_node=None; _pub=None
def _ensure():
    global _node,_pub
    if not rclpy.ok(): rclpy.init(args=None)
    if _node is None:
        _node = Node('cmdvel_drifts')
        _pub  = _node.create_publisher(Twist, '/cmd_vel', 10)

def _send(vx=0.0, vy=0.0, wz=0.0):
    _ensure()
    m=Twist(); m.linear.x=float(vx); m.linear.y=float(vy); m.angular.z=float(wz)
    _pub.publish(m)

def _hold(seconds, vx=0.0, vy=0.0, wz=0.0, rate=RATE_HZ):
    end=time.time()+float(seconds); dt=1.0/float(rate)
    while time.time()<end:
        _send(vx, vy, wz); time.sleep(dt)
    _send(0.0,0.0,0.0); time.sleep(GAP)

def DriftLeft(seconds=1.2):  _hold(seconds, vy=VY_LEFT,  wz=WZ_LEFT)
def DriftRight(seconds=1.2): _hold(seconds, vy=VY_RIGHT, wz=WZ_RIGHT)
def SpinLeft(seconds=1.0, w=1.6): _hold(seconds, wz=float(w))
def SpinRight(seconds=1.0, w=1.6): _hold(seconds, wz=-float(w))
def Forward(seconds=0.6, v=0.18):  _hold(seconds, vx=float(v))
def Back(seconds=0.4, v=0.14):     _hold(seconds, vx=-float(v))
def Stop(): _send(0,0,0)

def TinyFidget(duration_s=2.0, amp_v=0.02, amp_w=0.10, step_s=0.22):
    """Pixar-ish subtle sway while speaking."""
    end = time.time() + float(duration_s)
    while time.time() < end:
        _hold(step_s, vx=+amp_v, vy=0.0, wz=+amp_w, rate=RATE_HZ)
        _hold(step_s, vx=-amp_v, vy=0.0, wz=-amp_w, rate=RATE_HZ)
    Stop()

atexit.register(Stop)
