# robot_controller_api.py
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

def _qos_rel(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth
    )

def _rclpy_init_once():
    if not rclpy.ok():
        rclpy.init(args=None)

class RobotController(Node):
    """
    Thin ROS wrapper for:
      - cmd_vel
      - buzzer (bool topic)
    Notebook-safe:
      - init once
      - optional spin_once flush
      - close() for cleanup
    """
    def __init__(
        self,
        cmd_vel_topic: str = "/cmd_vel",
        buzzer_topic: str = "/buzzer",
        node_name: str = "robot_controller_api",
        flush_spin: bool = False,
        flush_sleep_s: float = 0.02,
    ):
        _rclpy_init_once()
        super().__init__(node_name)

        self.cmd_vel_topic = cmd_vel_topic
        self.buzzer_topic = buzzer_topic
        self.flush_spin = bool(flush_spin)
        self.flush_sleep_s = float(flush_sleep_s)

        self._cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, _qos_rel())
        self._buzz_pub = self.create_publisher(Bool, self.buzzer_topic, _qos_rel())

    def _flush(self):
        if self.flush_spin:
            try:
                rclpy.spin_once(self, timeout_sec=0.0)
            except Exception:
                pass
        if self.flush_sleep_s > 0:
            time.sleep(self.flush_sleep_s)

    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)
        self._flush()

    def stop(self):
        self.publish_cmd_vel(0.0, 0.0)

    def buzzer_on(self):
        self._buzz_pub.publish(Bool(data=True))
        self._flush()

    def buzzer_off(self):
        self._buzz_pub.publish(Bool(data=False))
        self._flush()

    def close(self):
        try:
            self.stop()
        except Exception:
            pass
        try:
            self.destroy_node()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False

