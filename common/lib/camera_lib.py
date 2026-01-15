# camera_lib.py
import time
from typing import Optional
import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState

# ---------- ROS init helper ----------
def _rclpy_init_once():
    if not rclpy.ok():
        rclpy.init(args=None)

# ---------- Camera node ----------
class Camera(Node):
    """
    Head/camera gimbal control over two PWM servos:
      - nod (pitch)  = servo_id 1
      - shake (yaw)  = servo_id 2

    Positions typically 1000..2000 with ~1500 centered.
    """

    def __init__(
        self,
        node_name: str = "cam_head_ctrl",
        nod_id: int = 1,
        shake_id: int = 2,
        center: int = 1500,
        speed_s: float = 0.20,
        min_pos: int = 1000,
        max_pos: int = 2000,
    ):
        _rclpy_init_once()
        super().__init__(node_name)
        self.pub = self.create_publisher(
            SetPWMServoState, "ros_robot_controller/pwm_servo/set_state", 10
        )
        self.nod_id = int(nod_id)
        self.shake_id = int(shake_id)
        self.center = int(center)
        self.speed_s = float(speed_s)
        self.min_pos = int(min_pos)
        self.max_pos = int(max_pos)

    # ------------ low-level ------------
    def _clamp(self, pos: int) -> int:
        return max(self.min_pos, min(self.max_pos, int(pos)))

    def _send(self, sid: int, pos: int, speed_s: Optional[float] = None):
        msg = SetPWMServoState()
        msg.duration = float(self.speed_s if speed_s is None else speed_s)

        st = PWMServoState()
        st.id = [int(sid)]
        st.position = [self._clamp(int(pos))]

        msg.state = [st]
        self.pub.publish(msg)

        # small wait for physical movement to complete
        time.sleep(msg.duration + 0.02)

    # ------------ primitives -----------
    def center_all(self, speed_s: Optional[float] = None):
        self._send(self.nod_id, self.center, speed_s)
        self._send(self.shake_id, self.center, speed_s)

    def set_pitch(self, pos: int, speed_s: Optional[float] = None):
        self._send(self.nod_id, pos, speed_s)

    def set_yaw(self, pos: int, speed_s: Optional[float] = None):
        self._send(self.shake_id, pos, speed_s)

    # ------------ gestures -------------
    def nod(self, depth: int = 300, speed_s: Optional[float] = None):
        """Down-up-down around center."""
        c = self.center
        self.set_pitch(c + depth, speed_s)
        self.set_pitch(c - depth, speed_s)
        self.set_pitch(c, speed_s)

    def shake(self, width: int = 300, speed_s: Optional[float] = None):
        """Left-right-left around center."""
        c = self.center
        self.set_yaw(c - width, speed_s)
        self.set_yaw(c + width, speed_s)
        self.set_yaw(c, speed_s)

    def wiggle(self, cycles: int = 2, amplitude: int = 200, speed_s: Optional[float] = None):
        """Friendly yaw wiggle."""
        c = self.center
        l = c - int(amplitude)
        r = c + int(amplitude)
        for _ in range(int(cycles)):
            self.set_yaw(l, speed_s)
            self.set_yaw(r, speed_s)
        self.set_yaw(c, speed_s)

    # small / subtle variants for “Pixar-ish” fidgets
    def tiny_wiggle(self, seconds: float = 2.0, amplitude: int = 90, speed_s: float = 0.12):
        """Very small continuous wiggle for 'seconds'."""
        end = time.time() + float(seconds)
        c = self.center
        l = c - int(amplitude)
        r = c + int(amplitude)
        while time.time() < end:
            self.set_yaw(l, speed_s)
            self.set_yaw(r, speed_s)
        self.set_yaw(c, speed_s)

    # quick looks
    def glance_left(self, amplitude: int = 250, hold_s: float = 0.15):
        c = self.center
        self.set_yaw(c - int(amplitude))
        time.sleep(hold_s)
        self.set_yaw(c)

    def glance_right(self, amplitude: int = 250, hold_s: float = 0.15):
        c = self.center
        self.set_yaw(c + int(amplitude))
        time.sleep(hold_s)
        self.set_yaw(c)

    # look up/down
    def look_up(self, amplitude: int = 250, hold_s: float = 0.15):
        c = self.center
        self.set_pitch(c + int(amplitude))
        time.sleep(hold_s)
        self.set_pitch(c)

    def look_down(self, amplitude: int = 250, hold_s: float = 0.15):
        c = self.center
        self.set_pitch(c - int(amplitude))
        time.sleep(hold_s)
        self.set_pitch(c)

# ---------- singleton accessor ----------
_CAM_SINGLETON: Optional[Camera] = None

def get_camera() -> Camera:
    global _CAM_SINGLETON
    if _CAM_SINGLETON is None:
        _CAM_SINGLETON = Camera()
    return _CAM_SINGLETON

# Optional auto-instance for notebooks
try:
    cam  # reuse if exists
except NameError:
    cam = get_camera()
    print("camera_lib ready: cam.nod(), cam.shake(), cam.wiggle(), cam.tiny_wiggle(), cam.center_all(), cam.glance_left/right(), cam.look_up/down()")
