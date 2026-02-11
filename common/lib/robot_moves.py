# robot_moves.py - 3.10 Lib
import time
from dataclasses import dataclass
from typing import Optional

from robot_controller_api import RobotController

@dataclass
class MoveDefaults:
    linear: float = 0.18
    turn: float = 0.9

class RobotMoves:
    def __init__(self, controller: Optional[RobotController] = None, defaults: MoveDefaults = MoveDefaults()):
        self.controller = controller or RobotController()
        self.defaults = defaults

    def drive(self, linear: float, angular: float, seconds: float):
        """Drive with linear m/s and angular rad/s for seconds."""
        end = time.time() + float(seconds)
        try:
            while time.time() < end:
                self.controller.publish_cmd_vel(linear, angular)
                time.sleep(0.05)
        finally:
            self.controller.stop()

    def forward(self, seconds: float = 1.0, speed: Optional[float] = None):
        self.drive(speed if speed is not None else self.defaults.linear, 0.0, seconds)

    def backward(self, seconds: float = 1.0, speed: Optional[float] = None):
        self.drive(-(speed if speed is not None else self.defaults.linear), 0.0, seconds)

    def left(self, seconds: float = 0.6, turn: Optional[float] = None):
        self.drive(0.0, turn if turn is not None else self.defaults.turn, seconds)

    def right(self, seconds: float = 0.6, turn: Optional[float] = None):
        self.drive(0.0, -(turn if turn is not None else self.defaults.turn), seconds)

    def stop(self):
        self.controller.stop()
