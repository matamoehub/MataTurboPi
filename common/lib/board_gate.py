# common/lib/board_gate.py
# Best-effort wake-up for the STM32 co-processor that gates the line-sensor
# accessory slot's I2C bus. The line sensor at 0x78 never answers until the
# Pi opens a serial handshake with the co-processor (HiwonderSDK Board()) and
# calls enable_reception(True) — confirmed on real hardware. Harmless no-op
# if the SDK/serial port isn't available (e.g. dev machine, or a ROS node
# already owns the port and has done this itself).
import sys
import time
from typing import Optional

_SDK_PATHS = ["/home/ubuntu/TurboPi", "/home/pi/TurboPi"]

_board: Optional[object] = None
_attempted = False


def ensure_board_enabled() -> bool:
    """
    Open the co-processor's serial link and enable reception, once per
    process. Returns True if the board was reached (this call or a prior
    one), False if unreachable — callers should proceed with the I2C read
    either way, since a ROS hardware-controller node already running may
    have done this already and be holding the serial port itself.
    """
    global _board, _attempted
    if _board is not None:
        return True
    if _attempted:
        return False
    _attempted = True

    for p in _SDK_PATHS:
        if p not in sys.path:
            sys.path.append(p)

    try:
        import HiwonderSDK.ros_robot_controller_sdk as rrc
        board = rrc.Board()
        board.enable_reception(True)
        time.sleep(0.1)
        _board = board
        return True
    except Exception:
        # SDK not present, or serial port already owned by another process
        # (e.g. the ROS ros_robot_controller node) — not fatal either way.
        return False
