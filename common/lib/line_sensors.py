# common/lib/line_sensors.py
# 4x IR line sensor via I2C, address 0x78, register 0x01 (confirmed on real
# hardware — 0x77 is a different device, the sonar+RGB-eyes MCU, see i2c_bus.py)
# Returns [s0,s1,s2,s3] as booleans, True = black line detected.
#
# Hardware is active-low: raw bit=1 means white/reflective ground, bit=0
# means black line or blocked. read() inverts this so True consistently
# means "line detected" for callers (line_follower_lib.py's PID/junction
# logic) — confirmed on real hardware: finger directly on a sensor reads
# raw 0, plain white mat with nothing under it reads raw all-1s.
#
# Robust against intermittent: OSError [Errno 121] Remote I/O error

import time
from typing import List, Optional

try:
    from smbus2 import SMBus
except ImportError as e:
    raise ImportError("Missing smbus2. Install with: sudo apt install -y python3-smbus || pip install smbus2") from e

from board_gate import ensure_board_enabled

# Known line-sensor addresses seen across the fleet — different units have
# answered at different addresses (0x78 is the documented default, 0x48 seen
# on at least one robot). 0x77 is excluded: that's the sonar+RGB-eyes MCU,
# a different device entirely (see i2c_bus.py).
_KNOWN_ADDRESSES = (0x78, 0x48)


def _detect_address(bus, register: int) -> int:
    for addr in _KNOWN_ADDRESSES:
        try:
            bus.read_byte_data(addr, register)
            return addr
        except OSError:
            continue
    raise OSError(
        f"No line sensor answered at any known address {[hex(a) for a in _KNOWN_ADDRESSES]} "
        f"on register 0x{register:02x}. Check wiring/mounting height, or pass address= explicitly."
    )


class LineSensors:
    # NOTE: Hiwonder hardware ships with address 0x78, register 0x01 (some
    # units answer at 0x48 instead — pass address=None, the default, to
    # auto-detect which one this robot uses).
    # The sensor will not answer on I2C until the STM32 co-processor's
    # serial link has been opened (see board_gate.py) — without that, every
    # read fails with OSError regardless of wiring/address.
    def __init__(self, bus_num: int = 1, address: Optional[int] = None, register: int = 0x01):
        ensure_board_enabled()
        self.bus_num = int(bus_num)
        self.register = int(register)
        self.bus = SMBus(self.bus_num)
        self.address = int(address) if address is not None else _detect_address(self.bus, self.register)

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass

    def _read_byte(self) -> int:
        return self.bus.read_byte_data(self.address, self.register)

    def read(self, retries: int = 4, retry_delay_s: float = 0.02) -> List[bool]:
        last_err: Optional[Exception] = None
        for _ in range(int(retries)):
            try:
                v = self._read_byte()
                # bit0..bit3, inverted (active-low hardware -> True = line)
                return [
                    not bool(v & 0x01),
                    not bool(v & 0x02),
                    not bool(v & 0x04),
                    not bool(v & 0x08),
                ]
            except OSError as e:
                last_err = e
                time.sleep(float(retry_delay_s))
        raise last_err  # type: ignore
