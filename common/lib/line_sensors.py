# common/lib/line_sensors.py
# 4x IR line sensor via I2C, address 0x78, register 0x01 (confirmed on real
# hardware — 0x77 is a different device, the sonar+RGB-eyes MCU, see i2c_bus.py)
# Returns [s0,s1,s2,s3] as booleans.
#
# Robust against intermittent: OSError [Errno 121] Remote I/O error

import time
from typing import List, Optional

try:
    from smbus2 import SMBus
except ImportError as e:
    raise ImportError("Missing smbus2. Install with: sudo apt install -y python3-smbus || pip install smbus2") from e

from board_gate import ensure_board_enabled


class LineSensors:
    # NOTE: Hiwonder hardware ships with address 0x78, register 0x01.
    # The sensor will not answer on I2C until the STM32 co-processor's
    # serial link has been opened (see board_gate.py) — without that, every
    # read fails with OSError regardless of wiring/address.
    def __init__(self, bus_num: int = 1, address: int = 0x78, register: int = 0x01):
        ensure_board_enabled()
        self.bus_num = int(bus_num)
        self.address = int(address)
        self.register = int(register)
        self.bus = SMBus(self.bus_num)

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
                # bit0..bit3
                return [
                    bool(v & 0x01),
                    bool(v & 0x02),
                    bool(v & 0x04),
                    bool(v & 0x08),
                ]
            except OSError as e:
                last_err = e
                time.sleep(float(retry_delay_s))
        raise last_err  # type: ignore
