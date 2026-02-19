import importlib.util
import sys
import types
from pathlib import Path


def _load_infrared_lib():
    # Stub line_sensors module for hardware-free tests.
    line_sensors_mod = types.ModuleType("line_sensors")

    class _FakeBus:
        def __init__(self):
            self._present = {0x77, 0x10}

        def read_byte(self, addr):
            if addr not in self._present:
                raise OSError("no device")
            return 1

    class _LineSensors:
        def __init__(self, bus_num=1, address=0x77, register=0x01):
            self.bus_num = bus_num
            self.address = address
            self.register = register
            self.bus = _FakeBus()

        def close(self):
            pass

        def read(self, retries=4, retry_delay_s=0.02):
            return [True, False, True, False]

        def _read_byte(self):
            return 0b0101

    line_sensors_mod.LineSensors = _LineSensors
    sys.modules["line_sensors"] = line_sensors_mod

    p = Path(__file__).resolve().parents[1] / "lib" / "infrared_lib.py"
    spec = importlib.util.spec_from_file_location("infrared_lib_for_test", str(p))
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_read_returns_booleans():
    infrared_lib = _load_infrared_lib()
    ir = infrared_lib.Infrared()
    assert ir.read() == [True, False, True, False]


def test_read_raw_returns_byte():
    infrared_lib = _load_infrared_lib()
    ir = infrared_lib.Infrared()
    assert ir.read_raw() == 0b0101


def test_scan_i2c_bus_finds_devices():
    infrared_lib = _load_infrared_lib()
    ir = infrared_lib.Infrared()
    devices = ir.scan_i2c_bus()
    assert "0x77" in devices
    assert "0x10" in devices

