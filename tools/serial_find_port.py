#!/usr/bin/env python3
"""
serial_find_port.py — Probe serial ports to find the TurboPi motor controller.

Run on the robot:
    python3 serial_find_port.py

What it does:
  - Tries each known port at 1 Mbaud
  - Sends a "stop all motors" packet (safe — zero speed)
  - Reports which port accepts the write without error
  - Does NOT require ROS
"""

import struct
import sys
import time

# ── Protocol constants (from fast-hiwonder/fast_sdk/board_sdk.py) ─────────────
MAGIC_1    = 0xAA
MAGIC_2    = 0x55
FUNC_MOTOR = 3
BAUD_RATE  = 1_000_000

CRC8_TABLE = [
      0,  94, 188, 226,  97,  63, 221, 131, 194, 156, 126,  32, 163, 253,  31,  65,
    157, 195,  33, 127, 252, 162,  64,  30,  95,   1, 227, 189,  62,  96, 130, 220,
     35, 125, 159, 193,  66,  28, 254, 160, 225, 191,  93,   3, 128, 222,  60,  98,
    190, 224,   2,  92, 223, 129,  99,  61, 124,  34, 192, 158,  29,  67, 161, 255,
     70,  24, 250, 164,  39, 121, 155, 197, 132, 218,  56, 102, 229, 187,  89,   7,
    219, 133, 103,  57, 186, 228,   6,  88,  25,  71, 165, 251, 120,  38, 196, 154,
    101,  59, 217, 135,   4,  90, 184, 230, 167, 249,  27,  69, 198, 152, 122,  36,
    248, 166,  68,  26, 153, 199,  37, 123,  58, 100, 134, 216,  91,   5, 231, 185,
    140, 210,  48, 110, 237, 179,  81,  15,  78,  16, 242, 172,  47, 113, 147, 205,
     17,  79, 173, 243, 112,  46, 204, 146, 211, 141, 111,  49, 178, 236,  14,  80,
    175, 241,  19,  77, 206, 144, 114,  44, 109,  51, 209, 143,  12,  82, 176, 238,
     50, 108, 142, 208,  83,  13, 239, 177, 240, 174,  76,  18, 145, 207,  45, 115,
    202, 148, 118,  40, 171, 245,  23,  73,   8,  86, 180, 234, 105,  55, 213, 139,
     87,   9, 235, 181,  54, 104, 138, 212, 149, 203,  41, 119, 244, 170,  72,  22,
    233, 183,  85,  11, 136, 214,  52, 106,  43, 117, 151, 201,  74,  20, 246, 168,
    116,  42, 200, 150,  21,  75, 169, 247, 182, 232,  10,  84, 215, 137, 107,  53,
]

def crc8(data: bytes) -> int:
    check = 0
    for b in data:
        check = CRC8_TABLE[check ^ b]
    return check

def build_motor_speed_packet(speeds):
    """
    speeds: list of (motor_id_1indexed, speed_float)
    Encodes as set_motor_speed command (sub-cmd 0x01).
    Motor IDs are sent 0-indexed on the wire.
    """
    payload = [0x01, len(speeds)]
    for motor_id, speed in speeds:
        payload.extend(struct.pack("<Bf", motor_id - 1, float(speed)))
    buf = [MAGIC_1, MAGIC_2, FUNC_MOTOR, len(payload)] + payload
    buf.append(crc8(bytes(buf[2:])))
    return bytes(buf)

# Safe test packet: all four motors at speed 0
STOP_PACKET = build_motor_speed_packet([(1, 0.0), (2, 0.0), (3, 0.0), (4, 0.0)])

CANDIDATE_PORTS = [
    "/dev/ttyAMA0",
    "/dev/ttyAMA1",
    "/dev/ttyS0",
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
    "/dev/ttyACM0",
    "/dev/ttyACM1",
]

def try_port(port_path):
    try:
        import serial
    except ImportError:
        print("ERROR: pyserial not installed. Run:  pip install pyserial")
        sys.exit(1)

    try:
        port = serial.Serial()
        port.baudrate = BAUD_RATE
        port.timeout  = 0.2
        port.rts      = False
        port.dtr      = False
        port.port     = port_path
        port.open()
    except Exception as e:
        print(f"  {port_path:20s}  OPEN FAILED  — {e}")
        return False

    try:
        port.write(STOP_PACKET)
        port.flush()
        time.sleep(0.05)
        # Read whatever comes back (may be empty if one-way protocol)
        response = port.read(port.in_waiting or 1)
        print(f"  {port_path:20s}  OK  — wrote {len(STOP_PACKET)} bytes"
              + (f", got {len(response)} bytes back" if response else ", no response (normal)"))
        return True
    except Exception as e:
        print(f"  {port_path:20s}  WRITE FAILED — {e}")
        return False
    finally:
        port.close()

print(f"\nProbing {len(CANDIDATE_PORTS)} ports at {BAUD_RATE:,} baud …\n")
found = []
for p in CANDIDATE_PORTS:
    ok = try_port(p)
    if ok:
        found.append(p)

print()
if found:
    print(f"Ports that accepted the packet: {found}")
    print(f"\nRecommended: set SERIAL_PORT = \"{found[0]}\" in your test scripts.")
else:
    print("No port accepted the packet. Check:")
    print("  1. Controller board is powered on")
    print("  2. UART is enabled  (sudo raspi-config → Interface → Serial Port → yes)")
    print("  3. Try:  ls /dev/tty*  to see what exists")
