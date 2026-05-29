#!/usr/bin/env python3
"""
serial_test_moves.py — Test basic robot movements over serial (no ROS).

Edit SERIAL_PORT below to match what serial_find_port.py found.

Run on the robot:
    python3 serial_test_moves.py

Movements tested:
  forward → backward → strafe left → strafe right → spin left → spin right → stop

Uses the same SIGN calibration as robot_moves.py: SIGN = [-1, +1, -1, +1]
Does NOT require ROS.
"""

import struct
import sys
import time

# ── Configure ─────────────────────────────────────────────────────────────────
SERIAL_PORT = "/dev/ttyAMA0"   # ← change if serial_find_port.py found another
BASE_SPEED  = 300              # matches robot_moves.py default
RUN_SECS    = 1.0
PAUSE_SECS  = 0.5
RATE_HZ     = 20               # packets per second while moving

# ── Protocol ──────────────────────────────────────────────────────────────────
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

def build_packet(speeds):
    """speeds: [(motor_id_1indexed, speed_float), ...]"""
    payload = [0x01, len(speeds)]
    for motor_id, speed in speeds:
        payload.extend(struct.pack("<Bf", motor_id - 1, float(speed)))
    buf = [MAGIC_1, MAGIC_2, FUNC_MOTOR, len(payload)] + payload
    buf.append(crc8(bytes(buf[2:])))
    return bytes(buf)

# ── Mecanum kinematics (mirrors robot_moves.py exactly) ───────────────────────
# Wheel order: FL, FR, RL, RR   IDs: 1, 2, 3, 4
# Hardware polarity correction
SIGN = [-1, +1, -1, +1]

def _pkt(fl, fr, rl, rr):
    speeds = [
        (1, SIGN[0] * fl),
        (2, SIGN[1] * fr),
        (3, SIGN[2] * rl),
        (4, SIGN[3] * rr),
    ]
    return build_packet(speeds)

STOP_PKT = _pkt(0, 0, 0, 0)

def _move_pkt(name):
    s = BASE_SPEED
    if   name == "forward":       return _pkt(+s, +s, +s, +s)
    elif name == "backward":      return _pkt(-s, -s, -s, -s)
    elif name == "strafe_left":   return _pkt(-s, +s, +s, -s)
    elif name == "strafe_right":  return _pkt(+s, -s, -s, +s)
    elif name == "spin_left":     return _pkt(-s, +s, -s, +s)
    elif name == "spin_right":    return _pkt(+s, -s, +s, -s)
    raise ValueError(name)

# ── Serial connection ──────────────────────────────────────────────────────────
try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run:  pip install pyserial")
    sys.exit(1)

print(f"Opening {SERIAL_PORT} at {BAUD_RATE:,} baud …")
try:
    port = serial.Serial()
    port.baudrate = BAUD_RATE
    port.timeout  = 0.2
    port.rts      = False
    port.dtr      = False
    port.port     = SERIAL_PORT
    port.open()
    print("OK\n")
except Exception as e:
    print(f"FAILED: {e}")
    print("Run serial_find_port.py first to find the correct port.")
    sys.exit(1)

def send(packet):
    port.write(packet)
    port.flush()

def stop():
    send(STOP_PKT)

def run_move(name, seconds=RUN_SECS):
    pkt = _move_pkt(name)
    print(f"  {name:<16s} {seconds}s …", end="", flush=True)
    dt    = 1.0 / RATE_HZ
    t_end = time.time() + seconds
    while time.time() < t_end:
        send(pkt)
        time.sleep(dt)
    stop()
    print("  OK")
    time.sleep(PAUSE_SECS)

# ── Test sequence ──────────────────────────────────────────────────────────────
SEQUENCE = [
    "forward",
    "backward",
    "strafe_left",
    "strafe_right",
    "spin_left",
    "spin_right",
]

try:
    stop()
    time.sleep(0.3)
    print(f"Running {len(SEQUENCE)} moves at speed={BASE_SPEED}, {RUN_SECS}s each\n")

    for move in SEQUENCE:
        run_move(move)

    print("\nAll moves complete.")
    print("If any move was wrong direction, flip the SIGN value for that wheel in robot_moves.py.")

except KeyboardInterrupt:
    print("\nInterrupted — stopping motors")
finally:
    stop()
    port.close()
    print("Port closed.")
