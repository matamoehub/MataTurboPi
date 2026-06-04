#!/usr/bin/env python3
"""
serial_test_motors.py — Spin each motor individually to verify wiring & direction.

Edit SERIAL_PORT below to match what serial_find_port.py found.

Run on the robot:
    python3 serial_test_motors.py

What it does:
  1. Spins FL for 0.5 s forward, stops
  2. Spins FR for 0.5 s forward, stops
  3. Spins RL for 0.5 s forward, stops
  4. Spins RR for 0.5 s forward, stops
  5. Reports which IDs moved — lets you confirm wiring matches FL/FR/RL/RR

Does NOT require ROS.
"""

import struct
import sys
import time

# ── Configure ─────────────────────────────────────────────────────────────────
SERIAL_PORT = "/dev/ttyAMA0"   # ← change if serial_find_port.py found another
TEST_SPEED  = 300              # match BASE_SPEED from robot_moves.py
RUN_SECS    = 0.5
PAUSE_SECS  = 0.8

# ── Protocol (from fast-hiwonder/fast_sdk/board_sdk.py) ───────────────────────
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

STOP_ALL = build_packet([(1, 0.0), (2, 0.0), (3, 0.0), (4, 0.0)])

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
    send(STOP_ALL)

# ── Test sequence ──────────────────────────────────────────────────────────────
MOTORS = [
    (1, "FL — Front Left"),
    (2, "FR — Front Right"),
    (3, "RL — Rear Left"),
    (4, "RR — Rear Right"),
]

try:
    stop()
    time.sleep(0.3)

    for motor_id, label in MOTORS:
        print(f"  Motor {motor_id}  ({label})  speed={TEST_SPEED} …", end="", flush=True)
        pkt = build_packet([(motor_id, float(TEST_SPEED))])
        t_end = time.time() + RUN_SECS
        while time.time() < t_end:
            send(pkt)
            time.sleep(0.05)
        stop()
        print("  stopped")
        time.sleep(PAUSE_SECS)

    print("\nDone. Check which wheels moved and note their positions.")
    print("Expected wiring (SIGN = [-1,+1,-1,+1] already applied in robot_moves.py):")
    print("  ID 1 = Front Left,  ID 2 = Front Right")
    print("  ID 3 = Rear Left,   ID 4 = Rear Right")

except KeyboardInterrupt:
    print("\nInterrupted")
finally:
    stop()
    port.close()
    print("Port closed.")
