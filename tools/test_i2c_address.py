#!/usr/bin/env python3
"""
test_i2c_address.py — Probe a specific I2C address/register over time from the
command line, to check whether a device's readings actually respond to
physical input (e.g. a line sensor's line) or are just static/noisy.

Run on the robot:
    python3 test_i2c_address.py                    # defaults: 0x48, bus 1, registers 0x00 and 0x01
    python3 test_i2c_address.py --address 0x77
    python3 test_i2c_address.py --address 0x48 --registers 0x00,0x01,0x02,0x03
    python3 test_i2c_address.py --address 0x48 --duration 20 --interval 0.1

What it does:
  - Reads each requested register in a loop for --duration seconds
  - Prints a timestamped line every time any register's value changes
  - Prints a plain read_byte() (no register) result too, for comparison
  - Ctrl+C stops cleanly
  - Does NOT require ROS or Jupyter — just smbus2
"""

import argparse
import sys
import time

try:
    from smbus2 import SMBus
except ImportError:
    sys.exit("Missing smbus2. Install with: sudo apt install -y python3-smbus  (or: pip install smbus2)")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bus", type=lambda s: int(s, 0), default=1, help="I2C bus number (default: 1)")
    p.add_argument("--address", type=lambda s: int(s, 0), default=0x48, help="I2C address, e.g. 0x48 (default: 0x48)")
    p.add_argument("--registers", type=str, default="0x00,0x01",
                   help="Comma-separated register list, e.g. 0x00,0x01,0x02 (default: 0x00,0x01)")
    p.add_argument("--duration", type=float, default=15.0, help="How long to read for, in seconds (default: 15)")
    p.add_argument("--interval", type=float, default=0.2, help="Seconds between reads (default: 0.2)")
    p.add_argument("--retries", type=int, default=4, help="Retries per read on OSError (default: 4)")
    p.add_argument("--retry-delay", type=float, default=0.02, help="Seconds between retries (default: 0.02)")
    return p.parse_args()


def read_with_retries(fn, retries, retry_delay):
    last_err = None
    for _ in range(retries):
        try:
            return fn(), None
        except OSError as e:
            last_err = e
            time.sleep(retry_delay)
    return None, last_err


def main():
    args = parse_args()
    registers = [int(r, 0) for r in args.registers.split(",") if r.strip()]

    print(f"Bus {args.bus}, address {hex(args.address)}, registers {[hex(r) for r in registers]}")
    print(f"Reading for {args.duration}s every {args.interval}s. Move whatever you're testing (e.g. the line) now.")
    print("Ctrl+C to stop early.\n")

    bus = SMBus(args.bus)
    last_values = {}
    fail_counts = {"plain": 0, **{r: 0 for r in registers}}
    end = time.time() + args.duration

    try:
        while time.time() < end:
            ts = time.strftime("%H:%M:%S")
            row = []

            plain, err = read_with_retries(lambda: bus.read_byte(args.address), args.retries, args.retry_delay)
            if err is not None:
                fail_counts["plain"] += 1
                row.append("plain=FAIL")
            else:
                row.append(f"plain=0x{plain:02x}")

            changed = False
            for reg in registers:
                val, err = read_with_retries(
                    lambda r=reg: bus.read_byte_data(args.address, r), args.retries, args.retry_delay
                )
                if err is not None:
                    fail_counts[reg] += 1
                    row.append(f"reg{hex(reg)}=FAIL")
                    continue
                row.append(f"reg{hex(reg)}=0x{val:02x} (0b{val:08b})")
                if last_values.get(reg) != val:
                    changed = True
                last_values[reg] = val

            if changed or not last_values:
                print(f"[{ts}]  " + "  ".join(row))

            time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        bus.close()

    print("\nDone.")
    print(f"Failures — plain read: {fail_counts['plain']}", end="")
    for reg in registers:
        print(f", reg{hex(reg)}: {fail_counts[reg]}", end="")
    print()
    print("\nWhat to look for:")
    print("  - Values that never change while you're moving the target under the")
    print("    sensor -> probably NOT the right device/register for this test.")
    print("  - A handful of retried failures is normal I2C flakiness; if EVERY")
    print("    read fails, that's wiring/address, not flakiness.")


if __name__ == "__main__":
    main()
