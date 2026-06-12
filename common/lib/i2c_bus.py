#!/usr/bin/env python3
"""Shared I2C transaction lock for the Hiwonder 0x77 device.

The Hiwonder sonar+RGB module exposes BOTH the ultrasonic sensor and the two
RGB "eye" LEDs on a single MCU at I2C address 0x77 on bus 1.  sonar_lib reads
it (trigger byte -> settle -> 2-byte read) and eyes_lib writes it (per-LED
register writes), often from different threads (e.g. the eyes blink thread
while a movement loop polls the sonar).

Each individual I2C transaction is serialised by the kernel, but a sonar read
is a *sequence* (write trigger, sleep, read result).  An eyes write landing
between the trigger and the read can disturb the MCU and is a plausible source
of the occasional 0x00FF "mid-measurement" sentinel.

This module provides one process-wide lock so sonar and eyes serialise their
*multi-step* bus access against each other.  Both libraries acquire it around
their bus work; anything else touching 0x77 should too.

Usage:
    from i2c_bus import bus_lock
    with bus_lock():
        ... do the full trigger+read or LED write sequence ...
"""
import builtins
import threading

# Stored on builtins so every copy of this module (it can be imported under
# slightly different module names by the lesson bootstrap) shares one lock.
_LOCK_ATTR = "_mata_i2c_0x77_lock"


def bus_lock() -> threading.Lock:
    """Return the process-wide lock guarding the 0x77 I2C device."""
    lock = getattr(builtins, _LOCK_ATTR, None)
    if lock is None:
        lock = threading.Lock()
        setattr(builtins, _LOCK_ATTR, lock)
    return lock
