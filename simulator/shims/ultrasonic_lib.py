#!/usr/bin/env python3
"""Simulator ultrasonic alias shim."""

from sonar_lib import Sonar, get_sonar, reset_sonar


Ultrasonic = Sonar


def get_ultrasonic(*args, **kwargs):
    return get_sonar(*args, **kwargs)
