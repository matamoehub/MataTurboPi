#!/usr/bin/env python3
"""
Compatibility alias for sonar_lib.
"""
__version__ = "1.1.0"


from sonar_lib import Sonar, get_sonar


Ultrasonic = Sonar


def get_ultrasonic(*args, **kwargs):
    return get_sonar(*args, **kwargs)
