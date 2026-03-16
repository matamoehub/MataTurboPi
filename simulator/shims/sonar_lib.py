#!/usr/bin/env python3
"""Simulator sonar shim backed by simulator state."""

from __future__ import annotations

import time
from typing import Optional

from simulator.core.sim_state import load_state


class Sonar:
    def __init__(self, topic: str = "sim/sonar", window: int = 5):
        self.topic = str(topic)
        self.window = int(window)

    def has_reading(self) -> bool:
        return load_state().get("sonar", {}).get("distance_mm") is not None

    def wait_for_reading(self, timeout_s: float = 2.0) -> int:
        end = time.time() + float(timeout_s)
        while time.time() < end:
            value = load_state().get("sonar", {}).get("distance_mm")
            if value is not None:
                return int(value)
            time.sleep(0.02)
        raise TimeoutError(f"No sonar reading available within {timeout_s}s")

    def get_distance_mm(self, filtered: bool = False) -> Optional[float]:
        value = load_state().get("sonar", {}).get("distance_mm")
        return None if value is None else float(value)

    def get_distance_cm(self, filtered: bool = False) -> Optional[float]:
        value = load_state().get("sonar", {}).get("distance_cm")
        return None if value is None else float(value)

    def is_closer_than(self, threshold_cm: float, filtered: bool = True) -> Optional[bool]:
        value = self.get_distance_cm(filtered=filtered)
        if value is None:
            return None
        return value <= float(threshold_cm)


_SONAR_SINGLETON: Optional[Sonar] = None


def get_sonar(topic: str = "sim/sonar", window: int = 5) -> Sonar:
    global _SONAR_SINGLETON
    if _SONAR_SINGLETON is None:
        _SONAR_SINGLETON = Sonar(topic=topic, window=window)
    return _SONAR_SINGLETON


def reset_sonar() -> None:
    global _SONAR_SINGLETON
    _SONAR_SINGLETON = None
