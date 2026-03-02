"""Drop-in simulator shim for common/lib/eyes_lib.py."""

from __future__ import annotations

from typing import Optional, Tuple

from simulator.core.sim_state import load_state, save_state

DEFAULT_INDICES = (0, 1)


class Eyes:
    def __init__(self, topic: Optional[str] = None, indices: Tuple[int, int] = DEFAULT_INDICES, node_name: str = "eyes_sim"):
        self.topic = topic or "/sim/eyes"
        self.left_i = int(indices[0])
        self.right_i = int(indices[1])
        self.node_name = node_name

    def _set(self, side: str, r: int, g: int, b: int) -> None:
        st = load_state()
        st["eyes"][side] = [int(r), int(g), int(b)]
        st["last_command"] = f"eyes_{side}"
        save_state(st)

    def set_both(self, r: int, g: int, b: int) -> None:
        st = load_state()
        rgb = [int(r), int(g), int(b)]
        st["eyes"]["left"] = rgb
        st["eyes"]["right"] = rgb
        st["last_command"] = "eyes_both"
        save_state(st)

    def set_left(self, r: int, g: int, b: int) -> None:
        self._set("left", r, g, b)

    def set_right(self, r: int, g: int, b: int) -> None:
        self._set("right", r, g, b)

    def set_index(self, idx: int, r: int, g: int, b: int) -> None:
        if int(idx) == self.left_i:
            self.set_left(r, g, b)
        elif int(idx) == self.right_i:
            self.set_right(r, g, b)

    def off(self) -> None:
        self.set_both(0, 0, 0)

    def blink(self, color=(255, 255, 255), period_s=0.25, duration_s=1.0) -> None:
        self.set_both(*color)

    def scan_indices(self, start: int = 0, end: int = 16, color=(0, 0, 255), hold_s: float = 0.35) -> None:
        self.set_both(*color)

    def diagnose(self) -> None:
        print("eyes_lib simulator active; topic:", self.topic)


_EYES_SINGLETON: Optional[Eyes] = None


def get_eyes(topic: Optional[str] = None, indices: Tuple[int, int] = DEFAULT_INDICES) -> Eyes:
    global _EYES_SINGLETON
    if _EYES_SINGLETON is None:
        _EYES_SINGLETON = Eyes(topic=topic, indices=indices)
    return _EYES_SINGLETON
