"""Drop-in simulator shim for common/lib/camera_lib.py."""

from __future__ import annotations

from typing import Optional

from simulator.core.sim_state import load_state, save_state


class Camera:
    def __init__(
        self,
        node_name: str = "cam_head_sim",
        nod_id: int = 1,
        shake_id: int = 2,
        center: int = 1500,
        speed_s: float = 0.20,
        min_pos: int = 1000,
        max_pos: int = 2000,
    ):
        self.node_name = node_name
        self.nod_id = int(nod_id)
        self.shake_id = int(shake_id)
        self.center = int(center)
        self.speed_s = float(speed_s)
        self.min_pos = int(min_pos)
        self.max_pos = int(max_pos)

        st = load_state()
        st["camera"]["center"] = self.center
        st["camera"]["min_pos"] = self.min_pos
        st["camera"]["max_pos"] = self.max_pos
        if "yaw" not in st["camera"]:
            st["camera"]["yaw"] = self.center
        if "pitch" not in st["camera"]:
            st["camera"]["pitch"] = self.center
        save_state(st)

    def _clamp(self, pos: int) -> int:
        return max(self.min_pos, min(self.max_pos, int(pos)))

    def _set(self, key: str, pos: int):
        st = load_state()
        st["camera"][key] = self._clamp(pos)
        st["last_command"] = f"camera_{key}"
        save_state(st)

    def _send(self, sid: int, pos: int, speed_s: Optional[float] = None):
        if int(sid) == self.nod_id:
            self._set("pitch", pos)
        elif int(sid) == self.shake_id:
            self._set("yaw", pos)

    def center_all(self, speed_s: Optional[float] = None):
        self._set("pitch", self.center)
        self._set("yaw", self.center)

    def set_pitch(self, pos: int, speed_s: Optional[float] = None):
        self._set("pitch", pos)

    def set_yaw(self, pos: int, speed_s: Optional[float] = None):
        self._set("yaw", pos)

    def nod(self, depth: int = 300, speed_s: Optional[float] = None):
        c = self.center
        self.set_pitch(c + depth, speed_s)
        self.set_pitch(c - depth, speed_s)
        self.set_pitch(c, speed_s)

    def shake(self, width: int = 300, speed_s: Optional[float] = None):
        c = self.center
        self.set_yaw(c - width, speed_s)
        self.set_yaw(c + width, speed_s)
        self.set_yaw(c, speed_s)

    def wiggle(self, cycles: int = 2, amplitude: int = 200, speed_s: Optional[float] = None):
        c = self.center
        for _ in range(int(cycles)):
            self.set_yaw(c - int(amplitude), speed_s)
            self.set_yaw(c + int(amplitude), speed_s)
        self.set_yaw(c, speed_s)

    def tiny_wiggle(self, seconds: float = 2.0, amplitude: int = 90, speed_s: float = 0.12):
        self.wiggle(cycles=max(1, int(seconds / max(0.01, speed_s))), amplitude=amplitude, speed_s=speed_s)

    def glance_left(self, amplitude: int = 250, hold_s: float = 0.15):
        self.set_yaw(self.center - int(amplitude))
        self.set_yaw(self.center)

    def glance_right(self, amplitude: int = 250, hold_s: float = 0.15):
        self.set_yaw(self.center + int(amplitude))
        self.set_yaw(self.center)

    def look_up(self, amplitude: int = 250, hold_s: float = 0.15):
        self.set_pitch(self.center + int(amplitude))
        self.set_pitch(self.center)

    def look_down(self, amplitude: int = 250, hold_s: float = 0.15):
        self.set_pitch(self.center - int(amplitude))
        self.set_pitch(self.center)


_CAM_SINGLETON: Optional[Camera] = None


def get_camera() -> Camera:
    global _CAM_SINGLETON
    if _CAM_SINGLETON is None:
        _CAM_SINGLETON = Camera()
    return _CAM_SINGLETON


try:
    cam
except NameError:
    cam = get_camera()
