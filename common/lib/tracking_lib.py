#!/usr/bin/env python3
"""
Client wrapper for the app object-tracking node.
"""
__version__ = "1.1.0"


from typing import Optional

from ros_service_client import (
    ROSServiceClient,
    clear_process_singleton,
    get_process_singleton,
    set_process_singleton,
)

try:
    from std_srvs.srv import SetBool, Trigger
    from interfaces.srv import SetFloat64, SetPoint
    from large_models_msgs.srv import SetString
except Exception as e:  # pragma: no cover - depends on robot runtime
    SetBool = None
    Trigger = None
    SetFloat64 = None
    SetPoint = None
    SetString = None
    _TRACKING_IMPORT_ERROR = e
else:
    _TRACKING_IMPORT_ERROR = None


def _require_interfaces() -> None:
    if (
        _TRACKING_IMPORT_ERROR is not None
        or Trigger is None
        or SetBool is None
        or SetFloat64 is None
        or SetPoint is None
        or SetString is None
    ):
        raise RuntimeError(f"Tracking interfaces are not available: {_TRACKING_IMPORT_ERROR}")


class ObjectTrackingClient(ROSServiceClient):
    def __init__(self, namespace: str = "/object_tracking"):
        _require_interfaces()
        super().__init__("object_tracking_client")
        self.ns = str(namespace).rstrip("/")

    def _svc(self, name: str) -> str:
        return f"{self.ns}/{name}"

    def ready(self, timeout_s: float = 2.0) -> bool:
        return self.wait_for_service(self._svc("init_finish"), Trigger, timeout_s=timeout_s)

    def enter(self, timeout_s: float = 3.0):
        return self.call(self._svc("enter"), Trigger, Trigger.Request(), timeout_s=timeout_s)

    def exit(self, timeout_s: float = 3.0):
        return self.call(self._svc("exit"), Trigger, Trigger.Request(), timeout_s=timeout_s)

    def set_running(self, enabled: bool = True, timeout_s: float = 3.0):
        req = SetBool.Request()
        req.data = bool(enabled)
        return self.call(self._svc("set_running"), SetBool, req, timeout_s=timeout_s)

    def start(self, timeout_s: float = 3.0):
        return self.set_running(True, timeout_s=timeout_s)

    def stop(self, timeout_s: float = 3.0):
        return self.set_running(False, timeout_s=timeout_s)

    def set_threshold(self, threshold: float, timeout_s: float = 3.0):
        req = SetFloat64.Request()
        req.data = float(threshold)
        return self.call(self._svc("set_threshold"), SetFloat64, req, timeout_s=timeout_s)

    def set_pan_tilt(self, enabled: bool = True, timeout_s: float = 3.0):
        req = SetBool.Request()
        req.data = bool(enabled)
        return self.call(self._svc("set_pan_tilt"), SetBool, req, timeout_s=timeout_s)

    def set_chassis_following(self, enabled: bool = True, timeout_s: float = 3.0):
        req = SetBool.Request()
        req.data = bool(enabled)
        return self.call(self._svc("set_chassis_following"), SetBool, req, timeout_s=timeout_s)

    def set_target_point(self, x_norm: float, y_norm: float, timeout_s: float = 3.0):
        req = SetPoint.Request()
        req.data.x = float(x_norm)
        req.data.y = float(y_norm)
        return self.call(self._svc("set_target_color"), SetPoint, req, timeout_s=timeout_s)

    def set_color_name(self, color_name: str, timeout_s: float = 3.0):
        req = SetString.Request()
        req.data = str(color_name)
        return self.call(self._svc("set_large_model_target_color"), SetString, req, timeout_s=timeout_s)

    def get_target_color(self, timeout_s: float = 3.0):
        return self.call(self._svc("get_target_color"), Trigger, Trigger.Request(), timeout_s=timeout_s)


def get_tracking(namespace: str = "/object_tracking") -> ObjectTrackingClient:
    key = "tracking_lib:client"
    inst = get_process_singleton(key)
    if inst is None:
        inst = set_process_singleton(key, ObjectTrackingClient(namespace=namespace))
    return inst


def reset_tracking() -> None:
    key = "tracking_lib:client"
    inst = get_process_singleton(key)
    if inst is not None:
        try:
            inst.close()
        except Exception:
            pass
    clear_process_singleton(key)
