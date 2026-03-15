#!/usr/bin/env python3
"""
Client wrapper for the app avoidance node.
"""

from typing import Optional

from ros_service_client import (
    ROSServiceClient,
    clear_process_singleton,
    get_process_singleton,
    set_process_singleton,
)

try:
    from std_srvs.srv import SetBool, Trigger
    from interfaces.srv import SetFloat64List
except Exception as e:  # pragma: no cover - depends on robot runtime
    SetBool = None
    Trigger = None
    SetFloat64List = None
    _AVOIDANCE_IMPORT_ERROR = e
else:
    _AVOIDANCE_IMPORT_ERROR = None


def _require_interfaces() -> None:
    if _AVOIDANCE_IMPORT_ERROR is not None or Trigger is None or SetBool is None or SetFloat64List is None:
        raise RuntimeError(f"Avoidance interfaces are not available: {_AVOIDANCE_IMPORT_ERROR}")


class AvoidanceClient(ROSServiceClient):
    def __init__(self, namespace: str = "/avoidance_node"):
        _require_interfaces()
        super().__init__("avoidance_client")
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

    def set_params(self, threshold_cm: float = 30.0, speed: float = 32.0, timeout_s: float = 3.0):
        req = SetFloat64List.Request()
        req.data = [float(threshold_cm), float(speed)]
        return self.call(self._svc("set_param"), SetFloat64List, req, timeout_s=timeout_s)


def get_avoidance(namespace: str = "/avoidance_node") -> AvoidanceClient:
    key = "avoidance_lib:client"
    inst = get_process_singleton(key)
    if inst is None:
        inst = set_process_singleton(key, AvoidanceClient(namespace=namespace))
    return inst


def reset_avoidance() -> None:
    key = "avoidance_lib:client"
    inst = get_process_singleton(key)
    if inst is not None:
        try:
            inst.close()
        except Exception:
            pass
    clear_process_singleton(key)
