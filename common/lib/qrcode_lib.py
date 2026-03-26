#!/usr/bin/env python3
"""
Client wrapper for the app QR-code node.
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
except Exception as e:  # pragma: no cover - depends on robot runtime
    SetBool = None
    Trigger = None
    _QRCODE_IMPORT_ERROR = e
else:
    _QRCODE_IMPORT_ERROR = None


def _require_interfaces() -> None:
    if _QRCODE_IMPORT_ERROR is not None or Trigger is None or SetBool is None:
        raise RuntimeError(f"QRCode interfaces are not available: {_QRCODE_IMPORT_ERROR}")


class QRCodeClient(ROSServiceClient):
    def __init__(self, namespace: str = "/qrcode"):
        _require_interfaces()
        super().__init__("qrcode_client")
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

    def start_recognition(self, enabled: bool = True, timeout_s: float = 3.0):
        req = SetBool.Request()
        req.data = bool(enabled)
        return self.call(self._svc("start_recognition"), SetBool, req, timeout_s=timeout_s)


def get_qrcode(namespace: str = "/qrcode") -> QRCodeClient:
    key = "qrcode_lib:client"
    inst = get_process_singleton(key)
    if inst is None:
        inst = set_process_singleton(key, QRCodeClient(namespace=namespace))
    return inst


def reset_qrcode() -> None:
    key = "qrcode_lib:client"
    inst = get_process_singleton(key)
    if inst is not None:
        try:
            inst.close()
        except Exception:
            pass
    clear_process_singleton(key)
