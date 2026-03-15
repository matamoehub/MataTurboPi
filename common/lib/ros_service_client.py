#!/usr/bin/env python3
"""
Shared ROS2 service-client helper for app node wrappers.
"""

import os
import threading
import time
import builtins
from typing import Any, Dict, Optional, Type

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
except Exception as e:  # pragma: no cover - depends on robot runtime
    rclpy = None
    SingleThreadedExecutor = None
    Node = object  # type: ignore[assignment]
    _RCLPY_IMPORT_ERROR = e
else:
    _RCLPY_IMPORT_ERROR = None


def _require_rclpy() -> None:
    if _RCLPY_IMPORT_ERROR is not None or rclpy is None:
        raise RuntimeError(f"ROS2 is not available: {_RCLPY_IMPORT_ERROR}")


def _rclpy_init_once() -> None:
    _require_rclpy()
    if not rclpy.ok():
        rclpy.init(args=None)


class ROSServiceClient(Node):
    """
    Small helper that keeps a node + executor thread alive for service wrappers.
    """

    def __init__(self, node_name: str):
        _rclpy_init_once()
        uniq = f"{node_name}_{os.getpid()}_{int(time.time() * 1000) % 100000}"
        super().__init__(uniq)
        self._clients: Dict[str, Any] = {}
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_stop = threading.Event()
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

    def _spin_loop(self) -> None:
        while not self._spin_stop.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception:
                time.sleep(0.05)

    def close(self) -> None:
        self._spin_stop.set()
        try:
            self._executor.remove_node(self)
        except Exception:
            pass
        try:
            self.destroy_node()
        except Exception:
            pass

    def _get_client(self, service_name: str, srv_type: Type[Any]):
        client = self._clients.get(service_name)
        if client is None:
            client = self.create_client(srv_type, service_name)
            self._clients[service_name] = client
        return client

    def wait_for_service(self, service_name: str, srv_type: Type[Any], timeout_s: float = 2.0) -> bool:
        client = self._get_client(service_name, srv_type)
        return bool(client.wait_for_service(timeout_sec=float(timeout_s)))

    def call(self, service_name: str, srv_type: Type[Any], request: Any, timeout_s: float = 3.0) -> Any:
        client = self._get_client(service_name, srv_type)
        if not client.wait_for_service(timeout_sec=float(timeout_s)):
            raise TimeoutError(f"Service unavailable: {service_name}")
        future = client.call_async(request)
        end = time.time() + float(timeout_s)
        while time.time() < end:
            if future.done():
                exc = future.exception()
                if exc is not None:
                    raise exc
                return future.result()
            time.sleep(0.02)
        raise TimeoutError(f"Timed out waiting for service response: {service_name}")


_REGISTRY_ATTR = "_mata_robot_singletons"


def _singleton_registry() -> Dict[str, Any]:
    registry = getattr(builtins, _REGISTRY_ATTR, None)
    if registry is None:
        registry = {}
        setattr(builtins, _REGISTRY_ATTR, registry)
    return registry


def get_process_singleton(key: str) -> Any:
    return _singleton_registry().get(str(key))


def set_process_singleton(key: str, value: Any) -> Any:
    _singleton_registry()[str(key)] = value
    return value


def clear_process_singleton(key: str) -> None:
    _singleton_registry().pop(str(key), None)
