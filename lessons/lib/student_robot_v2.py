#!/usr/bin/env python3
"""Compatibility shim for student_robot_v2.

The V2 implementation now lives in common/lib so all lessons and robot
workspaces resolve the same shared version.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

_COMMON_IMPL = Path(__file__).resolve().parents[2] / "common" / "lib" / "student_robot_v2.py"
_SPEC = importlib.util.spec_from_file_location("_student_robot_v2_common", _COMMON_IMPL)
if _SPEC is None or _SPEC.loader is None:
    raise ImportError(f"Unable to load common V2 library from {_COMMON_IMPL}")
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)

__all__ = getattr(
    _MODULE,
    "__all__",
    [name for name in dir(_MODULE) if not name.startswith("_") or name == "__version__"],
)

for _name in __all__:
    globals()[_name] = getattr(_MODULE, _name)
