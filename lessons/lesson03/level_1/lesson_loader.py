#!/usr/bin/env python3
"""
Notebook bootstrap helper for lesson folders.
Use in a notebook cell:
    from lesson_loader import setup
    setup()  # bootstraps + preloads common libs
"""

from pathlib import Path
import importlib
import os
import re
import sys
from typing import Dict, Optional

# Common lesson modules to preload from common/lib.
COMMON_MODULES = [
    "robot_moves",
    "eyes_lib",
    "camera_lib",
    "tts_lib",
    "sound",
    "line_sensors",
    "robot_controller_api",
    "infrared_lib",
    "line_follower_lib",
    "buzzer_lib",
]


def _find_repo_root(start: Path) -> Path:
    p = start.resolve()
    for _ in range(30):
        if (p / "common").is_dir() and (p / "lessons").is_dir():
            return p
        if p.parent == p:
            break
        p = p.parent
    raise FileNotFoundError(f"Could not find repo root from {start}")


def _parse_domain_from_name(name: str) -> Optional[str]:
    # Accept names like "robot-8", "turbopi08", "mata_t1_3" -> "8", "8", "3"
    m = re.search(r"(\d+)$", str(name).strip())
    if not m:
        return None
    return str(int(m.group(1)))


def _resolve_domain(default_domain: Optional[object]) -> str:
    # Priority:
    # 1) explicit setup(default_domain=...)
    # 2) ROS_DOMAIN_ID / ROBOT_DOMAIN_ID / ROBOT_NUMBER
    # 3) parse ROBOT_NAME / ROBOT_ID / HOSTNAME suffix digits
    # 4) fallback 0
    if default_domain is not None:
        return str(default_domain)

    for k in ("ROS_DOMAIN_ID", "ROBOT_DOMAIN_ID", "ROBOT_NUMBER"):
        v = os.environ.get(k)
        if v and str(v).strip().isdigit():
            return str(int(str(v).strip()))

    for k in ("ROBOT_NAME", "ROBOT_ID", "HOSTNAME"):
        v = os.environ.get(k)
        if v:
            parsed = _parse_domain_from_name(v)
            if parsed is not None:
                return parsed

    return "0"


def _expose_modules(mods: Dict[str, object]) -> None:
    # Expose modules into the caller notebook global scope (optional convenience).
    try:
        frame = sys._getframe(2)
        g = frame.f_globals
    except Exception:
        return

    g.update(mods)

    # Convenience aliases commonly used in lessons.
    if "robot_moves" in mods:
        g.setdefault("rm", mods["robot_moves"])


def setup(
    default_domain: Optional[object] = None,
    verbose: bool = True,
    preload_common: bool = True,
    expose_globals: bool = True,
):
    """
    Configure sys.path for common/lib + lessons/lib, run bootstrap,
    and optionally preload common lesson modules.

    default_domain:
      - None: infer from env (ROS_DOMAIN_ID / robot vars), fallback "0"
      - int/str: explicit domain override
    """
    try:
        start = Path.cwd()
    except FileNotFoundError:
        start = Path(os.environ.get("HOME", "/tmp"))

    root = _find_repo_root(start)
    common_lib = root / "common" / "lib"
    lessons_lib = root / "lessons" / "lib"

    for path in (common_lib, lessons_lib):
        s = str(path)
        if s not in sys.path:
            sys.path.insert(0, s)

    import bootstrap

    domain = _resolve_domain(default_domain)
    # Ensure downstream ROS imports pick the intended domain.
    os.environ["ROS_DOMAIN_ID"] = domain

    if hasattr(bootstrap, "init"):
        info = bootstrap.init(default_domain=domain, verbose=verbose)
    else:
        info = bootstrap.bootstrap(default_domain=domain, verbose=verbose)

    loaded: Dict[str, object] = {}
    if preload_common:
        for name in COMMON_MODULES:
            try:
                loaded[name] = importlib.import_module(name)
            except Exception as e:
                if verbose:
                    print(f"[lesson_loader] skip {name}: {e}")

        if expose_globals and loaded:
            _expose_modules(loaded)

        if verbose and loaded:
            print("Preloaded:", ", ".join(sorted(loaded.keys())))

    return {"bootstrap": info, "modules": loaded, "ros_domain_id": domain}
