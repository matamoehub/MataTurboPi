# demo_bootstrap.py
# Keeps notebooks clean: fixes broken cwd, finds repo root, adds sys.path,
# and cleans up ROS env (bad CycloneDDS URI).

from pathlib import Path
import os
import sys

BROKEN_CYCLONE_URI = "file:///etc/cyclonedds/config.xml"

def safe_start_dir() -> Path:
    try:
        return Path.cwd().resolve()
    except FileNotFoundError:
        home = os.environ.get("HOME")
        if home:
            return Path(home).resolve()
        return Path("/tmp").resolve()

def find_repo_root(start: Path) -> Path:
    p = start.resolve()
    for _ in range(25):
        if (p / "common").is_dir() and (p / "lessons").is_dir():
            return p
        if p.parent == p:
            break
        p = p.parent
    raise FileNotFoundError(f"Could not find repo root from start={start}")

def setup_paths() -> dict:
    start = safe_start_dir()
    root = find_repo_root(start)
    common_lib = root / "common" / "lib"
    lessons_lib = root / "lessons" / "lib"

    for path in (common_lib, lessons_lib):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))

    return {
        "START": str(start),
        "ROOT": str(root),
        "COMMON_LIB": str(common_lib),
        "LESSONS_LIB": str(lessons_lib),
    }

def setup_ros_env(default_domain: str = "0") -> dict:
    # Remove the known-bad CycloneDDS config
    if os.environ.get("CYCLONEDDS_URI") == BROKEN_CYCLONE_URI:
        os.environ.pop("CYCLONEDDS_URI", None)

    os.environ.setdefault("ROS_DOMAIN_ID", str(default_domain))
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")

    return {
        "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID"),
        "RMW_IMPLEMENTATION": os.environ.get("RMW_IMPLEMENTATION"),
        "CYCLONEDDS_URI": os.environ.get("CYCLONEDDS_URI"),
    }

def bootstrap(default_domain: str = "0", verbose: bool = True) -> dict:
    paths = setup_paths()
    env = setup_ros_env(default_domain=default_domain)

    info = {**paths, **env}
    if verbose:
        print("START:", info["START"])
        print("Repo root:", info["ROOT"])
        print("Using common lib:", info["COMMON_LIB"])
        print("Using lessons lib:", info["LESSONS_LIB"])
        print("ROS_DOMAIN_ID =", info["ROS_DOMAIN_ID"])
        print("RMW_IMPLEMENTATION =", info["RMW_IMPLEMENTATION"])
        print("CYCLONEDDS_URI =", info["CYCLONEDDS_URI"])
    return info
