# common/tests/00_bootstrap.py
from pathlib import Path
import sys

def add_repo_to_path() -> Path:
    """
    Finds the repo root by locating 'lessons/' and 'common/' folders
    and adds:
      - repo root (so `common` and `lessons` imports work)
      - common/lib (so `import camera_lib`, `import tts_lib`, etc works)
    """
    here = Path.cwd().resolve()
    for p in [here] + list(here.parents):
        if (p / "lessons").is_dir() and (p / "common").is_dir():
            repo_root = p
            common_lib = p / "common" / "lib"
            if str(repo_root) not in sys.path:
                sys.path.insert(0, str(repo_root))
            if str(common_lib) not in sys.path:
                sys.path.insert(0, str(common_lib))
            print("Repo root:", repo_root)
            print("Added to sys.path:", common_lib)
            return repo_root
    raise FileNotFoundError("Could not find repo root (needs lessons/ and common/)")

if __name__ == "__main__":
    add_repo_to_path())
