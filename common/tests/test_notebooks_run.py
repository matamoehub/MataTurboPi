import os
from pathlib import Path

import nbformat
import pytest
from nbclient import NotebookClient
from nbclient.exceptions import CellExecutionError


TEST_DIR = Path(__file__).resolve().parent

# Any notebooks you want to exclude from automated runs (optional)
SKIP = {
    # "06_animate.ipynb",
}

# If you want to mark some notebooks as hardware-only, put them here
HARDWARE = {
    # "04_eyes.ipynb",
    # "02_camera.ipynb",
}


def run_notebook(path: Path, timeout_s: int = 180) -> None:
    nb = nbformat.read(path, as_version=4)

    # Make execution behave more like a CI run
    client = NotebookClient(
        nb,
        timeout=timeout_s,
        kernel_name=os.environ.get("NB_KERNEL", "python3"),
        allow_errors=False,
    )

    # Execute in the notebook's folder so relative paths work
    client.execute(cwd=str(path.parent))


def notebook_paths():
    for p in sorted(TEST_DIR.glob("*.ipynb")):
        if p.name in SKIP:
            continue
        yield p


@pytest.mark.parametrize("nb_path", list(notebook_paths()), ids=lambda p: p.name)
def test_notebook_executes(nb_path: Path):
    # Optional skip for hardware notebooks unless explicitly enabled
    if nb_path.name in HARDWARE and os.environ.get("RUN_HARDWARE", "0") != "1":
        pytest.skip("hardware notebook (set RUN_HARDWARE=1 to run)")

    try:
        run_notebook(nb_path)
    except CellExecutionError as e:
        # Give a readable failure
        raise AssertionError(f"Notebook failed: {nb_path.name}\n\n{e}") from e
