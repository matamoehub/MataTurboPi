# MataTurboPi Simulator

Lightweight simulator for running lessons without the physical robot.

## Features

- 2D robot movement visualization
- Eye LED color simulation
- Camera pan/tilt simulation
- Works with existing lesson imports through shim modules

## Backend Switching Per Lesson

In every lesson notebook:

```python
from lesson_loader import setup
setup(backend="sim")   # simulator
# setup(backend="real")  # real robot
```

## Install and Run

### macOS

1. Install Python 3.11+ from [python.org](https://www.python.org/downloads/macos/).
2. Verify:

```bash
python3 --version
python3 -m tkinter
```

3. Install Jupyter:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install jupyterlab
```

4. Run simulator:

```bash
cd /Users/john/Documents/Code/MataTurboPi
python3 simulator/app.py
```

5. Start lessons:

```bash
cd /Users/john/Documents/Code/MataTurboPi
jupyter lab
```

### Windows 10/11

1. Install Python 3.11+ from [python.org](https://www.python.org/downloads/windows/). Check "Add Python to PATH".
2. Verify in PowerShell:

```powershell
python --version
python -m tkinter
```

3. Install Jupyter:

```powershell
python -m pip install --upgrade pip
python -m pip install jupyterlab
```

4. Run simulator (PowerShell):

```powershell
cd C:\path\to\MataTurboPi
python simulator\app.py
```

5. Start lessons in a second PowerShell window:

```powershell
cd C:\path\to\MataTurboPi
jupyter lab
```

### Linux (Ubuntu/Debian)

1. Install Python + Tk + venv + pip:

```bash
sudo apt update
sudo apt install -y python3 python3-tk python3-pip python3-venv
```

2. Install Jupyter:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install jupyterlab
```

3. Run simulator:

```bash
cd /path/to/MataTurboPi
python3 simulator/app.py
```

4. Start lessons in second terminal:

```bash
cd /path/to/MataTurboPi
jupyter lab
```

## Optional: Virtual Environment (all platforms)

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip jupyterlab
```

Windows PowerShell activate:

```powershell
.\.venv\Scripts\Activate.ps1
```

## Troubleshooting

- `ModuleNotFoundError: tkinter`: install Tk (`python3-tk` on Linux) or reinstall Python from python.org.
- Real backend import errors on non-robot machines are expected. Use `setup(backend="sim")`.
- If notebook was already running with old backend, rerun `setup(...)` or restart kernel.
