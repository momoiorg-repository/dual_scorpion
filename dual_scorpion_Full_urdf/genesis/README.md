# Genesis

This folder contains a self-contained URDF and mesh set for Genesis.

## Setup

```sh
python3 -m venv .venv
. .venv/bin/activate
python -m pip install "genesis-world==1.4.4" numpy
```

## Run

From this folder:

```sh
python run.py
python run.py --headless --steps 50
```

The GUI run opens the Genesis viewer. The headless run initializes on CPU and
executes the requested number of steps.

## Notes

- The robot has a fixed base and 16 actuated joints.
- Self-collision is enabled, with directly connected links excluded.
- Joint limits and controller values are provisional.
