# Genesis

This folder contains a self-contained URDF and mesh set for Genesis, with a
fixed base, 16 actuated joints, and 370 convex collision meshes.

## Setup

Use a Python environment supported by Genesis and install the tested version:

```sh
python3 -m venv .venv
. .venv/bin/activate
python -m pip install "genesis-world==1.4.4" numpy
```

## Run

From this folder:

```sh
python run.py
python run.py --pose fusion_current
python run.py --headless --steps 50
python run.py --headless --steps 50 --report validation_run.json
```

The GUI run uses the Genesis viewer. The headless run initializes on CPU and
executes the requested number of steps. `--report` writes a small JSON summary.

`neutral` sets all joint coordinates to zero. `fusion_current` loads the pose
captured from Fusion from `initial_state.json`.

## Collision and cache behavior

The runner enables self-collision and neutral collision while disabling collision
between adjacent links. Automatic mesh watertightening is disabled because the
input already contains prepared convex collision geometry.

Genesis and Taichi caches are redirected to a temporary directory that is removed
when the process exits. Dependencies installed in a project virtual environment
remain in that environment.

The verified run used Genesis 1.4.4 on CPU for 50 headless steps. This is a
runtime smoke test, not a full controller or hardware validation.
