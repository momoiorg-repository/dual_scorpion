# MuJoCo

This folder contains a self-contained MJCF model with 16 hinge joints, 16
position actuators, visual meshes, and 370 convex collision meshes.

## Setup

The supplied model was tested with MuJoCo 3.16.0 and NumPy. Install the pinned
dependencies in a virtual environment:

```sh
python3 -m venv .venv
. .venv/bin/activate
python -m pip install "mujoco==3.16.0" numpy
```

## Run

From this folder:

```sh
python run.py
python run.py --pose fusion_current
python run.py --headless --steps 1000
```

The default run opens the passive MuJoCo viewer at the `neutral` keyframe. The
viewer advances simulation and does not provide an interactive arm controller.
The headless run applies the selected keyframe, executes the requested number of
steps, and checks that all joint positions remain finite.

Available poses are `neutral` and `fusion_current`. `fusion_current` is the pose
captured from Fusion, not a calibrated home pose.

## Collision and control policy

`scorpion.xml` enables self-collision and excludes only the 16 directly connected
parent-child link pairs. Collision geometry is an approximation of the CAD parts.
Position gains, damping, force range, and joint limits are provisional simulation
values; they are not measured servo parameters.

The headless success message is a numerical smoke-test result. It is not a full
workspace collision check or evidence that a real robot is safe to operate.
