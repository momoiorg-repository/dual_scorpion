# MuJoCo

This folder contains a self-contained MJCF model and run script for Dual Scorpion.

## Setup

```sh
python3 -m venv .venv
. .venv/bin/activate
python -m pip install "mujoco==3.16.0" numpy
```

## Run

From this folder:

```sh
python run.py
python run.py --headless --steps 1000
```

The viewer starts from the neutral pose. The headless run executes the requested
number of simulation steps.

## Notes

The model is fixed at the base and has 16 actuated joints. Collision is enabled
for non-adjacent links. Joint limits, damping, controller gains, and force limits
are provisional values and should be replaced for a specific robot setup.
