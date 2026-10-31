# Isaac Sim

This folder contains OpenUSD assets and a run script for Isaac Sim.

## Files

- `scorpion.usd` is the default binary USD stage.
- `scorpion.usda` is the editable ASCII USD stage.
- `scorpion_current_pose.usda` applies an alternative captured pose.

## Run

From this folder, use Isaac Sim's bundled Python:

```sh
<ISAAC_SIM_ROOT>/python.sh run.py
<ISAAC_SIM_ROOT>/python.sh run.py --headless --steps 1000
```

The default pose is neutral. The `scorpion_current_pose.usda` overlay is optional.

## Notes

The asset set has been checked with OpenUSD. Isaac Sim and PhysX runtime behavior
has not been verified yet.
