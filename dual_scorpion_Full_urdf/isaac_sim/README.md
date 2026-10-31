# Isaac Sim and OpenUSD

This folder provides a neutral USD stage, an editable USD ASCII source stage, a
Fusion-current-pose overlay, and an Isaac Sim runner.

## Files

- `scorpion.usd` is the neutral binary USD stage used by default.
- `scorpion.usda` is the readable USD ASCII source stage.
- `scorpion_current_pose.usda` overlays the captured Fusion link transforms on
  `scorpion.usd`.
- `run.py` opens a stage, creates an Isaac Sim world, and runs physics steps.
- `initial_state.json` stores the captured joint coordinates in radians.

## Run with Isaac Sim

Run the script with Isaac Sim's bundled Python. From this folder:

```sh
<ISAAC_SIM_ROOT>/python.sh run.py
<ISAAC_SIM_ROOT>/python.sh run.py --pose fusion_current
<ISAAC_SIM_ROOT>/python.sh run.py --headless --steps 1000
```

The default pose is `neutral`. Use `--pose fusion_current` to open the captured
Fusion display pose.

## Validation status

OpenUSD successfully opened all three assets and found 17 rigid bodies, 16
revolute joints, one fixed joint, an articulation root, and meter units. Those
checks verify stage structure only.

Isaac Sim and PhysX were not launched during generation and validation. The
runner is provided for the next runtime test, not as evidence of successful
Isaac Sim physics or rendering.
