# Scorpion Simulation Model

This directory contains a portable simulation asset set generated from the
Fusion 360 design `scorpion v11`. The model is a bimanual robot with a fixed
base: 17 links and 16 actuated revolute joints, with seven joints on each arm.

The extracted model contains 70 unique BRep bodies. Four hidden obsolete
duplicate bodies whose names contain `_miss` are intentionally excluded. The
separate one-body-per-file STEP export is available in `../STEP/follower` when
that output directory is included with the repository. The root-level ST3215
servo assemblies are design references rather than link geometry, so they are
not included in the 70-body export.

> This is a CAD-derived simulation model, not a hardware certification. Mass,
> inertia, joint limits, drive gains, and collision geometry must be reviewed
> before use with a physical robot.

## Choose a simulator

Run the commands from this directory unless noted otherwise. Each engine folder
has its own dependencies, options, and validation notes.

| Target | Entry point | Quick check |
| --- | --- | --- |
| MuJoCo | `mujoco/run.py` | `cd mujoco && python3 run.py --headless --steps 1000` |
| Genesis | `genesis/run.py` | `cd genesis && python3 run.py --headless --steps 50` |
| ROS 2 / RViz | `urdf/launch/display.launch.py` | See [`urdf/README.md`](urdf/README.md) |
| Isaac Sim / OpenUSD | `isaac_sim/run.py` | See [`isaac_sim/README.md`](isaac_sim/README.md) |
| Gazebo Sim | `gazebo_jetty/run.sh` | `cd gazebo_jetty && sh run.sh` |

## Contents

| Path | Purpose |
| --- | --- |
| `urdf/` | Generic URDF, ROS 2 URDF, SRDF, launch files, visual meshes, and convex collision meshes |
| `mujoco/` | Self-contained MJCF model and passive-viewer/headless runner |
| `genesis/` | Self-contained Genesis URDF, meshes, SRDF, and headless/GUI runner |
| `isaac_sim/` | OpenUSD stage, current-pose overlay, and Isaac Sim runner |
| `gazebo_jetty/` | SDF 1.12 model, world, mesh resources, and launch script |
| `source/` | Fusion extraction metadata, collision policy, validation reports, and SHA-256 manifest |

The mesh files are duplicated between engine folders so that each folder can be
used independently. All model and mesh coordinates are in meters. URDF, SDF,
and USD use SI units.

## Link and joint layout

- `base_link` is fixed and contains the base, body, shoulder mounts, and motor cases.
- `r_link_1` through `r_link_8` form the right arm.
- `l_link_1` through `l_link_8` form the left arm.
- `joint0_*` through `joint7_*` are the 16 revolute joints. The `*_EE_*` names
  identify end-effector bodies in the terminal links; they are not extra joints.

`neutral` means all 16 joint coordinates are zero. `fusion_current` means the
pose captured from the Fusion display. The captured pose is useful for checking
the extraction, but it is not a calibrated home position.

Every engine folder includes `initial_state.json`, containing the captured joint
coordinates in radians.

## Collision setup

The visual model uses one high-refinement mesh per link. Collision is represented
by 370 convex pieces: CoACD compound decomposition for brackets and convex hulls
for enclosed motor parts. These shapes are approximations, not exact CAD solids.

Self-collision and external collision are enabled. The only intended exclusions
are the 16 directly connected parent-child link pairs. They are recorded in:

- `urdf/scorpion.srdf` and `urdf/collision_config.json`
- `genesis/scorpion.srdf` and `genesis/collision_config.json`
- `mujoco/scorpion.xml` (`<contact><exclude>` entries)
- `isaac_sim/scorpion.usda` filtered-pair relationships
- `gazebo_jetty/models/scorpion/model.sdf`

Many URDF consumers do not apply SRDF exclusions automatically. Enable
self-collision in the consumer and import or reproduce the supplied exclusion
policy. Do not enable collisions between every pair of links without checking
the connected clamp geometry.

## Mass and actuation assumptions

The CAD-derived total mass is approximately 16.09 kg. Frame and motor CAD bodies
retain their Fusion density, while printed bodies are assumed to be solid PLA at
1240 kg/m3. Joint efforts and speed limits are placeholders: 1 N m and 2 rad/s.
MuJoCo and Gazebo use provisional position gains of 10 and 0.5. MuJoCo caps
actuator force to +/-1 N m, while Gazebo uses the provisional joint effort limit.
Motor mass, infill, fasteners, wiring, and measured servo characteristics are not
included.

The mechanical joint range has not been calibrated, so the URDF joints are
continuous. Replace the placeholder limits and controller values before using
this model for hardware control or safety analysis.

## CAD checks and known limitations

The revised `joint6_R/L` and `joint7_R/L` bodies match their legacy mounting-hole
positions to within 4.51e-08 mm. Four same-link CAD intersections remain between
those revised bodies and their motor cases: about 115.6 mm3 for each joint6 and
about 305.3 mm3 for each joint7. Rigid-body self-collision cannot detect
collisions between bodies merged into one link. Review `source/cad_overlaps.json`
and verify the physical assembly before relying on the printed parts.

The collision sweep covered the neutral pose, the captured Fusion pose, and each
joint at +/-5 degrees. It did not cover the uncalibrated full mechanical range.

## Validation scope

- MuJoCo 3.16.0 compiled the model with 16 degrees of freedom and 16 actuators,
  matched 34 tested poses, and ran 1000 gravity steps with finite state.
- The MuJoCo collision probe detected all 17 links. The tested static poses had
  no penetration above 0.01 mm.
- Genesis 1.4.4 ran a CPU headless smoke test for 50 steps with finite state.
- OpenUSD opened all three USD assets and found 17 rigid bodies, 16 revolute
  joints, one fixed joint, and an articulation root.
- Gazebo SDF and URDF paths were parsed and checked structurally.
- Isaac Sim / PhysX and Gazebo Sim were not launched for this validation.

The exact test values and timestamps are in `source/validation.json`. The reports
show only the checks listed above; they do not establish full-workspace collision
clearance, controller performance, or safe operation on hardware.

## Verify the distributed files

From this directory:

```sh
shasum -a 256 -c source/SHA256SUMS
```

The repository-level `LICENSE` applies to this directory. See the project
README and acknowledgment section for attribution to SO-101.
