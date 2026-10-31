# Source and Validation Records

This folder preserves the machine-readable evidence used to build the simulator
assets. Paths inside the reports are relative to the individual engine folders.

| File | Purpose |
| --- | --- |
| `fusion_manifest.json` | Per-link body list, volume, mass, center of mass, inertia, and mesh deviation from `scorpion v11`; material labels are normalized to English |
| `fusion_readback.json` | Source document name, version, file ID, modified state, and healthy as-built joint count |
| `model.json` | Consolidated SI link and joint model used for export |
| `mesh_report.json` | Visual triangle counts and collision decomposition summary |
| `collision_config.json` | Self/external collision policy and the 16 adjacent exclusions |
| `alignment_check.json` | Mounting-hole alignment results for the revised joint6 and joint7 bodies |
| `cad_overlaps.json` | Remaining same-link CAD intersections between revised bodies and motor cases |
| `validation.json` | Combined MuJoCo, collision, URDF, Genesis, OpenUSD, and SDF validation record |
| `genesis_validation.json` | Standalone Genesis 1.4.4 CPU headless run summary |
| `SHA256SUMS` | SHA-256 checksums for the distributed package files |

## Verify the package

The paths in `SHA256SUMS` are relative to `dual_scorpion_Full_urdf`, not this
folder. From the package root:

```sh
shasum -a 256 -c source/SHA256SUMS
```

## Interpretation

The reports mix three different evidence levels:

1. CAD extraction and geometry checks.
2. Static file, XML, SDF, and OpenUSD structure checks.
3. MuJoCo and Genesis runtime smoke tests.

The validation reports explicitly identify Isaac Sim / PhysX and Gazebo Sim as
not runtime-tested. Treat the pose sweep as a limited extraction and collision
probe, not a full workspace clearance proof.
