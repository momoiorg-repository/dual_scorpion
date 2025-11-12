Dual_scorpion is the so-101 based integrated dual arm robot.
It is the independent fork of the so-101 design repository, and
is optimized for dual arm operation.
We added 2 DoF to the arm
created clavicle rod, spine and base.

dual_scorpion (CLI type `dual_scorpion`) extends the original SO-101 arm into a 7-DOF + gripper bimanual platform. The repository bundles printable parts, servo bring-up utilities, Hugging Face dataset hooks, and scripts for teleoperation, logging, and replay.

**References**
- Original SO-101 repository (Hugging Face LeRobot): https://github.com/huggingface/lerobot
- SO-101 hardware/software documentation on the Hugging Face Hub®: https://huggingface.co/docs/lerobot/so101
- Project overview video (YouTube): https://www.youtube.com/watch?v=a1u_bPGSeXs
- Hugging Face® is a registered trademark of Hugging Face, Inc.

## Contents
- [Highlights](#highlights)
- [Quickstart](#quickstart)
- [Hardware Bring-Up](#hardware-bring-up)
- [Configuration](#configuration)
- [Runtime Workflows](#runtime-workflows)
- [3D Printed Parts](#3d-printed-parts)
- [Contributing](#contributing)
- [License](#license)

## Highlights
- Dual 7-DOF follower arms with matching leader arms for real-time bimanual control.
- CLI workflows (`lerobot-*`) cover motor setup, calibration, teleoperation, recording, and replay.
- Hugging Face integration stores demonstrations, policies, and evaluation runs.
- `dual_scorpion_3d_printer_parts/` ships STL files for both follower and leader builds.

## Quickstart

### Requirements
- Linux or macOS host with Python 3.10+.
- `pip`, `venv`, and a recent `git`.
- Feetech/BX servo buses (through the `[feetech]` extra).

### Install
```bash
git clone https://github.com/momoiorg-repository/dual_scorpion.git
cd dual_scorpion
python -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -e ".[feetech]"
```

### Optional: authenticate with Hugging Face
```bash
huggingface-cli login
export HF_USER=<your-hf-username>
```

## Hardware Bring-Up
1. **Print & assemble** the follower and leader parts from `dual_scorpion_3d_printer_parts/`.
2. **Wire servos** to their controllers, mirroring IDs between left and right buses.
3. **Probe USB ports** (`lerobot-find-port`) to map `/dev/tty*` or `COM*` devices.

### Set Motor IDs
```bash
lerobot-setup-motors \
  --robot.type=dual_scorpion_follower \
  --robot.left_arm_port=/dev/tty.usbmodemLEFT \
  --robot.right_arm_port=/dev/tty.usbmodemRIGHT
```
The tool steps through each servo, writes the expected ID, and stores the layout.

### Calibrate Encoders
```bash
lerobot-calibrate \
  --robot.type=dual_scorpion_follower \
  --robot.left_arm_port=/dev/tty.usbmodemLEFT \
  --robot.right_arm_port=/dev/tty.usbmodemRIGHT \
  --robot.use_degrees=true
```
Follow the prompts to sweep every joint. Calibration artifacts are cached locally and reused by the runtime.

## Configuration
All follower parameters live in `src/lerobot/robots/dual_scorpion_follower/config_dual_scorpion_follower.py`. Every `lerobot-*` CLI accepts the same keys via `--robot.*` flags, or you can instantiate the config directly:

```python
from lerobot.robots.dual_scorpion_follower import DualScorpionFollowerConfig

config = DualScorpionFollowerConfig(
    right_arm_port="/dev/tty.usbmodemRIGHT",
    left_arm_port="/dev/tty.usbmodemLEFT",
    disable_torque_on_disconnect=True,
    max_relative_target=12,
    cameras={
        "top": {
            "type": "opencv",
            "index_or_path": 0,
            "width": 640,
            "height": 480,
            "fps": 30,
        },
    },
    use_degrees=True,
)
```

Key fields:
- `right_arm_port` / `left_arm_port` – serial device assigned to each servo daisy-chain.
- `disable_torque_on_disconnect` – drop torque when the driver exits (protects hardware at rest).
- `max_relative_target` – per-joint clamp that keeps relative commands within a safe offset (can be scalar or list).
- `cameras` – optional `dict[str, CameraConfig]` used for streaming or logging.
- `use_degrees` – choose between degrees and normalized joint units.

## Runtime Workflows

### Teleoperate
```bash
lerobot-teleoperate \
  --robot.type=dual_scorpion_follower \
  --robot.left_arm_port=/dev/tty.usbmodemFOLLOWER_L \
  --robot.right_arm_port=/dev/tty.usbmodemFOLLOWER_R \
  --teleop.type=dual_scorpion_leader \
  --teleop.left_arm_port=/dev/tty.usbmodemLEADER_L \
  --teleop.right_arm_port=/dev/tty.usbmodemLEADER_R \
  --display_data=true
```
The leader mirrors joint commands to the follower while streaming configured cameras.

### Record Datasets
```bash
lerobot-record \
  --robot.type=dual_scorpion_follower \
  --robot.left_arm_port=/dev/tty.usbmodemFOLLOWER_L \
  --robot.right_arm_port=/dev/tty.usbmodemFOLLOWER_R \
  --teleop.type=dual_scorpion_leader \
  --teleop.left_arm_port=/dev/tty.usbmodemLEADER_L \
  --teleop.right_arm_port=/dev/tty.usbmodemLEADER_R \
  --robot.id=scorpion_follower \
  --teleop.id=scorpion_leader \
  --dataset.repo_id=${HF_USER}/dual_scorpion_demo \
  --dataset.single_task="Pick and place" \
  --dataset.num_episodes=10
```
Samples are written locally and pushed to the Hugging Face Hub when `HF_USER` is set.

### Replay Policies or Datasets
```bash
lerobot-replay \
  --robot.type=dual_scorpion_follower \
  --robot.left_arm_port=/dev/tty.usbmodemFOLLOWER_L \
  --robot.right_arm_port=/dev/tty.usbmodemFOLLOWER_R \
  --dataset.repo_id=${HF_USER}/dual_scorpion_demo \
  --dataset.episode=0
```
For repeated evaluation runs, use `scripts/replay_loop.sh` after exporting `HF_USER`.

## 3D Printed Parts
`dual_scorpion_3d_printer_parts/` hosts the STL sets for both follower and leader builds (base plates, spine, clavicle rods, wrists, grippers, etc.).

## Acknowledgments
Dual_scorpion is designed based on the so-101 design. The so-101 development team has given us permission to create a derivative model, and we are very grateful to them.
- Built on the SO-101 open-hardware design from the Hugging Face LeRobot project (https://github.com/huggingface/lerobot).
- Documentation and calibration guidance reference the official SO-101 guide on the Hugging Face Hub® (https://huggingface.co/docs/lerobot/so101).
- This derivative work remains under the Apache License 2.0 in alignment with the upstream project’s licensing.

## Contributing
Issues and PRs are encouraged—especially around documentation gaps, new teleop tooling, and CAD improvements. Please open an issue before large hardware or API changes so we can coordinate on interfaces.

## License
Apache License 2.0 (see `LICENSE`). Use the assets freely within the terms, and consider sharing back improvements to benefit other dual_scorpion builders.

**Author:** CHEN JUNGMING (陳俊銘)
