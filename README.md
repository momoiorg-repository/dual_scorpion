Dual_scorpion is the so-101 based integrated dual arm robot.
It is the independent fork of the so-101 design repository, and
is optimized for dual arm operation.
We added 2 DoF to the arm
created clavicle rod, spine and base.

dual_scorpion (CLI types `dual_scorpion_follower` and `dual_scorpion_leader`) extends the original SO-101 arm into a 7-DOF + gripper bimanual platform. The repository bundles printable parts, servo bring-up utilities, Hugging Face dataset hooks, and scripts for teleoperation, logging, and replay.

**References**
- Original SO-101 repository (Hugging Face LeRobot): https://github.com/huggingface/lerobot
- SO-101 hardware/software documentation on the Hugging Face Hub®: https://huggingface.co/docs/lerobot/so101
- Project overview video (YouTube): https://www.youtube.com/watch?v=a1u_bPGSeXs
- Hugging Face® is a registered trademark of Hugging Face, Inc.

## Meta Quest arm teleoperation

The isolated Telegrip WebXR add-on and its local/remote Quest setup guide are
in [`telegrip_teleoperation/`](telegrip_teleoperation/README.md). That folder
contains arm teleoperation only and reuses this repository's existing Dual
Scorpion follower driver.

## Contents
- [Highlights](#highlights)
- [Quickstart](#quickstart)
- [Parts for Dual Scorpion Leader and Follower](#parts-for-dual-scorpion-leader-and-follower)
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
- Linux or macOS host with Python 3.12+.
- `pip`, `venv`, and a recent `git`.
- Feetech/BX servo buses (through the `[feetech]` extra).

### Install
```bash
git clone https://github.com/momoiorg-repository/dual_scorpion.git
cd dual_scorpion
python -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -e ".[core_scripts,feetech]"
```

### Optional: authenticate with Hugging Face
```bash
hf auth login
export HF_USER=<your-hf-username>
```

## Parts for Dual Scorpion Leader and Follower

This list is for building the Dual Scorpion setup, including both the Leader and Follower arms.

For this version, all motors are standardized as **STS3215 Servo 7.4V, 19kg·cm, 1:345 gear ratio**.

- Leader: `(7 + 1) × 2 = 16 motors`
- Follower: `(7 + 1) × 2 = 16 motors`
- Total: `(7 + 1) × 4 = 32 motors`

The body frame is assembled using metal frames and connectors.
Cameras are not included in this parts list.

> * It is also possible to use the 12V version of the servo, depending on your design, power supply, and torque requirements.

| Part | Amount | Unit Cost (US) | Buy US | Unit Cost (JPY) | Buy JP |
| --- | ---: | ---: | --- | ---: | --- |
| STS3215 Servo 7.4V, 19kg·cm, 1:345 gear | 32 | $13.89 | [Alibaba](https://www.alibaba.com/product-detail/Top-Seller-Low-Cost-Feetech-STS3215_1600999461525.html) | ￥2,980 | [Akizuki Denshi](https://akizukidenshi.com/catalog/g/g116312/) |
| Motor Control Board | 4 | $10.6 | [Amazon US](https://www.amazon.com/Waveshare-Integrates-Control-Circuit-Supports/dp/B0CTMM4LWK/) | ￥980 | [Akizuki Denshi](https://akizukidenshi.com/catalog/g/g131227/) |

### Frame Parts

In Japan, these frame parts were purchased at a local home center, but they are also available online.

| Japan Part | Buy JP | US Equivalent | Reference |
| --- | --- | --- | --- |
| SGF-0004 | [G-Fun Japan](https://store.g-fun.jp/item/detail.php?ItemNo=SGF-0004) | GFF-000 | [SUS America GFF-000](https://www.susamericainc.com/products/detail.php?item_no=GFF-000) |
| SGF-0265 | [G-Fun Japan](https://store.g-fun.jp/item/detail.php?ItemNo=SGF-0265) | GFJ-200 | [SUS America GFJ-200](https://www.susamericainc.com/products/detail.php?item_no=GFJ-200) |

### Approximate Dimensions

For the current setup, the approximate dimensions are:

- Body/frame rod height: about 60 cm
- Setup width: about 30–45 cm
- Recommended width: 45 cm

These dimensions are still approximate and can be adjusted depending on the build.

### Notes

- This BOM focuses on the motors, motor control boards, and body frame parts.
- Camera parts are intentionally omitted from this list.
- The current Dual Scorpion setup uses only **STS3215 7.4V 19kg·cm 1:345** servos for consistency.
- The 12V version can also be used, but the power supply and wiring should be selected accordingly.

## Hardware Bring-Up
1. **Print & assemble** the follower and leader parts from `dual_scorpion_3d_printer_parts/`.
2. **Wire servos** to their controllers, mirroring IDs between left and right buses.
3. **Probe USB ports** (`lerobot-find-port`) to map `/dev/tty*` or `COM*` devices.

### Set Motor IDs
The wizard flashes IDs in this order: RIGHT gripper → joint6 … joint0, then LEFT gripper → joint6 … joint0.

- **Both arms (normal)**  
  ```bash
  lerobot-setup-motors \
    --robot.type=dual_scorpion_follower \
    --robot.left_arm_port=/dev/tty.usbmodemLEFT \
    --robot.right_arm_port=/dev/tty.usbmodemRIGHT
  ```

- **Left arm only** (right side ignored; dummy port OK)  
  ```bash
  lerobot-setup-motors \
    --robot.type=dual_scorpion_follower \
    --robot.left_arm_port=/dev/tty.usbmodem5A680111991 \
    --robot.right_arm_port=/dev/null \
    --arm left
  ```

- **Right arm only**  
  ```bash
  lerobot-setup-motors \
    --robot.type=dual_scorpion_follower \
    --robot.right_arm_port=/dev/tty.usbmodemRIGHT \
    --robot.left_arm_port=/dev/null \
    --arm right
  ```

Notes:
- `--arm` accepts `left`, `right`, or `both` (default).
- Keep both port flags; the unused side can point to `/dev/null` (macOS/Linux) or `NUL` (Windows).
- If `--arm` doesn’t show up in `--help`, ensure you’re using the project venv binary: `source .venv/bin/activate && rehash`.

### Calibrate Encoders
```bash
lerobot-calibrate \
  --robot.type=dual_scorpion_follower \
  --robot.left_arm_port=/dev/tty.usbmodemLEFT \
  --robot.right_arm_port=/dev/tty.usbmodemRIGHT \
  --robot.use_degrees=true
```
Follow the prompts to sweep every joint. Calibration artifacts are cached locally and reused by the runtime.

To redo only one joint while keeping the rest of an existing calibration JSON, pass `--joints`.
For Dual Scorpion, `joint5` is accepted as an alias for the internal `joint5` key (motor ID 6).

```bash
lerobot-calibrate \
  --robot.type=dual_scorpion_follower \
  --robot.left_arm_port=/dev/tty.usbmodemLEFT \
  --robot.right_arm_port=/dev/tty.usbmodemRIGHT \
  --robot.id=scorpion_follower_parallel_gripper \
  --robot.use_degrees=true \
  --joints=joint5

lerobot-calibrate \
  --teleop.type=dual_scorpion_leader \
  --teleop.left_arm_port=/dev/tty.usbmodemLEFT \
  --teleop.right_arm_port=/dev/tty.usbmodemRIGHT \
  --teleop.id=scorpion_leader_parallel_gripper \
  --teleop.use_degrees=true \
  --joints=joint5
```

Use `--joints=right_joint5` or `--joints=left_joint5` if only one arm should be updated.
Only the selected JSON entries are updated, but the selected arm's motors are unlocked during the
procedure so the joint can be moved by hand. The live table still shows every joint on that arm; selected
rows are marked with `*`.

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
- `max_relative_target` – per-joint clamp that keeps relative commands within a safe offset (can be scalar or a dict keyed by names such as `right_joint0`).
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

## 3D Printed Parts
`dual_scorpion_3d_printer_parts/` hosts the STL sets for both follower and leader builds (base plates, spine, clavicle rods, wrists, grippers, etc.).

## Acknowledgments
Dual_scorpion is designed based on the design of so-101. The development team of so-101 agreed to create a derivative model, and we are deeply grateful to them.
- Built on the SO-101 open-hardware design from the Hugging Face LeRobot project (https://github.com/huggingface/lerobot).
- Documentation and calibration guidance reference the official SO-101 guide on the Hugging Face Hub® (https://huggingface.co/docs/lerobot/so101).
- This derivative work remains under the Apache License 2.0 in alignment with the upstream project’s licensing.

## Contributing
Issues and PRs are encouraged—especially around documentation gaps, new teleop tooling, and CAD improvements. Please open an issue before large hardware or API changes so we can coordinate on interfaces.

## License
Apache License 2.0 (see `LICENSE`). Use the assets freely within the terms, and consider sharing back improvements to benefit other dual_scorpion builders.

**Author:** CHEN JUNGMING (陳俊銘)
