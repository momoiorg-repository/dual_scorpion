# dual_scorpion
hardware design of SO-101 based dual arm robot

Dual_scorpion is the so-101 based integrated dual arm robot.
fork of the so-101 design repository
optimized for dual arm operation
added 2 DoF to the arm
created clavicle rod, spine and base


---

## Highlights

- Dual 7-DOF follower arm implementation with articulated grippers.
- Matching teleoperator (leader) arms for real-time bimanual control.
- CLI workflows for motor setup, calibration, teleoperation, recording, and replay (`lerobot-*` commands).
- Hugging Face dataset integration for storing demonstrations and policies.
- Open hardware: printable parts shipped under `dual_scorpion_3d_printer_parts/`.


---

## Installation

```bash
git clone https://github.com/syun88/dual_scropion.git
cd dual_scropion
python -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -e ".[feetech]"
```

Authenticate with Hugging Face if you plan to push datasets:

```bash
huggingface-cli login
export HF_USER=<your-hf-username>
```

---

## Hardware Bring-Up

1. **3D print and assemble** parts from `dual_scorpion_3d_printer_parts/`.
2. **Wire servos** to their controller boards, keeping IDs consistent between the left and right arms.
3. **Verify USB ports** with `lerobot-find-port` to identify `/dev/tty*` (Linux/macOS) or `COM*` (Windows).

### Set Motor IDs

```bash
lerobot-setup-motors \
  --robot.type=dual_scropion_follower \
  --robot.left_arm_port=/dev/tty.usbmodemXXXXX \
  --robot.right_arm_port=/dev/tty.usbmodemYYYYY
```

This utility walks through each servo and assigns the expected IDs.

### Calibrate Encoders

```bash
lerobot-calibrate \
  --robot.type=dual_scropion_follower \
  --robot.left_arm_port=/dev/tty.usbmodemXXXXX \
  --robot.right_arm_port=/dev/tty.usbmodemYYYYY \
  --robot.use_degrees=true
```

Follow the prompts to move each joint through its range of motion. Calibration files are written to your local cache and automatically re-used.

---

## Configuration

The follower configuration lives in `src/lerobot/robots/dual_scropion_follower/config_dual_scropion_follower.py`.

Key fields:
- `right_arm_port` / `left_arm_port`: USB device paths for each servo bus.
- `max_relative_target`: Optional per-joint safety clamp for relative commands.
- `cameras`: Mapping of camera IDs to `CameraConfig` definitions.
- `use_degrees`: Toggle between degree and normalized units for actions.

Create a Python config or CLI override to match your setup:

```python
from lerobot.robots.dual_scropion_follower import DualScropionFollowerConfig

config = DualScropionFollowerConfig(
    right_arm_port="/dev/tty.usbmodemRIGHT",
    left_arm_port="/dev/tty.usbmodemLEFT",
    use_degrees=True,
    cameras={
        "top": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30},
    },
    max_relative_target=12,
)
```

---

## Runtime Workflows

### Teleoperate

```bash
lerobot-teleoperate \
  --robot.type=dual_scropion_follower \
  --robot.left_arm_port=/dev/tty.usbmodemFOLLOWER_L \
  --robot.right_arm_port=/dev/tty.usbmodemFOLLOWER_R \
  --teleop.type=dual_scropion_leader \
  --teleop.left_arm_port=/dev/tty.usbmodemLEADER_L \
  --teleop.right_arm_port=/dev/tty.usbmodemLEADER_R \
  --display_data=true
```

The leader arm mirrors joint commands onto the follower and streams any configured camera feeds.

### Record Datasets

```bash
lerobot-record \
  --robot.type=dual_scropion_follower \
  --robot.left_arm_port=/dev/tty.usbmodemFOLLOWER_L \
  --robot.right_arm_port=/dev/tty.usbmodemFOLLOWER_R \
  --teleop.type=dual_scropion_leader \
  --teleop.left_arm_port=/dev/tty.usbmodemLEADER_L \
  --teleop.right_arm_port=/dev/tty.usbmodemLEADER_R \
  --robot.id=scorpion_follower \
  --teleop.id=scorpion_leader \
  --dataset.repo_id=${HF_USER}/dual_scropion_demo \
  --dataset.single_task="Pick and place" \
  --dataset.num_episodes=10
```

Frames, actions, and metadata are stored locally and optionally pushed to the Hugging Face Hub.

### Replay Policies or Datasets

```bash
lerobot-replay \
  --robot.type=dual_scropion_follower \
  --robot.left_arm_port=/dev/tty.usbmodemFOLLOWER_L \
  --robot.right_arm_port=/dev/tty.usbmodemFOLLOWER_R \
  --dataset.repo_id=${HF_USER}/dual_scropion_demo \
  --dataset.episode=0
```

Use `replay_loop.sh` for repeated evaluation runs after exporting `HF_USER`.

---

## 3D Printed Parts

The STL models under Follower `dual_scorpion_3d_printer_parts/Follower` and Leader cover base plates, joint mounts, gripper jaws, and wrist assemblies. Manufacture guidance (material, infill, hardware) is tracked in the project wiki under construction. Contributions with assembly notes, BOMs, and CAD updates are welcome.

---