"""Translate measured motor coordinates for the bundled Scorpion URDF."""

import math
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
SCORPION_MODEL_DIR = WORKSPACE_ROOT / "dual_scorpion_Full_urdf" / "default"


def supports_verified_view_mapping() -> bool:
    mocopi = WORKSPACE_ROOT / "telegrip_teleoperation/telegrip/mocopi"
    return (mocopi / "dataset/hardware_dataset.py").is_file()


def read_view_mapping(filename, robot):
    """Load a verified Full CAD mapping and optional measured gripper endpoints."""
    if not filename or not supports_verified_view_mapping():
        return {}
    if Path(filename).name != filename or Path(filename).suffix not in {".yaml", ".yml"}:
        raise ValueError("Use the name of a mapping YAML in config/mocopi/hardware.")

    import yaml
    from telegrip.mocopi.dataset.fingerprint import file_sha256
    from telegrip.mocopi.dataset.hardware_dataset import validate_mapping_for_robot

    path = WORKSPACE_ROOT / "config" / "mocopi" / "hardware" / filename
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("mujoco_model_sha256") != file_sha256(
        SCORPION_MODEL_DIR / "mujoco/scorpion.xml"
    ):
        raise ValueError("Mapping MuJoCo model hash differs from the selected model or is missing")
    validate_mapping_for_robot(document, robot)
    return document


def scorpion_joint_frame(positions, mapping):
    """Expose motor readings plus URDF radians wherever model calibration exists."""
    angles = {}
    for side, suffix in (("right", "R"), ("left", "L")):
        if side not in mapping.get("sides", {}):
            continue
        arm = mapping["sides"][side]
        for i, (sign, offset) in enumerate(zip(arm["sign"], arm["offset_deg"], strict=True)):
            value = positions.get(f"{side}_joint{i}.pos")
            if value is not None and math.isfinite(value):
                angles[f"joint{i}_{suffix}"] = math.radians((value - offset) / sign)

        limits = mapping.get("view_gripper_deg", {}).get(side, {})
        closed, opened = limits.get("closed"), limits.get("open")
        gripper = positions.get(f"{side}_gripper.pos")
        if (
            all(type(value) in (int, float) and math.isfinite(value) for value in (closed, opened, gripper))
            and 0 <= gripper <= 100
        ):
            angles[f"joint{len(arm['sign'])}_{suffix}"] = math.radians(
                closed + (opened - closed) * gripper / 100
            )
    return {"robot_model": "dual_scorpion", "joints": angles, "motor_positions": positions}


def recorded_joint_frames(repo_id: str, episode_index: int, robot_name: str | None):
    from . import episode_media
    from .hardware import scorpion_follower
    from .utils.config import get_robot_record, is_valid_robot_name

    dataset_dir = episode_media.resolve_dataset_dir(repo_id)
    info = episode_media.read_info(dataset_dir)
    robot_type = info.get("robot_type")
    if robot_type not in ("dual_scorpion_follower", "so101_dual_follower", "dual_scorpion_kinematic"):
        raise ValueError("This episode uses a different robot.")
    names = info["features"].get("observation.state", {}).get("names")
    if not isinstance(names, list) or not names or not all(isinstance(name, str) for name in names):
        raise ValueError("The episode has no named observation.state joints.")

    mapping = {}
    if robot_type == "dual_scorpion_kinematic":
        # Converted joint datasets store Full CAD angles directly in degrees.
        mapping = {"sides": {side: {"sign": [1] * 7, "offset_deg": [0] * 7} for side in ("left", "right")}}
    elif robot_name is not None and is_valid_robot_name(robot_name):
        record = get_robot_record(robot_name)
        if record and record.get("robot_model") == "dual_scorpion" and record.get("view_mapping"):
            from lerobot.robots import make_robot_from_config

            robot = make_robot_from_config(
                scorpion_follower(
                    record["follower_left_port"], record["follower_right_port"], record["follower_config"]
                )
            )
            mapping = read_view_mapping(record["view_mapping"], robot)

    frames = []
    for values in episode_media.load_episode_vectors(dataset_dir, episode_index, "observation.state"):
        positions = {key: value for key, value in zip(names, values, strict=True) if math.isfinite(value)}
        frames.append(scorpion_joint_frame(positions, mapping))
    return frames
