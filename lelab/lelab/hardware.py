"""LeLab bindings for the Scorpion devices registered in LeRobot."""

import os
from pathlib import Path
from typing import Literal

from pydantic import BaseModel

from lerobot.robots.dual_scorpion_follower import DualScorpionFollowerConfig
from lerobot.teleoperators.dual_scorpion_leader import DualScorpionLeaderConfig
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION

RobotModel = Literal["so101", "dual_scorpion"]
MAX_SCORPION_FPS = 30
SCORPION_STEP_LIMIT = 0.5
SCORPION_PORT_FIELDS = (
    "leader_left_port",
    "leader_right_port",
    "follower_left_port",
    "follower_right_port",
)


class HardwareRequest(BaseModel):
    robot_model: RobotModel = "so101"
    leader_left_port: str = ""
    leader_right_port: str = ""
    follower_left_port: str = ""
    follower_right_port: str = ""
    view_mapping: str = ""


def validate_ports(*ports: str) -> None:
    if not all(port and port.strip() == port for port in ports):
        raise ValueError("Select a serial port for each arm.")
    resolved = [os.path.normcase(os.path.realpath(port)) for port in ports]
    if len(set(resolved)) != len(ports):
        raise ValueError("Each arm must use a different serial port.")


def calibration_id(filename: str) -> str:
    identifier = filename.removesuffix(".json")
    if (
        not identifier
        or identifier.strip() != identifier
        or any(char in identifier for char in ("/", "\\", "..", "\0", ":"))
        or identifier == "."
    ):
        raise ValueError("Select an existing calibration or enter a valid calibration name.")
    return identifier


def calibration_dirs(side: Literal["leader", "follower"]) -> tuple[Path, ...]:
    if side == "leader":
        return (HF_LEROBOT_CALIBRATION / "teleoperators" / "dual_scorpion_leader",)
    return (
        HF_LEROBOT_CALIBRATION / "robots" / "dual_scorpion_follower",
        HF_LEROBOT_CALIBRATION / "robots" / "so101_dual_follower",
    )


def calibration_directory(side: Literal["leader", "follower"], identifier: str) -> Path:
    return next(
        (path for path in calibration_dirs(side) if (path / f"{identifier}.json").is_file()),
        calibration_dirs(side)[0],
    )


def scorpion_follower(left: str, right: str, filename: str, *, cameras=None, calibrating=False):
    validate_ports(left, right)
    identifier = calibration_id(filename)
    directory = (
        calibration_dirs("follower")[0] if calibrating else calibration_directory("follower", identifier)
    )
    return DualScorpionFollowerConfig(
        left_arm_port=left,
        right_arm_port=right,
        id=identifier,
        calibration_dir=directory,
        cameras=cameras or {},
        require_calibrated=not calibrating,
        max_relative_target=SCORPION_STEP_LIMIT,
    )


def scorpion_leader(left: str, right: str, filename: str, *, calibrating=False):
    validate_ports(left, right)
    return DualScorpionLeaderConfig(
        left_arm_port=left,
        right_arm_port=right,
        id=calibration_id(filename),
        calibration_dir=calibration_dirs("leader")[0],
        require_calibrated=not calibrating,
    )


def scorpion_pair(request, *, cameras=None):
    validate_ports(*(getattr(request, field) for field in SCORPION_PORT_FIELDS))
    return (
        scorpion_follower(
            request.follower_left_port, request.follower_right_port, request.follower_config, cameras=cameras
        ),
        scorpion_leader(request.leader_left_port, request.leader_right_port, request.leader_config),
    )
