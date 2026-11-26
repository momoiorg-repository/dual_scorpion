#!/usr/bin/env python

from lerobot.robots.dual_scorpion_follower import DualScorpionFollowerConfig
from lerobot.robots.utils import make_robot_from_config
from lerobot.scripts.lerobot_setup_motors import SetupConfig, setup_motors
from lerobot.teleoperators.dual_scorpion_leader import DualScorpionLeaderConfig
from lerobot.teleoperators.utils import make_teleoperator_from_config


def test_setup_motors_accepts_dual_scorpion_arm_selector(monkeypatch):
    from unittest.mock import Mock

    device = Mock()
    monkeypatch.setattr("lerobot.scripts.lerobot_setup_motors.make_robot_from_config", lambda _: device)
    cfg = SetupConfig(
        robot=DualScorpionFollowerConfig(right_arm_port="/dev/right", left_arm_port="/dev/left"),
        arm="left",
    )

    assert cfg.device.type == "dual_scorpion_follower"
    setup_motors(cfg)
    device.setup_motors.assert_called_once_with(arm="left")


def test_dual_scorpion_factories_are_registered(monkeypatch):
    class RobotSentinel:
        def __init__(self, config):
            self.config = config

    class TeleopSentinel:
        def __init__(self, config):
            self.config = config

    monkeypatch.setattr("lerobot.robots.dual_scorpion_follower.DualScorpionFollower", RobotSentinel)
    monkeypatch.setattr("lerobot.teleoperators.dual_scorpion_leader.DualScorpionLeader", TeleopSentinel)

    robot_config = DualScorpionFollowerConfig(right_arm_port="/dev/right", left_arm_port="/dev/left")
    teleop_config = DualScorpionLeaderConfig(right_arm_port="/dev/right", left_arm_port="/dev/left")

    assert isinstance(make_robot_from_config(robot_config), RobotSentinel)
    assert isinstance(make_teleoperator_from_config(teleop_config), TeleopSentinel)
