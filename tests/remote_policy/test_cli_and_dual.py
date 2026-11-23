import json
import os
import sys
import time
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from lerobot.remote_policy.adapters.dual_scorpion import (
    CAMERAS,
    FEATURES,
    POSITION_UNITS,
    DualScorpionAdapter,
)
from lerobot.remote_policy.adapters.mock import MockPolicy
from lerobot.remote_policy.cli import main
from lerobot.remote_policy.core import InferenceBackend, LocalTransport, PolicyClient
from lerobot.remote_policy.exchange import decode_observation, decode_output
from lerobot.remote_policy.schema import ABSOLUTE, PolicyCapabilities, PolicyError

REPO = Path(__file__).resolve().parents[2]


def test_cli_mock_file_exchange_and_dry_run(tmp_path, capsys):
    profile = str(REPO / "config/robot_policy/mock.yaml")
    obs = tmp_path / "observation.npz"
    prediction = tmp_path / "prediction.npz"
    assert main(["robots"]) == 0
    assert "dual_scorpion" in json.loads(capsys.readouterr().out)
    assert main(["policies"]) == 0
    assert {"pi05", "groot_n17", "mock"} <= set(json.loads(capsys.readouterr().out))
    assert main(["doctor", "--robot", "mock"]) == 0
    assert json.loads(capsys.readouterr().out)["devices"]["mock"]
    assert main(["capabilities", "--config", profile]) == 0
    assert json.loads(capsys.readouterr().out)["action_dim"] == 2
    assert main(["capture", "--config", profile, "--output", str(obs)]) == 0
    assert main(["predict", "--config", profile, "--input", str(obs), "--output", str(prediction)]) == 0
    assert main(["validate", "--config", profile, "--input", str(prediction), "--observation", str(obs)]) == 0
    assert (
        decode_observation(obs.read_bytes()).observation_id
        == decode_output(prediction.read_bytes()).observation_id
    )
    capsys.readouterr()
    assert main(["run", "--config", profile]) == 0
    assert json.loads(capsys.readouterr().out)["dry_run"] is True
    assert main(["run", "--config", profile, "--execute"]) == 2
    assert "requires execution.dry_run" in capsys.readouterr().err


def test_configured_policy_plugin_uses_unmodified_core(tmp_path, monkeypatch, capsys):
    class CustomPolicy(MockPolicy):
        def predict(self, inputs):
            return super().predict(inputs)

    monkeypatch.setitem(sys.modules, "test_robot_policy_plugin", SimpleNamespace(CustomPolicy=CustomPolicy))
    mock_profile = (REPO / "config/robot_policy/mock.yaml").read_text()
    profile = tmp_path / "plugin.yaml"
    profile.write_text(
        mock_profile.replace(
            "policy:\n  adapter: mock\n",
            "policy:\n  adapter: custom\n  plugin: test_robot_policy_plugin:CustomPolicy\n",
        )
    )
    assert main(["policies", "--config", str(profile)]) == 0
    assert "custom" in json.loads(capsys.readouterr().out)
    assert main(["run", "--config", str(profile)]) == 0
    assert json.loads(capsys.readouterr().out)["shape"] == [4, 2]


def _hold_client(robot, horizon=4):
    caps = robot.get_capabilities()
    policy = MockPolicy(
        PolicyCapabilities(
            "dual-hold",
            (caps.robot_type,),
            caps.images,
            caps.state_features,
            POSITION_UNITS,
            tuple(reversed(caps.action_features)),
            POSITION_UNITS,
            horizon,
            ABSOLUTE,
            30,
        )
    )
    return PolicyClient(LocalTransport(InferenceBackend(policy)))


def test_dual_scorpion_capture_and_executor_are_fail_closed(tmp_path, monkeypatch):
    pytest.importorskip("telegrip")
    config = tmp_path / "harness.yaml"
    config.write_text(
        "robot_config: unused.yaml\nrobot: {id: test, left_port: fake-left, right_port: fake-right}\n"
        "cameras:\n"
        + "".join(f"  - {{id: {name}, device: /dev/video{i}}}\n" for i, name in enumerate(CAMERAS))
    )
    robot = DualScorpionAdapter(config=config)
    capture_times = {}
    calls = []

    class FakeHardware:
        is_connected = False
        robot_id = "test"
        max_step_deg = 0.5

        def connect(self):
            calls.append("read_only_connect")
            self.is_connected = True

        def read(self):
            return SimpleNamespace(
                connected=True,
                calibrated=True,
                errors={},
                timestamp_monotonic_ns=time.monotonic_ns(),
                joints_deg={"right": tuple(range(7)), "left": tuple(range(-1, -8, -1))},
                gripper_positions={"right": 20, "left": 60},
            )

        def validate_calibration(self):
            return {"status": "invalid", "errors": ["test calibration file is absent"]}

    def read_rgb(name):
        captured_at = time.monotonic_ns()
        capture_times[name] = captured_at
        return captured_at, np.full((6, 7, 3), 25, np.uint8)

    hardware = FakeHardware()
    monkeypatch.setattr(robot, "_hardware", hardware)
    harness = SimpleNamespace(config=robot._config, cameras=SimpleNamespace(read_rgb=read_rgb))
    monkeypatch.setattr(robot, "_services", lambda: harness)
    monkeypatch.setattr(hardware, "execute_plan", lambda *a, **kw: pytest.fail("executed"), raising=False)
    observation = robot.get_observation("hold")
    assert calls == ["read_only_connect"]
    assert observation.state_names == FEATURES
    np.testing.assert_array_equal(observation.state, list(range(7)) + [20] + list(range(-1, -8, -1)) + [60])
    assert tuple(observation.images) == CAMERAS
    assert observation.image_timestamps_ns == capture_times
    assert observation.timestamp_ns >= max(capture_times.values())
    client = _hold_client(robot)
    output = client.predict(observation, robot=robot.get_capabilities())
    assert output.actions.shape == (4, 16)
    report = robot.validate_action(output, observation)
    assert report.checks["feedback_freshness"] == "passed"
    assert report.checks["gripper_range"] == "passed"
    assert any("scene geometry" in blocker for blocker in report.blockers)
    with pytest.raises(PolicyError, match="execution blocked"):
        robot.execute_action(output, observation)
    with pytest.raises(PolicyError, match="execution blocked"):
        robot.execute_action(output, replace(observation))
    robot.stop()
    assert robot.validate_action(output, observation).checks["emergency_stop"] == "latched"


def test_recorded_dual_scorpion_dataset_observation(tmp_path):
    root = os.getenv("ROBOT_POLICY_DATASET_ROOT")
    if not root:
        pytest.skip("set ROBOT_POLICY_DATASET_ROOT to the local 16D/3-camera LeRobot dataset")
    pytest.importorskip("datasets")
    from lerobot.remote_policy.dataset import capture_dataset_frame

    robot = DualScorpionAdapter()
    observation = capture_dataset_frame(robot.get_capabilities(), Path(root), None, 0)
    assert observation.clock == "dataset"
    assert observation.state.shape == (16,) and set(observation.images) == set(CAMERAS)
    assert observation.task
    output = _hold_client(robot, horizon=50).predict(observation, robot=robot.get_capabilities())
    assert output.actions.shape == (50, 16)
    assert robot.validate_action(output, observation).blockers
    path = tmp_path / "recorded.npz"
    assert main(["capture", "--robot", "dual_scorpion", "--dataset-root", root, "--output", str(path)]) == 0
    np.testing.assert_array_equal(decode_observation(path.read_bytes()).state, observation.state)
