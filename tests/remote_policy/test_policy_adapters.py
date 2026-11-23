import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from lerobot.remote_policy.adapters.groot_n17 import GrootN17Adapter
from lerobot.remote_policy.adapters.mock import MockRobot
from lerobot.remote_policy.adapters.pi05 import Pi05Adapter
from lerobot.remote_policy.core import InferenceBackend, LocalTransport, PolicyClient
from lerobot.remote_policy.schema import ABSOLUTE, PolicyCapabilities, PolicyError


def _capabilities():
    robot = MockRobot().get_capabilities()
    return PolicyCapabilities(
        "vla",
        ("mock_arm",),
        ("front",),
        ("joint", "gripper"),
        robot.state_units,
        ("joint", "gripper"),
        robot.action_units,
        3,
        ABSOLUTE,
        30,
        requires_task=True,
    )


def test_pi05_uses_saved_lerobot_processing_and_restores_relative_once(tmp_path, monkeypatch):
    pytest.importorskip("transformers")
    import torch

    from lerobot.configs import FeatureType, NormalizationMode, PolicyFeature
    from lerobot.policies.pi05 import PI05Config, PI05Policy
    from lerobot.processor import (
        AbsoluteActionsProcessorStep,
        AddBatchDimensionProcessorStep,
        DeviceProcessorStep,
        NormalizerProcessorStep,
        PolicyProcessorPipeline,
        RelativeActionsProcessorStep,
        UnnormalizerProcessorStep,
    )
    from lerobot.utils.constants import ACTION, OBS_STATE

    model_camera = "observation.images.trained_camera"
    caps = _capabilities()
    config = PI05Config(
        device="cpu",
        chunk_size=3,
        n_action_steps=3,
        use_relative_actions=True,
        action_feature_names=list(caps.action_features),
        input_features={
            OBS_STATE: PolicyFeature(FeatureType.STATE, (2,)),
            model_camera: PolicyFeature(FeatureType.VISUAL, (3, 8, 12)),
        },
        output_features={ACTION: PolicyFeature(FeatureType.ACTION, (2,))},
    )
    config.save_pretrained(tmp_path)
    (tmp_path / "model.safetensors").touch()

    stats = {OBS_STATE: {"mean": [2, 0], "std": [2, 25]}, ACTION: {"mean": [1, 42], "std": [2, 10]}}
    norm = {FeatureType.STATE: NormalizationMode.MEAN_STD, FeatureType.ACTION: NormalizationMode.MEAN_STD}
    relative = RelativeActionsProcessorStep(enabled=True, exclude_joints=["gripper"])
    pre = PolicyProcessorPipeline(
        name="policy_preprocessor",
        steps=[
            AddBatchDimensionProcessorStep(),
            relative,
            NormalizerProcessorStep(features=config.input_features, norm_map=norm, stats=stats),
            DeviceProcessorStep(device="cpu"),
        ],
    )
    post = PolicyProcessorPipeline(
        name="policy_postprocessor",
        steps=[
            UnnormalizerProcessorStep(features=config.output_features, norm_map=norm, stats=stats),
            AbsoluteActionsProcessorStep(enabled=True, relative_step=relative),
            DeviceProcessorStep(device="cpu"),
        ],
    )
    pre.save_pretrained(tmp_path)
    post.save_pretrained(tmp_path)

    class FakePiModel:
        def eval(self):
            return self

        def predict_action_chunk(self, batch):
            torch.testing.assert_close(batch[OBS_STATE], torch.tensor([[1.0, 2.0]]))
            assert batch[model_camera].shape == (1, 3, 8, 12)
            assert batch[model_camera].max().item() == 1.0
            return torch.zeros((1, 3, 2))

    def fake_load(path, *, config, require_weights):
        assert Path(path) == tmp_path and config.use_relative_actions and require_weights
        return FakePiModel()

    monkeypatch.setattr(PI05Policy, "from_pretrained", fake_load)
    adapter = Pi05Adapter(caps, images={model_camera: "front"}, device="cpu")
    adapter.load(str(tmp_path))
    obs = MockRobot(state_features=("gripper", "joint")).get_observation("move")
    obs = replace(obs, state=np.array([50, 4]), images={"front": np.full((8, 12, 3), 255, np.uint8)})
    output = PolicyClient(LocalTransport(InferenceBackend(adapter))).predict(obs)
    np.testing.assert_array_equal(output.actions, [[5, 42]] * 3)
    assert output.action_representation == ABSOLUTE


def test_groot_maps_official_batched_modalities_and_preserves_decoded_units(tmp_path, monkeypatch):
    caps = _capabilities()
    action_config = SimpleNamespace(
        rep=SimpleNamespace(name="RELATIVE"),
        type=SimpleNamespace(name="NON_EEF"),
        format=SimpleNamespace(name="DEFAULT"),
        state_key="arm",
    )
    gripper_config = SimpleNamespace(
        rep=SimpleNamespace(name="ABSOLUTE"),
        type=SimpleNamespace(name="NON_EEF"),
        format=SimpleNamespace(name="DEFAULT"),
        state_key=None,
    )
    modalities = {
        "video": SimpleNamespace(delta_indices=[0], modality_keys=["front_camera"]),
        "state": SimpleNamespace(delta_indices=[0], modality_keys=["arm", "hand"]),
        "language": SimpleNamespace(delta_indices=[0], modality_keys=["annotation.custom"]),
        "action": SimpleNamespace(
            delta_indices=[0, 1, 2],
            modality_keys=["arm", "hand"],
            action_configs=[action_config, gripper_config],
        ),
    }

    class FakeGrootPolicy:
        def __init__(self, *, model_path, embodiment_tag, device, strict):
            assert Path(model_path) == tmp_path and strict and embodiment_tag == "NEW_EMBODIMENT"
            assert device == "cuda:0"
            self.embodiment_tag = SimpleNamespace(value="new_embodiment")
            params = {"arm": {"dim": 1}, "hand": {"dim": 1}}
            self.processor = SimpleNamespace(
                state_action_processor=SimpleNamespace(
                    use_relative_action=True,
                    norm_params={"new_embodiment": {"state": params, "action": params}},
                )
            )

        def get_modality_config(self):
            return modalities

        def get_action(self, inputs):
            assert inputs["video"]["front_camera"].shape == (1, 1, 8, 12, 3)
            assert inputs["video"]["front_camera"].dtype == np.uint8
            np.testing.assert_array_equal(inputs["state"]["arm"], [[[4]]])
            np.testing.assert_array_equal(inputs["state"]["hand"], [[[50]]])
            assert inputs["language"] == {"annotation.custom": [["move"]]}
            return {"hand": np.full((1, 3, 1), 42, np.float32), "arm": np.full((1, 3, 1), 5, np.float32)}, {}

    monkeypatch.setitem(
        sys.modules, "gr00t.policy.gr00t_policy", SimpleNamespace(Gr00tPolicy=FakeGrootPolicy)
    )
    (tmp_path / "config.json").write_text('{"model_type":"Gr00tN1d7"}')
    adapter = GrootN17Adapter(
        caps,
        embodiment_tag="NEW_EMBODIMENT",
        images={"front_camera": "front"},
        state_groups={"arm": ["joint"], "hand": ["gripper"]},
        action_groups={"hand": ["gripper"], "arm": ["joint"]},
    )
    adapter.load(str(tmp_path))
    obs = MockRobot(state_features=("gripper", "joint")).get_observation("move")
    output = PolicyClient(LocalTransport(InferenceBackend(adapter))).predict(
        replace(obs, state=np.array([50, 4]))
    )
    np.testing.assert_array_equal(output.actions, [[5, 42]] * 3)

    action_config.type.name = "EEF"
    with pytest.raises(PolicyError, match="Unsupported GR00T action"):
        adapter.load(str(tmp_path))
