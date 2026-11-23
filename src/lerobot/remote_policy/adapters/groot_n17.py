"""NVIDIA Isaac-GR00T N1.7 policy API bridge."""

from __future__ import annotations

from typing import Any

import numpy as np

from ..exchange import decode_json, read_file
from ..schema import ABSOLUTE, PolicyCapabilities, PolicyError, RobotObservation, names, require_text
from .common import camera_mapping, checkpoint_directory, state_vector


class GrootN17Adapter:
    def __init__(
        self,
        capabilities: PolicyCapabilities,
        *,
        embodiment_tag: str,
        images: dict[str, str],
        state_groups: dict[str, list[str]],
        action_groups: dict[str, list[str]],
        device: str = "cuda:0",
        revision: str | None = None,
    ):
        if capabilities.action_representation != ABSOLUTE or not capabilities.requires_task:
            raise PolicyError("GR00T N1.7 requires a task and advertises decoded absolute joint positions")
        self._capabilities = capabilities
        require_text(embodiment_tag, "GR00T embodiment_tag")
        self._embodiment = embodiment_tag
        self._images = camera_mapping(images, capabilities.required_images)
        self._state = _groups(state_groups, capabilities.state_features, "state")
        self._actions = _groups(action_groups, capabilities.action_features, "action")
        self._action_order = tuple(name for group in self._actions.values() for name in group)
        self._device = device
        self._revision = revision

    def load(self, checkpoint: str | None) -> None:
        try:
            from gr00t.policy.gr00t_policy import Gr00tPolicy
        except ImportError as exc:
            raise PolicyError(
                "GR00T N1.7 requires the NVIDIA Isaac-GR00T N1.7 inference environment"
            ) from exc

        directory = checkpoint_directory(
            checkpoint,
            self._revision,
            allow_patterns=[
                "config.json",
                "*.safetensors",
                "*.safetensors.index.json",
                "processor/**",
                "*.json",
            ],
        )
        if decode_json(read_file(directory / "config.json", 64 * 1024)).get("model_type") != "Gr00tN1d7":
            raise PolicyError("GR00T N1.7 adapter requires an Isaac-GR00T Gr00tN1d7 checkpoint")
        self._policy = Gr00tPolicy(
            embodiment_tag=self._embodiment,
            model_path=str(directory),
            device=self._device,
            strict=True,
        )
        self._check_modalities()

    def _check_modalities(self) -> None:
        caps = self._capabilities
        config = self._policy.get_modality_config()
        if any(key not in config for key in ("video", "state", "action", "language")):
            raise PolicyError("GR00T checkpoint is missing a required modality")
        if any(list(config[key].delta_indices) != [0] for key in ("video", "state", "language")):
            raise PolicyError(
                "Unsupported GR00T input history. The protocol carries a single timestamp per feature"
            )
        if list(config["action"].delta_indices) != list(range(caps.action_horizon)):
            raise PolicyError("GR00T action offsets do not match the configured consecutive horizon")
        if not self._images:
            raise PolicyError("the GR00T N1.7 policy API requires at least one video stream")
        for key, mapping in (("video", self._images), ("state", self._state), ("action", self._actions)):
            if set(config[key].modality_keys) != set(mapping):
                raise PolicyError(f"GR00T {key} mapping does not match checkpoint modalities")
        languages = config["language"].modality_keys
        if not languages:
            raise PolicyError("the GR00T checkpoint does not name a language modality")
        self._language_key = languages[0]

        processor = self._policy.processor.state_action_processor
        params = processor.norm_params[self._policy.embodiment_tag.value]
        for key, mapping in (("state", self._state), ("action", self._actions)):
            for group, features in mapping.items():
                if int(params[key][group]["dim"]) != len(features):
                    raise PolicyError(f"GR00T {key} group {group!r} does not match checkpoint width")
        action_config = config["action"].action_configs
        if action_config is None or len(action_config) != len(self._actions):
            raise PolicyError("GR00T checkpoint must declare each action group's representation")
        for group, entry in zip(config["action"].modality_keys, action_config, strict=True):
            if entry.type.name != "NON_EEF" or entry.format.name != "DEFAULT":
                raise PolicyError(f"Unsupported GR00T action type or format for {group!r}")
            if entry.rep.name == "RELATIVE":
                state_names = self._state.get(entry.state_key or group)
                if (
                    not processor.use_relative_action
                    or state_names != self._actions[group]
                    or any(caps.state_units[name] != caps.action_units[name] for name in state_names)
                ):
                    raise PolicyError(
                        f"GR00T relative action reference for {group!r} does not match the contract"
                    )
            elif entry.rep.name != "ABSOLUTE":
                raise PolicyError(f"Unsupported GR00T action representation for {group!r}: {entry.rep.name}")

    def get_capabilities(self) -> PolicyCapabilities:
        return self._capabilities

    def preprocess(self, observation: RobotObservation) -> dict[str, Any]:
        return {
            "video": {key: observation.images[camera][None, None] for key, camera in self._images.items()},
            "state": {
                key: state_vector(observation, names)[None, None] for key, names in self._state.items()
            },
            "language": {self._language_key: [[observation.task]]},
        }

    def predict(self, inputs: dict[str, Any]) -> dict[str, np.ndarray]:
        # get_action includes the official checkpoint processor and decode_action.
        actions, _ = self._policy.get_action(inputs)
        return actions

    def postprocess(self, prediction: dict[str, np.ndarray]) -> np.ndarray:
        caps = self._capabilities
        chunks = []
        for group, features in self._actions.items():
            chunk = prediction.get(group)
            if not isinstance(chunk, np.ndarray) or chunk.shape != (1, caps.action_horizon, len(features)):
                raise PolicyError(f"GR00T action group {group!r} has the wrong chunk shape")
            chunks.append(chunk[0])
        return np.concatenate(chunks, axis=1)[
            :, [self._action_order.index(name) for name in caps.action_features]
        ]


def _groups(
    mapping: dict[str, list[str]], features: tuple[str, ...], label: str
) -> dict[str, tuple[str, ...]]:
    if not isinstance(mapping, dict) or not mapping:
        raise PolicyError(f"GR00T {label}_groups must be a non-empty object")
    result = {}
    for group, values in mapping.items():
        require_text(group, f"{label} group")
        result[group] = names(values, f"{label} group {group!r}")
    flattened = [name for group in result.values() for name in group]
    if set(flattened) != set(features) or len(flattened) != len(features):
        raise PolicyError(f"GR00T {label}_groups must cover the declared features without overlap")
    return result
