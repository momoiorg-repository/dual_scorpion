"""LeRobot π0.5 checkpoints with their saved official processor pipelines."""

from pathlib import Path

import numpy as np

from ..schema import ABSOLUTE, PolicyCapabilities, PolicyError, RobotObservation
from .common import camera_mapping, checkpoint_directory, state_vector


class Pi05Adapter:
    def __init__(
        self,
        capabilities: PolicyCapabilities,
        *,
        images: dict[str, str],
        device: str = "cuda",
        revision: str | None = None,
    ):
        if capabilities.action_representation != ABSOLUTE or not capabilities.requires_task:
            raise PolicyError("π0.5 requires a task and advertises absolute, postprocessed joint positions")
        self._capabilities = capabilities
        self._images = camera_mapping(images, capabilities.required_images)
        self._device = device
        self._revision = revision

    def load(self, checkpoint: str | None) -> None:
        import torch

        from lerobot.configs import NormalizationMode
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.factory import make_pre_post_processors
        from lerobot.policies.pi05 import PI05Config, PI05Policy
        from lerobot.processor.normalize_processor import NormalizerProcessorStep, UnnormalizerProcessorStep
        from lerobot.processor.relative_action_processor import (
            AbsoluteActionsProcessorStep,
            RelativeActionsProcessorStep,
        )
        from lerobot.utils.constants import ACTION, OBS_STATE

        caps = self._capabilities
        device = torch.device(self._device)
        if device.type == "cuda" and not torch.cuda.is_available():
            raise PolicyError("π0.5 requested CUDA but no CUDA device is available")
        directory = checkpoint_directory(
            checkpoint,
            self._revision,
            allow_patterns=["config.json", "model.safetensors", "policy_*.json", "policy_*.safetensors"],
        )
        config = PreTrainedConfig.from_pretrained(directory)
        if not isinstance(config, PI05Config) or config.use_peft:
            raise PolicyError("π0.5 requires a full LeRobot PI05 checkpoint")
        config.device = str(device)
        config.compile_model = False
        config.gradient_checkpointing = False
        self._check_checkpoint(config, ACTION, OBS_STATE)
        _require_files(directory)
        pre, post = make_pre_post_processors(
            config,
            pretrained_path=str(directory),
            preprocessor_overrides={"device_processor": {"device": str(device)}},
            postprocessor_overrides={"device_processor": {"device": "cpu"}},
        )
        for pipeline, step_type, key, feature_type in (
            (pre, NormalizerProcessorStep, OBS_STATE, config.input_features[OBS_STATE].type),
            (post, UnnormalizerProcessorStep, ACTION, config.output_features[ACTION].type),
        ):
            steps = [step for step in pipeline.steps if isinstance(step, step_type)]
            if len(steps) != 1:
                raise PolicyError(f"π0.5 checkpoint requires one official processor for {key}")
            if steps[0].norm_map.get(
                feature_type, NormalizationMode.IDENTITY
            ) != NormalizationMode.IDENTITY and not any(
                name.startswith(f"{key}.") for name in steps[0].state_dict()
            ):
                raise PolicyError(f"π0.5 checkpoint is missing saved normalization statistics for {key}")

        relative = [
            step for step in pre.steps if isinstance(step, RelativeActionsProcessorStep) and step.enabled
        ]
        absolute = [
            step for step in post.steps if isinstance(step, AbsoluteActionsProcessorStep) and step.enabled
        ]
        if config.use_relative_actions:
            if (
                len(relative) != 1
                or len(absolute) != 1
                or absolute[0].relative_step is not relative[0]
                or caps.state_features[: caps.action_dim] != caps.action_features
            ):
                raise PolicyError(
                    "π0.5 relative checkpoint requires paired processors and aligned state/action order"
                )
            if relative[0].action_names is None:
                if relative[0].exclude_joints and config.action_feature_names is None:
                    raise PolicyError(
                        "π0.5 relative checkpoint is missing action names for the exclusion mask"
                    )
                relative[0].action_names = config.action_feature_names
            elif tuple(relative[0].action_names) != caps.action_features:
                raise PolicyError("π0.5 processor action names do not match the contract")
        elif relative or absolute:
            raise PolicyError("π0.5 processor relative conversion disagrees with the checkpoint config")

        self._model = PI05Policy.from_pretrained(directory, config=config, require_weights=True).eval()
        self._pre, self._post = pre, post

    def _check_checkpoint(self, config, action_key: str, state_key: str) -> None:
        caps = self._capabilities
        if config.n_obs_steps != 1:
            raise PolicyError("Unsupported π0.5 input history. The protocol carries one observation")
        state = config.input_features.get(state_key)
        action = config.output_features.get(action_key)
        if state is None or state.shape != (len(caps.state_features),):
            raise PolicyError("π0.5 checkpoint state dimension does not match its contract")
        if action is None or action.shape != (caps.action_dim,) or config.chunk_size != caps.action_horizon:
            raise PolicyError("π0.5 checkpoint action dimension or chunk size does not match its contract")
        if (
            config.action_feature_names is not None
            and tuple(config.action_feature_names) != caps.action_features
        ):
            raise PolicyError("π0.5 checkpoint action feature order does not match its contract")
        actual_images = {
            key for key in config.image_features if not key.startswith("observation.images.empty_camera_")
        }
        if set(self._images) != actual_images:
            raise PolicyError("π0.5 image mapping does not match checkpoint input features")

    def get_capabilities(self) -> PolicyCapabilities:
        return self._capabilities

    def preprocess(self, observation: RobotObservation):
        import torch

        from lerobot.policies.utils import prepare_observation_for_inference
        from lerobot.utils.constants import OBS_STATE

        frame = {
            OBS_STATE: state_vector(observation, self._capabilities.state_features),
            **{key: observation.images[camera].copy() for key, camera in self._images.items()},
        }
        # LeRobot performs RGB layout/scaling here and applies the saved training transforms below.
        batch = prepare_observation_for_inference(frame, torch.device("cpu"), task=observation.task)
        return self._pre(batch)

    def predict(self, inputs):
        return self._model.predict_action_chunk(inputs)

    def postprocess(self, prediction) -> np.ndarray:
        # Keep the batch and time axes for official relative-action restoration.
        actions = self._post(prediction).detach().float().cpu().numpy()
        if actions.ndim != 3 or actions.shape[0] != 1:
            raise PolicyError("π0.5 expected a single batched action chunk")
        return actions[0]


def _require_files(directory: Path) -> None:
    for basename in ("model.safetensors", "policy_preprocessor.json", "policy_postprocessor.json"):
        if not (directory / basename).is_file():
            raise PolicyError(f"π0.5 checkpoint is missing {basename}")
