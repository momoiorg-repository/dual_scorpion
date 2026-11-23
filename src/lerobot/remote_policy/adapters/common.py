"""Shared utilities for model-side feature mapping and checkpoint resolution."""

from pathlib import Path

import numpy as np

from ..schema import PolicyError, RobotObservation, require_text


def checkpoint_directory(
    checkpoint: str | None,
    revision: str | None,
    *,
    allow_patterns: list[str] | None = None,
) -> Path:
    require_text(checkpoint, "policy checkpoint")
    path = Path(checkpoint).expanduser()
    if path.is_dir():
        return path.resolve()
    if path.is_absolute() or checkpoint.startswith((".", "~")):
        raise PolicyError(f"checkpoint directory does not exist: {path}")
    from huggingface_hub import snapshot_download

    return Path(snapshot_download(checkpoint, revision=revision, allow_patterns=allow_patterns))


def camera_mapping(images: dict[str, str], required_images: tuple[str, ...]) -> dict[str, str]:
    if not isinstance(images, dict):
        raise PolicyError("policy image mapping must be an object")
    for key, value in images.items():
        require_text(key, "model image key")
        require_text(value, "camera name")
    if set(images.values()) != set(required_images) or len(images) != len(required_images):
        raise PolicyError("policy image mapping must map every required camera exactly once")
    return dict(images)


def state_vector(observation: RobotObservation, names: tuple[str, ...]) -> np.ndarray:
    return observation.state[[observation.state_names.index(name) for name in names]]
