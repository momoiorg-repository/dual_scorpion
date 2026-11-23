"""Export recorded LeRobot dataset frames into the offline observation schema."""

from pathlib import Path

import numpy as np

from .schema import PolicyError, RobotCapabilities, RobotObservation


def capture_dataset_frame(
    robot: RobotCapabilities,
    root: Path,
    repo_id: str | None,
    index: int,
    *,
    task: str | None = None,
    video_backend: str = "pyav",
) -> RobotObservation:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.utils.constants import OBS_IMAGES, OBS_STATE

    if not (root / "meta" / "info.json").is_file():
        raise PolicyError("capture requires a local LeRobot dataset root")
    dataset = LeRobotDataset(
        repo_id=repo_id or root.name,
        root=root,
        return_uint8=True,
        video_backend=video_backend,
    )
    if not 0 <= index < len(dataset):
        raise PolicyError("dataset frame index is out of range")
    state_meta = dataset.features.get(OBS_STATE, {})
    state_names = tuple(state_meta.get("names") or ())
    if not state_names or not set(robot.state_features) <= set(state_names):
        raise PolicyError("dataset state feature names do not match the specified robot")
    available = {name.removeprefix(f"{OBS_IMAGES}.") for name in dataset.meta.camera_keys}
    if not set(robot.images) <= available:
        raise PolicyError("dataset lacks the requested robot cameras")

    frame = dataset[index]
    images = {}
    for name in robot.images:
        image = frame[f"{OBS_IMAGES}.{name}"].numpy()
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[0] != 3:
            raise PolicyError(f"{name}: expected a uint8 CHW image from LeRobotDataset")
        images[name] = np.ascontiguousarray(image.transpose(1, 2, 0))
    state = frame[OBS_STATE].numpy()
    selected = state[[state_names.index(name) for name in robot.state_features]]
    timestamp = round(float(frame["timestamp"]) * 1e9)
    return RobotObservation(
        robot_id=dataset.repo_id,
        robot_type=robot.robot_type,
        timestamp_ns=timestamp,
        state=selected,
        state_names=robot.state_features,
        state_units=robot.state_units,
        images=images,
        image_timestamps_ns=dict.fromkeys(images, timestamp),
        task=task if task is not None else frame.get("task"),
        clock="dataset",
        metadata={"source": "lerobot_dataset", "frame_index": index, "timestamps": "dataset_alignment"},
    )
