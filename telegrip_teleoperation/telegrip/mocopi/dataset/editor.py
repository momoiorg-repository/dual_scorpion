"""Rerun-compatible range selection for converted MOCOPI datasets."""

import hashlib
import json
from copy import deepcopy
from pathlib import Path

import numpy as np

SELECTION_SCHEMA = "dual_scorpion.mocopi.dataset_selection.v1"
HUMAN_SCHEMA = "dual_scorpion.mocopi.lerobot.v1"
DEFAULT_FEATURES = {"timestamp", "frame_index", "episode_index", "index", "task_index"}


def _manifest_path(root):
    return Path(root) / "meta" / "mocopi_dataset.json"


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temp.replace(path)


def load_dataset(root):
    """Open a finalized human MOCOPI dataset with its manifest."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = Path(root).resolve()
    manifest_file = _manifest_path(root)
    manifest = json.loads(manifest_file.read_text(encoding="utf-8"))
    if manifest.get("schema") != HUMAN_SCHEMA or manifest.get("gripper_recorded"):
        raise ValueError("Only finalized human MOCOPI datasets are supported")
    dataset = LeRobotDataset(manifest["repo_id"], root=root, video_backend="pyav")
    if dataset.num_episodes < 1 or len(dataset) < 2:
        raise ValueError("Dataset must contain at least one episode with two frames")
    return dataset, manifest


def episode_bounds(dataset, episode):
    if not isinstance(episode, int) or isinstance(episode, bool) or not 0 <= episode < dataset.num_episodes:
        raise ValueError(f"Episode must be between 0 and {dataset.num_episodes - 1}")
    metadata = dataset.meta.episodes[episode]
    start, end = int(metadata["dataset_from_index"]), int(metadata["dataset_to_index"])
    if not 0 <= start < end <= len(dataset):
        raise ValueError(f"Invalid dataset bounds for episode {episode}")
    return start, end


def validate_selection(dataset, root, selection):
    """Return normalized selections or raise before any output dataset is written."""
    if not isinstance(selection, dict) or selection.get("schema") != SELECTION_SCHEMA:
        raise ValueError("Unsupported dataset selection schema")
    expected_root = str(Path(root).resolve())
    if selection.get("dataset") != expected_root:
        raise ValueError(f"Selection belongs to another dataset: {selection.get('dataset')}")
    if selection.get("dataset_manifest_sha256") != _sha256(_manifest_path(root)):
        raise ValueError("Dataset manifest changed after the selection was created")
    if float(selection.get("fps", -1)) != float(dataset.fps):
        raise ValueError("Selection FPS does not match the dataset")
    rows = selection.get("selections")
    if not isinstance(rows, list) or not rows:
        raise ValueError("Selection needs at least one range")
    normalized = []
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("Each selection must be an object")
        episode, start, end = row.get("episode_index"), row.get("start_index"), row.get("end_index")
        if any(not isinstance(value, int) or isinstance(value, bool) for value in (episode, start, end)):
            raise ValueError("Episode and range indices must be integers")
        episode_start, episode_end = episode_bounds(dataset, episode)
        if not episode_start <= start < end <= episode_end:
            raise ValueError(f"Invalid selection range for episode {episode}: {start}..{end}")
        normalized.append(
            {"episode_index": episode, "start_index": start, "end_index": end}
        )
    normalized.sort(key=lambda item: (item["episode_index"], item["start_index"]))
    previous = None
    for item in normalized:
        if (
            previous is not None
            and item["episode_index"] == previous["episode_index"]
            and item["start_index"] <= previous["end_index"]
        ):
            raise ValueError(f"Overlapping selection ranges for episode {item['episode_index']}")
        previous = item
    result = deepcopy(selection)
    result.update(
        {
            "dataset": expected_root,
            "dataset_manifest_sha256": _sha256(_manifest_path(root)),
            "fps": float(dataset.fps),
            "selections": normalized,
        }
    )
    return result


def save_selection(path, selection):
    _atomic_json(path, selection)


def create_selection(root, dataset, ranges):
    """Create selections from half-open ``episode:local_start:local_end`` ranges."""
    selections = []
    for value in ranges:
        try:
            episode, start, end = (int(part) for part in value.split(":"))
        except (ValueError, AttributeError) as exc:
            raise ValueError(f"Invalid range {value!r}; expected EPISODE:START:END") from exc
        episode_start, episode_end = episode_bounds(dataset, episode)
        frame_count = episode_end - episode_start
        if not 0 <= start < end <= frame_count:
            raise ValueError(
                f"Invalid range {value!r}; use 0 <= START < END <= {frame_count} (END is exclusive)"
            )
        selections.append(
            {
                "episode_index": episode,
                "start_index": episode_start + start,
                "end_index": episode_start + end,
            }
        )
    selection = {
        "schema": SELECTION_SCHEMA,
        "dataset": str(Path(root).resolve()),
        "dataset_manifest_sha256": _sha256(_manifest_path(root)),
        "fps": float(dataset.fps),
        "selections": selections,
    }
    return validate_selection(dataset, root, selection)


def _frame_for_writer(source, index):
    """Re-shape decoded values to the feature contract expected by the writer."""
    row = source[index]
    data = {"task": row["task"]}
    for key, feature in source.meta.features.items():
        if key in DEFAULT_FEATURES:
            continue
        value = row[key]
        if feature.get("dtype") == "video":
            data[key] = value
            continue
        shape = tuple(feature.get("shape", ()))
        array = np.asarray(value)
        if shape == (1,):
            array = np.asarray([array.item()], dtype=np.dtype(feature["dtype"]))
        else:
            array = array.reshape(shape).astype(np.dtype(feature["dtype"]))
        data[key] = array
    return data


def apply_dataset_selection(selection_path, output, repo_id):
    """Copy selected source ranges into a new, independently verifiable dataset."""
    selection_path = Path(selection_path).resolve()
    selection = json.loads(selection_path.read_text(encoding="utf-8"))
    root = Path(selection.get("dataset", "")).resolve()
    source, manifest = load_dataset(root)
    selection = validate_selection(source, root, selection)
    output, repo_id = Path(output), repo_id.strip()
    if not repo_id:
        raise ValueError("repo_id must not be empty")
    if output.exists():
        raise FileExistsError(f"Edited dataset output already exists: {output}")

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    destination = LeRobotDataset.create(
        repo_id=repo_id,
        fps=source.fps,
        features=source.meta.features,
        root=output,
        robot_type="human_mocopi",
        use_videos=True,
        vcodec="h264",
        image_writer_threads=2,
    )
    edited_manifest = deepcopy(manifest)
    edited_manifest.update(
        {
            "repo_id": repo_id,
            "selection_schema": SELECTION_SCHEMA,
            "selection_file": str(selection_path),
            "selections": selection["selections"],
            "edited_from": {
                "dataset": str(root),
                "dataset_manifest_sha256": _sha256(_manifest_path(root)),
            },
            "edit_semantics": (
                "Frames are copied from contiguous source ranges. Actions remain the next measured "
                "human hand pose from the source dataset, even when the following source frame is excluded."
            ),
            "episodes": [],
        }
    )
    try:
        for item in selection["selections"]:
            start, end = item["start_index"], item["end_index"]
            for index in range(start, end):
                destination.add_frame(_frame_for_writer(source, index))
            destination.save_episode()
            source_episode = item["episode_index"]
            source_entry = manifest["episodes"][source_episode]
            first_grid = float(source[start]["observation.capture.grid_time_s"].item())
            edited_manifest["episodes"].append(
                {
                    "episode_index": len(edited_manifest["episodes"]),
                    "source_session": source_entry["source_session"],
                    "source_episode": source_episode,
                    "first_grid_time_s": first_grid,
                    "frames": end - start,
                    "source_dataset_from_index": start,
                    "source_dataset_to_index": end,
                    "source_episode_manifest": deepcopy(source_entry),
                }
            )
    finally:
        destination.finalize()
    _atomic_json(output / "meta" / "mocopi_dataset.json", edited_manifest)
    return edited_manifest
