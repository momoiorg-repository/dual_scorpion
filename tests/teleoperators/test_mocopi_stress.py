"""Hardware-free long-session stress tests for the mocopi pipeline."""

import json
import shutil
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "telegrip_teleoperation"))
from telegrip.mocopi.config import load_config
from telegrip.mocopi.dataset.capture import capture_session
from telegrip.mocopi.sources import fake_pair

CONFIG = Path(__file__).resolve().parents[2] / "config/mocopi/tracking.yaml"


def _camera_jitter(index):
    return 0.012 * ((index % 5) - 2) / 2


def _mocopi_jitter(index):
    return 0.012 * (((index + 2) % 5) - 2) / 2


def _dropped(index):
    return any(start <= index < start + 4 for start in (150, 300, 450))


def _write_two_minute_session(root, cfg, fps, frame_count):
    """Create a valid long raw session without sleeping for its logical duration."""
    import cv2

    capture_session(cfg, root, 0.05, "two minute synthetic stress", fake=True)
    shutil.rmtree(root / "images")
    (root / "images").mkdir()
    with (
        (root / "camera.jsonl").open("w", encoding="utf-8") as camera_file,
        (root / "mocopi.jsonl").open("w", encoding="utf-8") as mocopi_file,
    ):
        for index in range(frame_count):
            logical = index / fps
            camera_time = logical + 0.012 + _camera_jitter(index)
            image = np.zeros((32, 32, 3), dtype=np.uint8)
            image[:, :, index % 3] = (index * 7) % 256
            name = f"images/{index:08d}.jpg"
            if not cv2.imwrite(str(root / name), image, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                raise OSError(f"Failed to write stress image {name}")
            camera_file.write(
                json.dumps(
                    {"timestamp_s": camera_time, "file": name, "frame_id": index},
                    separators=(",", ":"),
                )
                + "\n"
            )
            if _dropped(index):
                continue
            mocopi_time = logical + 0.012 + _mocopi_jitter(index)
            sample, _ = fake_pair(logical, mocopi_time)
            mocopi_file.write(
                json.dumps(
                    {
                        "timestamp_s": mocopi_time,
                        "frame_id": index,
                        "bones": {str(key): value.tolist() for key, value in sample.bones.items()},
                    },
                    separators=(",", ":"),
                )
                + "\n"
            )
    metadata_path = root / "session.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata.update(
        camera_frames=frame_count,
        mocopi_frames=frame_count - 12,
        mocopi_discarded=12,
        mocopi_bad_packets=12,
        logical_duration_s=(frame_count - 1) / fps,
    )
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def test_two_minute_jitter_and_dropouts_survive_dataset_roundtrip(tmp_path):
    pytest.importorskip("cv2")
    dataset_type = pytest.importorskip("lerobot.datasets.lerobot_dataset").LeRobotDataset
    from telegrip.mocopi.dataset.convert import convert_sessions
    from telegrip.mocopi.dataset.evaluate import session_metrics
    from telegrip.mocopi.dataset.verify import verify_dataset

    fps = 5
    frame_count = 600
    cfg = load_config(CONFIG)
    cfg["camera"].update(width=32, height=32, fps=fps)
    raw = tmp_path / "two-minute-stress"
    _write_two_minute_session(raw, cfg, fps, frame_count)

    quality = session_metrics(raw, fps=fps, max_sync_s=0.04)
    assert quality["camera_frames"] == frame_count
    assert quality["mocopi_frames"] == frame_count - 12
    assert quality["mocopi_bad_packets"] == 12
    assert quality["unsynchronized_samples"] == 12
    assert quality["usable_segments"] == 4
    assert quality["usable_dataset_frames"] == frame_count - 16

    output = tmp_path / "stress-dataset"
    manifest = convert_sessions([raw], output, "local/mocopi-stress", fps=fps, max_sync_s=0.04)
    dataset = dataset_type("local/mocopi-stress", root=output, video_backend="pyav")
    assert len(dataset) == quality["usable_dataset_frames"]
    assert dataset.num_episodes == quality["usable_segments"] == len(manifest["episodes"])
    verification = verify_dataset(output, sample_frames=4)
    assert verification["status"] == "ok"

    episode_times = []
    for episode, entry in enumerate(manifest["episodes"]):
        start = int(dataset.meta.episodes[episode]["dataset_from_index"])
        end = int(dataset.meta.episodes[episode]["dataset_to_index"])
        times = np.asarray(
            dataset.hf_dataset[start:end]["observation.capture.grid_time_s"], dtype=float
        ).reshape(-1)
        # Grid timestamps are stored through the dataset's float32 pipeline.
        np.testing.assert_allclose(np.diff(times), 1 / fps, atol=1e-5)
        np.testing.assert_allclose(times[0], entry["first_grid_time_s"], atol=1e-5)
        episode_times.append(times)
    gaps = [
        following[0] - previous[-1]
        for previous, following in zip(episode_times, episode_times[1:], strict=False)
    ]
    np.testing.assert_allclose(gaps, [1.2, 1.2, 1.2], atol=1e-5)
