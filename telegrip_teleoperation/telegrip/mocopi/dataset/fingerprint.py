"""Identify the finalized LeRobot content used to build a replay plan."""

import hashlib
from pathlib import Path


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def dataset_fingerprint(root):
    """Hash metadata, frame data, and videos, including their relative paths.

    This identifies finalized v3 mocopi datasets, not arbitrary cache or
    auxiliary files. Missing content is an error rather than an empty hash.
    """
    root = Path(root)
    metadata = sorted(
        path for path in (root / "meta").rglob("*") if path.is_file() and path.suffix in (".json", ".parquet")
    )
    data = sorted((root / "data").rglob("*.parquet"))
    videos = sorted((root / "videos").rglob("*.mp4"))
    if not (root / "meta" / "info.json").is_file() or not (root / "meta" / "mocopi_dataset.json").is_file():
        raise ValueError("A finalized mocopi LeRobot dataset with metadata is required")
    if not data or not videos:
        raise ValueError("Mocopi dataset must contain finalized parquet data and head-camera video")
    digest = hashlib.sha256()
    for path in sorted([*metadata, *data, *videos], key=lambda value: value.relative_to(root).as_posix()):
        relative = path.relative_to(root).as_posix().encode("utf-8")
        digest.update(len(relative).to_bytes(8, "big"))
        digest.update(relative)
        digest.update(bytes.fromhex(file_sha256(path)))
    return digest.hexdigest()
