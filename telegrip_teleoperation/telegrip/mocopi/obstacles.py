"""Validated, operator-declared obstacle geometry for full-model previews."""

import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

SCHEMA = "dual_scorpion.mocopi.obstacles.v1"
MAX_OBSTACLES = 128


def _vector(document, key, name):
    value = np.asarray(document.get(key), dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError(f"Obstacle {name!r} needs three finite values in {key}")
    return value


def normalize_obstacles(document):
    """Validate and normalize a YAML/JSON obstacle scene without inferring geometry."""
    if not isinstance(document, dict) or document.get("schema") != SCHEMA:
        raise ValueError(f"Obstacle scene must declare schema {SCHEMA}")
    rows = document.get("obstacles")
    if not isinstance(rows, list) or len(rows) > MAX_OBSTACLES:
        raise ValueError(f"Obstacle scene needs a list of at most {MAX_OBSTACLES} boxes")
    result, names = [], set()
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("Each obstacle must be a mapping")
        name = str(row.get("name", "")).strip()
        if not name or "/" in name or name in names:
            raise ValueError("Obstacle names must be unique, nonempty, and cannot contain '/'")
        names.add(name)
        center = _vector(row, "center_m", name)
        half_size = _vector(row, "half_size_m", name)
        if np.any(half_size <= 0):
            raise ValueError(f"Obstacle {name!r} half_size_m values must be positive")
        rotation = _vector(row, "rotation_deg", name) if "rotation_deg" in row else np.zeros(3)
        result.append(
            {
                "name": name,
                "center_m": center.tolist(),
                "half_size_m": half_size.tolist(),
                "rotation_deg": rotation.tolist(),
            }
        )
    return result


def load_obstacles(path):
    path = Path(path)
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    return normalize_obstacles(document)


def obstacles_sha256(obstacles):
    encoded = json.dumps(obstacles, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
