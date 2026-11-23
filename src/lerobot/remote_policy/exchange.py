"""Bounded, pickle-free NPZ exchange for files and the inference wire."""

from __future__ import annotations

import io
import json
import math
import os
import tempfile
import zipfile
import zlib
from dataclasses import fields
from pathlib import Path
from typing import Any

import numpy as np
from numpy.lib import format as npy

from .schema import (
    MAX_CAMERAS,
    PROTOCOL_VERSION,
    InferenceRequest,
    PolicyError,
    PolicyOutput,
    RobotObservation,
)

MAX_PAYLOAD_BYTES = 64 * 1024 * 1024
MAX_JSON_BYTES = 64 * 1024


def _reject_constant(value: str) -> None:
    raise PolicyError(f"invalid JSON constant: {value}")


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result = {}
    for key, value in pairs:
        if key in result:
            raise PolicyError(f"duplicate JSON field: {key}")
        result[key] = value
    return result


def decode_json(data: bytes) -> dict[str, Any]:
    if len(data) > MAX_JSON_BYTES:
        raise PolicyError("JSON exceeds the size limit")
    try:
        result = json.loads(data, parse_constant=_reject_constant, object_pairs_hook=_unique_object)
    except (UnicodeError, ValueError, RecursionError) as exc:
        raise PolicyError("invalid exchange JSON") from exc
    if not isinstance(result, dict):
        raise PolicyError("exchange JSON must be an object")
    return result


def encode_json(document: dict[str, Any]) -> bytes:
    try:
        data = json.dumps(document, allow_nan=False, separators=(",", ":")).encode("utf-8")
    except (ValueError, TypeError, RecursionError) as exc:
        raise PolicyError("metadata must be finite and JSON-serializable") from exc
    if len(data) > MAX_JSON_BYTES:
        raise PolicyError("JSON exceeds the size limit")
    return data


def read_file(path: str | Path, limit: int = MAX_PAYLOAD_BYTES) -> bytes:
    with Path(path).open("rb") as stream:
        data = stream.read(limit + 1)
    if len(data) > limit:
        raise PolicyError("file exceeds the exchange size limit")
    return data


def write_file(path: str | Path, data: bytes) -> None:
    target = Path(path)
    with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as stream:
        temporary = Path(stream.name)
        try:
            stream.write(data)
            stream.close()
            os.replace(temporary, target)
        finally:
            temporary.unlink(missing_ok=True)


def encode_observation(observation: RobotObservation) -> bytes:
    return _encode_observation(observation, kind="observation")


def encode_request(request: InferenceRequest) -> bytes:
    return _encode_observation(
        request.observation,
        kind="request",
        request_id=request.request_id,
        capability_fingerprint=request.capability_fingerprint,
    )


def _encode_observation(observation: RobotObservation, kind: str, **extra: str) -> bytes:
    document = {
        field.name: getattr(observation, field.name)
        for field in fields(observation)
        if field.name not in {"state", "images"}
    }
    arrays = {f"image_{i}": image for i, image in enumerate(observation.images.values())}
    return _encode(
        kind,
        {"observation": document, "image_names": list(observation.images), **extra},
        {"state": observation.state, **arrays},
    )


def encode_output(output: PolicyOutput) -> bytes:
    document = {
        field.name: getattr(output, field.name) for field in fields(output) if field.name != "actions"
    }
    return _encode("output", document, {"actions": output.actions})


def _encode(kind: str, document: dict[str, Any], arrays: dict[str, np.ndarray]) -> bytes:
    header = encode_json({"protocol_version": PROTOCOL_VERSION, "kind": kind, "data": document})
    if len(header) + sum(array.nbytes for array in arrays.values()) > MAX_PAYLOAD_BYTES:
        raise PolicyError("arrays exceed the exchange size limit")
    buffer = io.BytesIO()
    np.savez(buffer, header=np.frombuffer(header, dtype=np.uint8), **arrays)
    if buffer.tell() > MAX_PAYLOAD_BYTES:
        raise PolicyError("payload exceeds the exchange size limit")
    return buffer.getvalue()


def decode_observation(blob: bytes) -> RobotObservation:
    document, arrays = _decode(blob, "observation")
    return _observation(document, arrays)


def decode_request(blob: bytes) -> InferenceRequest:
    document, arrays = _decode(blob, "request")
    try:
        request_id = document.pop("request_id")
        capability_fingerprint = document.pop("capability_fingerprint")
    except KeyError as exc:
        raise PolicyError("missing inference request fields") from exc
    return InferenceRequest(request_id, capability_fingerprint, _observation(document, arrays))


def _observation(document: dict[str, Any], arrays: dict[str, np.ndarray]) -> RobotObservation:
    try:
        if document.keys() != {"observation", "image_names"}:
            raise PolicyError("invalid observation envelope")
        image_names = document["image_names"]
        if (
            not isinstance(image_names, list)
            or len(image_names) > MAX_CAMERAS
            or any(not isinstance(name, str) for name in image_names)
            or len(image_names) != len(set(image_names))
        ):
            raise PolicyError("invalid observation camera list")
        image_keys = [f"image_{i}" for i in range(len(image_names))]
        if arrays.keys() != {"state", *image_keys}:
            raise PolicyError("observation NPZ arrays do not match the camera list")
        return RobotObservation(
            **document["observation"],
            state=arrays["state"],
            images={name: arrays[key] for name, key in zip(image_names, image_keys, strict=True)},
        )
    except TypeError as exc:
        raise PolicyError("invalid observation fields") from exc


def decode_output(blob: bytes) -> PolicyOutput:
    document, arrays = _decode(blob, "output")
    if arrays.keys() != {"actions"}:
        raise PolicyError("output NPZ must contain the actions array")
    try:
        return PolicyOutput(**document, actions=arrays["actions"])
    except TypeError as exc:
        raise PolicyError("invalid policy output fields") from exc


def _decode(blob: bytes, kind: str) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    if len(blob) > MAX_PAYLOAD_BYTES:
        raise PolicyError("payload exceeds the exchange size limit")
    try:
        with zipfile.ZipFile(io.BytesIO(blob)) as archive:
            entries = archive.infolist()
            members = {entry.filename: entry for entry in entries}
            if (
                len(members) != len(entries)
                or len(entries) > MAX_CAMERAS + 2
                or sum(entry.file_size for entry in entries) > MAX_PAYLOAD_BYTES
                or any(
                    entry.compress_type not in (zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED) for entry in entries
                )
            ):
                raise PolicyError("invalid or oversized NPZ archive")
            key = "header.npy"
            header = _read_array(archive, members.pop(key), limit=MAX_JSON_BYTES + 1024)
            if header.ndim != 1 or header.dtype != np.uint8:
                raise PolicyError("NPZ header must be a uint8 vector")
            envelope = decode_json(header.tobytes())
            if envelope.get("protocol_version") != PROTOCOL_VERSION or envelope.get("kind") != kind:
                raise PolicyError(f"expected a {kind} envelope with protocol {PROTOCOL_VERSION}")
            if not isinstance(envelope.get("data"), dict):
                raise PolicyError("NPZ envelope data must be an object")
            allowed = (
                {"actions.npy"}
                if kind == "output"
                else {"state.npy", *(f"image_{i}.npy" for i in range(MAX_CAMERAS))}
            )
            if members.keys() - allowed:
                raise PolicyError("unexpected NPZ array name")
            arrays = {name[:-4]: _read_array(archive, entry) for name, entry in members.items()}
            return envelope["data"], arrays
    except PolicyError:
        raise
    except (zipfile.BadZipFile, EOFError, KeyError, OSError, RuntimeError, ValueError, zlib.error) as exc:
        raise PolicyError("invalid NPZ exchange data") from exc


def _read_array(
    archive: zipfile.ZipFile,
    entry: zipfile.ZipInfo,
    *,
    limit: int = MAX_PAYLOAD_BYTES,
) -> np.ndarray:
    # Validate the declared shape and dtype against the member length before reading array data.
    if entry.file_size > limit:
        raise PolicyError("NPZ member exceeds the size limit")
    with archive.open(entry) as stream:
        version = npy.read_magic(stream)
        if version == (1, 0):
            shape, fortran_order, dtype = npy.read_array_header_1_0(stream)
        elif version == (2, 0):
            shape, fortran_order, dtype = npy.read_array_header_2_0(stream)
        else:
            raise PolicyError("unsupported NPY version")
        size = math.prod(shape) * dtype.itemsize
        if (
            not 1 <= len(shape) <= 3
            or any(dim <= 0 for dim in shape)
            or dtype.kind not in "fiu"
            or dtype.itemsize > 8
            or size != entry.file_size - stream.tell()
        ):
            raise PolicyError("unsafe or inconsistent NPY dtype, shape, or size")
        order = "F" if fortran_order else "C"
        return np.frombuffer(stream.read(), dtype=dtype).reshape(shape, order=order)
