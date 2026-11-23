"""Multi-camera acquisition and MCP image-content encoding."""

from __future__ import annotations

import base64
import hashlib
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np

from .config import CameraSpec


@dataclass(frozen=True)
class CameraFrame:
    camera_id: str
    timestamp_monotonic_ns: int
    width: int
    height: int
    mime_type: str
    image_bytes: bytes
    sha256: str
    coordinate_frame: str | None
    source: str
    properties: dict[str, Any]

    def to_mcp_content(self) -> dict[str, Any]:
        return {
            "type": "image",
            "data": base64.b64encode(self.image_bytes).decode("ascii"),
            "mimeType": self.mime_type,
        }

    def metadata(self) -> dict[str, Any]:
        return {
            "camera_id": self.camera_id,
            "timestamp_monotonic_ns": self.timestamp_monotonic_ns,
            "width": self.width,
            "height": self.height,
            "mime_type": self.mime_type,
            "sha256": self.sha256,
            "coordinate_frame": self.coordinate_frame,
            "source": self.source,
            "properties": dict(self.properties),
        }


class CameraError(RuntimeError):
    pass


class CameraService:
    """Open configured cameras on demand and return timestamped image content.

    Device paths, resolutions, and FPS are supplied by configuration. The
    service never claims a metric pose from a semantic image estimate.
    """

    def __init__(
        self,
        specs: tuple[CameraSpec, ...],
        *,
        backend_factory: Callable[[CameraSpec], Any] | None = None,
    ):
        self.specs = {spec.camera_id: spec for spec in specs}
        self._backend_factory = backend_factory
        self._backends: dict[str, Any] = {}
        self._last_frames: dict[str, CameraFrame] = {}
        self._last_errors: dict[str, str] = {}

    def list_cameras(self) -> list[dict[str, Any]]:
        return [self.status(camera_id) for camera_id in sorted(self.specs)]

    def status(self, camera_id: str) -> dict[str, Any]:
        if camera_id not in self.specs:
            raise CameraError(f"unknown camera {camera_id!r}")
        spec = self.specs[camera_id]
        last_frame = self._last_frames.get(camera_id)
        return {
            "camera_id": camera_id,
            "device": spec.device,
            "connected": camera_id in self._backends,
            "width": spec.width,
            "height": spec.height,
            "fps": spec.fps,
            "coordinate_frame": spec.coordinate_frame,
            "last_frame_monotonic_ns": None
            if last_frame is None
            else last_frame.timestamp_monotonic_ns,
            "last_error": self._last_errors.get(camera_id),
        }

    def capture(
        self,
        camera_ids: list[str] | None = None,
        *,
        max_age_s: float | None = None,
    ) -> list[CameraFrame]:
        requested = list(camera_ids) if camera_ids else list(self.specs)
        unknown = [camera_id for camera_id in requested if camera_id not in self.specs]
        if unknown:
            raise CameraError(f"unknown cameras: {unknown}")
        frames: list[CameraFrame] = []
        for camera_id in requested:
            try:
                frame = self._capture_one(camera_id)
                self._last_frames[camera_id] = frame
                self._last_errors.pop(camera_id, None)
                if max_age_s is not None and time.monotonic_ns() - frame.timestamp_monotonic_ns > max_age_s * 1e9:
                    raise CameraError(f"camera {camera_id!r} frame is older than {max_age_s}s")
                frames.append(frame)
            except Exception as exc:
                self._last_errors[camera_id] = str(exc)
                raise CameraError(f"camera {camera_id!r} capture failed: {exc}") from exc
        return frames

    def close(self) -> None:
        for backend in self._backends.values():
            close = getattr(backend, "close", None)
            if close is not None:
                close()
        self._backends.clear()

    def _capture_one(self, camera_id: str) -> CameraFrame:
        backend = self._backend(camera_id)
        timestamp, image = backend.read()
        image = np.asarray(image)
        if image.ndim != 3 or image.shape[2] < 3:
            raise ValueError("camera backend must return a 3-channel BGR image")
        encoded = _encode_jpeg(image)
        spec = self.specs[camera_id]
        return CameraFrame(
            camera_id=camera_id,
            timestamp_monotonic_ns=int(timestamp * 1e9),
            width=int(image.shape[1]),
            height=int(image.shape[0]),
            mime_type="image/jpeg",
            image_bytes=encoded,
            sha256=hashlib.sha256(encoded).hexdigest(),
            coordinate_frame=spec.coordinate_frame,
            source="real_camera_observation",
            properties={"device": spec.device, "configured_fps": spec.fps},
        )

    def _backend(self, camera_id: str) -> Any:
        if camera_id in self._backends:
            return self._backends[camera_id]
        spec = self.specs[camera_id]
        if self._backend_factory is not None:
            backend = self._backend_factory(spec)
        else:
            from ..camera import UvcCamera

            backend = UvcCamera(
                {
                    "device": spec.device,
                    "width": spec.width,
                    "height": spec.height,
                    "fps": spec.fps,
                    "fourcc": spec.fourcc,
                }
            )
        self._backends[camera_id] = backend
        return backend


def _encode_jpeg(image: np.ndarray) -> bytes:
    import cv2

    success, buffer = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if not success:
        raise RuntimeError("JPEG encoding failed")
    return buffer.tobytes()
