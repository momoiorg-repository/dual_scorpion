"""Non-blocking, session-scoped telemetry for TeleGrip.

Events are written as newline-delimited JSON (JSONL), which is both easy to
inspect by hand and straightforward to load with Python/pandas later.  The
control path only performs a non-blocking queue put; disk I/O happens on a
dedicated writer thread so telemetry cannot stall motor commands.
"""

from __future__ import annotations

import json
import logging
import queue
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

import numpy as np

logger = logging.getLogger(__name__)


def _json_safe(value: Any) -> Any:
    """Convert numpy and other common telemetry values to JSON-safe objects."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


class TelemetryRecorder:
    """Write timestamped telemetry events without blocking the control loop."""

    _STOP = object()

    def __init__(
        self,
        enabled: bool = True,
        log_dir: str = "logs/sessions",
        queue_size: int = 20_000,
    ):
        self.enabled = enabled
        path = Path(log_dir).expanduser()
        if not path.is_absolute():
            path = Path.cwd() / path
        self.log_dir = path
        self.queue: queue.Queue = queue.Queue(maxsize=queue_size)
        self.session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        self.path: Optional[Path] = None
        self._thread: Optional[threading.Thread] = None
        self._started = False
        self._dropped_events = 0

    def start(self) -> Optional[Path]:
        if not self.enabled or self._started:
            return self.path

        try:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            self.path = self.log_dir / f"telegrip-{self.session_id}.jsonl"
            self._thread = threading.Thread(
                target=self._writer_loop,
                name="telegrip-telemetry",
                daemon=True,
            )
            self._started = True
            self._thread.start()
            self.record(
                "session_start",
                session_id=self.session_id,
                format_version=1,
                clock_notes={
                    "wall_time": "UTC ISO-8601 for cross-system correlation",
                    "monotonic_ns": "local monotonic clock for latency calculations",
                },
            )
            logger.info("Telemetry recording to %s", self.path)
            return self.path
        except Exception as exc:
            self.enabled = False
            self._started = False
            logger.warning("Could not start telemetry recording: %s", exc)
            return None

    def record(self, event: str, **data: Any) -> None:
        if not self.enabled or not self._started:
            return

        payload = {
            "event": event,
            "session_id": self.session_id,
            "wall_time": datetime.now(timezone.utc).isoformat(timespec="microseconds"),
            "monotonic_ns": time.monotonic_ns(),
            **data,
        }
        try:
            self.queue.put_nowait(payload)
        except queue.Full:
            self._dropped_events += 1

    def stop(self) -> None:
        if not self._started:
            return

        self.record("session_end", dropped_events=self._dropped_events)
        try:
            self.queue.put(self._STOP, timeout=1.0)
        except queue.Full:
            logger.warning("Telemetry queue full during shutdown")
        if self._thread:
            self._thread.join(timeout=3.0)
        self._started = False

    def _writer_loop(self) -> None:
        try:
            with self.path.open("a", encoding="utf-8", buffering=1) as stream:
                while True:
                    item = self.queue.get()
                    if item is self._STOP:
                        break
                    stream.write(json.dumps(_json_safe(item), separators=(",", ":")) + "\n")
        except Exception as exc:
            logger.error("Telemetry writer stopped after an error: %s", exc)

