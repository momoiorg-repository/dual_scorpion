"""Local authorization and stop controls.

An LLM cannot arm the robot. Motion requires a token that is supplied only by
the operator outside model context, and stop is enforced locally.
"""

from __future__ import annotations

import secrets
import threading
from pathlib import Path


class AuthorizationError(RuntimeError):
    pass


class AuthorizationManager:
    def __init__(self, token_file: str | Path | None = None, token: str | None = None):
        self._token = token
        self.token_file: Path | None = None
        if token_file is not None:
            path = Path(token_file)
            self.token_file = path
            if not path.is_file():
                self._token = None
            else:
                value = path.read_text(encoding="utf-8").strip()
                if value:
                    self._token = value

    @property
    def required(self) -> bool:
        return self._token is not None

    def verify(self, token: str | None) -> None:
        if self._token is None:
            raise AuthorizationError("motion authorization is not configured")
        if not isinstance(token, str) or not token:
            raise AuthorizationError("motion authorization token is required")
        if not secrets.compare_digest(token, self._token):
            raise AuthorizationError("invalid motion authorization token")


class SafetyManager:
    def __init__(self, authorization: AuthorizationManager):
        self.authorization = authorization
        self._stop_event = threading.Event()

    @property
    def stop_event(self) -> threading.Event:
        return self._stop_event

    @property
    def stop_requested(self) -> bool:
        return self._stop_event.is_set()

    def request_stop(self) -> None:
        self._stop_event.set()

    def clear_stop(self) -> None:
        self._stop_event.clear()

    def authorize_motion(self, token: str | None) -> None:
        if self.stop_requested:
            raise AuthorizationError("motion is blocked by an active stop request")
        self.authorization.verify(token)
