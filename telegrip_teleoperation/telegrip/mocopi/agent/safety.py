"""One-shot local operation grants and stop controls.

No authorization secret is exposed to JOUZU, Gemini, or MCP tool arguments.
The operator creates a short-lived grant locally. The harness consumes and
deletes that grant before issuing a matching motor command.
"""

from __future__ import annotations

import json
import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any

GRANT_SCHEMA = "dual_scorpion.agent.operation_grant.v1"
OPERATIONS = {"gripper", "arm", "task_plan"}


class AuthorizationError(RuntimeError):
    pass


@dataclass(frozen=True)
class AuthorizationGrant:
    grant_id: str
    operation: str
    arms: tuple[str, ...]
    actions: tuple[str, ...]
    max_position_delta: float | None
    plan_id: str | None
    issued_at_unix: float
    expires_at_unix: float
    fingertip_clearance_confirmed: bool

    def to_status(self) -> dict[str, Any]:
        return {
            "grant_id": self.grant_id,
            "operation": self.operation,
            "arms": list(self.arms),
            "actions": list(self.actions),
            "max_position_delta": self.max_position_delta,
            "plan_id": self.plan_id,
            "issued_at_unix": self.issued_at_unix,
            "expires_at_unix": self.expires_at_unix,
            "fingertip_clearance_confirmed": self.fingertip_clearance_confirmed,
        }


class AuthorizationManager:
    """Load, validate, and atomically consume one local grant at a time."""

    def __init__(self, grant_file: str | Path | None = None):
        self.grant_file = Path(grant_file).expanduser() if grant_file is not None else None
        self._grants: dict[str, AuthorizationGrant] = {}
        self._consumed: set[str] = set()
        self._lock = threading.RLock()

    def status(self) -> dict[str, Any]:
        with self._lock:
            grant = self._load_file_grant() if self.grant_file is not None else None
            if grant is None:
                grant = next((self._grants[value] for value in reversed(self._grants)), None)
            return {
                "mode": "one_shot_local_operation_grants",
                "grant_file": None if self.grant_file is None else str(self.grant_file),
                "active_grant": None if grant is None else grant.to_status(),
                "secret_exposed_to_mcp": False,
            }

    def issue_grant(
        self,
        *,
        operation: str,
        arms: list[str],
        actions: list[str],
        expires_in_s: float,
        max_position_delta: float | None = None,
        plan_id: str | None = None,
        fingertip_clearance_confirmed: bool = False,
        replace: bool = False,
    ) -> AuthorizationGrant:
        if operation not in OPERATIONS:
            raise AuthorizationError(f"unsupported operation {operation!r}")
        if not arms or any(arm not in {"left", "right"} for arm in arms):
            raise AuthorizationError("grant arms must contain left and/or right")
        if len(arms) != len(set(arms)):
            raise AuthorizationError("grant arms must not contain duplicates")
        if not actions:
            raise AuthorizationError("grant actions must not be empty")
        if not 1 <= expires_in_s <= 900:
            raise AuthorizationError("grant expiry must be between 1 and 900 seconds")
        if max_position_delta is not None and not 0 < max_position_delta <= 100:
            raise AuthorizationError("grant max_position_delta must be in (0, 100]")
        if operation in {"arm", "task_plan"} and plan_id is None:
            raise AuthorizationError(f"{operation} grants require a plan ID")
        if operation in {"gripper", "task_plan"}:
            gripper_actions = {
                "open",
                "close",
                "set_position",
                "hold",
                "release",
                "grasp",
            }.intersection(actions)
            if gripper_actions and max_position_delta is None:
                raise AuthorizationError("gripper grants require max_position_delta")
            if gripper_actions and not fingertip_clearance_confirmed:
                raise AuthorizationError(
                    "operator must explicitly confirm fingertip clearance for gripper motion"
                )
        now = time.time()
        grant = AuthorizationGrant(
            grant_id=f"grant-{uuid.uuid4().hex[:12]}",
            operation=operation,
            arms=tuple(arms),
            actions=tuple(actions),
            max_position_delta=max_position_delta,
            plan_id=plan_id,
            issued_at_unix=now,
            expires_at_unix=now + expires_in_s,
            fingertip_clearance_confirmed=fingertip_clearance_confirmed,
        )
        with self._lock:
            if self.grant_file is None:
                self._grants[grant.grant_id] = grant
            else:
                if self.grant_file.exists() and not replace:
                    raise AuthorizationError(f"grant file already exists: {self.grant_file}")
                self.grant_file.parent.mkdir(parents=True, exist_ok=True)
                self.grant_file.write_text(
                    json.dumps(_grant_to_document(grant), ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8",
                )
        return grant

    def consume(
        self,
        *,
        operation: str,
        arms: list[str],
        actions: list[str],
        max_deltas: dict[str, float] | None = None,
        plan_id: str | None = None,
    ) -> AuthorizationGrant:
        with self._lock:
            grant = self._load_file_grant() if self.grant_file is not None else None
            source = "file"
            if grant is None and self._grants:
                grant_id = next(reversed(self._grants))
                grant = self._grants[grant_id]
                source = "memory"
            if grant is None:
                raise AuthorizationError("no local operation grant is available")
            if grant.grant_id in self._consumed:
                raise AuthorizationError("operation grant has already been consumed")
            if grant.operation != operation:
                raise AuthorizationError(
                    f"operation grant is for {grant.operation!r}, not {operation!r}"
                )
            if time.time() >= grant.expires_at_unix:
                self._remove_grant(grant.grant_id, source)
                raise AuthorizationError("operation grant expired")
            if plan_id is not None and grant.plan_id != plan_id:
                raise AuthorizationError("operation grant does not match the requested plan ID")
            missing_arms = sorted(set(arms) - set(grant.arms))
            if missing_arms:
                raise AuthorizationError(f"operation grant does not cover arms {missing_arms}")
            missing_actions = sorted(set(actions) - set(grant.actions))
            if missing_actions:
                raise AuthorizationError(f"operation grant does not cover actions {missing_actions}")
            deltas = max_deltas or {}
            if deltas:
                if grant.max_position_delta is None:
                    raise AuthorizationError("operation grant has no gripper position range")
                exceeded = {
                    arm: delta
                    for arm, delta in deltas.items()
                    if not 0 <= delta <= grant.max_position_delta
                }
                if exceeded:
                    raise AuthorizationError(
                        f"requested gripper movement exceeds grant range: {exceeded}"
                    )
                if not grant.fingertip_clearance_confirmed:
                    raise AuthorizationError(
                        "operator fingertip-clearance confirmation is missing"
                    )
            self._remove_grant(grant.grant_id, source)
            self._consumed.add(grant.grant_id)
            return grant

    def _load_file_grant(self) -> AuthorizationGrant | None:
        if self.grant_file is None or not self.grant_file.is_file():
            return None
        try:
            document = json.loads(self.grant_file.read_text(encoding="utf-8"))
            grant = _grant_from_document(document)
        except Exception as exc:
            raise AuthorizationError(f"invalid operation grant file: {exc}") from exc
        if grant.grant_id in self._consumed:
            return None
        return grant

    def _remove_grant(self, grant_id: str, source: str) -> None:
        if source == "file":
            if self.grant_file is not None and self.grant_file.exists():
                self.grant_file.unlink()
        else:
            self._grants.pop(grant_id, None)


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

    def authorize_gripper(
        self,
        *,
        action: str,
        arms: list[str],
        position_deltas: dict[str, float],
    ) -> AuthorizationGrant:
        self._check_stop()
        return self.authorization.consume(
            operation="gripper",
            arms=arms,
            actions=[action],
            max_deltas=position_deltas,
        )

    def authorize_arm(self, motion_plan_id: str, arms: list[str]) -> AuthorizationGrant:
        self._check_stop()
        return self.authorization.consume(
            operation="arm",
            arms=arms,
            actions=["move_arm_to_pose"],
            plan_id=motion_plan_id,
        )

    def authorize_task_plan(
        self,
        *,
        plan_id: str,
        arms: list[str],
        includes_arm_motion: bool,
        gripper_actions: list[str],
        gripper_deltas: dict[str, float],
    ) -> AuthorizationGrant:
        self._check_stop()
        actions = ["move_arm_to_pose"] if includes_arm_motion else []
        actions.extend(gripper_actions)
        actions = list(dict.fromkeys(actions))
        return self.authorization.consume(
            operation="task_plan",
            arms=arms,
            actions=actions,
            max_deltas=gripper_deltas,
            plan_id=plan_id,
        )

    def _check_stop(self) -> None:
        if self.stop_requested:
            raise AuthorizationError("motion is blocked by an active stop request")


def _grant_from_document(document: Any) -> AuthorizationGrant:
    if not isinstance(document, dict) or document.get("schema") != GRANT_SCHEMA:
        raise ValueError("operation grant schema mismatch")
    grant_id = document.get("grant_id")
    operation = document.get("operation")
    arms = document.get("arms")
    actions = document.get("actions")
    max_delta = document.get("max_position_delta")
    plan_id = document.get("plan_id")
    issued = document.get("issued_at_unix")
    expires = document.get("expires_at_unix")
    clearance = document.get("fingertip_clearance_confirmed")
    if not isinstance(grant_id, str) or not grant_id:
        raise ValueError("grant_id must be a non-empty string")
    if operation not in OPERATIONS:
        raise ValueError("unsupported grant operation")
    if not isinstance(arms, list) or not arms or any(item not in {"left", "right"} for item in arms):
        raise ValueError("grant arms must contain left and/or right")
    if not isinstance(actions, list) or not actions or not all(isinstance(item, str) for item in actions):
        raise ValueError("grant actions must be a non-empty string array")
    if max_delta is not None and (not isinstance(max_delta, (int, float)) or not 0 < max_delta <= 100):
        raise ValueError("grant max_position_delta is invalid")
    if plan_id is not None and not isinstance(plan_id, str):
        raise ValueError("grant plan_id must be a string")
    if not isinstance(issued, (int, float)) or not isinstance(expires, (int, float)):
        raise ValueError("grant timestamps must be numbers")
    if expires <= issued:
        raise ValueError("grant expiry must be later than issue time")
    if not isinstance(clearance, bool):
        raise ValueError("fingertip_clearance_confirmed must be boolean")
    return AuthorizationGrant(
        grant_id=grant_id,
        operation=operation,
        arms=tuple(arms),
        actions=tuple(actions),
        max_position_delta=None if max_delta is None else float(max_delta),
        plan_id=plan_id,
        issued_at_unix=float(issued),
        expires_at_unix=float(expires),
        fingertip_clearance_confirmed=clearance,
    )


def _grant_to_document(grant: AuthorizationGrant) -> dict[str, Any]:
    return {
        "schema": GRANT_SCHEMA,
        **grant.to_status(),
    }
