"""Validated task-plan contracts for the real-robot agent harness.

The plan is deliberately Cartesian/semantic at the LLM boundary. It cannot
contain direct motor commands; the deterministic planner converts validated
targets into joint trajectories and the execution layer independently checks
them against calibration, FK, limits, and feedback.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

PLAN_SCHEMA_ID = "dual_scorpion.agent.task_plan.v2"
SIDES = frozenset({"left", "right", "both"})
FORBIDDEN_PARAMETER_KEYS = {
    "joint_deg",
    "joint_command",
    "motor_command",
    "goal_position",
    "gripper_command",
    "gripper_deg",
}


class PlanValidationError(ValueError):
    """Raised when a model-generated plan is not executable as stated."""


@dataclass(frozen=True)
class PlanStep:
    id: str
    skill: str
    arms: str
    depends_on: list[str] = field(default_factory=list)
    parameters: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> PlanStep:
        if not isinstance(value, dict):
            raise PlanValidationError("each plan step must be an object")
        step_id = _required_string(value, "id", "step.id")
        skill = _required_string(value, "skill", f"step {step_id!r} skill")
        arms = _required_string(value, "arms", f"step {step_id!r} arms")
        if arms not in SIDES:
            raise PlanValidationError(f"step {step_id!r} has unsupported arms {arms!r}")
        depends_on = value.get("depends_on", [])
        if not isinstance(depends_on, list) or not all(isinstance(item, str) for item in depends_on):
            raise PlanValidationError(f"step {step_id!r} depends_on must be a string array")
        parameters = value.get("parameters", {})
        if not isinstance(parameters, dict):
            raise PlanValidationError(f"step {step_id!r} parameters must be an object")
        forbidden = sorted(FORBIDDEN_PARAMETER_KEYS.intersection(parameters))
        if forbidden:
            raise PlanValidationError(
                f"step {step_id!r} contains direct motor fields {forbidden}; "
                "use a validated Cartesian target or a relative motion"
            )
        return cls(step_id, skill, arms, list(depends_on), dict(parameters))

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "skill": self.skill,
            "arms": self.arms,
            "depends_on": list(self.depends_on),
            "parameters": dict(self.parameters),
        }


@dataclass(frozen=True)
class TaskPlan:
    schema: str
    goal: str
    goal_state: dict[str, Any]
    objects: list[str]
    environment: dict[str, Any]
    arm_roles: dict[str, str]
    steps: list[PlanStep]
    preconditions: list[str]
    success_conditions: list[dict[str, Any]]
    failure_conditions: list[dict[str, Any]]
    safety_constraints: list[str]
    replan_policy: dict[str, Any]
    provenance: dict[str, Any]

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> TaskPlan:
        if not isinstance(value, dict):
            raise PlanValidationError("task plan must be a JSON object")
        schema = _required_string(value, "schema", "plan.schema")
        if schema != PLAN_SCHEMA_ID:
            raise PlanValidationError(f"unsupported plan schema {schema!r}")
        goal = _required_string(value, "goal", "plan.goal")
        for name in ("goal_state", "environment", "arm_roles", "replan_policy", "provenance"):
            if name not in value or not isinstance(value[name], dict):
                raise PlanValidationError(f"plan.{name} must be an object")
        for name in ("objects", "preconditions", "success_conditions", "failure_conditions", "safety_constraints"):
            if name not in value or not isinstance(value[name], list):
                raise PlanValidationError(f"plan.{name} must be an array")
        if not all(isinstance(item, str) for item in value["objects"]):
            raise PlanValidationError("plan.objects must contain object IDs")
        if not all(isinstance(item, str) for item in value["preconditions"]):
            raise PlanValidationError("plan.preconditions must contain strings")
        for name in ("success_conditions", "failure_conditions"):
            if not all(isinstance(item, dict) for item in value[name]):
                raise PlanValidationError(f"plan.{name} must contain objects")
        if not all(isinstance(item, str) for item in value["safety_constraints"]):
            raise PlanValidationError("plan.safety_constraints must contain strings")
        steps_value = value.get("steps", [])
        if not isinstance(steps_value, list):
            raise PlanValidationError("plan.steps must be an array")
        steps = [PlanStep.from_dict(item) for item in steps_value]
        return cls(
            schema=schema,
            goal=goal,
            goal_state=dict(value["goal_state"]),
            objects=list(value["objects"]),
            environment=dict(value["environment"]),
            arm_roles=dict(value["arm_roles"]),
            steps=steps,
            preconditions=list(value["preconditions"]),
            success_conditions=[dict(item) for item in value["success_conditions"]],
            failure_conditions=[dict(item) for item in value["failure_conditions"]],
            safety_constraints=list(value["safety_constraints"]),
            replan_policy=dict(value["replan_policy"]),
            provenance=dict(value["provenance"]),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": self.schema,
            "goal": self.goal,
            "goal_state": dict(self.goal_state),
            "objects": list(self.objects),
            "environment": dict(self.environment),
            "arm_roles": dict(self.arm_roles),
            "steps": [step.to_dict() for step in self.steps],
            "preconditions": list(self.preconditions),
            "success_conditions": [dict(item) for item in self.success_conditions],
            "failure_conditions": [dict(item) for item in self.failure_conditions],
            "safety_constraints": list(self.safety_constraints),
            "replan_policy": dict(self.replan_policy),
            "provenance": dict(self.provenance),
        }


def validate_task_plan(
    plan: TaskPlan,
    *,
    available_skills: set[str],
    world: Any,
) -> list[str]:
    """Return all missing or unsafe plan inputs. Empty means structurally valid."""
    errors: list[str] = []
    step_ids = [step.id for step in plan.steps]
    if len(step_ids) != len(set(step_ids)):
        errors.append("step IDs are not unique")
    known_steps = set(step_ids)
    for step in plan.steps:
        if step.skill not in available_skills:
            errors.append(f"step {step.id!r} requires unavailable skill {step.skill!r}")
        for dependency in step.depends_on:
            if dependency not in known_steps:
                errors.append(f"step {step.id!r} depends on unknown step {dependency!r}")
            elif dependency == step.id:
                errors.append(f"step {step.id!r} depends on itself")
    if _has_cycle(plan.steps):
        errors.append("step dependencies contain a cycle")
    if not plan.steps:
        errors.append("plan has no executable steps")
    for object_id in plan.objects:
        if object_id not in world.objects:
            errors.append(f"plan references unobserved object {object_id!r}")
    for step in plan.steps:
        if step.skill != "move_arm_to_pose":
            continue
        target_keys = {
            "relative_position_m",
            "target_object_id",
            "target_position_m",
        }.intersection(step.parameters)
        if not target_keys:
            errors.append(f"step {step.id!r} has no supported Cartesian target")
        if "target_object_id" in step.parameters:
            object_id = step.parameters["target_object_id"]
            if object_id not in world.objects:
                errors.append(f"step {step.id!r} references unobserved object {object_id!r}")
            elif world.objects[object_id].position_m is None:
                errors.append(
                    f"step {step.id!r} uses object {object_id!r} without a validated metric pose"
                )
        if "target_position_m" in step.parameters and not step.parameters.get("pose_validation"):
            errors.append(
                f"step {step.id!r} has an absolute metric target without pose_validation"
            )
        if "relative_position_m" in step.parameters:
            vector = np.asarray(step.parameters["relative_position_m"], dtype=float)
            if vector.shape != (3,) or not np.isfinite(vector).all() or np.linalg.norm(vector) > 0.35:
                errors.append(f"step {step.id!r} relative_position_m is invalid or exceeds 0.35 m")
    for condition in plan.success_conditions:
        kind = condition.get("type")
        if kind not in {"tcp_position_error", "joint_tracking_error", "operator_confirmation", "no_stop"}:
            errors.append(f"unknown success condition type {kind!r}")
        if kind in {"tcp_position_error", "joint_tracking_error"}:
            tolerance = condition.get("max_value")
            if not _positive_finite(tolerance):
                errors.append(f"success condition {kind!r} requires positive max_value")
    return errors


def _has_cycle(steps: list[PlanStep]) -> bool:
    graph = {step.id: set(step.depends_on) for step in steps}
    complete: set[str] = set()
    active: set[str] = set()

    def visit(node: str) -> bool:
        if node in complete:
            return False
        if node in active:
            return True
        active.add(node)
        if any(visit(dependency) for dependency in graph[node]):
            return True
        active.remove(node)
        complete.add(node)
        return False

    return any(visit(node) for node in graph)


def _required_string(value: dict[str, Any], key: str, label: str) -> str:
    result = value.get(key)
    if not isinstance(result, str) or not result.strip():
        raise PlanValidationError(f"{label} must be a non-empty string")
    return result


def _positive_finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and np.isfinite(value) and value > 0
