"""Structured plan receiver for JOUZU/Gemini.

The harness intentionally contains no LLM client. JOUZU performs natural
language and image reasoning, then submits the resulting JSON plan here.
"""

from __future__ import annotations

from typing import Any

from .schema import PLAN_SCHEMA_ID, TaskPlan, validate_task_plan
from .skills import SkillRegistry, built_in_skill_registry


class TaskPlanReceiver:
    def __init__(self, registry: SkillRegistry | None = None):
        self.registry = registry or built_in_skill_registry()

    def receive(self, value: dict[str, Any], world: Any) -> tuple[TaskPlan, list[str]]:
        plan = TaskPlan.from_dict(value)
        errors = validate_task_plan(
            plan,
            available_skills=self.registry.available_names(),
            world=world,
        )
        return plan, errors


def build_planning_context(world: Any) -> dict[str, Any]:
    """Return the deterministic context an external planner needs."""
    return {
        "plan_schema_id": PLAN_SCHEMA_ID,
        "world_state": world.to_dict(),
        "available_skills": built_in_skill_registry().specs(),
        "policy": {
            "natural_language_reasoning": "external_jouzu_gemini",
            "motion_generation": "deterministic_harness",
            "direct_motor_commands": "forbidden",
            "semantic_image_estimates": "not_metric_coordinates",
        },
    }
