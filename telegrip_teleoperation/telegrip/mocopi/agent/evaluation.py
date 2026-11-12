"""Machine-readable evaluation and replanning context for real executions."""

from __future__ import annotations

from typing import Any

import numpy as np

EVALUATION_SCHEMA_ID = "dual_scorpion.agent.real_evaluation.v1"


def evaluate_execution(report: dict[str, Any], plan: Any) -> dict[str, Any]:
    """Evaluate declared conditions only from robot feedback and harness state.

    Semantic image success is intentionally not inferred here. JOUZU/Gemini may
    interpret before/after images separately, but that judgement is not treated
    as a measured robot result.
    """
    checks: list[dict[str, Any]] = []
    frames = report.get("frames", [])
    for condition in plan.success_conditions:
        kind = condition.get("type")
        if kind == "tcp_position_error":
            tolerance = float(condition["max_value"])
            errors = [
                float(frame["arms"][side]["tcp_position_error_m"])
                for frame in frames
                for side in ("left", "right")
                if "tcp_position_error_m" in frame.get("arms", {}).get(side, {})
            ]
            passed = bool(errors) and max(errors, default=np.inf) <= tolerance
            checks.append(
                {
                    "condition": condition,
                    "passed": passed,
                    "observed_max_value": max(errors, default=None),
                    "values": errors,
                }
            )
        elif kind == "joint_tracking_error":
            tolerance = float(condition["max_value"])
            errors = [
                float(frame["arms"][side]["max_joint_error_deg"])
                for frame in frames
                for side in ("left", "right")
                if "max_joint_error_deg" in frame.get("arms", {}).get(side, {})
            ]
            passed = bool(errors) and max(errors, default=np.inf) <= tolerance
            checks.append(
                {
                    "condition": condition,
                    "passed": passed,
                    "observed_max_value": max(errors, default=None),
                    "values": errors,
                }
            )
        elif kind == "no_stop":
            passed = report.get("status") == "complete" and report.get("failure") is None
            checks.append({"condition": condition, "passed": passed, "report_status": report.get("status")})
        elif kind == "operator_confirmation":
            checks.append(
                {
                    "condition": condition,
                    "passed": False,
                    "reason": "operator_confirmation cannot be inferred from motor feedback",
                }
            )
        else:
            checks.append({"condition": condition, "passed": False, "reason": "unsupported condition"})
    success = report.get("status") == "complete" and bool(checks) and all(item["passed"] for item in checks)
    return {
        "schema": EVALUATION_SCHEMA_ID,
        "status": "success" if success else "failure",
        "checks": checks,
        "metrics": {
            "frames": len(frames),
            "measured_fps": report.get("measured_fps"),
            "gripper_commanded": report.get("gripper_commanded", False),
        },
        "ground_truth": {
            "joint_angles": "measured_encoder_feedback",
            "tcp_pose": "computed_from_measured_joints_and_urdf",
            "semantic_task_success": "requires_external_multimodal_judgement",
        },
        "llm_self_assessment_used": False,
    }


def build_replan_context(
    plan: Any,
    report: dict[str, Any],
    evaluation: dict[str, Any],
    world_state: dict[str, Any],
) -> dict[str, Any]:
    return {
        "schema": "dual_scorpion.agent.replan_context.v1",
        "goal": plan.goal,
        "plan": plan.to_dict(),
        "execution_status": report.get("status"),
        "failure": report.get("failure"),
        "failed_frames": report.get("frames", [])[-1:] if report.get("frames") else [],
        "evaluation": evaluation,
        "current_world_state": world_state,
        "needed_observations": [
            "fresh camera images",
            "fresh calibrated joint readings",
            "validated metric object pose if the next step references an object",
        ],
        "constraints": [
            "do not invent object IDs or coordinates",
            "do not emit direct joint commands",
            "preserve local safety and authorization checks",
        ],
    }
