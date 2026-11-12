---
name: dual-scorpion-real-agent
description: Plan and safely operate the Dual Scorpion real robot through the local MCP harness. Use this skill when the user asks to inspect robot or camera state, submit a structured task plan, validate motion, execute real-arm motion, or recover from an execution failure.
metadata:
  robot: dual-scorpion
  execution_layer: local-mcp
  simulation_only: false
---

# Dual Scorpion real-robot harness

This skill is an operating guide. It does not implement kinematics, control, or safety; all calculations and commands are delegated to the local MCP tools.

1. Read the current robot and camera status with `get_status`. Do not assume a camera or motor bus is connected.
2. Use `get_world_state` and `capture_images` before planning. Image frames are semantic observations, not metric coordinates. A 3D target is valid only when it comes from a measured or validated geometry source and is marked with `coordinate_validation`.
3. Use `update_object_estimates` for semantic labels or externally validated poses. Never invent an object ID, position, orientation, uncertainty, or confidence.
4. Produce a JSON task plan using schema `dual_scorpion.agent.task_plan.v2` and submit it with `submit_task_plan`. Do not emit joint angles, motor goals, or gripper commands in the plan.
5. Use `plan_motion` only after the plan is valid. The deterministic planner performs FK/IK, joint-limit, continuity, and trajectory validation.
6. Real motion requires the operator's local authorization token. Call `connect_robot` or `execute_motion` only when the user has explicitly authorized that operation and supplied the required token. Never ask the harness to infer or create authorization.
7. Monitor execution with `get_execution_report`. If execution fails, call `get_replan_context`, refresh observations, and generate a new plan that respects the reported constraints.
8. Call `stop_motion` immediately when the user requests a stop. Local safety checks take precedence over any model reasoning.

Gripper control is currently `unavailable` in this harness. Do not represent gripper operation as supported. The current motion backend commands joint0–joint6 only and never sends a gripper command.

Detailed tool contracts are in [references/mcp-tools.md](references/mcp-tools.md). Safety and authorization requirements are in [references/safety.md](references/safety.md).
