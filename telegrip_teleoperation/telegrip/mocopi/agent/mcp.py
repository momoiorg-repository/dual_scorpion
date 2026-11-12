"""Minimal MCP-compatible JSON-RPC server for JOUZU.

The server uses the MCP stdio transport's JSON-RPC core, so no provider SDK is
required. It exposes read-only observation tools separately from motion tools;
motion and robot connection require an operator-supplied authorization token.
"""

from __future__ import annotations

import json
import sys
from typing import Any

from .harness import ScorpionAgentHarness

MCP_PROTOCOL_VERSION = "2025-06-18"


class ScorpionMcpServer:
    def __init__(self, harness: ScorpionAgentHarness):
        self.harness = harness

    def run(self) -> None:
        for line in sys.stdin:
            if not line.strip():
                continue
            request = json.loads(line)
            response = self.handle(request)
            if response is None:
                continue
            sys.stdout.write(json.dumps(response, ensure_ascii=False, separators=(",", ":")) + "\n")
            sys.stdout.flush()

    def handle(self, request: dict[str, Any]) -> dict[str, Any] | None:
        method = request.get("method")
        request_id = request.get("id")
        try:
            if method == "initialize":
                result = {
                    "protocolVersion": MCP_PROTOCOL_VERSION,
                    "capabilities": {"tools": {"listChanged": False}},
                    "serverInfo": {"name": "dual-scorpion-agent", "version": "1.0.0"},
                }
            elif method == "notifications/initialized":
                return None
            elif method == "ping":
                result = {}
            elif method == "tools/list":
                result = {"tools": self.tools()}
            elif method == "tools/call":
                result = self.call_tool(
                    request["params"]["name"],
                    request["params"].get("arguments", {}),
                )
            else:
                return self._error(request_id, -32601, f"unknown method {method!r}")
        except Exception as exc:
            return self._error(request_id, -32000, str(exc))
        if request_id is None:
            return None
        return {"jsonrpc": "2.0", "id": request_id, "result": result}

    def tools(self) -> list[dict[str, Any]]:
        return [
            _tool(
                "get_status",
                "Return robot, calibration, measured-scene, camera, safety, execution, and skill availability.",
                {"type": "object", "properties": {}, "additionalProperties": False},
            ),
            _tool(
                "connect_robot",
                "Validate calibration, connect the follower buses, and read current encoders without homing. The local server must already hold operator authorization.",
                {
                    "type": "object",
                    "properties": {},
                    "additionalProperties": False,
                },
            ),
            _tool(
                "get_world_state",
                "Return the current provenance-labelled WorldState.",
                {"type": "object", "properties": {}, "additionalProperties": False},
            ),
            _tool(
                "read_gripper_state",
                "Read the calibrated normalized position, state, load, and current of one gripper.",
                {
                    "type": "object",
                    "properties": {"arm": {"type": "string", "enum": ["left", "right"]}},
                    "required": ["arm"],
                    "additionalProperties": False,
                },
            ),
            _tool(
                "list_skills",
                "List Robot Skills, contracts, hardware requirements, and implementation status.",
                {"type": "object", "properties": {}, "additionalProperties": False},
            ),
            _tool(
                "capture_images",
                "Capture camera frames and return MCP image content plus timestamps and metadata.",
                {
                    "type": "object",
                    "properties": {
                        "camera_ids": {"type": "array", "items": {"type": "string"}},
                        "max_age_s": {"type": "number", "exclusiveMinimum": 0},
                    },
                    "additionalProperties": False,
                },
            ),
            _tool(
                "update_object_estimates",
                "Store semantic or externally validated object estimates. Metric 3D poses require coordinate_validation.",
                {
                    "type": "object",
                    "properties": {"estimates": {"type": "array", "items": {"type": "object"}}},
                    "required": ["estimates"],
                    "additionalProperties": False,
                },
            ),
            _tool(
                "submit_task_plan",
                "Validate and store a structured task plan. Direct motor commands are rejected.",
                {
                    "type": "object",
                    "properties": {"plan": {"type": "object"}},
                    "required": ["plan"],
                    "additionalProperties": False,
                },
            ),
            _tool(
                "plan_motion",
                "Generate and validate a deterministic joint trajectory against the measured table/stick scene.",
                {
                    "type": "object",
                    "properties": {
                        "plan_id": {"type": "string"},
                        "fps": {"type": "integer", "minimum": 1, "maximum": 120},
                    },
                    "required": ["plan_id"],
                    "additionalProperties": False,
                },
            ),
            _tool(
                "execute_motion",
                "Start a real-robot motion execution job. The local server must already hold operator authorization.",
                {
                    "type": "object",
                    "properties": {
                        "motion_plan_id": {"type": "string"},
                        "capture_before_after": {"type": "boolean"},
                    },
                    "required": ["motion_plan_id"],
                    "additionalProperties": False,
                },
            ),
            _tool(
                "control_gripper",
                "Open, close, set, hold, release, or attempt a generic monitored grasp on one or both grippers. The local server must already hold operator authorization.",
                {
                    "type": "object",
                    "properties": {
                        "action": {
                            "type": "string",
                            "enum": ["open", "close", "set_position", "hold", "release", "grasp"],
                        },
                        "arms": {"type": "string", "enum": ["left", "right", "both"]},
                        "position": {"type": "number", "minimum": 0, "maximum": 100},
                    },
                    "required": ["action", "arms"],
                    "additionalProperties": False,
                },
            ),
            _tool(
                "execute_task_plan",
                "Execute a validated task plan in dependency order, combining arm motion and gripper skills. The local server must already hold operator authorization.",
                {
                    "type": "object",
                    "properties": {
                        "plan_id": {"type": "string"},
                        "capture_before_after": {"type": "boolean"},
                    },
                    "required": ["plan_id"],
                    "additionalProperties": False,
                },
            ),
            _tool(
                "stop_motion",
                "Set the local stop event for the active execution job.",
                {"type": "object", "properties": {}, "additionalProperties": False},
            ),
            _tool(
                "get_execution_report",
                "Return execution status and quantitative robot feedback.",
                {
                    "type": "object",
                    "properties": {"execution_id": {"type": "string"}},
                    "required": ["execution_id"],
                    "additionalProperties": False,
                },
            ),
            _tool(
                "get_replan_context",
                "Return failure, current state, and required observations for JOUZU-side replanning.",
                {
                    "type": "object",
                    "properties": {"execution_id": {"type": "string"}},
                    "required": ["execution_id"],
                    "additionalProperties": False,
                },
            ),
        ]

    def call_tool(self, name: str, arguments: dict[str, Any]) -> dict[str, Any]:
        if name == "get_status":
            return _structured(self.harness.status())
        if name == "connect_robot":
            return _structured(self.harness.connect_robot())
        if name == "get_world_state":
            return _structured(self.harness.world_state().to_dict())
        if name == "read_gripper_state":
            return _structured(self.harness.read_gripper_state(arguments["arm"]))
        if name == "list_skills":
            return _structured({"skills": self.harness.registry.specs()})
        if name == "capture_images":
            frames = self.harness.capture_images(
                arguments.get("camera_ids"),
                max_age_s=arguments.get("max_age_s"),
            )
            content = [frame.to_mcp_content() for frame in frames]
            content.append(
                {
                    "type": "text",
                    "text": json.dumps({"frames": [frame.metadata() for frame in frames]}, ensure_ascii=False),
                }
            )
            return {
                "content": content,
                "structuredContent": {"frames": [frame.metadata() for frame in frames]},
            }
        if name == "update_object_estimates":
            return _structured(self.harness.update_object_estimates(arguments["estimates"]))
        if name == "submit_task_plan":
            return _structured(self.harness.submit_task_plan(arguments["plan"]))
        if name == "plan_motion":
            return _structured(self.harness.plan_motion(arguments["plan_id"], fps=arguments.get("fps", 30)))
        if name == "execute_motion":
            return _structured(
                self.harness.execute_motion(
                    arguments["motion_plan_id"],
                    capture_before_after=arguments.get("capture_before_after", False),
                )
            )
        if name == "control_gripper":
            return _structured(
                self.harness.control_gripper(
                    action=arguments["action"],
                    arms=arguments["arms"],
                    position=arguments.get("position"),
                )
            )
        if name == "execute_task_plan":
            return _structured(
                self.harness.execute_task_plan(
                    arguments["plan_id"],
                    capture_before_after=arguments.get("capture_before_after", False),
                )
            )
        if name == "stop_motion":
            return _structured(self.harness.stop_motion())
        if name == "get_execution_report":
            return _structured(self.harness.execution_report(arguments["execution_id"]))
        if name == "get_replan_context":
            report_result = self.harness.execution_report(arguments["execution_id"])
            plan = self.harness.plans.get(report_result["plan_id"])
            if plan is None:
                raise RuntimeError("original task plan is unavailable")
            from .evaluation import build_replan_context, evaluate_execution

            report = report_result.get("report") or {}
            evaluation = evaluate_execution(report, plan)
            context = build_replan_context(plan, report, evaluation, self.harness.world_state().to_dict())
            return _structured(context)
        raise KeyError(f"unknown tool {name!r}")

    def _error(self, request_id: Any, code: int, message: str) -> dict[str, Any]:
        return {
            "jsonrpc": "2.0",
            "id": request_id,
            "error": {"code": code, "message": message},
        }


def _tool(name: str, description: str, input_schema: dict[str, Any]) -> dict[str, Any]:
    return {"name": name, "description": description, "inputSchema": input_schema}


def _structured(value: Any) -> dict[str, Any]:
    return {
        "content": [{"type": "text", "text": json.dumps(value, ensure_ascii=False)}],
        "structuredContent": value,
    }
