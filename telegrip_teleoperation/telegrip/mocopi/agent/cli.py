"""Command-line entry point for the real-robot agent harness."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from .config import HarnessConfig
from .harness import ScorpionAgentHarness
from .mcp import ScorpionMcpServer
from .skills import built_in_skill_registry


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.command == "skills":
        print(json.dumps(built_in_skill_registry().specs(), ensure_ascii=False, indent=2))
        return 0

    config = HarnessConfig.from_file(args.config)
    with ScorpionAgentHarness(config) as harness:
        if args.command == "mcp":
            ScorpionMcpServer(harness).run()
            return 0
        if args.command == "status":
            return _print(harness.status())
        if args.command == "world":
            return _print(harness.world_state().to_dict())
        if args.command == "capture":
            frames = harness.capture_images(args.camera_ids)
            output_dir = Path(args.output_dir)
            output_dir.mkdir(parents=True, exist_ok=True)
            rows = []
            for frame in frames:
                image_path = output_dir / f"{frame.camera_id}-{frame.timestamp_monotonic_ns}.jpg"
                image_path.write_bytes(frame.image_bytes)
                rows.append({**frame.metadata(), "path": str(image_path)})
            return _print({"status": "success", "frames": rows})
        if args.command == "validate-plan":
            plan = _load_json(args.plan)
            result = harness.submit_task_plan(plan)
            return _print(result, nonzero_on_rejection=True)
        if args.command == "plan":
            result = harness.plan_motion(args.plan_id, fps=args.fps)
            return _print(result)
        if args.command == "report":
            return _print(harness.execution_report(args.execution_id))
    parser.error(f"unknown command {args.command!r}")
    return 2


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Dual Scorpion real-robot agent harness")
    parser.add_argument("--config", default="config/mocopi/agent.example.yaml")
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("skills", help="print Robot Skill contracts")
    subparsers.add_parser("status", help="print robot, camera, and safety status")
    subparsers.add_parser("world", help="print the current WorldState")

    capture = subparsers.add_parser("capture", help="capture configured camera images")
    capture.add_argument("--camera-ids", nargs="*", default=None)
    capture.add_argument("--output-dir", default="outputs/agent/images")

    validate = subparsers.add_parser("validate-plan", help="validate a structured JSON plan")
    validate.add_argument("--plan", required=True)

    plan = subparsers.add_parser("plan", help="generate and validate a motion plan")
    plan.add_argument("--plan-id", required=True)
    plan.add_argument("--fps", type=int, default=30)

    report = subparsers.add_parser("report", help="read an execution report")
    report.add_argument("--execution-id", required=True)

    subparsers.add_parser("mcp", help="run the MCP stdio server")
    return parser


def _load_json(path: str) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("JSON root must be an object")
    return value


def _print(value: Any, *, nonzero_on_rejection: bool = False) -> int:
    print(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))
    if nonzero_on_rejection and value.get("status") == "rejected":
        return 1
    return 0
