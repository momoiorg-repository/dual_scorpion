"""Command-line entry point for the real-robot agent harness."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

from .config import HarnessConfig
from .harness import ScorpionAgentHarness
from .mcp import ScorpionMcpServer
from .safety import AuthorizationManager
from .skills import built_in_skill_registry


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.command == "skills":
        print(json.dumps(built_in_skill_registry().specs(), ensure_ascii=False, indent=2))
        return 0

    config = HarnessConfig.from_file(args.config)
    if args.command == "authorize-gripper":
        return _print(
            _authorize_gripper(config, args).to_status(),
        )
    if args.command == "authorize-arm":
        return _print(
            _authorize_arm(config, args).to_status(),
        )
    if args.command == "authorize-task-plan":
        return _print(
            _authorize_task_plan(config, args).to_status(),
        )
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
        if args.command == "gripper-state":
            harness.connect_robot()
            return _print(
                {
                    "status": "connected",
                    "gripper": harness.read_gripper_state(args.arm),
                }
            )
        if args.command == "gripper-test":
            result = _run_limited_gripper_test(harness, args)
            _print(result)
            return 0 if result["status"] in {"complete", "no_motion_required"} else 1
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

    authorize_gripper = subparsers.add_parser(
        "authorize-gripper",
        help="create a one-shot local grant for limited gripper motion",
    )
    authorize_gripper.add_argument("--arm", required=True, choices=("left", "right"))
    authorize_gripper.add_argument(
        "--action",
        default="set_position",
        choices=("open", "close", "set_position", "hold", "release", "grasp"),
    )
    authorize_gripper.add_argument("--max-position-delta", type=float, default=1.0)
    authorize_gripper.add_argument("--expires-in", type=float, default=300.0)
    authorize_gripper.add_argument("--confirm-fingertip-clearance", action="store_true")
    authorize_gripper.add_argument("--replace", action="store_true")

    authorize_arm = subparsers.add_parser(
        "authorize-arm",
        help="create a one-shot local grant for a validated arm motion plan",
    )
    authorize_arm.add_argument("--motion-plan-id", required=True)
    authorize_arm.add_argument("--expires-in", type=float, default=300.0)
    authorize_arm.add_argument("--replace", action="store_true")

    authorize_task = subparsers.add_parser(
        "authorize-task-plan",
        help="create a one-shot local grant for a validated task plan",
    )
    authorize_task.add_argument("--plan-id", required=True)
    authorize_task.add_argument(
        "--arm",
        action="append",
        choices=("left", "right"),
        required=True,
    )
    authorize_task.add_argument(
        "--gripper-action",
        action="append",
        choices=("open", "close", "set_position", "hold", "release", "grasp"),
        default=[],
    )
    authorize_task.add_argument("--max-position-delta", type=float)
    authorize_task.add_argument("--include-arm-motion", action="store_true")
    authorize_task.add_argument("--confirm-fingertip-clearance", action="store_true")
    authorize_task.add_argument("--expires-in", type=float, default=300.0)
    authorize_task.add_argument("--replace", action="store_true")

    gripper_state = subparsers.add_parser("gripper-state", help="connect and read one gripper")
    gripper_state.add_argument("--arm", required=True, choices=("left", "right"))

    gripper_test = subparsers.add_parser(
        "gripper-test",
        help="connect, read, move one gripper by an explicitly signed small delta, and stop",
    )
    gripper_test.add_argument("--arm", required=True, choices=("left", "right"))
    gripper_test.add_argument("--delta", type=float, required=True)

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


def _authorize_gripper(config: HarnessConfig, args: argparse.Namespace):
    manager = AuthorizationManager(config.operation_grants_file)
    return manager.issue_grant(
        operation="gripper",
        arms=[args.arm],
        actions=[args.action],
        expires_in_s=args.expires_in,
        max_position_delta=args.max_position_delta,
        fingertip_clearance_confirmed=args.confirm_fingertip_clearance,
        replace=args.replace,
    )


def _authorize_arm(config: HarnessConfig, args: argparse.Namespace):
    manager = AuthorizationManager(config.operation_grants_file)
    return manager.issue_grant(
        operation="arm",
        arms=["left", "right"],
        actions=["move_arm_to_pose"],
        expires_in_s=args.expires_in,
        plan_id=args.motion_plan_id,
        replace=args.replace,
    )


def _authorize_task_plan(config: HarnessConfig, args: argparse.Namespace):
    manager = AuthorizationManager(config.operation_grants_file)
    actions = list(args.gripper_action)
    if args.include_arm_motion:
        actions.append("move_arm_to_pose")
    return manager.issue_grant(
        operation="task_plan",
        arms=list(dict.fromkeys(args.arm)),
        actions=actions,
        expires_in_s=args.expires_in,
        max_position_delta=args.max_position_delta,
        plan_id=args.plan_id,
        fingertip_clearance_confirmed=args.confirm_fingertip_clearance,
        replace=args.replace,
    )


def _run_limited_gripper_test(harness: ScorpionAgentHarness, args: argparse.Namespace):
    harness.connect_robot()
    before = harness.read_gripper_state(args.arm)
    target = float(before["position"]) + float(args.delta)
    gripper = harness.config.gripper
    if not gripper.min_position <= target <= gripper.max_position:
        raise ValueError(
            f"target {target} is outside [{gripper.min_position}, {gripper.max_position}]"
        )
    result = harness.control_gripper(
        action="set_position",
        arms=args.arm,
        position=target,
    )
    after = harness.read_gripper_state(args.arm)
    command = result["results"][0]
    harness.stop_motion()
    return {
        "status": command["status"],
        "stop_reason": command["stop_reason"],
        "arm": args.arm,
        "requested_delta": float(args.delta),
        "initial_position": command["initial_position"],
        "target_position": command["target_position"],
        "final_position": command["final_position"],
        "measured_movement": command["measured_movement"],
        "position_error": command["position_error"],
        "position_reached": command["position_reached"],
        "motion_required": command["motion_required"],
        "motion_observed": command["motion_observed"],
        "motion_observation_threshold": command["motion_observation_threshold"],
        "command_count": command["command_count"],
        "motion_command_count": command["motion_command_count"],
        "before": before,
        "command": result,
        "after": after,
        "stop_requested": True,
        "timestamp_unix": time.time(),
    }
