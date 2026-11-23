"""Installable CLI for capture, policy inference, and local validation."""

from __future__ import annotations

import argparse
import json
import sys
from contextlib import closing
from dataclasses import asdict
from pathlib import Path
from typing import Any

import yaml

from .config import POLICIES, ROBOTS, Profile
from .exchange import (
    decode_observation,
    decode_output,
    encode_observation,
    encode_output,
    read_file,
    write_file,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="robot-policy", description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    def command(name: str, help_text: str, *, robot: bool = False, policy: bool = False):
        sub = commands.add_parser(name, help=help_text)
        sub.add_argument("--config", type=Path, help="trusted local YAML profile")
        if robot:
            sub.add_argument("--robot", help="robot adapter name")
        if policy:
            sub.add_argument("--policy", help="local adapter or expected remote policy ID")
        return sub

    command("robots", "list robot adapters")
    command("policies", "list policy adapters")
    command("doctor", "inspect robot configuration without opening motor ports", robot=True)
    caps = command("capabilities", "get or negotiate policy capabilities", robot=True, policy=True)
    caps.add_argument("--output", type=Path, help="save capabilities as JSON")
    capture = command(
        "capture", "capture a read-only observation or export a local dataset frame", robot=True
    )
    capture.add_argument("--output", required=True, type=Path)
    capture.add_argument("--task")
    capture.add_argument("--dataset-root", type=Path)
    capture.add_argument("--dataset-id")
    capture.add_argument("--frame", type=int, default=0)
    capture.add_argument("--video-backend", default="pyav")
    predict = command("predict", "predict from an observation file", robot=True, policy=True)
    predict.add_argument("--input", required=True, type=Path)
    predict.add_argument("--output", required=True, type=Path)
    validate = command("validate", "validate a prediction against its reference observation", robot=True)
    validate.add_argument("--input", required=True, type=Path)
    validate.add_argument("--observation", required=True, type=Path)
    run = command("run", "negotiate, capture, infer, and dry-run one chunk", robot=True, policy=True)
    run.add_argument("--task")
    run.add_argument("--output", type=Path)
    run.add_argument("--execute", action="store_true", help="request locally authorized actuation")
    command("serve", "serve a loaded policy over authenticated gRPC", policy=True)
    return parser


def _show(document: Any) -> None:
    print(json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False))


def _action_summary(output) -> dict[str, Any]:
    return {
        "request_id": output.request_id,
        "observation_id": output.observation_id,
        "policy_id": output.policy_id,
        "shape": list(output.actions.shape),
        "fps": output.fps,
    }


def _optional_robot_capabilities(profile: Profile, override: str | None):
    if override is None and not profile.section("robot").get("adapter"):
        return None
    with closing(profile.robot(override)) as robot:
        return robot.get_capabilities()


def _serve(profile: Profile, policy: str | None) -> None:
    from .grpc_transport import create_server

    listen = profile.section("inference").get("listen", "grpc://127.0.0.1:50051")
    token = profile.token()
    backend = profile.backend(policy)
    server, port = create_server(
        backend,
        listen,
        token,
        tls_certificate=profile.pem("tls_certificate"),
        tls_key=profile.pem("tls_key"),
    )
    server.start()
    try:
        print(f"Serving {backend.get_capabilities().policy_id} on port {port}", flush=True)
        server.wait_for_termination()
    finally:
        server.stop(grace=0).wait()


def _run(args: argparse.Namespace, profile: Profile) -> None:
    if args.command in {"robots", "policies"}:
        name = "robot" if args.command == "robots" else "policy"
        registered = set(ROBOTS if name == "robot" else POLICIES)
        configured = profile.section(name)
        if configured.get("plugin") and configured.get("adapter"):
            registered.add(configured["adapter"])
        _show(sorted(registered))
    elif args.command == "doctor":
        with closing(profile.robot(args.robot)) as robot:
            doctor = getattr(robot, "doctor", None)
            document = {"capabilities": asdict(robot.get_capabilities())}
            if doctor is not None:
                document["devices"] = doctor()
            _show(document)
    elif args.command == "capture":
        with closing(profile.robot(args.robot)) as robot:
            if args.dataset_root is None:
                observation = robot.get_observation(args.task)
            else:
                from .dataset import capture_dataset_frame

                observation = capture_dataset_frame(
                    robot.get_capabilities(),
                    args.dataset_root,
                    args.dataset_id,
                    args.frame,
                    task=args.task,
                    video_backend=args.video_backend,
                )
        write_file(args.output, encode_observation(observation))
        _show(
            {
                "observation_id": observation.observation_id,
                "state_dim": len(observation.state),
                "cameras": list(observation.images),
            }
        )
    elif args.command == "capabilities":
        robot_caps = _optional_robot_capabilities(profile, args.robot)
        client = profile.client(args.policy)
        with closing(client.transport):
            document = client.capabilities(robot_caps).to_dict()
        if args.output is not None:
            write_file(args.output, (json.dumps(document, indent=2, allow_nan=False) + "\n").encode("utf-8"))
        _show(document)
    elif args.command == "predict":
        robot_caps = _optional_robot_capabilities(profile, args.robot)
        observation = decode_observation(read_file(args.input))
        client = profile.client(args.policy)
        with closing(client.transport):
            output = client.predict(observation, robot=robot_caps)
        write_file(args.output, encode_output(output))
        _show(_action_summary(output))
    elif args.command == "validate":
        output = decode_output(read_file(args.input))
        observation = decode_observation(read_file(args.observation))
        with closing(profile.robot(args.robot)) as robot:
            validation = robot.validate_action(output, observation)
        _show({"dry_run": True, **asdict(validation)})
    elif args.command == "run":
        dry_run = profile.dry_run(args.execute)
        client = profile.client(args.policy)
        with closing(client.transport), closing(profile.robot(args.robot)) as robot:
            output, validation = client.run(robot, task=args.task, dry_run=dry_run)
        if args.output is not None:
            write_file(args.output, encode_output(output))
        _show({**_action_summary(output), **asdict(validation), "dry_run": dry_run})
    elif args.command == "serve":
        _serve(profile, args.policy)


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        _run(args, Profile.load(args.config))
    except KeyboardInterrupt:
        return 0 if args.command == "serve" else 130
    except (ValueError, OSError, RuntimeError, ImportError, TypeError, yaml.YAMLError) as exc:
        print(f"robot-policy: {exc}", file=sys.stderr)
        return 2
    return 0
