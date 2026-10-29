"""Command-line tools for human demonstration recording and conversion."""

import argparse
import json
import logging
from pathlib import Path

from ..config import camera_device, load_config
from .capture import capture_session


def parser():
    result = argparse.ArgumentParser(description="mocopi + head-camera demonstration datasets")
    result.add_argument("--config", default="config/mocopi/tracking.yaml")
    sub = result.add_subparsers(dest="command", required=True)
    capture = sub.add_parser(
        "capture", help="Record raw camera and mocopi streams; no IK or robot connection"
    )
    capture.add_argument("--output", required=True, help="New local session folder")
    capture.add_argument("--duration", type=float, default=30)
    capture.add_argument("--task", required=True, help="Description of the demonstrated task")
    capture.add_argument("--camera", type=camera_device, help="Camera index or device path")
    capture.add_argument("--fake", action="store_true", help="Use visibly synthetic camera and motion")
    convert = sub.add_parser("convert", help="Create a local LeRobot v3 dataset")
    convert.add_argument(
        "--input", action="append", required=True, help="Raw session folder; repeat for each"
    )
    convert.add_argument("--output", required=True, help="New local dataset folder")
    convert.add_argument("--repo-id", required=True, help="Local LeRobot repo id, e.g. local/mocopi-demo")
    convert.add_argument("--fps", type=int, default=30)
    convert.add_argument("--max-sync-s", type=float, default=0.08)
    convert.add_argument("--allow-mixed-sources", action="store_true")
    evaluate = sub.add_parser("evaluate", help="Quantify raw recording quality and wrist movement")
    evaluate.add_argument("--input", required=True, help="Raw session folder")
    evaluate.add_argument("--output", help="Optional JSON report")
    evaluate.add_argument("--fps", type=int, default=30)
    evaluate.add_argument("--max-sync-s", type=float, default=0.08)
    evaluate.add_argument("--workspace", help="Optional YAML with left/right min_m and max_m")
    return result


def main():
    args = parser().parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s: %(message)s")
    try:
        config = load_config(args.config, camera=getattr(args, "camera", None))
        if args.command == "capture":
            result = capture_session(config, args.output, args.duration, args.task, fake=args.fake)
        elif args.command == "convert":
            from .convert import convert_sessions

            result = convert_sessions(
                args.input,
                args.output,
                args.repo_id,
                fps=args.fps,
                max_sync_s=args.max_sync_s,
                allow_mixed_sources=args.allow_mixed_sources,
            )
        elif args.command == "evaluate":
            from .evaluate import session_metrics

            workspace = None
            if args.workspace:
                import yaml

                workspace = yaml.safe_load(Path(args.workspace).read_text())
            result = session_metrics(
                args.input, fps=args.fps, max_sync_s=args.max_sync_s, workspace=workspace
            )
            if args.output:
                output = Path(args.output)
                output.parent.mkdir(parents=True, exist_ok=True)
                with output.open("x", encoding="utf-8") as file:
                    file.write(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    except (ValueError, RuntimeError, OSError) as exc:
        parser().exit(1, f"Error: {exc}\n")


if __name__ == "__main__":
    main()
