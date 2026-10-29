"""Command-line tools for human demonstration recording and conversion."""

import argparse
import json
import logging

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
    return result


def main():
    args = parser().parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s: %(message)s")
    try:
        config = load_config(args.config, camera=getattr(args, "camera", None))
        if args.command == "capture":
            result = capture_session(config, args.output, args.duration, args.task, fake=args.fake)
            print(json.dumps(result, ensure_ascii=False, indent=2))
    except (ValueError, RuntimeError, OSError) as exc:
        parser().exit(1, f"Error: {exc}\n")


if __name__ == "__main__":
    main()
