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
    capture.add_argument("--head-source", choices=["mocopi", "slam"], default="mocopi")
    capture.add_argument("--image-port", type=int, default=12354, help="Local rectified image stream port")
    capture.add_argument("--reference-id", help="Identifier for this mocopi coordinate/calibration epoch")
    convert = sub.add_parser("convert", help="Create a local LeRobot v3 dataset")
    convert.add_argument(
        "--input", action="append", required=True, help="Raw session folder; repeat for each"
    )
    convert.add_argument("--output", required=True, help="New local dataset folder")
    convert.add_argument("--repo-id", required=True, help="Local LeRobot repo id, e.g. local/mocopi-demo")
    convert.add_argument("--fps", type=int, default=30)
    convert.add_argument("--max-sync-s", type=float, default=0.08)
    convert.add_argument("--allow-mixed-sources", action="store_true")
    convert.add_argument("--allow-unverified-reference", action="store_true")
    evaluate = sub.add_parser("evaluate", help="Quantify raw recording quality and wrist movement")
    evaluate.add_argument("--input", required=True, help="Raw session folder")
    evaluate.add_argument("--output", help="Optional JSON report")
    evaluate.add_argument("--fps", type=int, default=30)
    evaluate.add_argument("--max-sync-s", type=float, default=0.08)
    evaluate.add_argument("--workspace", help="Optional YAML with left/right min_m and max_m")
    metrics = sub.add_parser("metrics-csv", help="Combine saved evaluation reports into one comparison CSV")
    metrics.add_argument("--input", action="append", required=True, help="Saved JSON report; repeat for each")
    metrics.add_argument("--output", required=True, help="New CSV file to write")
    plot = sub.add_parser("metrics-plot", help="Render selected report metrics as a dependency-free SVG")
    plot.add_argument("--input", action="append", required=True, help="Saved JSON report; repeat for each")
    plot.add_argument(
        "--metric",
        action="append",
        required=True,
        help="Flattened report metric, e.g. tracking.left.position_m.max; repeat for each",
    )
    plot.add_argument("--output", required=True, help="New SVG file to write")
    verify = sub.add_parser("verify", help="Read a finalized dataset and check structure and semantics")
    verify.add_argument("--dataset", required=True, help="Finalized mocopi dataset folder")
    verify.add_argument("--sample-frames", type=int, default=8, help="Sampled frames per episode (1..200)")
    verify.add_argument("--output", help="Optional JSON report")
    replay = sub.add_parser("replay-sim", help="Replay an episode in MuJoCo and quantify tracking")
    replay.add_argument("--dataset", required=True)
    replay.add_argument("--episode", type=int, default=0)
    replay.add_argument("--headless", action="store_true")
    replay.add_argument("--fast", action="store_true", help="Do not wait for recorded frame times")
    replay.add_argument("--slowdown", type=int, default=1, help="Hold each frame for 1..10 simulation ticks")
    replay.add_argument("--no-camera", action="store_true", help="Do not open the recorded camera window")
    replay.add_argument("--left-urdf", help="Override the left-arm URDF for this replay")
    replay.add_argument("--right-urdf", help="Override the right-arm URDF for this replay")
    replay.add_argument("--visual-urdf", help="Optional compatible assembly URDF for CAD visuals")
    replay.add_argument("--sim-model", choices=["full", "legacy"], help="Full native CAD or legacy arm URDFs")
    replay.add_argument("--mujoco-model", help="Override the full MuJoCo XML model path")
    replay.add_argument(
        "--start-pose",
        choices=["work", "backwards", "config"],
        help="Override the configured initial robot pose for this plan",
    )
    replay.add_argument(
        "--obstacles",
        help="Optional YAML with operator-declared box obstacles for the full model",
    )
    replay.add_argument("--plan", help="Write a new robot joint plan JSON")
    replay.add_argument("--report", help="Write a new simulation metrics JSON")
    joints = sub.add_parser(
        "convert-joints", help="Create a separate dataset of simulated joint trajectories"
    )
    joints.add_argument("--dataset", required=True, help="Source human mocopi LeRobot dataset")
    joints.add_argument("--plan", action="append", required=True, help="Plan for a source episode; repeat")
    joints.add_argument("--output", required=True, help="New local dataset folder")
    joints.add_argument("--repo-id", required=True)
    joints.add_argument("--left-urdf", help="Left-arm URDF used by the plans")
    joints.add_argument("--right-urdf", help="Right-arm URDF used by the plans")
    joints.add_argument("--mujoco-model", help="Full MuJoCo XML used by the plans")
    joints.add_argument("--max-sim-error-m", type=float, default=0.025)
    joints.add_argument("--max-sim-orientation-error-deg", type=float)
    feedback = sub.add_parser("convert-real", help="Create a dataset from recorded robot feedback")
    feedback.add_argument(
        "--run", action="append", nargs=2, metavar=("PLAN", "REPORT"), required=True, help="Repeat per run"
    )
    feedback.add_argument("--output", required=True, help="New local dataset folder")
    feedback.add_argument("--repo-id", required=True)
    real = sub.add_parser("replay-real", help="Validate or explicitly replay a plan on the real follower")
    real.add_argument("--plan", required=True)
    real.add_argument("--execute", action="store_true", help="Connect and move the real arm joints")
    real.add_argument("--left-port", help="Left follower serial device")
    real.add_argument("--right-port", help="Right follower serial device")
    real.add_argument("--robot-id", help="ID used for the existing robot motor calibration")
    real.add_argument("--calibration-dir", help="Override the existing robot calibration directory")
    real.add_argument("--report", help="New JSON output for measured real-robot results")
    real.add_argument("--allow-synthetic", action="store_true", help="Permit a plan sourced from --fake")
    real.add_argument("--max-sim-error-m", type=float, default=0.025)
    real.add_argument(
        "--max-sim-orientation-error-deg",
        type=float,
        help="Optionally reject plans exceeding this simulated wrist angle error",
    )
    real.add_argument("--left-urdf", help="Left-arm URDF used to create the plan")
    real.add_argument("--right-urdf", help="Right-arm URDF used to create the plan")
    real.add_argument("--mujoco-model", help="Full MuJoCo XML used for simulation-only validation")
    compare = sub.add_parser(
        "compare-real", help="Compare real feedback with simulation and measured TCP points"
    )
    compare.add_argument("--plan", required=True)
    compare.add_argument("--report", required=True)
    compare.add_argument("--reference-csv", help="Optional physical TCP measurements in robot-base metres")
    compare.add_argument("--output", help="New JSON comparison report")
    compare.add_argument("--left-urdf", help="Left-arm URDF used in the plan")
    compare.add_argument("--right-urdf", help="Right-arm URDF used in the plan")
    return result


def main():
    args = parser().parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s: %(message)s")
    try:
        config = load_config(args.config, camera=getattr(args, "camera", None))
        if args.command == "capture":
            result = capture_session(
                config,
                args.output,
                args.duration,
                args.task,
                fake=args.fake,
                head_source=args.head_source,
                image_port=args.image_port,
                reference_id=args.reference_id,
            )
        elif args.command == "convert":
            from .convert import convert_sessions

            result = convert_sessions(
                args.input,
                args.output,
                args.repo_id,
                fps=args.fps,
                max_sync_s=args.max_sync_s,
                allow_mixed_sources=args.allow_mixed_sources,
                allow_unverified_reference=args.allow_unverified_reference,
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
        elif args.command == "metrics-csv":
            from .report_csv import export_metrics_csv

            result = export_metrics_csv(args.input, args.output)
        elif args.command == "metrics-plot":
            from .report_plot import export_metrics_svg

            result = export_metrics_svg(args.input, args.metric, args.output)
        elif args.command == "verify":
            from .verify import verify_dataset

            result = verify_dataset(args.dataset, sample_frames=args.sample_frames)
            if args.output:
                output = Path(args.output)
                output.parent.mkdir(parents=True, exist_ok=True)
                with output.open("x", encoding="utf-8") as file:
                    file.write(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
            if result["errors"]:
                print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
                parser().exit(1, f"Error: dataset verification failed: {', '.join(result['errors'])}\n")
        elif args.command == "replay-sim":
            from .replay import replay_sim

            custom_urdf = bool(args.left_urdf or args.right_urdf or args.visual_urdf)
            sim_model = args.sim_model or ("legacy" if custom_urdf else "full")
            if (sim_model == "full" and custom_urdf) or (sim_model == "legacy" and args.mujoco_model):
                raise ValueError("Select either the full MuJoCo XML or the legacy arm URDFs")
            if args.mujoco_model:
                config["robot"]["mujoco_full_model"] = str(Path(args.mujoco_model).resolve())
            for side in ("left", "right"):
                path = getattr(args, f"{side}_urdf")
                if path:
                    config["robot"]["urdf"][side] = str(Path(path).resolve())
            if args.visual_urdf:
                config["robot"]["visual_urdf"] = str(Path(args.visual_urdf).resolve())
            elif args.left_urdf or args.right_urdf:
                config["robot"]["visual_urdf"] = None
            for path in (args.plan, args.report):
                if path and Path(path).exists():
                    raise FileExistsError(f"Output already exists: {path}")
            plan, result = replay_sim(
                config,
                args.dataset,
                args.episode,
                gui=not args.headless,
                realtime=not args.fast,
                show_camera=not args.no_camera,
                slowdown=args.slowdown,
                sim_model=sim_model,
                start_pose=args.start_pose,
                obstacles=args.obstacles,
            )
            for path, document in ((args.plan, plan), (args.report, result)):
                if path:
                    output = Path(path)
                    output.parent.mkdir(parents=True, exist_ok=True)
                    with output.open("x", encoding="utf-8") as file:
                        file.write(json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
        elif args.command == "convert-joints":
            from .joint_dataset import export_joint_dataset

            if args.mujoco_model:
                config["robot"]["mujoco_full_model"] = str(Path(args.mujoco_model).resolve())
            for side in ("left", "right"):
                path = getattr(args, f"{side}_urdf")
                if path:
                    config["robot"]["urdf"][side] = str(Path(path).resolve())
            result = export_joint_dataset(
                config,
                args.dataset,
                args.plan,
                args.output,
                args.repo_id,
                max_sim_error_m=args.max_sim_error_m,
                max_sim_orientation_error_deg=args.max_sim_orientation_error_deg,
            )
        elif args.command == "convert-real":
            from .real_dataset import export_real_dataset

            result = export_real_dataset(args.run, args.output, args.repo_id)
        elif args.command == "replay-real":
            from .real import replay_real

            if args.mujoco_model:
                config["robot"]["mujoco_full_model"] = str(Path(args.mujoco_model).resolve())
            for side in ("left", "right"):
                path = getattr(args, f"{side}_urdf")
                if path:
                    config["robot"]["urdf"][side] = str(Path(path).resolve())
            real_result = replay_real(
                config,
                args.plan,
                execute=args.execute,
                left_port=args.left_port,
                right_port=args.right_port,
                robot_id=args.robot_id,
                calibration_dir=args.calibration_dir,
                report=args.report,
                allow_synthetic=args.allow_synthetic,
                max_sim_error_m=args.max_sim_error_m,
                max_sim_orientation_error_deg=args.max_sim_orientation_error_deg,
            )
            if args.execute:
                result = {
                    "mode": "real",
                    "status": real_result["status"],
                    "frames": len(real_result["frames"]),
                    "measured_fps": real_result["measured_fps"],
                    "tracking": real_result["tracking"],
                    "report": args.report,
                }
            else:
                result = real_result
        elif args.command == "compare-real":
            from .compare import compare_real

            for side in ("left", "right"):
                path = getattr(args, f"{side}_urdf")
                if path:
                    config["robot"]["urdf"][side] = str(Path(path).resolve())
            result = compare_real(config, args.plan, args.report, args.reference_csv)
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
