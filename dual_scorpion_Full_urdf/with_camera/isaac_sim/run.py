"""Open the camera-equipped articulation with Isaac Sim's bundled Python."""

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--steps", type=int, default=1000)
    parser.add_argument("--pose", choices=("neutral", "fusion_current"), default="neutral")
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("--steps must be positive")

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless})
    try:
        import omni.usd
        from isaacsim.core.api import World

        name = "scorpion_current_pose.usda" if args.pose == "fusion_current" else "scorpion.usd"
        path = Path(__file__).resolve().with_name(name)
        if not omni.usd.get_context().open_stage(str(path)):
            raise RuntimeError(f"Cannot open USD stage {path}")
        for _ in range(5):
            app.update()
        world = World(stage_units_in_meters=1.0)
        world.reset()
        for _ in range(args.steps):
            if not app.is_running():
                break
            world.step(render=not args.headless)
    finally:
        app.close()


if __name__ == "__main__":
    main()
