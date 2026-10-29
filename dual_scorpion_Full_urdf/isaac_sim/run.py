"""Run with Isaac Sim's python.sh; the USD already contains articulation and physics."""
from pathlib import Path
import argparse

p=argparse.ArgumentParser()
p.add_argument('--headless',action='store_true')
p.add_argument('--steps',type=int,default=1000)
p.add_argument('--pose',choices=['neutral','fusion_current'],default='neutral')
args=p.parse_args()
from isaacsim import SimulationApp
app=SimulationApp({'headless':args.headless})
import omni.usd
from isaacsim.core.api import World
filename='scorpion_current_pose.usda' if args.pose=='fusion_current' else 'scorpion.usd'
omni.usd.get_context().open_stage(str(Path(__file__).with_name(filename)))
for _ in range(5):
    app.update()
world=World(stage_units_in_meters=1.0)
world.reset()
for _ in range(args.steps):
    world.step(render=not args.headless)
app.close()
