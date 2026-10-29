"""Run with an environment containing mujoco and numpy. No dependencies are installed here."""
from pathlib import Path
import argparse
import time
import numpy as np
import mujoco

p = argparse.ArgumentParser()
p.add_argument('--headless', action='store_true')
p.add_argument('--steps', type=int, default=1000)
p.add_argument('--pose', choices=['neutral', 'fusion_current'], default='neutral')
args = p.parse_args()
model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name('scorpion.xml')))
data = mujoco.MjData(model)
key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, args.pose)
mujoco.mj_resetDataKeyframe(model, data, key)
if args.headless:
    for _ in range(args.steps):
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all()
    print('MuJoCo:', model.nq, 'DOF;', args.steps, 'steps; finite=', bool(np.isfinite(data.qpos).all()))
else:
    import mujoco.viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0, 0.08, 0.23]
        viewer.cam.distance = 1.2
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        viewer.opt.geomgroup[3] = 0
        while viewer.is_running():
            start = time.time()
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0, model.opt.timestep - (time.time()-start)))
