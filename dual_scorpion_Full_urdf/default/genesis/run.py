"""Genesis 1.4.4 loader. Use --headless for a CPU smoke test."""
from pathlib import Path
import argparse
import json
import tempfile
import os

p = argparse.ArgumentParser()
p.add_argument('--headless', action='store_true')
p.add_argument('--steps', type=int, default=500)
p.add_argument('--pose', choices=['neutral', 'fusion_current'], default='neutral')
p.add_argument('--report', type=Path)
args = p.parse_args()
with tempfile.TemporaryDirectory(prefix='scorpion_genesis_') as cache:
    os.environ.setdefault('GS_CACHE_FILE_PATH', cache)
    os.environ.setdefault('TI_OFFLINE_CACHE_FILE_PATH', cache+'/taichi')
    import numpy as np
    import genesis as gs
    gs.init(backend=gs.cpu, logging_level='warning')
    scene = gs.Scene(sim_options=gs.options.SimOptions(dt=0.002, substeps=2),
                     rigid_options=gs.options.RigidOptions(enable_self_collision=True, enable_neutral_collision=True,
                                                          enable_adjacent_collision=False,
                                                          max_collision_pairs=512),
                     viewer_options=gs.options.ViewerOptions(camera_pos=(1,-1,0.8), camera_lookat=(0,0,0.23)),
                     show_viewer=not args.headless)
    scene.add_entity(gs.morphs.Plane(pos=(0,0,-0.03)),material=gs.materials.Rigid(friction=0.6))
    robot = scene.add_entity(gs.morphs.URDF(file=str(Path(__file__).with_name('scorpion.urdf')),
                            fixed=True, merge_fixed_links=False, recompute_inertia=False,
                            convexify=False, decimate=False, watertighten=None),
                            material=gs.materials.Rigid(friction=0.6))
    scene.build()
    names = [f'joint{i}_{side}' for side in ('R','L') for i in range(8)]
    dofs = [robot.get_joint(name).dofs_idx_local[0] for name in names]
    state = json.loads(Path(__file__).with_name('initial_state.json').read_text())
    q = np.array([state[name] if args.pose=='fusion_current' else 0.0 for name in names])
    robot.set_dofs_position(q, dofs_idx_local=dofs)
    robot.set_dofs_kp(np.full(16,10.0), dofs_idx_local=dofs)
    robot.set_dofs_kv(np.full(16,0.5), dofs_idx_local=dofs)
    robot.set_dofs_force_range(np.full(16,-1.0),np.full(16,1.0),dofs_idx_local=dofs)
    robot.control_dofs_position(q,dofs_idx_local=dofs)
    for _ in range(args.steps):
        scene.step()
    current = robot.get_dofs_position().detach().cpu().numpy()
    assert np.isfinite(current).all()
    print('Genesis:', robot.n_dofs, 'DOF;', args.steps, 'steps; finite=', bool(np.isfinite(current).all()))
    if args.report:
        args.report.write_text(json.dumps({'engine':'Genesis','version':gs.__version__,'backend':'cpu',
            'headless':args.headless,'dofs':robot.n_dofs,'links':robot.n_links,'geoms':robot.n_geoms,
            'steps':args.steps,'state_finite':bool(np.isfinite(current).all()),
            'self_collision':True,'neutral_collision':True,'adjacent_collision':False,
            'watertighten_disabled':True,'runtime_tested':True},indent=2))
    gs.destroy()
