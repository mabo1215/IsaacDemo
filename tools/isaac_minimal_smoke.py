"""Small Isaac Sim startup/USD smoke test.

This intentionally excludes PhysX, ROS 2, Replicator, and World so it can
separate Kit/USD startup problems from scene/video problems on Windows.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from isaacsim.simulation_app import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--output', type=Path, default=Path('outputs/minimal_smoke.usd'))
    return parser.parse_args()


args = parse_args()
if args.headless:
    # Isaac Sim 4.5's Windows UI helpers can dereference a null viewport in
    # --no-window mode. This test has no UI dependency.
    SimulationApp._prepare_ui = lambda self: None
    SimulationApp._wait_for_viewport = lambda self: None

app = SimulationApp({
    'headless': args.headless,
    'renderer': 'RayTracedLighting',
    'physics_gpu': -1,
    'multi_gpu': False,
    'extra_args': [
        '--/physics/suppressReadback=false',
        '--/renderer/multiGpu/enabled=false',
    ],
    'width': 320,
    'height': 180,
})

try:
    import omni.usd
    from pxr import Gf, UsdGeom

    stage = omni.usd.get_context().get_stage()
    root = UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(root.GetPrim())
    cube = UsdGeom.Cube.Define(stage, '/World/SmokeCube')
    cube.GetSizeAttr().Set(1.0)
    cube.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    cube.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.5))
    for _ in range(5):
        app.update()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    stage.Export(str(args.output))
    print(f'[OK] minimal Isaac USD smoke: {args.output}')
finally:
    app.close()
