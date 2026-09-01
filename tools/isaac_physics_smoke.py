"""Isaac Sim 4.5 PhysX-only smoke test (no ROS 2 and no Replicator)."""

from __future__ import annotations

from pathlib import Path

from isaacsim.simulation_app import SimulationApp

SimulationApp._prepare_ui = lambda self: None
SimulationApp._wait_for_viewport = lambda self: None
app = SimulationApp({
    'headless': True,
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
    from pxr import Gf, UsdGeom, UsdPhysics
    from isaacsim.core.api import World

    stage = omni.usd.get_context().get_stage()
    root = UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(root.GetPrim())
    for path, size, position in (
        ('/World/Ground', (8.0, 8.0, 0.1), (0.0, 0.0, -0.05)),
        ('/World/Block', (0.5, 0.5, 0.5), (0.0, 0.0, 2.0)),
    ):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.GetSizeAttr().Set(1.0)
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        cube.AddTranslateOp().Set(Gf.Vec3d(*position))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        if path.endswith('Block'):
            UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
            UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr().Set(1.0)

    print('[MARK] before World')
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0, rendering_dt=1.0 / 30.0)
    print('[MARK] before reset')
    world.reset()
    print('[MARK] after reset')
    for index in range(10):
        world.step(render=True)
        print(f'[MARK] after step {index + 1}')
    out = Path('outputs/physics_smoke.usd')
    out.parent.mkdir(parents=True, exist_ok=True)
    stage.Export(str(out))
    print(f'[OK] PhysX smoke: {out}')
finally:
    app.close()
