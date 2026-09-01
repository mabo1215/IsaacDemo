"""Replicator-only smoke test with one USD camera and one cube."""

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

writer = None
try:
    import omni.replicator.core as rep
    import omni.usd
    from pxr import Gf, UsdGeom

    stage = omni.usd.get_context().get_stage()
    root = UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(root.GetPrim())
    cube = UsdGeom.Cube.Define(stage, '/World/SmokeCube')
    cube.GetSizeAttr().Set(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.5))
    camera = UsdGeom.Camera.Define(stage, '/World/RenderCamera')
    camera.CreateFocalLengthAttr().Set(24.0)
    camera.AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(3.0, -4.0, 2.5),
            Gf.Vec3d(0.0, 0.0, 0.5),
            Gf.Vec3d(0.0, 0.0, 1.0),
        )
    )
    print('[MARK] before render product')
    render_product = rep.create.render_product(str(camera.GetPath()), (320, 180))
    print('[MARK] after render product')
    output = Path('outputs/replicator_smoke')
    output.mkdir(parents=True, exist_ok=True)
    writer = rep.WriterRegistry.get('BasicWriter')
    print('[MARK] before writer initialize')
    writer.initialize(output_dir=str(output), rgb=True)
    print('[MARK] before writer attach')
    writer.attach([render_product])
    print('[MARK] after writer attach')
    for index in range(3):
        app.update()
        rep.orchestrator.step(rt_subframes=1)
        print(f'[MARK] after frame {index + 1}')
    print('[OK] Replicator smoke completed')
finally:
    if writer is not None:
        writer.detach()
    app.close()
