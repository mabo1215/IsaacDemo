"""Create a render product but do not start Replicator orchestration."""

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
    camera.AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(3.0, -4.0, 2.5),
            Gf.Vec3d(0.0, 0.0, 0.5),
            Gf.Vec3d(0.0, 0.0, 1.0),
        )
    )
    print('[MARK] before render product')
    product = rep.create.render_product(str(camera.GetPath()), (320, 180))
    print(f'[MARK] after render product: {product}')
    for index in range(5):
        app.update()
        print(f'[MARK] after update {index + 1}')
    Path('outputs/renderproduct_smoke.txt').write_text('render product created\n', encoding='utf-8')
    print('[OK] render product smoke completed')
finally:
    app.close()
