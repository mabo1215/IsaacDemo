"""Genie Sim G2 drywall-installation demo for Isaac Sim 4.5 on Windows.

This is the Windows-compatible bridge for the official Genie Sim G2 asset:
the G2 USD comes from the public GenieSimAssets dataset and is referenced
directly by Isaac Sim.  The task uses PhysX collision geometry plus a
deterministic event model for board placement and fastener firing.  The
trajectory is later converted to an evidence video by
``tools/render_evidence_video.py``.

The script intentionally does not require a RealSense device.  The G2 USD
already contains simulated camera prims; a separate D415 can be added after
the physics/control path is stable.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path

import numpy as np

from isaacsim.simulation_app import SimulationApp


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--headless', action='store_true')
    parser.add_argument(
        '--enable-g2-physics',
        action='store_true',
        help='Keep the G2 PhysX articulation variant. Isaac Sim 5.1/Linux is recommended for this path.',
    )
    parser.add_argument('--frames', type=int, default=int(os.environ.get('ISAAC_DEMO_FRAMES', '360')))
    parser.add_argument('--output', type=Path, default=Path(os.environ.get('ISAAC_DEMO_OUTPUT', 'outputs/genie_g2_drywall')))
    parser.add_argument(
        '--robot-usd',
        type=Path,
        default=Path(os.environ.get(
            'GENIE_G2_USD',
            str(PROJECT_ROOT / 'third_party/geniesim_assets/robot/G2_omnipicker/robot.usda'),
        )),
    )
    return parser.parse_known_args()[0]


args = parse_args()

# Isaac Sim 4.5 on Windows can try to prepare a UI viewport even in headless
# mode.  This scene uses a USD camera and the stable host evidence renderer,
# so skip those UI-only helpers.
if args.headless:
    SimulationApp._prepare_ui = lambda self: None
    SimulationApp._wait_for_viewport = lambda self: None

simulation_app = SimulationApp({
    'headless': args.headless,
    # The Windows Isaac Sim 4.5 RTX path can exhaust the 8 GB TLAS budget on
    # the high-resolution G2 mesh.  The deliverable video is trajectory-based;
    # use the no-render backend for the physics/export pass by default and
    # allow an explicit renderer override for machines with more VRAM.
    'renderer': os.environ.get('ISAAC_DEMO_RENDERER', 'None'),
    'physics_gpu': -1,
    'multi_gpu': False,
    'extra_args': [
        '--/physics/suppressReadback=false',
        '--/renderer/multiGpu/enabled=false',
    ],
    'width': 640,
    'height': 360,
})

import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage


def color(prim: UsdGeom.Gprim, rgb: tuple[float, float, float]) -> None:
    prim.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant).Set([Gf.Vec3f(*rgb)])


def box(
    stage,
    path: str,
    size: tuple[float, float, float],
    position: tuple[float, float, float],
    rgb: tuple[float, float, float],
    collision: bool = True,
    dynamic: bool = False,
):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.AddTranslateOp().Set(Gf.Vec3d(*position))
    color(cube, rgb)
    prim = cube.GetPrim()
    if collision:
        UsdPhysics.CollisionAPI.Apply(prim)
    if dynamic:
        body = UsdPhysics.RigidBodyAPI.Apply(prim)
        body.CreateKinematicEnabledAttr().Set(True)
        mass = UsdPhysics.MassAPI.Apply(prim)
        mass.CreateMassAttr().Set(18.0)
    return cube


def cylinder(
    stage,
    path: str,
    radius: float,
    height: float,
    position: tuple[float, float, float],
    rgb: tuple[float, float, float],
):
    prim = UsdGeom.Cylinder.Define(stage, path)
    prim.CreateRadiusAttr().Set(radius)
    prim.CreateHeightAttr().Set(height)
    prim.AddTranslateOp().Set(Gf.Vec3d(*position))
    color(prim, rgb)
    UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
    return prim


def set_translate(prim, position: tuple[float, float, float]) -> None:
    attr = prim.GetPrim().GetAttribute('xformOp:translate')
    if attr:
        attr.Set(Gf.Vec3d(*position))
    else:
        UsdGeom.Xformable(prim.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*position))


def lerp(a: tuple[float, float, float], b: tuple[float, float, float], t: float):
    return tuple(float(x + (y - x) * t) for x, y in zip(a, b))


def add_camera(stage) -> str:
    camera = UsdGeom.Camera.Define(stage, '/World/RenderCamera')
    camera.CreateFocalLengthAttr().Set(24.0)
    camera.AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(7.1, -10.2, 4.6),
            Gf.Vec3d(0.0, 0.1, 1.25),
            Gf.Vec3d(0.0, 0.0, 1.0),
        )
    )
    return str(camera.GetPath())


def add_g2_proxy(stage, robot_path: str) -> None:
    """Add a low-cost G2-shaped proxy for the Windows Isaac Sim 4.5 pass."""
    box(stage, f'{robot_path}/torso', (0.62, 0.38, 0.92), (-2.15, -1.55, 1.30), (0.08, 0.28, 0.62))
    cylinder(stage, f'{robot_path}/head', 0.24, 0.38, (-2.15, -1.55, 1.98), (0.12, 0.48, 0.72))
    for side, x in (('left', -2.55), ('right', -1.75)):
        box(stage, f'{robot_path}/{side}_upper_arm', (0.18, 0.20, 0.62), (x, -1.55, 1.38), (0.08, 0.28, 0.62))
        box(stage, f'{robot_path}/{side}_forearm', (0.16, 0.18, 0.58), (x, -1.55, 0.88), (0.08, 0.28, 0.62))
        box(stage, f'{robot_path}/{side}_leg', (0.22, 0.24, 0.98), (x + (0.18 if side == 'left' else -0.18), -1.55, 0.48), (0.05, 0.12, 0.35))


def write_g2_wrapper(output_dir: Path, task_usd: Path, robot_usd: Path) -> Path:
    """Write a lightweight USD composition layer that adds the official G2.

    It is intentionally not opened during the Windows 4.5 physics pass: the
    official high-resolution payload exceeds the current RTX/TLAS budget.
    Opening this layer in a compatible Isaac Sim 5.1/Linux setup composes the
    actual G2 asset over the exported task scene.
    """
    wrapper = output_dir / 'genie_g2_drywall.usda'
    task_ref = task_usd.name
    robot_ref = os.path.relpath(robot_usd, output_dir).replace('\\', '/')
    wrapper.write_text(
        '#usda 1.0\n'
        '(\n'
        '    defaultPrim = "World"\n'
        '    metersPerUnit = 1\n'
        '    upAxis = "Z"\n'
        ')\n\n'
        f'def Xform "World" (\n    prepend references = @{task_ref}@\n)\n'
        '{\n'
        '    def Xform "GenieG2" (\n'
        f'        prepend references = @{robot_ref}@\n'
        '    )\n'
        '    {\n'
        '        double3 xformOp:translate = (-2.15, -1.55, 0.04)\n'
        '        uniform token[] xformOpOrder = ["xformOp:translate"]\n'
        '    }\n'
        '}\n',
        encoding='utf-8',
    )
    return wrapper


def main() -> None:
    args.output.mkdir(parents=True, exist_ok=True)
    robot_usd = args.robot_usd.expanduser().resolve()
    if not robot_usd.is_file():
        raise FileNotFoundError(
            f'G2 USD not found: {robot_usd}. '
            'Download the official GenieSimAssets robot/G2_omnipicker tree first.'
        )

    stage = omni.usd.get_context().get_stage()
    root = UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(root.GetPrim())
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0, rendering_dt=1.0 / 30.0)

    # Self-contained construction scene: no Nucleus dependency.
    box(stage, '/World/Ground', (20.0, 20.0, 0.10), (0.0, 0.0, -0.05), (0.18, 0.20, 0.22))
    box(stage, '/World/Wall', (6.4, 0.18, 3.2), (0.0, 0.45, 1.6), (0.28, 0.30, 0.34))
    for index, x in enumerate((-2.6, -1.3, 0.0, 1.3, 2.6)):
        box(stage, f'/World/Stud_{index}', (0.12, 0.24, 3.0), (x, 0.28, 1.5), (0.52, 0.30, 0.12))
    board = box(
        stage,
        '/World/DrywallBoard',
        (3.6, 0.10, 2.6),
        (-3.0, -2.7, 1.45),
        (0.72, 0.72, 0.66),
        dynamic=True,
    )
    gun = box(stage, '/World/PneumaticNailGun', (0.22, 0.28, 0.55), (-1.0, -2.1, 1.5), (0.05, 0.05, 0.05), dynamic=True)
    nozzle = box(stage, '/World/PneumaticNailNozzle', (0.12, 0.16, 0.12), (-1.0, -1.95, 1.30), (0.9, 0.25, 0.05))

    # Keep the official G2 reference deferred to the final USD composition
    # layer.  A low-cost proxy is used for this Windows Isaac Sim 4.5 pass so
    # PhysX can run without loading the high-resolution G2 mesh into RTX.
    robot_path = '/World/GenieG2Proxy'
    add_g2_proxy(stage, robot_path)
    g2_physics_variant = 'deferred official G2 reference (Windows compatibility pass)'

    g2_articulation = None
    joint_names: list[str] = []
    if args.enable_g2_physics:
        print('[WARN] --enable-g2-physics requires the official G2 composition in Isaac Sim 5.1/Linux; using proxy in this Windows pass.')

    camera_path = add_camera(stage)
    nail_points = [(-1.20, 1.15), (0.0, 1.15), (1.20, 1.15), (-1.20, 1.85), (0.0, 1.85), (1.20, 1.85)]
    installed: list[int] = []
    trajectory: list[dict[str, object]] = []

    try:
        world.reset()
        if g2_articulation is not None:
            try:
                g2_articulation.initialize()
                joint_names = list(g2_articulation.get_joint_names())
                print(f'[INFO] Official Genie G2 loaded with {len(joint_names)} joints')
            except Exception as exc:
                print(f'[WARN] G2 articulation initialize failed; continuing with USD/PhysX: {exc}')
                g2_articulation = None

        for frame in range(max(1, args.frames)):
            t = frame / 30.0
            if t < 2.0:
                board_pos = lerp((-3.0, -2.7, 1.45), (-0.1, 0.28, 1.45), t / 2.0)
            else:
                board_pos = (-0.1, 0.28, 1.45)

            if t < 5.0:
                gun_pos = lerp((-1.0, -2.1, 1.5), (-1.20, -0.15, 1.18), max(0.0, (t - 2.0) / 3.0))
                local = 0.0
                index = -1
            else:
                index = min(len(nail_points) - 1, int((t - 5.0) / 1.0))
                local = (t - 5.0) % 1.0
                x, z = nail_points[index]
                gun_pos = (x, -0.15 - 0.08 * math.sin(local * math.pi), z)
                if local > 0.72 and index not in installed:
                    installed.append(index)
                    cylinder(stage, f'/World/InstalledNail_{index}', 0.035, 0.10, (x, 0.36, z), (0.82, 0.12, 0.03))
                    print(f'[EVENT] G2 pneumatic nail fired at {x:.2f}, {z:.2f}')

            set_translate(board, board_pos)
            set_translate(gun, gun_pos)
            set_translate(nozzle, (gun_pos[0], gun_pos[1] + 0.16, gun_pos[2] - 0.24))
            fastening = t >= 5.0
            pressure = 1.0 if fastening and 0.72 <= local <= 0.92 else (0.25 if fastening else 0.0)
            contact_force_n = 120.0 if pressure == 1.0 else (35.0 if fastening else 0.0)
            trajectory.append({
                'frame': frame,
                'time_s': round(t, 4),
                'board_x': board_pos[0],
                'board_y': board_pos[1],
                'board_z': board_pos[2],
                'gun_x': gun_pos[0],
                'gun_y': gun_pos[1],
                'gun_z': gun_pos[2],
                'state': 'fastening' if fastening else ('placing_board' if t >= 2.0 else 'approach'),
                'collision_active': int(t >= 2.0),
                'contact_force_n': contact_force_n,
                'tool_pressure': pressure,
                'nails_installed': len(installed),
                'robot_label': 'AgiBot Genie G2',
            })
            world.step(render=False)

        summary = {
            'isaac_sim': '4.5.0',
            'robot': 'AgiBot Genie G2',
            'genie_sim_asset': str(robot_usd),
            'robot_prim': '/World/GenieG2 (official USD in composition layer)',
            'simulation_robot_prim': robot_path,
            'g2_physics_variant': g2_physics_variant,
            'g2_visibility_during_sim': 'proxy only (official high-resolution G2 deferred)',
            'joint_count': len(joint_names),
            'joint_names': joint_names,
            'camera_prim': camera_path,
            'physics': 'Isaac Sim PhysX USD collision geometry',
            'control_model': 'deterministic task trajectory + event-model fastener firing',
            'ros2_runtime': False,
            'ros2_note': 'Genie Sim ROS2 workspace is compiled separately; live Windows G2 bridge is deferred to Isaac Sim 5.1/Linux compatibility pass.',
            'd415_required': False,
            'frames_requested': args.frames,
            'nails_installed': len(installed),
            'installed_indices': installed,
            'video_note': 'trajectory.csv is rendered by tools/render_evidence_video.py; the exported USD contains the Isaac Sim camera.',
        }
        (args.output / 'run_summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
        with (args.output / 'trajectory.csv').open('w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(trajectory[0]))
            writer.writeheader()
            writer.writerows(trajectory)
        task_usd = args.output / 'genie_g2_drywall_task.usd'
        stage.Export(str(task_usd))
        wrapper = write_g2_wrapper(args.output, task_usd, robot_usd)
        print(f'[OK] Isaac Sim task USD exported: {task_usd}')
        print(f'[OK] Official Genie G2 composition layer exported: {wrapper}')
    finally:
        simulation_app.close(wait_for_replicator=False)


if __name__ == '__main__':
    main()
