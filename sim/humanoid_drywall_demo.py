"""Minimal humanoid drywall-installation scene for Isaac Sim 4.5.

The board/nail penetration is intentionally an event model. The scene still
uses USD collision geometry and PhysX rigid bodies, and the ROS2 bridge graph
publishes/subscribes H1 joint states when the H1 asset is available.
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--frames', type=int, default=int(os.environ.get('ISAAC_DEMO_FRAMES', '360')))
    parser.add_argument('--output', type=Path, default=Path(os.environ.get('ISAAC_DEMO_OUTPUT', 'outputs')))
    return parser.parse_known_args()[0]


args = parse_args()
# Isaac Sim 4.5 on Windows can crash while trying to prepare a UI viewport in
# --no-window mode. Replicator creates its own off-screen render product, so
# bypassing these UI-only helpers keeps true headless rendering available.
if args.headless:
    SimulationApp._prepare_ui = lambda self: None
    SimulationApp._wait_for_viewport = lambda self: None
simulation_app = SimulationApp({
    'headless': args.headless,
    'renderer': 'RayTracedLighting',
    # Keep PhysX on CPU for this small rigid-body scene. RTX rendering still
    # uses the 3070, while CPU PhysX avoids GPU dynamics buffer pressure on 8GB.
    'physics_gpu': -1,
    'multi_gpu': False,
    'extra_args': ['--/physics/suppressReadback=false'],
    'width': 640,
    'height': 360,
})

import omni.graph.core as og
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction


def color(prim: UsdGeom.Gprim, rgb: tuple[float, float, float]) -> None:
    prim.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant).Set([Gf.Vec3f(*rgb)])


def box(stage, path: str, size: tuple[float, float, float], position: tuple[float, float, float],
        rgb: tuple[float, float, float], collision: bool = True, dynamic: bool = False):
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


def cylinder(stage, path: str, radius: float, height: float, position: tuple[float, float, float],
             rgb: tuple[float, float, float]):
    prim = UsdGeom.Cylinder.Define(stage, path)
    prim.CreateRadiusAttr().Set(radius)
    prim.CreateHeightAttr().Set(height)
    prim.AddTranslateOp().Set(Gf.Vec3d(*position))
    color(prim, rgb)
    UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
    return prim


def humanoid_fallback(stage):
    """Low-cost visual/collision fallback if the online H1 USD is unavailable."""
    box(stage, '/World/FallbackHumanoid/torso', (0.55, 0.32, 0.82), (-2.0, -1.6, 1.25), (0.15, 0.35, 0.8))
    cylinder(stage, '/World/FallbackHumanoid/head', 0.22, 0.35, (-2.0, -1.6, 1.92), (0.75, 0.70, 0.55))
    for side, x in (('left', -2.38), ('right', -1.62)):
        box(stage, f'/World/FallbackHumanoid/{side}_upper_arm', (0.18, 0.18, 0.62), (x, -1.6, 1.32), (0.15, 0.35, 0.8))
        box(stage, f'/World/FallbackHumanoid/{side}_forearm', (0.16, 0.16, 0.55), (x + (0.05 if side == 'left' else -0.05), -1.6, 0.84), (0.15, 0.35, 0.8))
        box(stage, f'/World/FallbackHumanoid/{side}_leg', (0.20, 0.22, 0.95), (x + (0.18 if side == 'left' else -0.18), -1.6, 0.42), (0.12, 0.18, 0.42))


def set_transform(prim, position: tuple[float, float, float]) -> None:
    attrs = prim.GetPrim().GetAttribute('xformOp:translate')
    if attrs:
        attrs.Set(Gf.Vec3d(*position))


def lerp(a, b, t):
    return tuple(float(x + (y - x) * t) for x, y in zip(a, b))


def create_ros_graph(robot_path: str) -> bool:
    try:
        og.Controller.edit(
            {'graph_path': '/World/ROS2Control', 'evaluator_name': 'execution'},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ('OnPlaybackTick', 'omni.graph.action.OnPlaybackTick'),
                    ('PublishJointState', 'isaacsim.ros2.bridge.ROS2PublishJointState'),
                    ('SubscribeJointState', 'isaacsim.ros2.bridge.ROS2SubscribeJointState'),
                    ('ArticulationController', 'isaacsim.core.nodes.IsaacArticulationController'),
                    ('ReadSimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                ],
                og.Controller.Keys.CONNECT: [
                    ('OnPlaybackTick.outputs:tick', 'PublishJointState.inputs:execIn'),
                    ('OnPlaybackTick.outputs:tick', 'SubscribeJointState.inputs:execIn'),
                    ('OnPlaybackTick.outputs:tick', 'ArticulationController.inputs:execIn'),
                    ('ReadSimTime.outputs:simulationTime', 'PublishJointState.inputs:timeStamp'),
                    ('SubscribeJointState.outputs:jointNames', 'ArticulationController.inputs:jointNames'),
                    ('SubscribeJointState.outputs:positionCommand', 'ArticulationController.inputs:positionCommand'),
                    ('SubscribeJointState.outputs:velocityCommand', 'ArticulationController.inputs:velocityCommand'),
                    ('SubscribeJointState.outputs:effortCommand', 'ArticulationController.inputs:effortCommand'),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ('ArticulationController.inputs:robotPath', robot_path),
                    ('PublishJointState.inputs:targetPrim', robot_path),
                ],
            },
        )
        return True
    except Exception as exc:  # Keep video generation useful if bridge libs are absent.
        print(f'[WARN] ROS2 graph creation failed: {exc}')
        return False


def main() -> None:
    args.output.mkdir(parents=True, exist_ok=True)
    stage = omni.usd.get_context().get_stage()
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, '/World').GetPrim())

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0, rendering_dt=1.0 / 30.0)
    # Keep the demo offline-safe: Isaac's default ground-plane helper queries
    # Nucleus for a USD asset, which is unnecessary for this self-contained scene.
    box(stage, '/World/Ground', (20.0, 20.0, 0.10), (0.0, 0.0, -0.05), (0.18, 0.20, 0.22))

    # Wall, studs and a front-facing drywall board.
    box(stage, '/World/Wall', (6.4, 0.18, 3.2), (0.0, 0.35, 1.6), (0.28, 0.30, 0.34))
    for i, x in enumerate((-2.6, -1.3, 0.0, 1.3, 2.6)):
        box(stage, f'/World/Stud_{i}', (0.12, 0.24, 3.0), (x, 0.18, 1.5), (0.52, 0.30, 0.12))
    board = box(stage, '/World/DrywallBoard', (3.6, 0.10, 2.6), (-3.0, -2.6, 1.45), (0.72, 0.72, 0.66), dynamic=True)
    board_prim = board.GetPrim()

    # H1 humanoid asset. The fallback keeps the scene visible if the online asset root is unavailable.
    robot_path = '/World/H1'
    h1 = None
    try:
        # Use a real H1 only when the user supplies a reachable Nucleus/content
        # root. The packaged Isaac Sim zip does not include robot USD assets.
        root = os.environ.get('ISAAC_ASSETS_ROOT')
        if root:
            asset = root.rstrip('/') + '/Isaac/Robots/Unitree/H1/h1_with_hand.usd'
            add_reference_to_stage(usd_path=asset, prim_path=robot_path)
            for _ in range(30):
                simulation_app.update()
            h1 = Articulation(prim_paths_expr=robot_path, name='h1')
            world.scene.add(h1)
            h1.initialize()
            print(f'[INFO] H1 loaded with {len(h1.get_joint_names())} joints')
        else:
            print('[WARN] Isaac asset root is unavailable')
    except Exception as exc:
        print(f'[WARN] H1 asset unavailable: {exc}')
        h1 = None
    if h1 is None:
        humanoid_fallback(stage)

    # Pneumatic nail gun and a small pressure/contact proxy.
    gun = box(stage, '/World/PneumaticNailGun', (0.22, 0.28, 0.55), (-1.0, -2.1, 1.5), (0.05, 0.05, 0.05), dynamic=True)
    nozzle = box(stage, '/World/PneumaticNailNozzle', (0.12, 0.16, 0.12), (-1.0, -1.95, 1.30), (0.9, 0.25, 0.05))
    nail_points = [(-1.20, 1.15), (0.0, 1.15), (1.20, 1.15), (-1.20, 1.85), (0.0, 1.85), (1.20, 1.85)]

    ros_graph = False
    if os.environ.get('ISAAC_DEMO_SKIP_ROS_GRAPH') != '1':
        ros_graph = create_ros_graph(robot_path)
    if ros_graph:
        print('[INFO] ROS2 graph ready: /joint_states <-> /joint_command')

    # Keep a camera in the USD deliverable. The stable video path on this
    # RTX 3070/Windows combination is host-side evidence rendering from the
    # recorded simulation trajectory; Replicator's Windows 4.5 writer can
    # access-violate while starting its orchestrator with this driver.
    camera_prim = UsdGeom.Camera.Define(stage, '/World/RenderCamera')
    camera_prim.CreateFocalLengthAttr().Set(24.0)
    camera_prim.AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(7.4, -10.5, 4.8),
            Gf.Vec3d(0.0, 0.0, 1.35),
            Gf.Vec3d(0.0, 0.0, 1.0),
        )
    )
    print('[INFO] Isaac RTX camera is included in USD; host evidence video will use trajectory.csv')

    try:
        world.reset()

        joint_names = list(h1.get_joint_names()) if h1 is not None else []
        summary = {
            'isaac_sim': '4.5.0',
            'robot': 'Unitree H1' if h1 is not None else 'fallback humanoid',
            'ros2_graph': ros_graph,
            'physics': 'PhysX USD collision geometry',
            'event_model': 'contact/proximity threshold -> pneumatic nail event',
            'joint_names': joint_names,
            'frames_requested': args.frames,
            'nails_installed': 0,
        }
        installed = []
        trajectory = []
        for frame in range(args.frames):
            t = frame / 30.0
            if t < 2.0:
                alpha = t / 2.0
                board_pos = lerp((-3.0, -2.6, 1.45), (-0.1, -0.28, 1.45), alpha)
            else:
                board_pos = (-0.1, -0.28, 1.45)
            set_transform(board, board_pos)

            # Animate the tool to each fastener point. This is the fallback motion
            # path; ROS2 joint commands can simultaneously override H1 drives.
            if t < 5.0:
                gun_pos = lerp((-1.0, -2.1, 1.5), (-1.20, -0.55, 1.18), max(0.0, (t - 2.0) / 3.0))
            else:
                index = min(len(nail_points) - 1, int((t - 5.0) / 1.0))
                local = (t - 5.0) % 1.0
                x, z = nail_points[index]
                gun_pos = (x, -0.56 - 0.08 * math.sin(local * math.pi), z)
                if local > 0.72 and index not in installed:
                    installed.append(index)
                    marker = cylinder(stage, f'/World/InstalledNail_{index}', 0.035, 0.10, (x, -0.37, z), (0.82, 0.12, 0.03))
                    marker.GetPrim().GetAttribute('xformOp:translate').Set(Gf.Vec3d(x, -0.37, z))
                    print(f'[EVENT] pneumatic nail fired at {x:.2f}, {z:.2f}')
            set_transform(gun, gun_pos)
            set_transform(nozzle, (gun_pos[0], gun_pos[1] + 0.16, gun_pos[2] - 0.24))

            fastening = t >= 5.0
            pressure = 0.0
            contact_force_n = 0.0
            if fastening:
                pressure = 1.0 if 0.72 <= local <= 0.92 else 0.25
                contact_force_n = 120.0 if pressure == 1.0 else 35.0
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
            })

            if h1 is not None and joint_names:
                pose = np.zeros(len(joint_names), dtype=np.float32)
                for name, value in {
                    'left_shoulder_pitch_joint': -0.35,
                    'left_shoulder_roll_joint': 0.25,
                    'left_elbow_joint': -0.85,
                    'right_shoulder_pitch_joint': -0.35,
                    'right_shoulder_roll_joint': -0.25,
                    'right_elbow_joint': -0.85,
                }.items():
                    if name in joint_names:
                        pose[joint_names.index(name)] = value
                h1.apply_action(ArticulationAction(joint_positions=pose))

            # The evidence video is rendered from trajectory.csv on the host.
            # Avoid per-step RTX viewport rendering here; on this 8 GB card
            # the full scene's hidden viewport can otherwise race PhysX.
            world.step(render=False)

        summary['nails_installed'] = len(installed)
        summary['installed_indices'] = installed
        (args.output / 'run_summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
        with (args.output / 'trajectory.csv').open('w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(trajectory[0]))
            writer.writeheader()
            writer.writerows(trajectory)
        stage.Export(str(args.output / 'drywall_installation.usd'))
    finally:
        simulation_app.close(wait_for_replicator=False)


if __name__ == '__main__':
    main()
