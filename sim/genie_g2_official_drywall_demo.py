"""Isaac Sim RTX demo: official AgiBot Genie G2 installs drywall.

This Linux/Isaac Sim 5.1 entry point is intentionally separate from the
Windows compatibility demo.  It loads the official GenieSimAssets G2 USD
directly, keeps the drywall/fastener event model deterministic, and records
the scene with Isaac Sim Replicator so the resulting MP4 is a native RTX
render rather than a host-side diagram.
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
    parser.add_argument('--frames', type=int, default=int(os.environ.get('ISAAC_DEMO_FRAMES', '180')))
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument(
        '--output',
        type=Path,
        default=Path(os.environ.get('ISAAC_DEMO_OUTPUT', 'outputs/genie_g2_official_drywall')),
    )
    parser.add_argument(
        '--robot-usd',
        type=Path,
        default=Path(os.environ.get(
            'GENIE_G2_USD',
            'third_party/geniesim_assets/robot/G2_omnipicker/robot.usda',
        )),
    )
    return parser.parse_args()


args = parse_args()

if args.headless:
    # Avoid UI-only viewport setup on a headless remote host.
    SimulationApp._prepare_ui = lambda self: None
    SimulationApp._wait_for_viewport = lambda self: None

simulation_app = SimulationApp({
    'headless': args.headless,
    'renderer': os.environ.get('ISAAC_DEMO_RENDERER', 'RayTracedLighting'),
    'physics_gpu': -1,
    'multi_gpu': False,
    'extra_args': [
        '--/physics/suppressReadback=false',
        '--/renderer/multiGpu/enabled=false',
    ],
    'width': 640,
    'height': 360,
})

import carb
import omni.replicator.core as rep
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage


def material(stage, name: str, rgb: tuple[float, float, float], roughness: float = 0.7):
    material_path = f'/World/Looks/{name}'
    mat = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f'{material_path}/Shader')
    shader.CreateIdAttr().Set('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return mat


def bind_material(prim, mat) -> None:
    UsdShade.MaterialBindingAPI(prim.GetPrim()).Bind(mat)


def box(
    stage,
    path: str,
    size: tuple[float, float, float],
    position: tuple[float, float, float],
    mat,
    collision: bool = True,
    kinematic: bool = False,
):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*position))
    # USD applies xform ops in the authored order.  Translation must precede
    # scale; the old order scaled the requested position as well, placing the
    # wall/board above the nail coordinates and the ground into the robot.
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    bind_material(cube, mat)
    prim = cube.GetPrim()
    if collision:
        UsdPhysics.CollisionAPI.Apply(prim)
    if kinematic:
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
    mat,
    collision: bool = True,
):
    prim = UsdGeom.Cylinder.Define(stage, path)
    prim.CreateRadiusAttr().Set(radius)
    prim.CreateHeightAttr().Set(height)
    prim.AddTranslateOp().Set(Gf.Vec3d(*position))
    # Cylinder axis is Z by default; rotate it so the fastener points into the wall.
    prim.AddRotateXYZOp().Set(Gf.Vec3f(90.0, 0.0, 0.0))
    bind_material(prim, mat)
    if collision:
        UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
    return prim


def set_translate(prim, position: tuple[float, float, float]) -> None:
    xform = UsdGeom.Xformable(prim.GetPrim())
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(*position))
            return
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))


def set_orient(prim, orientation: Gf.Quatd) -> None:
    xform = UsdGeom.Xformable(prim.GetPrim())
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            op.Set(orientation)
            return
    xform.AddOrientOp().Set(orientation)


def yaw_quaternion(yaw: float) -> Gf.Quatd:
    return Gf.Quatd(
        math.cos(yaw / 2.0),
        Gf.Vec3d(0.0, 0.0, math.sin(yaw / 2.0)),
    )


def set_scripted_root_pose(
    robot_prim,
    robot_pos: tuple[float, float, float],
    robot_yaw: float,
    articulation: Articulation | None = None,
    articulation_reference_position: np.ndarray | None = None,
    articulation_reference_orientation: np.ndarray | None = None,
    robot_reference_position: tuple[float, float, float] | None = None,
) -> None:
    """Keep the imported G2 USD root upright after joint writes.

    The official G2 articulation's PhysX root body is body_link5 inside the
    imported chain, not the outer ``/World/GenieG2`` Xform. Moving the
    articulation view root directly to the ground would place the torso at
    z=0 and collapse the robot. When an initialized view is supplied, its
    physical root is translated by the same ground delta while retaining its
    original height and orientation from the official asset.
    """
    orientation = yaw_quaternion(robot_yaw)
    set_translate(robot_prim, robot_pos)
    set_orient(robot_prim, orientation)
    if (
        articulation is not None
        and articulation_reference_position is not None
        and articulation_reference_orientation is not None
        and robot_reference_position is not None
    ):
        delta = np.asarray(robot_pos, dtype=np.float32) - np.asarray(
            robot_reference_position,
            dtype=np.float32,
        )
        articulation.set_world_poses(
            positions=articulation_reference_position + delta.reshape(1, 3),
            orientations=articulation_reference_orientation,
        )


def update_held_tool(
    head,
    handle,
    nozzle,
    hand_pos: tuple[float, float, float],
    head_pos: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Keep one compact nail gun body attached between hand and nozzle.

    The base walks to each ground work position, so the gun can remain a
    short held tool.  There is deliberately no long colored proxy chain from
    the hand to the wall.
    """
    set_translate(handle, hand_pos)
    start = np.asarray(hand_pos, dtype=np.float64)
    end = np.asarray(head_pos, dtype=np.float64)
    set_translate(head, tuple((start + end) * 0.5))
    nozzle_pos = (head_pos[0], head_pos[1] + 0.16, head_pos[2] - 0.25)
    set_translate(nozzle, nozzle_pos)
    return nozzle_pos


def hand_reference(robot_pos: tuple[float, float, float]) -> tuple[float, float, float]:
    """Approximate the official G2 right gripper in the selected work pose."""
    return (robot_pos[0] + 0.60, robot_pos[1] - 0.30, robot_pos[2] + 1.55)


def smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def articulation_joint_names(articulation: Articulation) -> list[str]:
    for attribute in ('get_dof_names', 'dof_names', '_dof_names'):
        if not hasattr(articulation, attribute):
            continue
        value = getattr(articulation, attribute)
        try:
            names = value() if callable(value) else value
            if names is not None:
                return [str(name) for name in names]
        except (AttributeError, TypeError):
            continue
    return []


def apply_arm_pose(
    target: np.ndarray,
    joint_names: list[str],
    pose: str,
    active_index: int = -1,
) -> None:
    # The official G2 asset has seven revolute joints per arm.  The fastening
    # poses deliberately sweep the shoulder and elbow as the active point
    # moves left/centre/right and lower/upper, so the visible arms follow the
    # held tool instead of staying in one imported T-pose.
    basic_values = {
        'folded': (0.0, 0.60, -1.10, 0.65, 0.0, 0.0, 0.0),
        'ready': (0.0, 0.35, -0.90, 0.50, 0.0, 0.0, 0.0),
    }
    if pose in basic_values:
        values = basic_values[pose]
    else:
        point_index = max(0, active_index)
        # Fasteners are ordered column-major: lower and upper at the same
        # ground position are completed before the mobile base moves on.
        upper_row = point_index % 2 == 1
        column_offset = point_index // 2
        pressing = pose.startswith('press_')
        if upper_row:
            # Raise the elbow for the upper row; the mobile base already
            # provides the large horizontal alignment motion.
            values = (
                0.0,
                0.86 + 0.08 * column_offset + (0.04 if pressing else 0.0),
                -1.28 - 0.08 * column_offset - (0.06 if pressing else 0.0),
                0.70 + 0.06 * column_offset + (0.04 if pressing else 0.0),
                0.0,
                0.0,
                0.0,
            )
        else:
            values = (
                0.0,
                0.48 + 0.10 * column_offset + (0.04 if pressing else 0.0),
                -0.90 - 0.10 * column_offset - (0.06 if pressing else 0.0),
                0.52 + 0.08 * column_offset + (0.04 if pressing else 0.0),
                0.0,
                0.0,
                0.0,
            )
    for side in ('l', 'r'):
        for joint_index, joint_value in enumerate(values, start=1):
            # idx21..27 and idx61..67 are the stable names in the official G2 USD.
            prefix = 2 if side == 'l' else 6
            exact_name = f'idx{prefix}{joint_index}_arm_{side}_joint{joint_index}'
            for index, candidate in enumerate(joint_names):
                if candidate == exact_name:
                    target[0, index] = joint_value
                    break


def set_controlled_joint_pose(
    articulation: Articulation,
    joint_names: list[str],
    base_positions: np.ndarray,
    arm_pose: str,
    active_index: int = -1,
) -> None:
    target = base_positions.copy()
    apply_arm_pose(target, joint_names, arm_pose, active_index)
    gripping = arm_pose == 'ready' or arm_pose.startswith('working') or arm_pose.startswith('press_')
    grip_value = 0.22 if gripping else 0.0
    # The official asset uses mimic joints for the inner fingers.  Only target
    # the outer drive; writing both members of the mimic pair can destabilize
    # PhysX during the first step.
    for name in (
        'idx41_gripper_l_outer_joint1',
        'idx81_gripper_r_outer_joint1',
    ):
        if name in joint_names:
            target[0, joint_names.index(name)] = grip_value
    articulation.set_joint_positions(target)
    articulation.set_joint_position_targets(target)
    articulation.set_joint_velocities(np.zeros_like(target))


def neutralize_joint_drives(stage, robot_path: str) -> int:
    """Disable USD drives; the task writes arm joint positions explicitly.

    The official G2 USD contains drives for wheels, body, grippers and arms.
    Their reaction torque can overturn the free-root articulation when the
    scripted base and upper-row arm poses change.  The per-frame direct joint
    position writes still animate the official articulation while removing
    that accumulated drive torque.
    """
    changed = 0
    prefix = robot_path.rstrip('/') + '/'
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(prefix):
            continue
        drive_values = {
            'drive:angular:physics:stiffness': 0.0,
            'drive:angular:physics:damping': 0.0,
            'drive:angular:physics:maxForce': 0.0,
            'drive:linear:physics:stiffness': 0.0,
            'drive:linear:physics:damping': 0.0,
            'drive:linear:physics:maxForce': 0.0,
        }
        for attribute_name in (
            'drive:angular:physics:stiffness',
            'drive:angular:physics:damping',
            'drive:angular:physics:maxForce',
            'drive:linear:physics:stiffness',
            'drive:linear:physics:damping',
            'drive:linear:physics:maxForce',
        ):
            attribute = prim.GetAttribute(attribute_name)
            if attribute.IsValid():
                attribute.Set(drive_values[attribute_name])
                changed += 1
    return changed


def lerp(a: tuple[float, float, float], b: tuple[float, float, float], t: float):
    return tuple(float(x + (y - x) * t) for x, y in zip(a, b))


def add_camera(stage) -> str:
    camera = UsdGeom.Camera.Define(stage, '/World/RenderCamera')
    camera.CreateFocalLengthAttr().Set(30.0)
    camera.AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(5.2, -13.5, 3.6),
            Gf.Vec3d(-0.1, 0.10, 1.35),
            Gf.Vec3d(0.0, 0.0, 1.0),
        ).GetInverse()
    )
    return str(camera.GetPath())


def add_lighting(stage) -> None:
    dome = UsdLux.DomeLight.Define(stage, '/World/DomeLight')
    dome.CreateIntensityAttr().Set(900.0)
    dome.CreateColorAttr().Set(Gf.Vec3f(0.72, 0.78, 0.90))
    key = UsdLux.DistantLight.Define(stage, '/World/KeyLight')
    key.CreateIntensityAttr().Set(2500.0)
    key.CreateAngleAttr().Set(0.5)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-35.0, -25.0, -25.0))


def main() -> None:
    args.output.mkdir(parents=True, exist_ok=True)
    robot_usd = args.robot_usd.expanduser().resolve()
    if not robot_usd.is_file():
        raise FileNotFoundError(f'Official G2 USD not found: {robot_usd}')

    stage = omni.usd.get_context().get_stage()
    world_root = UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(world_root.GetPrim())
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0, rendering_dt=1.0 / args.fps)

    looks = {}
    for key, rgb, roughness in (
        ('ground', (0.12, 0.14, 0.18), 0.7),
        ('wall', (0.28, 0.31, 0.37), 0.7),
        ('stud', (0.55, 0.30, 0.10), 0.7),
        ('drywall', (0.72, 0.72, 0.67), 0.9),
        ('tool', (0.035, 0.04, 0.05), 0.35),
        ('nozzle', (0.88, 0.22, 0.035), 0.35),
        ('nail', (0.82, 0.08, 0.025), 0.4),
    ):
        looks[key] = material(stage, key.title(), rgb, roughness)

    # Self-contained drywall work cell.  All static scene geometry has PhysX collision.
    box(stage, '/World/Ground', (20.0, 20.0, 0.10), (0.0, 0.0, -0.05), looks['ground'])
    box(stage, '/World/Wall', (6.4, 0.18, 3.2), (0.0, 0.65, 1.6), looks['wall'])
    for index, x in enumerate((-2.6, -1.3, 0.0, 1.3, 2.6)):
        box(stage, f'/World/Stud_{index}', (0.12, 0.24, 3.0), (x, 0.48, 1.5), looks['stud'])

    board = box(
        stage,
        '/World/DrywallBoard',
        (3.6, 0.10, 2.6),
        (-3.0, -2.7, 1.45),
        looks['drywall'],
        kinematic=True,
    )
    gun = box(
        stage,
        '/World/PneumaticNailGun',
        (0.22, 0.58, 0.38),
        (-1.0, -2.1, 1.5),
        looks['tool'],
        kinematic=True,
    )
    tool_handle = box(
        stage,
        '/World/PneumaticNailGunHandle',
        (0.16, 0.22, 0.22),
        (-1.0, -2.1, 1.5),
        looks['tool'],
        collision=False,
    )
    nozzle = box(
        stage,
        '/World/PneumaticNailNozzle',
        (0.12, 0.16, 0.12),
        (-1.0, -1.95, 1.30),
        looks['nozzle'],
        collision=False,
    )

    # Load the actual AgiBot Genie G2 asset, including its visual, PhysX and sensor payloads.
    robot_path = '/World/GenieG2'
    add_reference_to_stage(usd_path=str(robot_usd), prim_path=robot_path)
    robot_prim = stage.GetPrimAtPath(robot_path)
    if not robot_prim.IsValid():
        raise RuntimeError(f'G2 reference did not create {robot_path}')
    robot_start = (-2.8, -3.0, 0.04)
    # Move close enough that the extended arm and the held tool can reach the
    # board surface without a disconnected free-floating gun proxy.
    # The first work position is replaced with the target-specific ground
    # positions after the six fastening points are declared below.
    robot_work = (-1.80, -0.32, 0.04)
    robot_yaw = math.pi / 2.0
    set_scripted_root_pose(robot_prim, robot_start, robot_yaw)
    print(f'[INFO] Official G2 reference: {robot_usd}')

    # Let reference and payloads resolve before querying bounds or creating the render product.
    for _ in range(60):
        simulation_app.update()

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    aligned = bbox_cache.ComputeWorldBound(robot_prim).ComputeAlignedBox()
    print(f'[INFO] G2 bbox min={aligned.GetMin()} max={aligned.GetMax()}')

    # The task uses an explicit root trajectory and joint-position targets.
    # Keep the official articulation dynamic (PhysX does not support a
    # kinematic body inside this articulation), but disable root gravity so the
    # scripted approach cannot fall while the pose controller is being applied.
    base_link = stage.GetPrimAtPath(f'{robot_path}/base_link')
    if base_link.IsValid():
        gravity_attr = base_link.GetAttribute('physxRigidBody:disableGravity')
        if gravity_attr.IsValid():
            gravity_attr.Set(True)
    drive_attributes_configured = neutralize_joint_drives(stage, robot_path)

    g2_articulation = Articulation(prim_paths_expr=robot_path, name='genie_g2')
    joint_names: list[str] = []
    base_joint_positions: np.ndarray | None = None
    articulation_reference_position: np.ndarray | None = None
    articulation_reference_orientation: np.ndarray | None = None
    articulation_status = 'not_initialized'
    robot_gravity_disabled = False
    try:
        world.reset()
        g2_articulation.initialize()
        joint_names = articulation_joint_names(g2_articulation)
        base_joint_positions = np.asarray(
            g2_articulation.get_joint_positions(), dtype=np.float32
        ).copy()
        if base_joint_positions.ndim == 1:
            base_joint_positions = base_joint_positions.reshape(1, -1)
        articulation_reference_position = np.asarray(
            g2_articulation.get_world_poses()[0],
            dtype=np.float32,
        ).reshape(1, 3).copy()
        articulation_reference_orientation = np.asarray(
            g2_articulation.get_world_poses()[1],
            dtype=np.float32,
        ).reshape(1, 4).copy()
        # This Isaac Sim 5.1 Articulation view exposes the multi-articulation
        # API, so disable gravity for every body and clamp the root 6D velocity
        # through the view rather than the SingleArticulation convenience API.
        g2_articulation.set_body_disable_gravity(
            np.ones((1, g2_articulation.num_bodies), dtype=np.uint8)
        )
        g2_articulation.set_velocities(np.zeros((1, 6), dtype=np.float32))
        robot_gravity_disabled = True
        articulation_status = 'initialized'
        print(f'[INFO] G2 articulation initialized with {len(joint_names)} joints')
    except Exception as exc:
        articulation_status = f'initialize_failed: {exc}'
        print(f'[WARN] G2 articulation initialization failed; keeping official USD visual/collision payload: {exc}')

    add_lighting(stage)
    camera_path = add_camera(stage)
    render_product = rep.create.render_product(camera_path, (640, 360))
    frames_dir = (args.output / 'frames').resolve()
    carb.settings.get_settings().set(
        '/omni/replicator/backends/disk/root_dir',
        str(args.output.resolve()),
    )
    writer = rep.WriterRegistry.get('BasicWriter')
    writer.initialize(output_dir=str(frames_dir), rgb=True)
    writer.attach([render_product])

    # Keep the board on the wall for the whole demonstration.  The target
    # markers and installed nails use the board-facing surface, so perspective
    # cannot make them appear above/below the board as in the previous run.
    board_pos = (-0.1, 0.28, 1.45)
    set_translate(board, board_pos)
    nail_points = [
        (-1.20, 1.15),
        (-1.20, 1.85),
        (0.0, 1.15),
        (0.0, 1.85),
        (1.20, 1.15),
        (1.20, 1.85),
    ]
    target_y = 0.235
    for index, (x, z) in enumerate(nail_points):
        cylinder(
            stage,
            f'/World/FastenerTarget_{index}',
            0.055,
            0.015,
            (x, target_y, z),
            looks['nail'],
            collision=False,
        )

    # Each nail point has a corresponding ground work position.  The right
    # gripper reference is about +0.60 m in world X from the root at yaw 90°,
    # so the base walks to x = nail_x - 0.60 before the arm reaches forward.
    robot_work_positions = [(x - 0.60, -0.32, 0.04) for x, _ in nail_points]
    robot_work = robot_work_positions[0]
    installed: list[int] = []
    trajectory: list[dict[str, object]] = []
    total_duration = args.frames / float(args.fps)
    approach_duration = min(3.5, max(2.5, total_duration * 0.18))
    ready_duration = min(1.5, max(1.0, total_duration * 0.08))
    fastening_start = approach_duration + ready_duration
    fastening_duration = max(0.8, (total_duration - fastening_start) / len(nail_points))
    wheel_distance = 0.0

    try:
        for frame in range(max(1, args.frames)):
            t = frame / float(args.fps)
            if t < approach_duration:
                progress = smoothstep(t / approach_duration)
                robot_pos = lerp(robot_start, robot_work, progress)
                gun_pos = hand_reference(robot_pos)
                state = 'walk_to_drywall'
                active_index = -1
                active_local = 0.0
                arm_pose = 'folded'
                wheel_distance += math.dist(robot_start, robot_work) / max(1, int(approach_duration * args.fps))
            elif t < fastening_start:
                robot_pos = robot_work
                gun_pos = hand_reference(robot_pos)
                state = 'align_and_grasp_tool'
                active_index = -1
                active_local = 0.0
                arm_pose = 'ready'
            else:
                elapsed = t - fastening_start
                active_index = min(len(nail_points) - 1, int(elapsed / fastening_duration))
                active_local = min(1.0, (elapsed - active_index * fastening_duration) / fastening_duration)
                x, z = nail_points[active_index]
                current_work = robot_work_positions[active_index]
                previous_work = (
                    robot_work
                    if active_index == 0
                    else robot_work_positions[active_index - 1]
                )
                target = (x, target_y - 0.10, z + 0.25)
                if active_local < 0.28:
                    # First move the mobile base to the ground point for this
                    # fastener while the arm keeps the tool close to the body.
                    robot_pos = lerp(
                        previous_work,
                        current_work,
                        smoothstep(active_local / 0.28),
                    )
                    gun_pos = hand_reference(robot_pos)
                    state = 'walk_to_fastener'
                    arm_pose = 'ready'
                elif active_local < 0.38:
                    robot_pos = current_work
                    gun_pos = hand_reference(robot_pos)
                    state = 'align_tool'
                    arm_pose = 'ready'
                else:
                    robot_pos = current_work
                    entry = hand_reference(robot_pos)
                    reach_fraction = smoothstep(min(1.0, (active_local - 0.38) / 0.24))
                    gun_pos = lerp(entry, target, reach_fraction)
                    arm_pose = f'working_{active_index}'
                    if active_local < 0.62:
                        state = 'move_to_fastener'
                    elif active_local < 0.72:
                        state = 'press_fastener'
                        arm_pose = f'press_{active_index}'
                    else:
                        state = 'hold_fastener'
                if active_local >= 0.72 and active_index not in installed:
                    installed.append(active_index)
                    cylinder(
                        stage,
                        f'/World/InstalledNail_{active_index}',
                        0.035,
                        0.10,
                        (x, target_y + 0.035, z),
                        looks['nail'],
                    )
                    print(f'[EVENT] G2 pneumatic nail fired at index={active_index} x={x:.2f}, z={z:.2f}')

            if t >= fastening_start + len(nail_points) * fastening_duration:
                state = 'task_complete'
                robot_pos = robot_work_positions[-1]
                gun_pos = hand_reference(robot_pos)
                active_index = len(nail_points) - 1
                active_local = 1.0
                arm_pose = 'ready'

            set_scripted_root_pose(
                robot_prim,
                robot_pos,
                robot_yaw,
                g2_articulation if articulation_status == 'initialized' else None,
                articulation_reference_position,
                articulation_reference_orientation,
                robot_start,
            )
            set_translate(board, board_pos)

            if base_joint_positions is not None and joint_names:
                set_controlled_joint_pose(
                    g2_articulation,
                    joint_names,
                    base_joint_positions,
                    arm_pose,
                    active_index,
                )
                g2_articulation.set_velocities(np.zeros((1, 6), dtype=np.float32))

            fastening = state in ('press_fastener', 'hold_fastener')
            pressure = 1.0 if fastening else (0.25 if state == 'move_to_fastener' else 0.0)
            planned_nozzle_pos = (gun_pos[0], gun_pos[1] + 0.16, gun_pos[2] - 0.25)
            ground_target = robot_work_positions[max(0, active_index)]
            trajectory.append({
                'frame': frame,
                'time_s': round(t, 4),
                'board_x': board_pos[0],
                'board_y': board_pos[1],
                'board_z': board_pos[2],
                'gun_x': gun_pos[0],
                'gun_y': gun_pos[1],
                'gun_z': gun_pos[2],
                'nozzle_x': planned_nozzle_pos[0],
                'nozzle_y': planned_nozzle_pos[1],
                'nozzle_z': planned_nozzle_pos[2],
                'robot_x': robot_pos[0],
                'robot_y': robot_pos[1],
                'robot_z': robot_pos[2],
                'ground_target_x': ground_target[0],
                'ground_target_y': ground_target[1],
                'ground_column_index': (active_index // 2) if active_index >= 0 else -1,
                'ground_row': (
                    'upper' if active_index >= 0 and active_index % 2 == 1
                    else ('lower' if active_index >= 0 else 'none')
                ),
                'robot_yaw_deg': 90.0,
                'active_fastener_index': active_index,
                'state': state,
                'arm_pose': arm_pose,
                'collision_active': int(state in ('press_fastener', 'hold_fastener')),
                'contact_force_n': 120.0 if pressure == 1.0 else (35.0 if fastening else 0.0),
                'tool_pressure': pressure,
                'nails_installed': len(installed),
                'tool_grasped': int(state not in ('walk_to_drywall',)),
                'robot_label': 'AgiBot Genie G2 (official USD)',
            })

            world.step(render=False)
            # PhysX advances the dynamic articulation first; apply the
            # commanded pose immediately before the render so the official
            # G2 arms cannot be overwritten by the uncontrolled default drive.
            if base_joint_positions is not None and joint_names:
                g2_articulation.set_velocities(np.zeros((1, 6), dtype=np.float32))
                set_controlled_joint_pose(
                    g2_articulation,
                    joint_names,
                    base_joint_positions,
                    arm_pose,
                    active_index,
                )
                g2_articulation.set_velocities(np.zeros((1, 6), dtype=np.float32))
            # Re-apply the scripted root after the dynamic physics step. The
            # articulation-level teleport is what prevents roll/pitch drift
            # when the arm joint targets change during fastening.
            set_scripted_root_pose(
                robot_prim,
                robot_pos,
                robot_yaw,
                g2_articulation if articulation_status == 'initialized' else None,
                articulation_reference_position,
                articulation_reference_orientation,
                robot_start,
            )
            if base_joint_positions is not None and joint_names:
                g2_articulation.set_velocities(np.zeros((1, 6), dtype=np.float32))
            simulation_app.update()
            hand_pos = hand_reference(robot_pos)
            nozzle_pos = update_held_tool(
                gun,
                tool_handle,
                nozzle,
                hand_pos,
                gun_pos,
            )
            rep.orchestrator.step(rt_subframes=1)

        # Allow the last render write to flush before detaching the writer.
        for _ in range(8):
            simulation_app.update()
    finally:
        writer.detach()

    with (args.output / 'trajectory.csv').open('w', newline='', encoding='utf-8') as handle:
        csv_writer = csv.DictWriter(handle, fieldnames=list(trajectory[0]))
        csv_writer.writeheader()
        csv_writer.writerows(trajectory)

    task_usd = args.output / 'genie_g2_official_drywall.usd'
    stage.Export(str(task_usd))
    summary = {
        'isaac_sim': '5.1.0',
        'renderer': os.environ.get('ISAAC_DEMO_RENDERER', 'RayTracedLighting'),
        'robot': 'AgiBot Genie G2',
        'official_g2_usd': str(robot_usd),
        'robot_prim': robot_path,
        'g2_articulation': articulation_status,
        'joint_count': len(joint_names),
        'camera_prim': camera_path,
        'physics': 'Isaac Sim PhysX; official G2 collision payload plus wall/board/tool collision geometry',
        'control_model': 'deterministic drywall placement + pneumatic fastener event model',
        'ros2_runtime': False,
        'd415_required': False,
        'frames_requested': args.frames,
        'fps': args.fps,
        'motion_model': 'column-major ground-point base trajectory (lower+upper per column) plus official G2 joint-position targets; outer root and correctly offset physical articulation root locked upright, with gravity disabled for scripted stability',
        'drive_attributes_configured': drive_attributes_configured,
        'robot_gravity_disabled': robot_gravity_disabled,
        'arm_position_drive': {
            'enabled': False,
            'mode': 'direct articulation joint-position writes after each physics step',
            'stiffness': 0.0,
            'damping': 0.0,
            'max_force': 0.0,
        },
        'robot_start': robot_start,
        'robot_work': robot_work,
        'ground_work_positions': robot_work_positions,
        'robot_yaw_deg': 90.0,
        'fastener_order': nail_points,
        'state_durations_s': {
            'walk_to_drywall': approach_duration,
            'align_and_grasp_tool': ready_duration,
            'per_fastener': fastening_duration,
        },
        'nails_installed': len(installed),
        'installed_indices': installed,
        'native_render_frames': str(frames_dir),
        'task_usd': str(task_usd),
    }
    (args.output / 'run_summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(f'[OK] native Isaac Sim frames: {frames_dir}')
    print(f'[OK] task USD: {task_usd}')
    print(f'[OK] nails installed: {len(installed)}/{len(nail_points)}')


try:
    main()
finally:
    simulation_app.close(wait_for_replicator=False)
