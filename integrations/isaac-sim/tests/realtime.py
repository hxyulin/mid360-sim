# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Run with Isaac python.sh: actual 1 kHz physics callback acquisition."""

import argparse
import json
from pathlib import Path
import sys
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--steps", type=int, default=2000)
parser.add_argument("--batch-steps", type=int, choices=(1, 10, 100), default=1)
parser.add_argument("--static-trace", action="store_true")
parser.add_argument("--pose-source", choices=("usd", "physics"), default="usd")
parser.add_argument("--moving-obstacle", action="store_true")
parser.add_argument("--hybrid", action="store_true")
parser.add_argument("--static-boxes", type=int, default=0)
parser.add_argument("--rigid-trace", action="store_true")
args = parser.parse_args()
if not 100 <= args.steps <= 10000 or args.steps % 100:
    parser.error("steps must be a multiple of 100 in 100..10000")
if args.rigid_trace and not args.hybrid:
    parser.error("rigid-trace requires hybrid")
if args.hybrid and (not args.moving_obstacle or args.static_trace):
    parser.error("hybrid requires moving-obstacle and cannot use static-trace")
if args.static_trace and args.moving_obstacle:
    parser.error("static tracing cannot include moving optical geometry")
if not 0 <= args.static_boxes <= 512:
    parser.error("static-boxes must be in 0..512")
from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "multi_gpu": False})
try:
    import numpy as np
    import omni.usd
    from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
    from isaacsim.core.api import SimulationContext
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("omni.warp.core")
    sys.path.insert(
        0, str(Path(__file__).resolve().parents[1] / "exts" / "hxyulin.mid360")
    )
    from mid360.isaac.extension import Extension, create_sensor, latest_frame, _pose

    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")
    box = UsdGeom.Cube.Define(stage, "/World/Enclosure")
    box.CreateSizeAttr(20.0)
    UsdGeom.Xform.Define(stage, "/World/Details")
    for i in range(args.static_boxes):
        detail = UsdGeom.Cube.Define(stage, f"/World/Details/Box{i}")
        detail.CreateSizeAttr(0.15)
        detail.AddTranslateOp().Set(
            Gf.Vec3d(-5 + (i % 16) * 0.6, 3 + (i // 16) * 0.2, 0.2)
        )
    if args.moving_obstacle:
        obstacle = UsdGeom.Cube.Define(stage, "/World/Obstacle")
        obstacle.CreateSizeAttr(0.7)
        obstacle.AddTranslateOp().Set(Gf.Vec3d(2.0, -1.0, 0.8))
        obstacle_body = UsdPhysics.RigidBodyAPI.Apply(obstacle.GetPrim())
        PhysxSchema.PhysxRigidBodyAPI.Apply(
            obstacle.GetPrim()
        ).CreateSleepThresholdAttr(0.0)
        obstacle_body.CreateVelocityAttr(Gf.Vec3f(0.0, 0.1, 0.0))
        obstacle_body.CreateAngularVelocityAttr(Gf.Vec3f(0.0, 0.0, 15.0))
        UsdPhysics.MassAPI.Apply(obstacle.GetPrim()).CreateMassAttr(1.0)
    mount = UsdGeom.Xform.Define(stage, "/World/Robot")
    mount.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.8))
    rigid = UsdPhysics.RigidBodyAPI.Apply(mount.GetPrim())
    PhysxSchema.PhysxRigidBodyAPI.Apply(mount.GetPrim()).CreateSleepThresholdAttr(0.0)
    rigid.CreateVelocityAttr(Gf.Vec3f(0.1, 0, 0))
    rigid.CreateAngularVelocityAttr(Gf.Vec3f(0, 0, 28.6478898))
    UsdPhysics.MassAPI.Apply(mount.GetPrim()).CreateMassAttr(1.0)
    sim = SimulationContext(
        physics_dt=0.001, rendering_dt=0.1, stage_units_in_meters=1.0
    )
    sim.get_physics_context().set_gravity(0.0)
    create_sensor(
        stage,
        path="/World/Robot/Mid360",
        roots=(
            "/World/Enclosure",
            "/World/Details",
            *(("/World/Obstacle",) if args.moving_obstacle else ()),
        ),
        batch_steps=args.batch_steps,
        static_trace=args.static_trace,
        dynamic_roots=("/World/Obstacle",) if args.hybrid else (),
        rigid_trace=args.rigid_trace,
    )
    sim.initialize_physics()
    sim.play()

    import omni.physx

    physx_interface = omni.physx.get_physx_interface()

    native_pose_samples = []
    native_pose_reads = 0

    def physics_pose(stage, path):
        global native_pose_reads
        # This benchmark mount has identity extrinsics under exactly one body.
        # Do not reuse this mapping for arbitrary nested robot sensor mounts.
        assert path == "/World/Robot/Mid360"
        state = physx_interface.get_rigidbody_transformation("/World/Robot")
        native = (tuple(state["position"]), tuple(state["rotation"]))
        native_pose_reads += 1
        if native_pose_reads % 100 == 0:
            usd = _pose(stage, path)
            position_error = float(np.linalg.norm(np.array(native[0]) - usd[0]))
            qa, qb = np.array(native[1]), np.array(usd[1])
            qa, qb = qa / np.linalg.norm(qa), qb / np.linalg.norm(qb)
            rotation_error = float(
                min(np.linalg.norm(qa - qb), np.linalg.norm(qa + qb))
            )
            # Diagnostic, not an equivalence assumption: native PhysX and USD
            # writeback may represent different callback phases. Do not use the
            # throughput probe as validation of a drop-in pose substitution.
            native_pose_samples.append((position_error, rotation_error))
        return native

    def run(sensing):
        extension = None
        if sensing:
            extension = Extension(
                pose_reader=physics_pose if args.pose_source == "physics" else None
            )
            extension.on_startup("realtime-benchmark")
        for _ in range(100):
            sim.step(render=False)
        if extension:
            first_index = (
                latest_frame("/World/Robot/Mid360").first_index + 200 * args.batch_steps
            )
        durations, pending, messages, points = [], [], 0, 0
        before_pose = np.array(
            UsdGeom.XformCache()
            .GetLocalToWorldTransform(mount.GetPrim())
            .ExtractTranslation()
        )
        start = time.perf_counter()
        for i in range(args.steps):
            before = time.perf_counter()
            sim.step(render=False)
            if extension and (i + 1) % args.batch_steps == 0:
                frame = latest_frame("/World/Robot/Mid360")
                assert (
                    frame is not None and len(frame.ranges_m) == 200 * args.batch_steps
                )
                assert (
                    frame.first_index == first_index + (i + 1 - args.batch_steps) * 200
                ), (i, frame.first_index, first_index, extension.batch_counts)
                assert frame.valid.all()
                fields = frame.livox_fields()
                fields["offset_time"] += ((i + 1 - args.batch_steps) % 100) * 1_000_000
                pending.append(fields)
                if (i + 1) % 100 == 0:
                    cloud = {
                        key: np.concatenate([p[key] for p in pending])
                        for key in (
                            "xyz_m",
                            "offset_time",
                            "line",
                            "tag",
                            "reflectivity",
                        )
                    }
                    assert len(cloud["xyz_m"]) == 20_000
                    assert int(cloud["offset_time"][-1]) == 99_995_000
                    points += len(cloud["xyz_m"])
                    messages += 1
                    pending.clear()
            durations.append((time.perf_counter() - before) * 1000)
        elapsed = time.perf_counter() - start
        after_pose = np.array(
            UsdGeom.XformCache()
            .GetLocalToWorldTransform(mount.GetPrim())
            .ExtractTranslation()
        )
        if args.pose_source == "physics":
            native = physics_pose(stage, "/World/Robot/Mid360")
            np.testing.assert_allclose(native[0], after_pose, rtol=0, atol=1e-5)
            usd_rotation = _pose(stage, "/World/Robot/Mid360")[1]
            qa, qb = np.array(native[1]), np.array(usd_rotation)
            qa, qb = qa / np.linalg.norm(qa), qb / np.linalg.norm(qb)
            assert min(np.linalg.norm(qa - qb), np.linalg.norm(qa + qb)) < 1e-5
        assert np.linalg.norm(after_pose - before_pose) > 0.005, (
            before_pose,
            after_pose,
        )
        if extension:
            extension.on_shutdown()
        return dict(
            wall_s=elapsed,
            simulated_s=args.steps * 0.001,
            real_time_factor=args.steps * 0.001 / elapsed,
            step_median_ms=float(np.median(durations)),
            step_p95_ms=float(np.percentile(durations, 95)),
            steps_over_1ms=int(np.sum(np.array(durations) > 1)),
            messages=messages,
            points=points,
        )

    report = dict(
        physics_dt_s=0.001,
        rays_per_step=200,
        batch_steps=args.batch_steps,
        static_trace=args.static_trace,
        pose_source=args.pose_source,
        moving_obstacle=args.moving_obstacle,
        static_boxes=args.static_boxes,
        hybrid=args.hybrid,
        rigid_trace=args.rigid_trace,
        publish_period_s=0.1,
        scope="moving mount, actual extension callbacks + physics + host arrays; no rendering/transport",
        baseline=run(False),
        sensor=run(True),
        checks="moving USD mount, callback counts, emission continuity, 100 ms aggregation",
        native_pose_comparison=(
            None
            if not native_pose_samples
            else dict(
                samples=len(native_pose_samples),
                max_position_error_m=max(p[0] for p in native_pose_samples),
                max_sign_invariant_quaternion_error=max(
                    p[1] for p in native_pose_samples
                ),
            )
        ),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print("MID360_REALTIME " + json.dumps(report), flush=True)
except BaseException:
    import traceback

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".failure.txt").write_text(traceback.format_exc())
    raise
finally:
    app.close()
