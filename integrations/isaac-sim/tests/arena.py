# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Benchmark caller-owned RM visual geometry with moving neutral CAD robots.

Uses one existing PhysX scene and source triangles. Bodies are unconstrained
free movers in zero gravity: no wheel/suspension/contact dynamics are claimed.
Field mechanisms remain in their supplied rest poses. No source assets shipped.
"""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--input", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--mode", choices=("combined", "hybrid", "rigid"), default="rigid")
parser.add_argument("--robots", type=int, choices=(1, 4, 8), default=4)
parser.add_argument("--steps", type=int, default=2000)
parser.add_argument("--verify", action="store_true")
parser.add_argument("--physics-threads", type=int, choices=(0, 1, 4, 8, 32))
parser.add_argument("--profile-cpu", action="store_true")
parser.add_argument(
    "--physics-driver", choices=("context", "manager"), default="context"
)
parser.add_argument("--physics-batch-steps", type=int, choices=(1, 10, 100), default=1)
parser.add_argument(
    "--backend-file",
    type=Path,
    help="External backend.py with adjacent nominal.json for controlled source comparisons",
)
args = parser.parse_args()
if args.physics_driver != "manager" and args.physics_batch_steps != 1:
    parser.error("batched stepping requires --physics-driver manager")
if not 100 <= args.steps <= 10000 or args.steps % 100:
    parser.error("steps must be a multiple of 100 in 100..10000")
from isaacsim import SimulationApp

app = SimulationApp(
    {
        "headless": True,
        "multi_gpu": False,
        "extra_args": (
            []
            if args.physics_threads is None
            else [f"--/persistent/physics/numThreads={args.physics_threads}"]
        ),
    }
)
try:
    import numpy as np
    import carb
    import omni.usd

    if args.physics_threads is not None:
        assert (
            carb.settings.get_settings().get("/persistent/physics/numThreads")
            == args.physics_threads
        )
    from pxr import Gf, Vt, UsdGeom, UsdPhysics, PhysxSchema
    from isaacsim.core.api import SimulationContext
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("omni.warp.core")
    sys.path.insert(
        0, str(Path(__file__).resolve().parents[1] / "exts" / "hxyulin.mid360")
    )
    from mid360.isaac import backend, extension as extension_module
    from mid360.isaac.extension import Extension, create_sensor, latest_frame, _pose

    if args.backend_file is not None:
        spec = importlib.util.spec_from_file_location(
            "arena_baseline", args.backend_file
        )
        backend = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = backend
        spec.loader.exec_module(backend)
        # Only this benchmark process substitutes an acquisition implementation;
        # physics, extension scheduling, assets and workload remain unchanged.
        for name in (
            "Mid360Sensor",
            "StaticSceneBatch",
            "HybridSensor",
            "RigidHybridSensor",
        ):
            setattr(extension_module, name, getattr(backend, name))
    Mid360Sensor, RigidHybridSensor = backend.Mid360Sensor, backend.RigidHybridSensor

    data = np.load(args.input, allow_pickle=False)
    metadata = json.loads(str(data["metadata"]))
    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, "/World/Field")
    UsdGeom.Xform.Define(stage, "/World/Robots")

    def mesh(path, points, faces):
        value = UsdGeom.Mesh.Define(stage, path)
        value.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        value.CreateDoubleSidedAttr(True)
        value.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points))
        value.CreateFaceVertexCountsAttr(
            Vt.IntArray.FromNumpy(np.full(len(faces), 3, dtype=np.int32))
        )
        value.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.ravel()))
        return value

    for part in metadata["static"]:
        index = part["index"]
        mesh(f"/World/Field/Part{index}", data[f"p{index}"], data[f"t{index}"])
    bodies = []
    placements = [
        (-4.0, -3.0),
        (-2.0, -1.0),
        (1.0, -3.0),
        (4.0, 1.0),
        (-5.0, 2.0),
        (3.0, 3.0),
        (6.0, -2.0),
        (-7.0, -2.0),
    ]
    for i in range(args.robots):
        path = f"/World/Robots/Robot{i}"
        body = UsdGeom.Xform.Define(stage, path)
        body.AddTranslateOp().Set(Gf.Vec3d(*placements[i], 0.0))
        physical = UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        PhysxSchema.PhysxRigidBodyAPI.Apply(body.GetPrim()).CreateSleepThresholdAttr(
            0.0
        )
        physical.CreateVelocityAttr(
            Gf.Vec3f(0.15 if i % 2 == 0 else -0.1, 0.03 * (i % 3), 0.0)
        )
        physical.CreateAngularVelocityAttr(Gf.Vec3f(0.0, 0.0, 10.0 + i * 3.0))
        UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(20.0)
        mesh(path + "/Visual", data["robot_p"], data["robot_t"])
        bodies.append(body.GetPrim())
    sensor_path = "/World/Robots/Robot0/Mid360"
    marker = create_sensor(
        stage,
        path=sensor_path,
        roots=("/World/Field", "/World/Robots"),
        batch_steps=100,
        dynamic_roots=("/World/Robots",) if args.mode != "combined" else (),
        rigid_trace=args.mode == "rigid",
    )
    UsdGeom.Xformable(marker).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.45))
    sim = SimulationContext(
        physics_dt=0.001, rendering_dt=0.1, stage_units_in_meters=1.0
    )
    sim.get_physics_context().set_gravity(0.0)
    sim.initialize_physics()
    sim.play()

    def advance_one():
        if args.physics_driver == "context":
            sim.step(render=False)
        else:
            SimulationManager.step()

    def body_positions():
        cache = UsdGeom.XformCache()
        return np.array(
            [cache.GetLocalToWorldTransform(b).ExtractTranslation() for b in bodies]
        )

    def run(sensing):
        extension = None
        if sensing:
            extension = Extension()
            extension.on_startup("arena-benchmark")
        if args.physics_driver == "manager":
            SimulationManager.step(steps=100)
        else:
            for _ in range(100):
                advance_one()
        first = 0 if not sensing else latest_frame(sensor_path).first_index + 20000
        before_positions = body_positions()
        durations, sampled_frames = [], []
        count, points, messages = 0, 0, 0
        digest = hashlib.sha256()
        start = time.perf_counter()
        last_step_end = start

        def after_step(i):
            nonlocal count, points, messages, last_step_end
            if sensing and (i + 1) % 100 == 0:
                frame = latest_frame(sensor_path)
                assert frame is not None and len(frame.ranges_m) == 20000
                assert frame.first_index == first + (i + 1 - 100) * 200
                assert np.isfinite(frame.ranges_m).all()
                digest.update(frame.ranges_m.tobytes())
                fields = frame.livox_fields()
                assert np.all(fields["offset_time"] < 100_000_000)
                count += len(frame.ranges_m)
                points += len(fields["xyz_m"])
                messages += 1
                if len(sampled_frames) < 2:
                    sampled_frames.append(frame)
            durations.append((time.perf_counter() - last_step_end) * 1000)
            last_step_end = time.perf_counter()

        for batch_start in range(0, args.steps, args.physics_batch_steps):
            if args.physics_driver == "manager":
                SimulationManager.step(
                    steps=args.physics_batch_steps,
                    callback=lambda step, total: after_step(batch_start + step - 1),
                )
            else:
                advance_one()
                after_step(batch_start)
        wall = time.perf_counter() - start
        displacement = np.linalg.norm(body_positions() - before_positions, axis=1)
        assert np.all(displacement > 0.005), displacement
        if extension:
            extension.on_shutdown()
            assert (
                points > 0
                and count == args.steps * 200
                and messages == args.steps // 100
            )
        return dict(
            wall_s=wall,
            simulated_s=args.steps * 0.001,
            real_time_factor=args.steps * 0.001 / wall,
            step_median_ms=float(np.median(durations)),
            step_p95_ms=float(np.percentile(durations, 95)),
            steps_over_1ms=int(np.sum(np.array(durations) > 1)),
            scheduled_rays=count,
            range_sha256=digest.hexdigest() if sensing else None,
            valid_returns=points,
            messages=messages,
            robot_displacement_m=displacement.tolist(),
        )

    baseline = run(False)
    if args.profile_cpu:
        import cProfile
        import pstats
        import io

        profile = cProfile.Profile()
        profile.enable()
        measured = run(True)
        profile.disable()
        stream = io.StringIO()
        pstats.Stats(profile, stream=stream).sort_stats("cumtime").print_stats(30)
        args.output.with_suffix(".profile.txt").write_text(stream.getvalue())
    else:
        measured = run(True)
    verification = None
    if args.verify:
        reference = Mid360Sensor(stage, roots=("/World/Field", "/World/Robots"))
        candidate = RigidHybridSensor(
            Mid360Sensor(stage, roots=("/World/Field",), min_range_m=0.0),
            Mid360Sensor(stage, roots=("/World/Robots",), min_range_m=0.0),
        )
        fixed_reference = Mid360Sensor(stage, roots=("/World/Field",))
        expected = []
        dynamic_hits = 0
        previous = _pose(stage, sensor_path)
        for i in range(100):
            advance_one()
            current = _pose(stage, sensor_path)
            expected.append(
                reference.scan(
                    i * 200, 50, previous, current, stride=4, pose_fraction=(0.0, 0.98)
                )
            )
            candidate.scan(
                i * 200, 50, previous, current, stride=4, pose_fraction=(0.0, 0.98)
            )
            fixed = fixed_reference.scan(
                i * 200, 50, previous, current, stride=4, pose_fraction=(0.0, 0.98)
            )
            union = expected[-1].ranges_m
            dynamic_hits += int(
                np.sum(
                    (union >= 0.1)
                    & ((fixed.ranges_m < 0) | (union < fixed.ranges_m - 1e-4))
                )
            )
            previous = current
        actual = candidate.flush()
        expected_ranges = np.concatenate([f.ranges_m for f in expected])
        np.testing.assert_array_equal(actual.valid, expected_ranges >= 0)
        np.testing.assert_allclose(
            actual.ranges_m, expected_ranges, rtol=0.0, atol=1e-3
        )
        np.testing.assert_array_equal(
            actual.timestamps_ns, np.concatenate([f.timestamps_ns for f in expected])
        )
        assert dynamic_hits > 0, "no actual CAD robot hits in verification sample"
        verification = dict(
            sampled_rays=len(expected_ranges),
            valid_returns=int(actual.valid.sum()),
            max_range_difference_m=float(
                np.max(np.abs(actual.ranges_m - expected_ranges))
            ),
            same_miss_mask=True,
            dynamic_occluding_hits=dynamic_hits,
        )
    report = dict(
        mode=args.mode,
        physics_driver=args.physics_driver,
        physics_batch_steps=args.physics_batch_steps,
        physics_threads=carb.settings.get_settings().get(
            "/persistent/physics/numThreads"
        ),
        cpu_profiled=args.profile_cpu,
        backend_sha256=hashlib.sha256(Path(backend.__file__).read_bytes()).hexdigest(),
        robots=args.robots,
        physics_dt_s=0.001,
        rays_per_s=200000,
        output_period_s=0.1,
        static_triangles=metadata["static_triangles"],
        robot_triangles_each=metadata["robot"]["triangles"],
        total_triangles=metadata["static_triangles"]
        + args.robots * metadata["robot"]["triangles"],
        static_source_meshes=len(metadata["static"]),
        dynamic_source_meshes=args.robots,
        asset_metadata=metadata,
        verification=verification,
        scope="full visual arena at rest; merged neutral CAD robot geometry; free moving roots; no wheels/contacts/rendering/ROS/network",
        baseline=baseline,
        sensor=measured,
        checks="all robot roots move; continuous scheduled emissions; 100 ms frames; finite misses/hits; nonempty returns",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(
        "MID360_ARENA "
        + json.dumps({k: v for k, v in report.items() if k != "asset_metadata"}),
        flush=True,
    )
except BaseException:
    import traceback

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".failure.txt").write_text(traceback.format_exc())
    raise
finally:
    app.close()
