# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Run with Isaac Sim's python.sh; real GPU queries, no downloaded assets."""

import argparse
import json
from pathlib import Path
import statistics
import time

parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "multi_gpu": False})
try:
    import numpy as np
    import omni.usd
    from pxr import UsdGeom, Gf
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("omni.warp.core")
    import sys

    sys.path.insert(
        0, str(Path(__file__).resolve().parents[1] / "exts" / "hxyulin.mid360")
    )
    from mid360.isaac.backend import Mid360Sensor, StaticSceneBatch, MODEL

    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")
    box = UsdGeom.Cube.Define(stage, "/World/Enclosure")
    box.CreateSizeAttr(20.0)
    # Rays originate inside this box: this also verifies backface/two-sided hits.
    sensor = Mid360Sensor(stage)
    origin = np.array([0.0, 0.0, 0.8])
    pose = (tuple(origin), (0, 0, 0, 1))
    durations = []
    for n in range(35):
        start = time.perf_counter()
        frame = sensor.scan(n * 20000, 20000, pose)
        elapsed = (time.perf_counter() - start) * 1000
        if n >= 5:
            durations.append(elapsed)
        expected = np.min(
            np.where(
                frame.directions > 0,
                (10 - origin) / frame.directions,
                (-10 - origin) / frame.directions,
            ),
            axis=1,
        )
        assert np.allclose(frame.ranges_m, expected, rtol=2e-5, atol=2e-4)
    # Cross-language optical fixtures include the exact 18.2h clock boundary.
    errors = []
    for fixture in MODEL["reference_rays"]:
        f = sensor.scan(int(fixture["index"]), 1, pose)
        difference = float(np.linalg.norm(f.directions[0] - fixture["direction"]))
        assert difference < 2e-5, (fixture, difference)
        errors.append(difference)
    # Batch partition/stride invariance; original miss timestamps remain intact.
    full = sensor.scan(7, 1000, pose, stride=3)
    a = sensor.scan(7, 400, pose, stride=3)
    b = sensor.scan(1207, 600, pose, stride=3)
    assert np.array_equal(full.directions, np.r_[a.directions, b.directions])
    assert np.array_equal(full.ranges_m, np.r_[a.ranges_m, b.ranges_m])
    # Dynamic rigid transform update must change ranges without rebuilding BVH.
    box.AddTranslateOp().Set(Gf.Vec3d(1, 0, 0))
    moved = sensor.scan(7, 1000, pose, stride=3)
    assert not np.array_equal(moved.ranges_m, full.ranges_m)
    box.GetOrderedXformOps()[0].Set(Gf.Vec3d(0, 0, 0))
    # Near opaque geometry suppresses farther walls, rather than restarting rays.
    near = UsdGeom.Cube.Define(stage, "/World/Near")
    near.CreateSizeAttr(0.1)
    near.AddTranslateOp().Set(Gf.Vec3d(*origin))
    blocked = Mid360Sensor(stage).scan(0, 100, pose)
    assert not blocked.valid.any()
    assert blocked.livox_fields()["timebase"] == 0
    # Per-ray sensor translation interpolation, with independently analytic ranges.
    translated = sensor.scan(0, 2000, pose, ((1, 0, 0.8), (0, 0, 0, 1)))
    origins = np.column_stack(
        [np.linspace(0, 1, 2000), np.zeros(2000), np.full(2000, 0.8)]
    )
    expected = np.min(
        np.where(
            translated.directions > 0,
            (10 - origins) / translated.directions,
            (-10 - origins) / translated.directions,
        ),
        axis=1,
    )
    assert np.allclose(translated.ranges_m, expected, rtol=2e-5, atol=2e-4)
    # Sensor rotation uses xyzw and is interpolated at emission time.
    q = (np.sqrt(0.5), 0, 0, np.sqrt(0.5))
    rotated = sensor.scan(0, 2000, (tuple(origin), q))
    world_dirs = np.column_stack(
        [rotated.directions[:, 0], -rotated.directions[:, 2], rotated.directions[:, 1]]
    )
    expected = np.min(
        np.where(
            world_dirs > 0, (10 - origin) / world_dirs, (-10 - origin) / world_dirs
        ),
        axis=1,
    )
    assert np.allclose(rotated.ranges_m, expected, rtol=2e-5, atol=2e-4)
    # Nonuniform object scale and a real triangulated USD Mesh share one BVH.
    scale_op = box.AddScaleOp()
    scale_op.Set(Gf.Vec3d(1, 2, 0.5))
    scaled = Mid360Sensor(stage, exclude=("/World/Near",)).scan(0, 2000, pose)
    bounds = np.array([10, 20, 5])
    expected = np.min(
        np.where(
            scaled.directions > 0,
            (bounds - origin) / scaled.directions,
            (-bounds - origin) / scaled.directions,
        ),
        axis=1,
    )
    assert np.allclose(scaled.ranges_m, expected, rtol=2e-5, atol=2e-4)
    scale_op.Set(Gf.Vec3d(1, 1, 1))
    triangle = UsdGeom.Mesh.Define(stage, "/World/TriangleWall")
    triangle.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    triangle.CreatePointsAttr([(2, -20, -20), (2, 20, -20), (2, 20, 20), (2, -20, 20)])
    triangle.CreateFaceVertexCountsAttr([3, 3])
    triangle.CreateFaceVertexIndicesAttr([0, 1, 2, 0, 2, 3])
    multi = Mid360Sensor(stage, exclude=("/World/Near",)).scan(0, 2000, pose)
    box_expected = np.min(
        np.where(
            multi.directions > 0,
            (10 - origin) / multi.directions,
            (-10 - origin) / multi.directions,
        ),
        axis=1,
    )
    plane_expected = np.where(
        multi.directions[:, 0] > 0, 2 / multi.directions[:, 0], np.inf
    )
    assert np.allclose(
        multi.ranges_m, np.minimum(box_expected, plane_expected), rtol=2e-5, atol=2e-4
    )
    stage.RemovePrim("/World/TriangleWall")
    # Never silently use control cages or ignore unsupported optical instances.
    bad = UsdGeom.Mesh.Define(stage, "/World/Subdivision")
    try:
        Mid360Sensor(stage, exclude=("/World/Near",))
        raise AssertionError("subdivision should be rejected")
    except ValueError as error:
        assert "subdivision" in str(error)
    stage.RemovePrim("/World/Subdivision")
    UsdGeom.PointInstancer.Define(stage, "/World/Instances")
    try:
        Mid360Sensor(stage, exclude=("/World/Near",))
        raise AssertionError("point instancer should be rejected")
    except ValueError as error:
        assert "PointInstancer" in str(error)
    stage.RemovePrim("/World/Instances")
    # Deferred readback must retain each query's scene, not the final scene.
    first = sensor.scan(200, 200, pose)
    sensor.scan(200, 200, pose, defer_readback=True)
    box.GetOrderedXformOps()[0].Set(Gf.Vec3d(0.1, 0, 0))
    sensor.scan(400, 200, pose, defer_readback=True)
    combined = sensor.flush()
    second = sensor.scan(400, 200, pose)
    np.testing.assert_allclose(combined.ranges_m[:200], first.ranges_m, atol=1e-5)
    np.testing.assert_allclose(combined.ranges_m[200:], second.ranges_m, atol=1e-5)
    np.testing.assert_array_equal(combined.indices, np.arange(200, 600))
    saved = combined.ranges_m.copy()
    sensor.scan(600, 200, pose, defer_readback=True)
    try:
        sensor.scan(0, 200, pose)
        raise AssertionError("mixed pending/immediate scan accepted")
    except ValueError:
        pass
    sensor.flush()
    assert sensor.flush() is None
    np.testing.assert_array_equal(combined.ranges_m, saved)
    box.GetOrderedXformOps()[0].Set(Gf.Vec3d(0, 0, 0))
    parent_op = UsdGeom.Xformable(stage.GetPrimAtPath("/World")).AddTranslateOp()
    unchanged = sensor.scan(0, 200, pose)
    parent_op.Set(Gf.Vec3d(0.2, 0, 0))
    changed_parent = sensor.scan(0, 200, pose)
    assert not np.array_equal(unchanged.ranges_m, changed_parent.ranges_m)
    parent_op.Set(Gf.Vec3d(0, 0, 0))
    # Growing a queued buffer preserves the already queried prefix.
    sensor.scan(0, 12000, pose, defer_readback=True)
    sensor.scan(12000, 12000, pose, defer_readback=True)
    grown = sensor.flush()
    expected = sensor.scan(0, 24000, pose)
    np.testing.assert_array_equal(grown.ranges_m, expected.ranges_m)
    # Static tracing batches retain per-segment, non-linear translation/rotation.
    static = StaticSceneBatch(sensor)
    cursor = 17
    expected_parts = []
    poses = [
        pose,
        ((0.2, -0.1, 0.8), (0, 0, 0.6, 0.8)),
        ((-0.3, 0.4, 0.9), (0, 0.6, 0, 0.8)),
        ((0.1, 0.2, 0.7), (0.6, 0, 0, 0.8)),
    ]
    for i, count in enumerate((197, 201, 203)):
        expected_parts.append(
            sensor.scan(
                cursor,
                count,
                poses[i],
                poses[i + 1],
                stride=3,
                pose_fraction=(0.1, 0.8),
            )
        )
        static.scan(
            cursor, count, poses[i], poses[i + 1], stride=3, pose_fraction=(0.1, 0.8)
        )
        cursor += count * 3
    traced = static.flush()
    np.testing.assert_array_equal(
        traced.directions, np.concatenate([p.directions for p in expected_parts])
    )
    np.testing.assert_allclose(
        traced.ranges_m, np.concatenate([p.ranges_m for p in expected_parts]), atol=1e-5
    )
    np.testing.assert_array_equal(
        traced.timestamps_ns, np.concatenate([p.timestamps_ns for p in expected_parts])
    )
    assert static.flush() is None
    retained_ranges, retained_directions = (
        traced.ranges_m.copy(),
        traced.directions.copy(),
    )
    # Alternating larger/smaller batches must not expose capacity tails or
    # overwrite a caller's retained host frame when device buffers are reused.
    for count in (3, 22000, 7):
        static.scan(0, count, pose)
        current = static.flush()
        reference = sensor.scan(0, count, pose)
        assert len(current.ranges_m) == count
        np.testing.assert_array_equal(current.ranges_m, reference.ranges_m)
        np.testing.assert_array_equal(traced.ranges_m, retained_ranges)
        np.testing.assert_array_equal(traced.directions, retained_directions)

    # The GPU subinterval path agrees with explicit CPU endpoint interpolation.
    def interpolate_pose(a, b, fraction):
        position = tuple(x + (y - x) * fraction for x, y in zip(a[0], b[0]))
        qa = Gf.Quatd(a[1][3], Gf.Vec3d(*a[1][:3]))
        qb = Gf.Quatd(b[1][3], Gf.Vec3d(*b[1][:3]))
        q = Gf.Slerp(fraction, qa, qb)
        return position, (*q.GetImaginary(), q.GetReal())

    gpu_fraction = sensor.scan(
        123, 101, poses[1], poses[2], pose_fraction=(0.123, 0.987)
    )
    cpu_fraction = sensor.scan(
        123,
        101,
        interpolate_pose(poses[1], poses[2], 0.123),
        interpolate_pose(poses[1], poses[2], 0.987),
    )
    np.testing.assert_allclose(gpu_fraction.ranges_m, cpu_fraction.ranges_m, atol=2e-4)
    # The static-only path must reject moving optical geometry, not trace old
    # poses against its new location. Ordinary per-step tracing still supports it.
    static.scan(0, 200, pose)
    box.GetOrderedXformOps()[0].Set(Gf.Vec3d(0.1, 0, 0))
    try:
        static.flush()
        raise AssertionError("changed static geometry accepted")
    except ValueError as error:
        assert "geometry changed" in str(error)
    box.GetOrderedXformOps()[0].Set(Gf.Vec3d(0, 0, 0))
    # PhysX writeback need not emit authoring notices. Never cache such meshes
    # as static or let the static tracing architecture accept them.
    from pxr import UsdPhysics

    UsdPhysics.RigidBodyAPI.Apply(box.GetPrim())
    physical = Mid360Sensor(stage, roots=("/World/Enclosure",))
    assert not physical._cache_transforms
    try:
        StaticSceneBatch(physical)
        raise AssertionError("physics-driven optical geometry accepted as static")
    except ValueError:
        pass
    box.GetPrim().RemoveAPI(UsdPhysics.RigidBodyAPI)
    # Exercise the actual Kit extension on the simulation timeline, including
    # paused time and STOP/reset, not just direct Python backend calls.
    stage.RemovePrim("/World/Near")
    from pxr import UsdPhysics
    import omni.kit.app, omni.timeline

    manager = omni.kit.app.get_app().get_extension_manager()
    manager.add_path(str(Path(__file__).resolve().parents[1] / "exts"))
    assert manager.set_extension_enabled_immediate("hxyulin.mid360", True)
    from mid360.isaac.extension import create_sensor, latest_frame

    create_sensor(stage)
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(15):
        app.update()
    live = latest_frame()
    assert live is not None and len(live.ranges_m) > 0
    assert live.valid.any()
    timeline.pause()
    for _ in range(3):
        app.update()
    paused = latest_frame()
    for _ in range(3):
        app.update()
    assert latest_frame() is paused
    timeline.stop()
    for _ in range(3):
        app.update()
    assert latest_frame() is None
    manager.set_extension_enabled_immediate("hxyulin.mid360", False)
    report = dict(
        isaac_version="6.2.0",
        device="cuda:0",
        rays_per_batch=20000,
        warmup_batches=5,
        measured_batches=30,
        median_ms=statistics.median(durations),
        p95_ms=float(np.percentile(durations, 95)),
        max_reference_vector_difference=max(errors),
        checks="GPU enclosure intersections, Rust fixtures, long-cycle wrap, chunk/stride invariance, dynamic transforms, near-hit occlusion, rolling sensor translation/rotation, nonuniform scale, combined triangle mesh BVH, deferred readback ownership/growth/moving geometry/ancestor invalidation, segmented static tracing equivalence/change rejection/GPU pose fractions/physics ancestor rejection, Kit extension acquisition/pause/reset",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print("MID360_RESULT " + json.dumps(report), flush=True)
except BaseException:
    import traceback

    traceback.print_exc()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".failure.txt").write_text(traceback.format_exc())
    raise
finally:
    app.close()
