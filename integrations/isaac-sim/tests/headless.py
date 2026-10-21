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
    from mid360.isaac.backend import Mid360Sensor, MODEL

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
        checks="GPU enclosure intersections, Rust fixtures, long-cycle wrap, chunk/stride invariance, dynamic transforms, near-hit occlusion, rolling sensor translation/rotation, nonuniform scale, combined triangle mesh BVH, Kit extension acquisition/pause/reset",
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
