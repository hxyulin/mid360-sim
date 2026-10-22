# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""GPU split-scene equivalence, moving occlusion and near-hit shadow checks."""

import argparse
import json
from pathlib import Path
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--rigid-trace", action="store_true")
args = parser.parse_args()
from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "multi_gpu": False})
try:
    import numpy as np
    import omni.usd
    from pxr import Gf, UsdGeom
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("omni.warp.core")
    sys.path.insert(
        0, str(Path(__file__).resolve().parents[1] / "exts" / "hxyulin.mid360")
    )
    from mid360.isaac.backend import Mid360Sensor, HybridSensor, RigidHybridSensor

    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, "/World/Static")
    walls = UsdGeom.Cube.Define(stage, "/World/Static/Walls")
    walls.CreateSizeAttr(20.0)
    blocker = UsdGeom.Cube.Define(stage, "/World/Static/Near")
    blocker.CreateSizeAttr(0.1)
    fixed_op = blocker.AddTranslateOp()
    fixed_op.Set(Gf.Vec3d(-3, 0, 0.8))
    moving = UsdGeom.Cube.Define(stage, "/World/Moving")
    moving.CreateSizeAttr(2.0)
    move_op = moving.AddTranslateOp()
    move_op.Set(Gf.Vec3d(2.5, 0, 0.8))
    scale_op = moving.AddScaleOp()
    scale_op.Set(Gf.Vec3d(1, 1, 1))
    rotate_op = moving.AddRotateXYZOp()
    extra = UsdGeom.Cube.Define(stage, "/World/Moving/Extra")
    extra.CreateSizeAttr(0.6)
    extra.AddTranslateOp().Set(Gf.Vec3d(0, 2, 0))
    reference = Mid360Sensor(stage)
    static_reference = Mid360Sensor(stage, roots=("/World/Static",))

    def make_hybrid():
        factory = RigidHybridSensor if args.rigid_trace else HybridSensor
        return factory(
            Mid360Sensor(stage, roots=("/World/Static",), min_range_m=0.0),
            Mid360Sensor(stage, roots=("/World/Moving",), min_range_m=0.0),
        )

    hybrid = make_hybrid()
    expected, cursor = [], 7
    moving_hits = 0
    for step in range(25):
        move_op.Set(Gf.Vec3d(1.5 + 0.03 * step, 0.5 * np.sin(step * 0.4), 0.8))
        scale_op.Set(Gf.Vec3d((0.7 + step * 0.01) * (-1 if step % 2 else 1), 0.9, 1.2))
        rotate_op.Set(Gf.Vec3f(step * 3.0, step * 2.0, step * 5.0))
        a = ((0.01 * step, 0, 0.8), (0, 0, np.sin(step * 0.01), np.cos(step * 0.01)))
        b = (
            (0.01 * (step + 1), 0.002, 0.8),
            (0, 0, np.sin((step + 1) * 0.01), np.cos((step + 1) * 0.01)),
        )
        count = 193 + step % 3
        expected.append(
            reference.scan(cursor, count, a, b, stride=3, pose_fraction=(0.1, 0.9))
        )
        hybrid.scan(cursor, count, a, b, stride=3, pose_fraction=(0.1, 0.9))
        fixed = static_reference.scan(
            cursor, count, a, b, stride=3, pose_fraction=(0.1, 0.9)
        )
        union = expected[-1].ranges_m
        moving_hits += int(
            np.sum(
                (union >= 0.1)
                & ((fixed.ranges_m < 0) | (union < fixed.ranges_m - 1e-5))
            )
        )
        cursor += count * 3
    actual = hybrid.flush()
    expected_ranges = np.concatenate([f.ranges_m for f in expected])
    np.testing.assert_allclose(actual.ranges_m, expected_ranges, rtol=2e-6, atol=2e-5)
    np.testing.assert_array_equal(
        actual.directions, np.concatenate([f.directions for f in expected])
    )
    np.testing.assert_array_equal(
        actual.timestamps_ns, np.concatenate([f.timestamps_ns for f in expected])
    )
    assert moving_hits > 100
    saved = actual.ranges_m.copy()
    pose = ((0, 0, 0.8), (0, 0, 0, 1))
    # A nearby moving shell suppresses all farther static walls.
    move_op.Set(Gf.Vec3d(0, 0, 0.8))
    scale_op.Set(Gf.Vec3d(0.025, 0.025, 0.025))
    hybrid.scan(0, 200, pose)
    near_dynamic = hybrid.flush()
    assert not near_dynamic.valid.any()
    np.testing.assert_array_equal(
        near_dynamic.ranges_m, reference.scan(0, 200, pose).ranges_m
    )
    # Reverse the partitions: nearby static shell suppresses a farther mover.
    move_op.Set(Gf.Vec3d(2.5, 0, 0.8))
    scale_op.Set(Gf.Vec3d(1, 1, 1))
    fixed_op.Set(Gf.Vec3d(0, 0, 0.8))
    hybrid.scan(0, 200, pose)
    near_static = hybrid.flush()
    assert not near_static.valid.any()
    np.testing.assert_array_equal(
        near_static.ranges_m, reference.scan(0, 200, pose).ranges_m
    )
    np.testing.assert_array_equal(actual.ranges_m, saved)
    assert hybrid.flush() is None
    # Static edits during pending acquisition fail closed, not partial union.
    hybrid.scan(0, 200, pose)
    fixed_op.Set(Gf.Vec3d(-3, 0, 0.8))
    try:
        hybrid.flush()
        raise AssertionError("changed static partition accepted")
    except ValueError as error:
        assert "geometry changed" in str(error)
    try:
        hybrid.flush()
        raise AssertionError("failed hybrid reused")
    except ValueError as error:
        assert "failed" in str(error) and "hybrid" in str(error)
    report = dict(
        rigid_trace=args.rigid_trace,
        points=len(expected_ranges),
        moving_hits=moving_hits,
        moving_source_meshes=len(hybrid.dynamic.paths),
        max_range_difference_m=float(np.max(np.abs(actual.ranges_m - expected_ranges))),
        checks="union equivalence across multiple nested translating/rotating/reflected/scaled meshes and turning sensor; stride/fractions/time; near static and dynamic occlusion; owned outputs; static-change failure",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print("MID360_HYBRID " + json.dumps(report), flush=True)
except BaseException:
    import traceback

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".failure.txt").write_text(traceback.format_exc())
    raise
finally:
    app.close()
