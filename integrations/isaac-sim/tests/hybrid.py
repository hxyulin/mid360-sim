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
    from pxr import Gf, Usd, UsdGeom
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("omni.warp.core")
    sys.path.insert(
        0, str(Path(__file__).resolve().parents[1] / "exts" / "hxyulin.mid360")
    )
    from mid360.isaac.backend import (
        HybridSensor,
        Mid360Sensor,
        RigidHybridSensor,
        SceneTransformHistory,
    )

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
    extra.AddRotateXYZOp().Set(Gf.Vec3f(11, 23, 37))
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
    if args.rigid_trace:
        # Both consumers retain the same captures, with independent sensor poses,
        # fractions and flush boundaries. Later writes cannot change old rows.
        left, right = make_hybrid(), make_hybrid()
        history = SceneTransformHistory(stage, left.dynamic.paths, generation=17)
        snapshots, expected_left, expected_right = [], [], []

        def rejected(operation, message):
            try:
                operation()
            except ValueError:
                return
            raise AssertionError(message)

        for step in range(4):
            move_op.Set(Gf.Vec3d(2 + 0.1 * step, 0, 0.8))
            snapshot = history.capture(("physics-writeback", step))
            assert history.capture(("physics-writeback", step)) is snapshot
            snapshots.append(snapshot)
            rejected(
                lambda snapshot=snapshot: snapshot.inverses.setflags(write=True),
                "writable snapshot",
            )
            end = ((0.02 * step, 0, 0.8), (0, 0, np.sin(0.1), np.cos(0.1)))
            expected_left.append(reference.scan(step * 200, 200, pose, end))
            expected_right.append(
                reference.scan(step * 200, 200, end, pose, pose_fraction=(0.2, 0.7))
            )
            left.scan(step * 200, 200, pose, end, scene_snapshot=snapshot)
            right.scan(
                step * 200,
                200,
                end,
                pose,
                pose_fraction=(0.2, 0.7),
                scene_snapshot=snapshot,
            )
            if step % 2 == 1:
                frame = left.flush()
                np.testing.assert_allclose(
                    frame.ranges_m,
                    np.concatenate([f.ranges_m for f in expected_left[-2:]]),
                    rtol=2e-6,
                    atol=2e-5,
                )
        old = snapshots[0].inverses.copy()
        move_op.Set(Gf.Vec3d(9, 0, 0.8))
        assert history.capture(("physics-writeback", 0)) is snapshots[0]
        np.testing.assert_array_equal(snapshots[0].inverses, old)
        del left, history, snapshots
        frame = right.flush()
        np.testing.assert_allclose(
            frame.ranges_m,
            np.concatenate([f.ranges_m for f in expected_right]),
            rtol=2e-6,
            atol=2e-5,
        )
        np.testing.assert_array_equal(
            frame.timestamps_ns,
            np.concatenate([f.timestamps_ns for f in expected_right]),
        )
        # Wrong ordered partition/stage/generation cannot silently supply poses.
        mismatch = make_hybrid()
        other = SceneTransformHistory(
            stage, tuple(reversed(mismatch.dynamic.paths)), 18
        )
        rejected(
            lambda: mismatch.scan(0, 200, pose, scene_snapshot=other.capture(0)),
            "wrong partition accepted",
        )
        overlapping = make_hybrid()
        first_generation = SceneTransformHistory(stage, overlapping.dynamic.paths, 18)
        next_generation = SceneTransformHistory(stage, overlapping.dynamic.paths, 19)
        overlapping.scan(0, 200, pose, scene_snapshot=first_generation.capture(0))
        overlapping.scan(200, 200, pose, scene_snapshot=next_generation.capture(0))
        assert overlapping.flush() is not None
        # A history must outlive either consumer without a history/snapshot cycle.
        import weakref

        temporary = SceneTransformHistory(stage, overlapping.dynamic.paths, 20)
        retained = temporary.capture(0)
        owner = weakref.ref(temporary)
        del temporary
        assert owner() is not None
        del retained
        assert owner() is None
        singular = make_hybrid()
        scale_op.Set(Gf.Vec3d(0, 1, 1))
        rejected(lambda: singular.scan(0, 200, pose), "singular source accepted")
        scale_op.Set(Gf.Vec3d(1, 1, 1))
        removed = make_hybrid()
        old_history = SceneTransformHistory(stage, removed.dynamic.paths, 19)
        old_snapshot = old_history.capture(0)
        removed.scan(0, 200, pose, scene_snapshot=old_snapshot)
        stage.RemovePrim("/World/Moving/Extra")
        UsdGeom.Cube.Define(stage, "/World/Moving/Extra").CreateSizeAttr(0.6)
        rejected(lambda: old_history.capture(0), "replacement history accepted")
        rejected(removed.flush, "replacement pending history accepted")
        rebuilt = make_hybrid()
        rebuilt.scan(0, 200, pose)
        assert rebuilt.flush() is not None
        bounded = SceneTransformHistory(stage, rebuilt.dynamic.paths, 20)
        for index in range(100):
            bounded.capture(index)
        rejected(lambda: bounded.capture(100), "unbounded history accepted")

        for kind in ("nan", "inf", "perspective", "determinant_overflow"):
            validation_stage = Usd.Stage.CreateInMemory()
            cube = UsdGeom.Cube.Define(validation_stage, "/Cube")
            matrix = Gf.Matrix4d(1)
            if kind == "perspective":
                matrix[0, 3] = 0.5
            elif kind == "determinant_overflow":
                matrix[0, 0] = matrix[1, 1] = 1e200
            else:
                matrix[0, 0] = float(kind)
            cube.AddTransformOp().Set(matrix)
            invalid = SceneTransformHistory(validation_stage, ["/Cube"], 0)
            rejected(
                lambda invalid=invalid: invalid.capture(0),
                "invalid source matrix accepted",
            )

        for attribute, edited in (
            ("holeIndices", [0]),
            ("subdivisionScheme", "catmullClark"),
        ):
            validation_stage = Usd.Stage.CreateInMemory()
            mesh = UsdGeom.Mesh.Define(validation_stage, "/Mesh")
            mesh.CreatePointsAttr([(0, 0, 0), (1, 0, 0), (0, 1, 0)])
            mesh.CreateFaceVertexCountsAttr([3])
            mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
            mesh.CreateHoleIndicesAttr([])
            mesh.CreateSubdivisionSchemeAttr("none")
            invalid = SceneTransformHistory(validation_stage, ["/Mesh"], 0)
            invalid.capture(0)
            mesh.GetPrim().GetAttribute(attribute).Set(edited)
            rejected(
                lambda invalid=invalid: invalid.capture(1),
                "unsupported mesh edit accepted",
            )

        # Actual native physics writeback, followed by one shared capture.
        # No second dynamics world: this caller advances the existing USD scene.
        import omni.physx
        from isaacsim.core.api import SimulationContext
        from pxr import PhysxSchema, UsdPhysics

        rigid = UsdPhysics.RigidBodyAPI.Apply(moving.GetPrim())
        rigid.CreateVelocityAttr(Gf.Vec3f(0.1, 0, 0))
        UsdPhysics.MassAPI.Apply(moving.GetPrim()).CreateMassAttr(1.0)
        PhysxSchema.PhysxRigidBodyAPI.Apply(moving.GetPrim()).CreateSleepThresholdAttr(
            0.0
        )
        sim = SimulationContext(
            physics_dt=0.001, rendering_dt=0.1, stage_units_in_meters=1.0
        )
        sim.get_physics_context().set_gravity(0.0)
        sim.initialize_physics()
        sim.play()
        sim.step(render=False)
        native_reference = Mid360Sensor(stage)
        native_a, native_b = make_hybrid(), make_hybrid()
        physics_history = SceneTransformHistory(stage, native_a.dynamic.paths, 21)
        physics_expected = []
        initial_x = None
        for step in range(4):
            sim.step(render=False)
            physics_x = omni.physx.get_physx_interface().get_rigidbody_transformation(
                "/World/Moving"
            )["position"][0]
            usd_x = (
                UsdGeom.XformCache()
                .GetLocalToWorldTransform(moving.GetPrim())
                .ExtractTranslation()[0]
            )
            assert abs(physics_x - usd_x) < 1e-5, "USD writeback unavailable"
            if initial_x is None:
                initial_x = physics_x
            snapshot = physics_history.capture(step)
            physics_expected.append(native_reference.scan(step * 200, 200, pose))
            native_a.scan(step * 200, 200, pose, scene_snapshot=snapshot)
            native_b.scan(step * 200, 200, pose, scene_snapshot=snapshot)
        assert physics_x > initial_x + 0.0002
        for consumer in (native_a, native_b):
            np.testing.assert_allclose(
                consumer.flush().ranges_m,
                np.concatenate([f.ranges_m for f in physics_expected]),
                rtol=2e-6,
                atol=2e-5,
            )
        sim.stop()

    report = dict(
        rigid_trace=args.rigid_trace,
        points=len(expected_ranges),
        moving_hits=moving_hits,
        moving_source_meshes=len(hybrid.dynamic.paths),
        max_range_difference_m=float(np.max(np.abs(actual.ranges_m - expected_ranges))),
        checks="union equivalence across multiple nested translating/rotating/reflected/scaled meshes and turning sensor; stride/fractions/time; near static and dynamic occlusion; owned outputs; static-change failure; rigid shared lifetime, identity, singular/removal/recreation and bounds; actual physics writeback shared by two sensors",
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
