# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Query independent MoCap replay fixture through actual Warp paths."""

import argparse
from pathlib import Path
import sys
import json
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--repeats", type=int, choices=(1, 3), default=1)
parser.add_argument(
    "--modes",
    nargs="+",
    choices=("ordinary", "segmented", "history"),
    default=["ordinary", "segmented", "history"],
)
parser.add_argument("--no-usd-writeback", action="store_true")
parser.add_argument("--input", type=Path, required=True)
parser.add_argument("--physics-threads", type=int, choices=(0, 1, 4, 8, 32))
parser.add_argument(
    "--physics-driver", choices=("context", "manager"), default="context"
)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
from isaacsim import SimulationApp

extra_args = []
if args.physics_threads is not None:
    extra_args.append(f"--/persistent/physics/numThreads={args.physics_threads}")
if args.no_usd_writeback:
    extra_args.append("--/physics/updateToUsd=false")
app = SimulationApp({"headless": True, "multi_gpu": False, "extra_args": extra_args})

try:
    import numpy as np
    import carb

    if args.physics_threads is not None:
        assert (
            carb.settings.get_settings().get("/persistent/physics/numThreads")
            == args.physics_threads
        )
    if args.no_usd_writeback:
        assert not carb.settings.get_settings().get("/physics/updateToUsd")
    import omni.usd
    from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
    import omni.physx
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.api import SimulationContext

    sys.path.insert(
        0, str(Path(__file__).resolve().parents[1] / "exts" / "hxyulin.mid360")
    )
    from mid360.isaac.backend import Mid360Sensor, StaticSceneBatch

    fixture = np.load(args.input)
    bp, bq = fixture["boundary_p"], fixture["boundary_q"]
    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")
    for axis in range(3):
        for sign in (-1, 1):
            box = UsdGeom.Cube.Define(stage, f"/World/Wall{axis}_{sign+1}")
            box.CreateSizeAttr(1.0)
            half = [20.2] * 3
            half[axis] = 0.2
            p = [0.0] * 3
            p[axis] = sign * 10.1
            box.AddTranslateOp().Set(Gf.Vec3d(*p))
            box.AddScaleOp().Set(Gf.Vec3d(*half))
            box.CreateDoubleSidedAttr(True)
    probe = UsdGeom.Xform.Define(stage, "/Probe")
    probe.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.8))
    rigid = UsdPhysics.RigidBodyAPI.Apply(probe.GetPrim())
    rigid.CreateVelocityAttr(Gf.Vec3f(0.1, 0.0, 0.0))
    UsdPhysics.MassAPI.Apply(probe.GetPrim()).CreateMassAttr(1.0)
    PhysxSchema.PhysxRigidBodyAPI.Apply(probe.GetPrim()).CreateSleepThresholdAttr(0.0)
    sim = SimulationContext(
        physics_dt=0.001, rendering_dt=0.1, stage_units_in_meters=1.0
    )
    sim.get_physics_context().set_gravity(0.0)
    sim.initialize_physics()
    sim.play()
    # Verify bulk history ownership, indexing, and validation before timings.
    test = StaticSceneBatch(Mid360Sensor(stage))
    poses = np.column_stack([bp[:3], bq[:3]])
    counts = np.array([3, 5], dtype=np.int32)
    expected = StaticSceneBatch(Mid360Sensor(stage))
    for first, count, k in ((7, 3, 0), (16, 5, 1)):
        expected.scan(
            first,
            count,
            (bp[k], bq[k]),
            (bp[k + 1], bq[k + 1]),
            stride=3,
            pose_fraction=(0.0, (count - 1) / count),
        )
    golden = expected.flush()
    test.scan_history(7, counts, poses, stride=3)
    poses[:] = np.nan
    counts[:] = 99
    try:
        test.scan(31, 1, (bp[0], bq[0]))
        raise AssertionError("bulk history must require flush")
    except ValueError:
        pass
    actual = test.flush()
    np.testing.assert_array_equal(actual.indices, golden.indices)
    np.testing.assert_allclose(actual.ranges_m, golden.ranges_m, rtol=0, atol=1e-5)
    for bad in ([0], [1.5], np.full(101, 1)):
        try:
            test.scan_history(0, bad, np.column_stack([bp[:2], bq[:2]]))
            raise AssertionError("bad history accepted")
        except ValueError:
            pass
    timings = {}
    for repeat, mode in ((r, m) for r in range(args.repeats) for m in args.modes):
        sensor = Mid360Sensor(stage)
        if mode in ("segmented", "history"):
            sensor = StaticSceneBatch(sensor)
        parts = []
        sink = args.output.with_name(args.output.stem + "-" + mode + ".xyz.bin").open(
            "wb"
        )
        sensor_s = 0.0
        probe_before = omni.physx.get_physx_interface().get_rigidbody_transformation(
            "/Probe"
        )["position"][0]
        start = time.perf_counter()
        for i in range(2000):
            if args.physics_driver == "context":
                sim.step(render=False)
            else:
                SimulationManager.step()

            def pose(k):
                return (bp[k], bq[k])

            before = time.perf_counter()
            if mode != "history":
                sensor.scan(
                    i * 200,
                    200,
                    pose(i),
                    pose(i + 1),
                    defer_readback=True,
                    pose_fraction=(0.0, 0.995),
                )
            if (i + 1) % 100 == 0:
                if mode == "history":
                    first = i - 99
                    sensor.scan_history(
                        first * 200,
                        np.full(100, 200, dtype=np.int32),
                        np.column_stack([bp[first : i + 2], bq[first : i + 2]]),
                    )
                frame = sensor.flush()
                np.testing.assert_array_equal(
                    frame.indices, np.arange(i * 200 - 19800, (i + 1) * 200)
                )
                assert frame.valid.all()
                parts.append(frame.ranges_m.copy())
                sink.write(
                    np.asarray(
                        frame.directions * frame.ranges_m[:, None], dtype="<f4"
                    ).tobytes()
                )
            sensor_s += time.perf_counter() - before
        elapsed = time.perf_counter() - start
        sink.close()
        probe_after = omni.physx.get_physx_interface().get_rigidbody_transformation(
            "/Probe"
        )["position"][0]
        np.testing.assert_allclose(probe_after - probe_before, 0.2, rtol=0, atol=1e-4)
        timing_key = mode if args.repeats == 1 else f"{mode}-{repeat}"
        timings[timing_key] = dict(
            usd_writeback=not args.no_usd_writeback,
            physics_driver=args.physics_driver,
            physics_threads=args.physics_threads,
            probe_displacement_m=probe_after - probe_before,
            wall_s=elapsed,
            sensor_s=sensor_s,
            simulated_s=2.0,
            real_time_factor=2.0 / elapsed,
            scope="physics steps + native queries + owned XYZ sink; sensor_s excludes physics; no renderer/ROS",
        )
        np.save(
            args.output.with_name(args.output.stem + "-" + mode + ".npy"),
            np.concatenate(parts),
        )
        print("GT_REPLAY", mode, 400000, timings[timing_key], flush=True)
    args.output.with_suffix(".timing.json").write_text(
        json.dumps(timings, indent=2) + "\n"
    )
except BaseException:
    import traceback

    args.output.with_suffix(".failure.txt").write_text(traceback.format_exc())
    raise
finally:
    app.close()
