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
args = parser.parse_args()
if not 100 <= args.steps <= 10000 or args.steps % 100:
    parser.error("steps must be a multiple of 100 in 100..10000")
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
    from mid360.isaac.extension import Extension, create_sensor, latest_frame

    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")
    box = UsdGeom.Cube.Define(stage, "/World/Enclosure")
    box.CreateSizeAttr(20.0)
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
        roots=("/World/Enclosure",),
        batch_steps=args.batch_steps,
    )
    sim.initialize_physics()
    sim.play()

    def run(sensing):
        extension = None
        if sensing:
            extension = Extension()
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
        publish_period_s=0.1,
        scope="moving mount, actual extension callbacks + physics + host arrays; no rendering/transport",
        baseline=run(False),
        sensor=run(True),
        checks="moving USD mount, callback counts, emission continuity, 100 ms aggregation",
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
