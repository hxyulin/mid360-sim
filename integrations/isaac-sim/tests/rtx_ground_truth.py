# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Replay MoCap on native RTX; preserve raw stamps, validity and endpoint poses."""

import argparse
from pathlib import Path
import json
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--physics-threads", type=int, choices=(0, 1, 8))
parser.add_argument("--input", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--render-steps", type=int, choices=(1, 10, 100), default=10)
parser.add_argument("--readback", choices=("poll", "writer"), default="writer")
parser.add_argument("--allow-incomplete", action="store_true")
parser.add_argument("--no-motion-bvh", action="store_true")
args = parser.parse_args()
args.output.parent.mkdir(parents=True, exist_ok=True)
from isaacsim import SimulationApp

app = SimulationApp(
    {
        "headless": True,
        "multi_gpu": False,
        "enable_motion_bvh": not args.no_motion_bvh,
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

    if args.physics_threads is not None:
        assert (
            carb.settings.get_settings().get("/persistent/physics/numThreads")
            == args.physics_threads
        )
    import omni.usd
    from pxr import Gf, UsdGeom
    from scipy.spatial.transform import Rotation, Slerp
    from isaacsim.core.api import SimulationContext
    from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data
    from rtx_profile import create_profile

    fixture = np.load(args.input)
    bp, bq = fixture["boundary_p"], fixture["boundary_q"]
    times = np.arange(2001) * 0.001
    rot = Slerp(times, Rotation.from_quat(bq))

    def pose(at):
        t = np.clip(np.asarray(at) - 1.0, 0.0, 2.0)
        p = np.column_stack(
            [np.interp(np.atleast_1d(t), times, bp[:, i]) for i in range(3)]
        )
        return p, rot(np.atleast_1d(t)).as_quat()

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
            size = [20.2] * 3
            size[axis] = 0.2
            center = [0.0] * 3
            center[axis] = sign * 10.1
            box.AddTranslateOp().Set(Gf.Vec3d(*center))
            box.AddScaleOp().Set(Gf.Vec3d(*size))
            box.CreateDoubleSidedAttr(True)
    lidar, sensor = create_profile(
        "rotary",
        np.zeros(32),
        np.linspace(-7.0, 52.0, 32),
        tick_rate=1000 / args.render_steps,
    )
    sim = SimulationContext(
        physics_dt=0.001,
        rendering_dt=args.render_steps * 0.001,
        stage_units_in_meters=1.0,
    )
    sim.initialize_physics()
    sim.play()

    def set_pose(at):
        p, q = pose(at)
        lidar.set_world_poses(positions=p, orientations=q[:, [3, 0, 1, 2]])

    set_pose(sim.current_time)
    records = []
    seen = set()

    def consume(gmo):
        stamp = int(gmo.timestampNs)
        if gmo.numElements and stamp not in seen:
            seen.add(stamp)
            if 1_000_000_000 <= stamp < 3_000_000_000:
                records.append(
                    dict(
                        timestamp_ns=stamp,
                        count=int(gmo.numElements),
                        x=np.asarray(gmo.x).copy(),
                        y=np.asarray(gmo.y).copy(),
                        z=np.asarray(gmo.z).copy(),
                        offsets=np.asarray(gmo.timeOffsetNs).copy(),
                        read_sim_ns=round(sim.current_time * 1e9),
                        start_ns=int(gmo.frameStart.timestampNs),
                        end_ns=int(gmo.frameEnd.timestampNs),
                    )
                )

    if args.readback == "writer":
        import omni.replicator.core as rep

        class CaptureWriter(rep.Writer):
            def __init__(self):
                self.data_structure = "renderProduct"
                self.annotators = [rep.annotators.get("GenericModelOutput")]

            def write(self, data):
                for payload in data.get("renderProducts", {}).values():
                    raw = payload.get("GenericModelOutput")
                    if isinstance(raw, dict):
                        raw = raw.get("data")
                    if raw is not None:
                        consume(parse_generic_model_output_data(raw))

        rep.WriterRegistry.register(CaptureWriter)
        sensor.attach_writer("CaptureWriter")
    render_wall = 0.0
    loop_start = time.perf_counter()
    for step in range(3400):
        sim.step(render=False)
        if (step + 1) % args.render_steps:
            continue
        set_pose(sim.current_time)
        before = time.perf_counter()
        sim.render()
        if args.readback == "poll":
            raw, _ = sensor.get_data("generic-model-output")
            if raw is not None:
                consume(parse_generic_model_output_data(raw))
        render_wall += time.perf_counter() - before
    loop_wall = time.perf_counter() - loop_start
    if not args.allow_incomplete:
        assert len(records) == 20, [(r["timestamp_ns"], r["count"]) for r in records]
    for r in records:
        assert r["count"] == 20000, r["count"]
        np.testing.assert_array_equal(np.sort(r["offsets"]), np.arange(20000) * 5000)
    if not args.allow_incomplete:
        np.testing.assert_array_equal(
            [r["timestamp_ns"] for r in records], np.arange(10, 30) * 100_000_000
        )
    arrays = {
        f"{key}{i}": r[key]
        for i, r in enumerate(records)
        for key in ("x", "y", "z", "offsets")
    }
    np.savez_compressed(args.output.with_suffix(".npz"), **arrays)
    report = dict(
        readback=args.readback,
        physics_threads=args.physics_threads,
        loop_wall_s=loop_wall,
        simulated_s=3.4,
        real_time_factor=3.4 / loop_wall,
        complete=len(records) == 20,
        expected_scans=20,
        missing_scan_stamps_ns=sorted(
            set((np.arange(10, 30) * 100_000_000).tolist())
            - {r["timestamp_ns"] for r in records}
        ),
        motion_bvh=not args.no_motion_bvh,
        render_step_ns=args.render_steps * 1_000_000,
        tick_rate_hz=1000 / args.render_steps,
        profile="custom repeating rotary 32 fixed elevations; 200k scheduled/s",
        source_metadata=json.loads(str(fixture["metadata"])),
        render_readback_wall_s=render_wall,
        records=[
            {k: v for k, v in r.items() if k not in ("x", "y", "z", "offsets")}
            for r in records
        ],
        scope="real measured motion in exact synthetic room; native RTX; no material noise; no physical accuracy claim",
    )
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print("RTX_MOCAP", args.render_steps, [r["count"] for r in records], flush=True)
except BaseException:
    import traceback

    args.output.with_suffix(".failure.txt").write_text(traceback.format_exc())
    raise
finally:
    app.close()
