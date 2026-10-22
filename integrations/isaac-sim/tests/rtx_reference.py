# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Compare native RTX and Warp acquisition; run with Isaac python.sh.

The default custom rotary profile matches MID-360 rate, range and field of
view, but uses 32 fixed elevation channels; it is not a NVIDIA/Livox preset.
The experimental nominal profile repeats the first 100 ms of our pattern as
20,000 virtual solid-state emitters and can initialize very slowly.
Noise is disabled to isolate acquisition. --backend warp uses our ordinary
nonrepeating optics, with static segmented tracing in the identical enclosure.
"""
import argparse
import json
from pathlib import Path
import sys
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--backend", choices=("rtx", "warp"), default="rtx")
parser.add_argument("--motion-bvh", action="store_true")
parser.add_argument("--moving", action="store_true")
parser.add_argument("--profile", choices=("rotary", "nominal"), default="rotary")
parser.add_argument("--frames", type=int, default=20)
args = parser.parse_args()
if not 1 <= args.frames <= 100:
    parser.error("frames must be in 1..100")
args.output.parent.mkdir(parents=True, exist_ok=True)
from isaacsim import SimulationApp

app = SimulationApp(
    {"headless": True, "multi_gpu": False, "enable_motion_bvh": args.motion_bvh}
)
try:
    import numpy as np
    import omni.usd
    from pxr import Gf, UsdGeom
    from isaacsim.core.api import SimulationContext
    from isaacsim.sensors.experimental.rtx import (
        parse_generic_model_output_data,
        SUPPORTED_LIDAR_CONFIGS,
    )

    # Share the existing pattern implementation, not a second copy of optics.
    sys.path.insert(
        0, str(Path(__file__).resolve().parents[1] / "exts" / "hxyulin.mid360")
    )
    from mid360.isaac.backend import Mid360Sensor, StaticSceneBatch

    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")
    # Explicit outward boxes enclose the mount; use two-sided visible surfaces.
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
    reference = Mid360Sensor(stage)
    f = reference.scan(0, 20000, ((0, 0, 0), (0, 0, 0, 1)))
    d = f.directions
    az = -np.degrees(np.arctan2(d[:, 1], d[:, 0]))
    el = np.degrees(np.arcsin(d[:, 2]))
    n = 20000 if args.profile == "nominal" else 32
    if args.profile == "rotary":
        az, el = np.zeros(n), np.linspace(-7.0, 52.0, n)
    if args.backend == "rtx":
        from rtx_profile import create_profile

        lidar, sensor = create_profile(args.profile, az, el)
    else:
        sensor = StaticSceneBatch(reference)
    # Lidar's standard authored transform operations are mutated in place.
    sim = SimulationContext(
        physics_dt=0.001, rendering_dt=0.1, stage_units_in_meters=1.0
    )
    sim.initialize_physics()
    sim.play()
    velocity = 1.0 if args.moving else 0.0
    omega = 0.5 if args.moving else 0.0
    samples, durations, complete_durations = [], [], []

    def set_pose(t):
        if args.backend != "rtx":
            return
        lidar.set_world_poses(
            positions=np.array([[velocity * t, 0.0, 0.8]]),
            orientations=np.array(
                [[np.cos(omega * t / 2), 0.0, 0.0, np.sin(omega * t / 2)]]
            ),
        )

    acquisition_epoch_ns = round(sim.current_time * 1e9)
    print("RTX_READY", args.profile, flush=True)
    # Render warmup establishes both motion BVH endpoints before measuring.
    for i in range(10 + args.frames):
        full_start = time.perf_counter()
        for step in range(100):
            t = sim.current_time
            sim.step(render=False)
            if args.backend == "warp":

                def pose(at):
                    return (
                        (velocity * at, 0.0, 0.8),
                        (0.0, 0.0, np.sin(omega * at / 2), np.cos(omega * at / 2)),
                    )

                sensor.scan(
                    i * 20000 + step * 200,
                    200,
                    pose(t),
                    pose(sim.current_time),
                    epoch_ns=acquisition_epoch_ns,
                    pose_fraction=(0.0, 0.995),
                )
        set_pose(sim.current_time)
        before = time.perf_counter()
        sim.render()
        if args.backend == "warp":
            frame = sensor.flush()
            sample = dict(
                timestamp_ns=int(frame.timestamps_ns[0]),
                count=20000,
                x=np.degrees(
                    np.arctan2(frame.directions[:, 1], frame.directions[:, 0])
                ),
                y=np.degrees(np.arcsin(frame.directions[:, 2])),
                z=frame.ranges_m.copy(),
                emitter=(frame.indices % 4).astype(np.uint32),
                offsets=(frame.timestamps_ns - frame.timestamps_ns[0]).astype(
                    np.uint32
                ),
                frame_start=None,
                frame_end=None,
            )
            sample["read_sim_time_ns"] = round(sim.current_time * 1e9)
            if i >= 10:
                samples.append(sample)
        else:
            raw, info = sensor.get_data("generic-model-output")
            if raw is not None:
                gmo = parse_generic_model_output_data(raw)
                if gmo.numElements:
                    sample = dict(
                        timestamp_ns=int(gmo.timestampNs),
                        count=int(gmo.numElements),
                        x=np.asarray(gmo.x).copy(),
                        y=np.asarray(gmo.y).copy(),
                        z=np.asarray(gmo.z).copy(),
                        emitter=np.asarray(gmo.emitterId).copy(),
                        offsets=np.asarray(gmo.timeOffsetNs).copy(),
                        frame_start=dict(
                            timestamp_ns=int(gmo.frameStart.timestampNs),
                            position=np.asarray(gmo.frameStart.posM).tolist(),
                            orientation=np.asarray(gmo.frameStart.orientation).tolist(),
                        ),
                        frame_end=dict(
                            timestamp_ns=int(gmo.frameEnd.timestampNs),
                            position=np.asarray(gmo.frameEnd.posM).tolist(),
                            orientation=np.asarray(gmo.frameEnd.orientation).tolist(),
                        ),
                    )
                    sample["read_sim_time_ns"] = round(sim.current_time * 1e9)
                    if i >= 10:
                        samples.append(sample)
        print("RTX_FRAME", i, samples[-1]["count"] if samples else 0, flush=True)
        if i >= 10:
            durations.append(time.perf_counter() - before)
            complete_durations.append(time.perf_counter() - full_start)
    assert len(samples) == args.frames, (len(samples), args.frames)
    assert all(s["count"] == 20000 for s in samples)
    np.testing.assert_array_equal(
        np.diff([s["timestamp_ns"] for s in samples]),
        np.full(len(samples) - 1, 100_000_000),
    )
    for sample in samples:
        np.testing.assert_array_equal(
            np.sort(sample["offsets"]), np.arange(20000) * 5000
        )
    errors = []
    frozen_errors = []
    for sample in samples:
        a, e = np.radians(sample["x"].astype(float)), np.radians(
            sample["y"].astype(float)
        )
        local = np.column_stack(
            [np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)]
        )
        for frozen, destination in ((False, errors), (True, frozen_errors)):
            t = sample["timestamp_ns"] * 1e-9 + (
                0 if frozen else sample["offsets"] * 1e-9
            )
            yaw = omega * t
            world = np.column_stack(
                [
                    np.cos(yaw) * local[:, 0] - np.sin(yaw) * local[:, 1],
                    np.sin(yaw) * local[:, 0] + np.cos(yaw) * local[:, 1],
                    local[:, 2],
                ]
            )
            origin = np.zeros_like(world)
            origin[:, 0] = velocity * t
            origin[:, 2] = 0.8
            with np.errstate(divide="ignore"):
                distances = (np.where(world > 0, 10.0, -10.0) - origin) / world
                distances[np.abs(world) < 1e-12] = np.inf
                expected = np.min(distances, axis=1)
            keep = np.isfinite(sample["z"]) & (sample["z"] > 0) & np.isfinite(expected)
            destination.extend((sample["z"][keep] - expected[keep]).tolist())
    assert errors and np.isfinite(errors).all()
    # Save raw GMO coordinates; GMO x/y/z may be spherical according to flags.
    np.savez_compressed(
        args.output.with_suffix(".npz"),
        **{
            f"{k}{i}": s[k]
            for i, s in enumerate(samples)
            for k in ("x", "y", "z", "offsets", "emitter")
        },
    )
    report = dict(
        acquisition_epoch_ns=acquisition_epoch_ns,
        backend=args.backend,
        motion_bvh=args.motion_bvh,
        moving=args.moving,
        profile="nonrepeating-nominal" if args.backend == "warp" else args.profile,
        manufacturer_preset=False,
        shipped_livox_configs=[
            k
            for k in SUPPORTED_LIDAR_CONFIGS
            if "livox" in k.lower() or "mid360" in k.lower()
        ],
        frames=len(samples),
        frame_start=samples[0]["frame_start"],
        frame_end=samples[0]["frame_end"],
        range_rms_m=float(np.sqrt(np.mean(np.square(errors)))),
        frozen_pose_range_rms_m=float(np.sqrt(np.mean(np.square(frozen_errors)))),
        completed_scan_readback_lag_ms=[
            (s["read_sim_time_ns"] - s["timestamp_ns"] - 100_000_000) / 1e6
            for s in samples
        ],
        complete_wall_s=float(sum(complete_durations)),
        real_time_factor=args.frames * 0.1 / sum(complete_durations),
        counts=[s["count"] for s in samples],
        valid_counts=[
            int(np.count_nonzero(np.isfinite(s["z"]) & (s["z"] > 0))) for s in samples
        ],
        timestamps_ns=[s["timestamp_ns"] for s in samples],
        offsets_min_max_ns=[
            int(samples[0]["offsets"].min()),
            int(samples[0]["offsets"].max()),
        ],
        gmo_coordinate_type="CoordsType.SPHERICAL",
        gmo_frame="FrameOfReference.SENSOR",
        raw_examples={
            k: samples[0][k][:5].tolist() for k in ("x", "y", "z", "offsets", "emitter")
        },
        coordinate_ranges=[
            [float(samples[0][k].min()), float(samples[0][k].max())]
            for k in ("x", "y", "z")
        ],
        render_readback_s=float(sum(durations)),
        render_readback_p95_ms=float(np.percentile(durations, 95) * 1000),
        scope="100 ms render intervals, 1 ms physics; total includes physics/render/owned host GMO; no transport",
        checks="20,000 scheduled ranges/frame; valid counts; offsets; analytic continuous-pose enclosure comparison",
    )
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print("MID360_RTX " + json.dumps(report), flush=True)
except BaseException:
    import traceback

    args.output.with_suffix(".failure.txt").write_text(traceback.format_exc())
    raise
finally:
    app.close()
