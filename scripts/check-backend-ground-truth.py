#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Replay independent MoCap motion in a known box; no real-scene accuracy claim."""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "integrations/python"))
from mid360_sim.pattern import directions


def ranges(p, q, rays):
    """Independent double-precision ray/box reference, inner faces at +/-10 m."""
    from scipy.spatial.transform import Rotation

    world = Rotation.from_quat(q).apply(rays)
    with np.errstate(divide="ignore", invalid="ignore"):
        distance = (np.where(world > 0, 10.0, -10.0) - p) / world
    distance[np.abs(world) < 1e-14] = np.inf
    return np.min(distance, axis=1)


def prepare(args):
    from scipy.spatial.transform import Rotation

    spec = importlib.util.spec_from_file_location(
        "comparison", ROOT / "scripts/compare-motion-ground-truth.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    gt = module.Trajectory(args.gt)
    # Select by motion only, never by any backend's error. Reject GT gaps >20 ms.
    candidates = []
    for start in np.arange(0.1, gt.t[-1] - 2.1, 0.05):
        if not gt.covered(start, start + 2):
            continue
        t = start + np.arange(201) * 0.01
        _, r = gt.pose(t)
        score = np.mean((r[:-1].inv() * r[1:]).magnitude() ** 2)
        candidates.append((score, start))
    if not candidates:
        raise ValueError("no gap-free two-second trajectory")
    _, start = max(candidates)
    t = start + np.arange(400000) * 5e-6
    p, r = gt.pose(t)
    rays_per_step = args.step_us // 5
    steps = 400000 // rays_per_step
    step_ns = args.step_us * 1000
    bp, br = gt.pose(start + np.arange(steps + 1) * (step_ns * 1e-9))
    origin = bp[0].copy()
    p += np.array([0.0, 0.0, 0.8]) - origin
    bp += np.array([0.0, 0.0, 0.8]) - origin
    if np.max(abs(p)) >= 9:
        raise ValueError("trajectory leaves reference enclosure")
    d = np.concatenate([directions(i, 200000) for i in (0, 200000)])
    reference = ranges(p, r.as_quat(), d)
    index = np.arange(400000) // rays_per_step
    a, b = bp[index], bp[index + 1]
    qa, qb = br[index], br[index + 1]
    fraction = (np.arange(400000) % rays_per_step) / rays_per_step
    interp_p = a + (b - a) * fraction[:, None]
    interp_r = qa * Rotation.from_rotvec(
        (qa.inv() * qb).as_rotvec() * fraction[:, None]
    )
    metadata = dict(
        source="JEPLO EIL_Mask2 independent 100 Hz Qualisys LiDAR-frame poses",
        source_sha256=hashlib.sha256(args.gt.read_bytes()).hexdigest(),
        start_relative_s=float(start),
        duration_s=2.0,
        rays=400000,
        step_ns=step_ns,
        translation_shift_m=(np.array([0.0, 0.0, 0.8]) - origin).tolist(),
        gaps_over_20ms=0,
        max_gt_gap_s=float(
            np.max(np.diff(gt.t)[(gt.t[:-1] >= start) & (gt.t[1:] <= start + 2)])
        ),
        scope="real measured motion, synthetic exact enclosure, nominal optics; not real scene/range validation",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.output,
        boundary_p=bp,
        boundary_q=br.as_quat(),
        reference=reference,
        hold_start=ranges(a, qa.as_quat(), d),
        hold_end=ranges(b, qb.as_quat(), d),
        interpolated=ranges(interp_p, interp_r.as_quat(), d),
        metadata=json.dumps(metadata),
    )
    np.savetxt(
        args.output.with_suffix(".csv"),
        np.column_stack([np.arange(steps + 1) * step_ns, bp, br.as_quat()]),
        fmt=["%d"] + ["%.17g"] * 7,
    )
    args.output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata, indent=2))


def mujoco_run(args):
    import mujoco
    from mid360_sim.mujoco import MujocoSensor

    fixture = np.load(args.input)
    boxes = []
    for axis in range(3):
        for sign in (-1, 1):
            p = [0.0] * 3
            p[axis] = sign * 10.1
            half = [10.1] * 3
            half[axis] = 0.1
            boxes.append(
                f'<geom type="box" pos="{" ".join(map(str,p))}" size="{" ".join(map(str,half))}"/>'
            )
    model = mujoco.MjModel.from_xml_string(
        "<mujoco><worldbody>"
        + "".join(boxes)
        + '<body mocap="true"><site name="lidar"/></body></worldbody></mujoco>'
    )
    data = mujoco.MjData(model)
    sensor = MujocoSensor(model, data, "lidar")
    output = []
    bp, bq = fixture["boundary_p"], fixture["boundary_q"]
    rays_per_step = json.loads(str(fixture["metadata"]))["step_ns"] // 5000
    before = time.perf_counter()
    for i in range(400000 // rays_per_step):
        data.mocap_pos[0] = bp[i + 1]
        q = bq[i + 1]
        data.mocap_quat[0] = q[[3, 0, 1, 2]]
        mujoco.mj_forward(model, data)
        if args.rolling:
            frame = sensor.scan_rolling(
                i * rays_per_step,
                rays_per_step,
                (bp[i], bq[i]),
                (bp[i + 1], bq[i + 1]),
                origin_tolerance_m=args.origin_tolerance_m,
                pose_fraction=(0.0, (rays_per_step - 1) / rays_per_step),
            )
        else:
            frame = sensor.scan(i * rays_per_step, rays_per_step)
        output.append(frame.ranges_m)
    elapsed = time.perf_counter() - before
    np.save(args.output, np.concatenate(output))
    timing = dict(
        wall_s=elapsed,
        simulated_s=2.0,
        real_time_factor=2.0 / elapsed,
        rolling=args.rolling,
        origin_tolerance_m=args.origin_tolerance_m,
        step_ns=json.loads(str(fixture["metadata"]))["step_ns"],
        mujoco=mujoco.__version__,
        scope="prescribed poses, native queries and owned arrays; no dynamics integration, rendering or transport",
    )
    args.output.with_suffix(".timing.json").write_text(
        json.dumps(timing, indent=2) + "\n"
    )
    print(json.dumps(timing))


def evaluate(args):
    fixture = np.load(args.input)
    if args.backend == "gazebo":
        points = np.fromfile(
            args.result,
            dtype=np.dtype(
                [("xyz", "<f4", (3,)), ("offset", "<u4"), ("meta", "u1", (4,))]
            ),
        )
        measured = np.linalg.norm(points["xyz"].astype(float), axis=1)
        np.testing.assert_array_equal(
            points["offset"], (np.arange(len(measured)) % 20000) * 5000
        )
        np.testing.assert_array_equal(
            points["meta"][:, 2], np.arange(len(measured)) % 4
        )
    else:
        measured = np.load(args.result)
    if (
        measured.shape != (400000,)
        or not np.isfinite(measured).all()
        or not (measured > 0).all()
    ):
        raise ValueError("incomplete/nonfinite acquisition")

    def metrics(error):
        return dict(
            rms_m=float(np.sqrt(np.mean(error**2))),
            p95_m=float(np.percentile(abs(error), 95)),
            max_m=float(np.max(abs(error))),
        )

    report = dict(
        backend=args.backend,
        metadata=json.loads(str(fixture["metadata"])),
        points=len(measured),
        actual_reference=metrics(measured - fixture["reference"]),
        numerical_policy_agreement={
            key: metrics(measured - fixture[key])
            for key in ("hold_start", "hold_end", "interpolated")
        },
    )
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)
    a = sub.add_parser("prepare")
    a.add_argument("--gt", type=Path, required=True)
    a.add_argument("--output", type=Path, required=True)
    a.add_argument("--step-us", type=int, choices=(50, 200, 1000), default=1000)
    a.set_defaults(run=prepare)
    a = sub.add_parser("mujoco")
    a.add_argument("--input", type=Path, required=True)
    a.add_argument("--output", type=Path, required=True)
    a.add_argument("--rolling", action="store_true")
    a.add_argument("--origin-tolerance-m", type=float, default=0.0005)
    a.set_defaults(run=mujoco_run)
    a = sub.add_parser("evaluate")
    a.add_argument("--input", type=Path, required=True)
    a.add_argument("--result", type=Path, required=True)
    a.add_argument("--backend", choices=("mujoco", "gazebo", "isaac"), required=True)
    a.add_argument("--output", type=Path, required=True)
    a.set_defaults(run=evaluate)
    args = p.parse_args()
    args.run(args)
