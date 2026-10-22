#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Separate acquisition hold error from native intersection error on MoCap replay."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


reference = load("reference", ROOT / "scripts/check-backend-ground-truth.py")
comparison = load("comparison", ROOT / "scripts/compare-motion-ground-truth.py")


def metrics(x):
    x = np.asarray(x)
    if not len(x) or not np.isfinite(x).all():
        raise ValueError("invalid errors")
    return dict(
        count=len(x),
        rms_mm=float(np.sqrt(np.mean(x * x)) * 1000),
        p95_mm=float(np.percentile(abs(x), 95) * 1000),
        max_mm=float(np.max(abs(x)) * 1000),
    )


def run(args):
    fixture = np.load(args.fixture)
    metadata = json.loads(str(fixture["metadata"]))
    assert hashlib.sha256(args.gt.read_bytes()).hexdigest() == metadata["source_sha256"]
    gt = comparison.Trajectory(args.gt)
    start = metadata["start_relative_s"]
    shift = np.array(metadata["translation_shift_m"])

    def pose(t):
        p, r = gt.pose(start + np.asarray(t))
        return p + shift, r

    d = np.concatenate([reference.directions(i, 200000) for i in (0, 200000)])
    t = np.arange(400000) * 5e-6
    p, r = pose(t)
    expected = fixture["reference"]
    index = (gt.t[:-1] >= start) & (gt.t[1:] <= start + 2)
    dt = np.diff(gt.t)[index]
    omega = (gt.r[:-1].inv() * gt.r[1:]).magnitude()[index] / dt
    speed = np.linalg.norm(np.diff(gt.p, axis=0)[index], axis=1) / dt
    policies = {}
    for step_us in (1000, 200, 50):
        step_ns = step_us * 1000
        point_ns = np.arange(400000) * 5000
        base = (point_ns // step_ns) * step_ns * 1e-9
        for label, offset in (("start", 0.0), ("midpoint", 0.5), ("end", 1.0)):
            hp, hr = pose(base + step_us * 1e-6 * offset)
            predicted = reference.ranges(hp, hr.as_quat(), d)
            policies[f"hold_{step_us}us_{label}"] = metrics(predicted - expected)
    # Attribute rotation/translation by holding one part at step end, retaining
    # exact per-emission reference for the other. Contributions need not add.
    end = ((np.arange(400000) // 200) + 1) * 0.001
    ep, er = pose(end)
    attribution = dict(
        rotation_hold_only=metrics(reference.ranges(p, er.as_quat(), d) - expected),
        translation_hold_only=metrics(reference.ranges(ep, r.as_quat(), d) - expected),
    )
    # Compare long-range and glancing rays with the same 1 ms endpoint hold.
    full_error = fixture["hold_end"] - expected
    world = r.apply(d)
    hit = p + world * expected[:, None]
    axis = np.argmax(abs(hit), axis=1)
    incidence = np.abs(world[np.arange(len(world)), axis])
    subsets = {}
    for name, mask in [
        ("range_under_12m", expected < 12),
        ("range_12m_plus", expected >= 12),
        ("incidence_cos_under_0.7", incidence < 0.7),
        ("incidence_cos_0.7_plus", incidence >= 0.7),
    ]:
        subsets[name] = metrics(full_error[mask])
    rtx = []
    for path in args.rtx:
        report = json.loads(path.read_text())
        raw = np.load(path.with_suffix(".npz"))
        errors = []
        frame_errors = []
        clock_checks = {str(us): [] for us in (-1000, -500, 0, 500, 1000)}
        policy_checks = {name: [] for name in ("1ms_start", "1ms_midpoint", "1ms_end")}
        invalid = 0
        for i, record in enumerate(report["records"]):
            stamp = record["timestamp_ns"]
            at = (stamp + raw[f"offsets{i}"].astype(np.int64)) * 1e-9 - 1.0
            a, e = np.radians(raw[f"x{i}"].astype(float)), np.radians(
                raw[f"y{i}"].astype(float)
            )
            local = np.column_stack(
                [np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)]
            )
            actual = raw[f"z{i}"].astype(float)
            valid = np.isfinite(actual) & (actual > 0)
            invalid += int(np.sum(~valid))
            pp, rr = pose(at)
            target = reference.ranges(pp, rr.as_quat(), local)
            err = actual[valid] - target[valid]
            errors.extend(err.tolist())
            frame_errors.append(metrics(err))
            for us, values in clock_checks.items():
                pp, rr = pose(np.clip(at + int(us) * 1e-6, 0.0, 2.0))
                values.extend(
                    (
                        actual[valid] - reference.ranges(pp, rr.as_quat(), local)[valid]
                    ).tolist()
                )
            # Diagnostic alternatives only; never rewrite acquisition stamps.
            for name, offset in (
                ("1ms_start", 0.0),
                ("1ms_midpoint", 0.0005),
                ("1ms_end", 0.001),
            ):
                query = (
                    np.floor(
                        (stamp + raw[f"offsets{i}"].astype(np.int64) - 1_000_000_000)
                        / 1_000_000
                    )
                    * 0.001
                    + offset
                )
                pp, rr = pose(np.clip(query, 0.0, 2.0))
                policy_checks[name].extend(
                    (
                        actual[valid] - reference.ranges(pp, rr.as_quat(), local)[valid]
                    ).tolist()
                )
        rtx.append(
            dict(
                file=path.name,
                motion_bvh=report["motion_bvh"],
                render_step_ns=report["render_step_ns"],
                tick_rate_hz=report["tick_rate_hz"],
                complete=report.get("complete", len(report["records"]) == 20),
                scheduled=sum(v["count"] for v in report["records"]),
                valid=sum(v["count"] for v in report["records"]) - invalid,
                invalid=invalid,
                error=metrics(errors),
                per_scan=frame_errors,
                clock_sensitivity_us={k: metrics(v) for k, v in clock_checks.items()},
                held_pose_agreement={k: metrics(v) for k, v in policy_checks.items()},
                readback_age_ms=[
                    (v["read_sim_ns"] - v["timestamp_ns"] - 100000000) / 1e6
                    for v in report["records"]
                ],
            )
        )
    output = dict(
        metadata=metadata,
        policy_errors=policies,
        attribution=attribution,
        subsets=subsets,
        speed_m_s=dict(
            median=float(np.median(speed)),
            p95=float(np.percentile(speed, 95)),
            max=float(max(speed)),
        ),
        angular_deg_s=dict(
            median=float(np.degrees(np.median(omega))),
            p95=float(np.degrees(np.percentile(omega, 95))),
            max=float(np.degrees(max(omega))),
        ),
        rtx=rtx,
    )
    args.output.write_text(json.dumps(output, indent=2, allow_nan=False) + "\n")
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--gt", type=Path, required=True)
    p.add_argument("--fixture", type=Path, required=True)
    p.add_argument("--rtx", type=Path, nargs="*", default=[])
    p.add_argument("--output", type=Path, required=True)
    run(p.parse_args())
