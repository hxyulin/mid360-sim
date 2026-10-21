#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Diagnose the recorded 128 Hz motion-policy error and cheap causal curvature.

Exploratory analysis of the existing evaluation points, not a runtime change or
new independent validation. Uses one earlier pose and the existing interval
endpoints. No future pose beyond the interval end, retiming or optical fitting.
"""

import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

spec = importlib.util.spec_from_file_location(
    "comparison", Path(__file__).with_name("compare-motion-ground-truth.py")
)
comparison = importlib.util.module_from_spec(spec)
spec.loader.exec_module(comparison)


def rotation_policy(previous, start, end, fraction, weight=0.0, guard=False):
    """Blend SLERP with three-pose quadratic curvature in start-local log space.

    Poses are uniformly spaced; fraction is in [0,1]. This low-angle local
    approximation can overshoot. Optional guard rejects nonforward curvature
    when previous and current angular increments do not align positively.
    """
    forward = (start.inv() * end).as_rotvec()
    backward = (start.inv() * previous).as_rotvec()
    f = np.asarray(fraction)[:, None]
    amount = np.full(len(fraction), weight)
    if guard:
        amount[np.einsum("ni,ni->n", -backward, forward) <= 0] = 0
    correction = 0.5 * amount[:, None] * (backward + forward) * f * (f - 1)
    return start * Rotation.from_rotvec(forward * f + correction)


def stats(error):
    """Metre statistics compatible with the original comparison."""
    return comparison.metrics(error)


def controlled():
    """Fixed-axis acceleration and an exact stop expose benefit and overshoot."""
    step = 1 / 128
    t = np.linspace(0, step, 2001)
    f = t / step
    omega, accel = np.pi, 4 * np.pi
    point = np.tile([10.0, 0.0, 0.0], (len(t), 1))
    out = {}
    for name, theta in [
        ("smooth_acceleration", lambda t: omega * t + 0.5 * accel * t**2),
        ("stop_at_boundary", lambda t: omega * np.minimum(t, 0)),
    ]:
        knots = Rotation.from_euler("z", theta(np.array([-step, 0, step])))
        previous, start, end = [
            Rotation.from_quat(np.tile(knots[i].as_quat(), (len(t), 1)))
            for i in range(3)
        ]
        truth = Rotation.from_euler("z", theta(t)).apply(point)
        out[name] = {}
        for label, weight, guard in [
            ("linear", 0, False),
            ("half_curvature", 0.5, False),
            ("guarded_half_curvature", 0.5, True),
        ]:
            r = rotation_policy(previous, start, end, f, weight, guard)
            out[name][label] = stats(np.linalg.norm(r.apply(point) - truth, axis=1))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("evaluation", type=Path)
    ap.add_argument("trajectory", type=Path)
    ap.add_argument("--output", required=True, type=Path)
    args = ap.parse_args()
    if args.output.exists():
        raise ValueError("output must be new")
    data = np.load(args.evaluation)
    gt = comparison.Trajectory(args.trajectory)
    t, x, ref = data["times"], data["local"], data["reference"]
    step = 1 / 128
    a = np.floor(t / step) * step
    f = (t - a) / step
    p0, r0 = gt.pose(a)
    p1, r1 = gt.pose(a + step)
    _, rm = gt.pose(a - step)
    position = p0 + (p1 - p0) * f[:, None]
    baseline_rotation = rotation_policy(rm, r0, r1, f)
    baseline_error = np.linalg.norm(baseline_rotation.apply(x) + position - ref, axis=1)
    truth_p, truth_r = gt.pose(t)
    high = data["high"]
    held = data["heldout"]
    next_knot = np.searchsorted(gt.t, a, side="right")
    crossing = gt.t[next_knot] < (a + step)
    report = {
        "scope": "Exploratory reuse of previous held-out samples, not new independent validation or native simulator execution.",
        "interval_hz": 128,
        "higher_motion_points": int(high.sum()),
        "decomposition_higher_motion_m": {
            "translation_only": stats(
                np.linalg.norm(truth_r.apply(x) + position - ref, axis=1)[high]
            ),
            "rotation_only": stats(
                np.linalg.norm(baseline_rotation.apply(x) + truth_p - ref, axis=1)[high]
            ),
        },
        "reference_knots": {
            "crossing_point_fraction": float(np.mean(crossing[high])),
            "squared_error_fraction_in_crossing_intervals": float(
                np.sum(baseline_error[high & crossing] ** 2)
                / np.sum(baseline_error[high] ** 2)
            ),
        },
        "candidates": {},
        "range_bins": {},
        "controlled": controlled(),
    }
    for name, weight, guard in [
        ("linear", 0, False),
        ("half_rotation_curvature", 0.5, False),
        ("full_rotation_curvature", 1, False),
        ("guarded_half_rotation_curvature", 0.5, True),
    ]:
        r = rotation_policy(rm, r0, r1, f, weight, guard)
        world = r.apply(x) + position
        error = np.linalg.norm(world - ref, axis=1)
        info = {
            g: stats(error[mask])
            for g, mask in [
                ("training", ~held),
                ("heldout_all", held),
                ("heldout_high_motion", high),
            ]
        }
        wins = []
        for i in np.unique(data["ids"][high]):
            keep = high & (data["ids"] == i)
            wins.append(
                np.mean(error[keep] ** 2) < np.mean(baseline_error[keep] ** 2) - 1e-20
            )
        floor = data["floor"] & high
        info["higher_motion_clouds_improved"] = int(sum(wins))
        info["higher_motion_clouds"] = len(wins)
        info["real_floor_residual_m"] = stats(
            (np.einsum("ni,i->n", world, data["normal"]) - data["offset"])[floor]
        )
        report["candidates"][name] = info
    report["grid_phase_sensitivity"] = []
    for phase in [0, step / 4, step / 2, 3 * step / 4]:
        start_time = np.floor((t - phase) / step) * step + phase
        fraction = (t - start_time) / step
        pp, rr = gt.pose(start_time)
        ep, er = gt.pose(start_time + step)
        _, previous = gt.pose(start_time - step)
        pos = pp + (ep - pp) * fraction[:, None]
        row = {"phase_s": phase}
        for label, weight, guard in [("linear", 0, False), ("guarded_half", 0.5, True)]:
            quat = rotation_policy(previous, rr, er, fraction, weight, guard)
            row[label] = stats(np.linalg.norm(quat.apply(x) + pos - ref, axis=1)[high])
        report["grid_phase_sensitivity"].append(row)
    ranges = np.linalg.norm(x, axis=1)
    for lo, hi in [(0.5, 2), (2, 5), (5, 15)]:
        report["range_bins"][f"{lo:g}-{hi:g}m"] = stats(
            baseline_error[high & (ranges >= lo) & (ranges < hi)]
        )
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
