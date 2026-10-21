#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Evaluate the official plugin's snapshot policy against recorded MoCap motion.

This is a source-derived acquisition-policy proxy, NOT execution of Gazebo 9
or a reproduction of official CSV rays. Sweep four 10 Hz callback-grid phases
and both endpoint timestamp assignments because upstream has no per-point time.
"""

import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np

spec = importlib.util.spec_from_file_location(
    "motion_comparison", Path(__file__).with_name("compare-motion-ground-truth.py")
)
motion = importlib.util.module_from_spec(spec)
spec.loader.exec_module(motion)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "evaluation", type=Path, help="evaluation.npz from recorded-motion comparison"
    )
    ap.add_argument("trajectory", type=Path)
    ap.add_argument("--output", required=True, type=Path)
    args = ap.parse_args()
    if args.output.exists():
        raise ValueError("output must be new")
    gt = motion.Trajectory(args.trajectory)
    data = np.load(args.evaluation)
    times, local, reference = data["times"], data["local"], data["reference"]
    # Every policy sees the exact same points, including across grid phases.
    covered = data["heldout"].copy()
    phases = [0, 0.025, 0.05, 0.075]
    windows = []
    for phase in phases:
        begin = np.floor((times - phase) / 0.1) * 0.1 + phase
        end = begin + 0.1
        starts, inverse = np.unique(begin, return_inverse=True)
        valid = np.array([gt.covered(a, a + 0.1) for a in starts])
        covered &= valid[inverse]
        windows.append((begin, end))
    report = {
        "scope": "source-derived 10 Hz snapshot policy proxy; not native official plugin execution or optical comparison",
        "upstream_revision": "1cce1073633a062b92e30243a4c2920e45551bb5",
        "period_s": 0.1,
        "grid_phase_s": phases,
        "groups": {},
    }
    for label, selected in [
        ("heldout_all", covered),
        ("heldout_high_motion", covered & data["high"]),
    ]:
        t, xyz, ref = times[selected], local[selected], reference[selected]
        section = {
            "points": int(selected.sum()),
            "clouds": len(np.unique(data["ids"][selected])),
            "snapshot": [],
        }
        for phase, (begin, end) in zip(phases, windows):
            for endpoint, tt in [("start", begin), ("end", end)]:
                p, r = gt.pose(tt[selected])
                world = r.apply(xyz) + p
                section["snapshot"].append(
                    {
                        "phase_s": phase,
                        "endpoint_assignment": endpoint,
                        "point_displacement_m": motion.metrics(
                            np.linalg.norm(world - ref, axis=1)
                        ),
                    }
                )
        p, r = gt.sampled(t, 1 / 128, True)
        section["our_interpolation_128hz_m"] = motion.metrics(
            np.linalg.norm(r.apply(xyz) + p - ref, axis=1)
        )
        report["groups"][label] = section
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
