#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Evaluate actual upstream XYZ with explicit, diagnostic nominal timing.

Upstream PointCloud has NO per-point times. The 5 us correspondence here is
assigned by this matched 20k@10Hz harness, not decoded from native messages.
"""

import argparse
import importlib.util
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    s = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(s)
    s.loader.exec_module(m)
    return m


p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--input", type=Path, required=True)
p.add_argument("--fixture", type=Path, required=True)
p.add_argument("--gt", type=Path, required=True)
p.add_argument("--output", type=Path, required=True)
a = p.parse_args()
r = load("reference", ROOT / "scripts/check-backend-ground-truth.py")
g = load("comparison", ROOT / "scripts/compare-motion-ground-truth.py")
f = np.load(a.fixture)
meta = json.loads(str(f["metadata"]))
gt = g.Trajectory(a.gt)
xyz = np.fromfile(a.input, dtype="<f4").reshape(-1, 3).astype(float)
assert xyz.shape == (400000, 3)
measured = np.linalg.norm(xyz, axis=1)
valid = np.isfinite(measured) & (measured > 0)
indices = np.flatnonzero(valid)
directions = xyz[valid] / measured[valid, None]
t = meta["start_relative_s"] + indices * 5e-6
pos, rot = gt.pose(t)
pos += meta["translation_shift_m"]
expected = r.ranges(pos, rot.as_quat(), directions)
end = indices // 20000 * 100 + 100
snapshot = r.ranges(f["boundary_p"][end], f["boundary_q"][end], directions)


def metrics(v):
    return dict(
        rms_m=float(np.sqrt(np.mean(v * v))),
        mean_m=float(np.mean(v)),
        p95_m=float(np.percentile(abs(v), 95)),
        max_m=float(np.max(abs(v))),
    )


report = dict(
    source_revision="1cce1073633a062b92e30243a4c2920e45551bb5",
    valid=int(valid.sum()),
    scheduled=400000,
    rolling_correspondence=metrics(measured[valid] - expected),
    snapshot_agreement=metrics(measured[valid] - snapshot),
    timing_policy="harness-assigned uniform 5us offsets at 20k/10Hz; upstream has no per-point timestamps",
    metadata=meta,
)
a.output.write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2))
