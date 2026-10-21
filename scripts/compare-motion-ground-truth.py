#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Compare rolling acquisition policies to an independent LiDAR-frame TUM trace.

Read-only ROS2 CustomMsg extraction. Evaluates recorded local returns under
pose policies; no optical refit, clock correction, per-cloud ICP or map building.
Reference is linear-position/SLERP interpolation of supplied GT, not continuous
physical truth. Plane references are estimated from separate low-motion scans.
"""

import argparse
from collections import Counter
from decimal import Decimal
import hashlib
import json
from pathlib import Path
import sqlite3

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


def metrics(values):
    """Report absolute error magnitudes, in metres for point/plane residuals."""
    x = np.asarray(values)
    if len(x) == 0 or not np.isfinite(x).all():
        raise ValueError("empty or nonfinite evaluation group")
    return dict(
        count=len(x),
        rms=float(np.sqrt(np.mean(x**2))),
        median=float(np.median(abs(x))),
        p95=float(np.percentile(abs(x), 95)),
        max=float(np.max(abs(x))),
    )


class Trajectory:
    """Read explicit xyzw LiDAR-to-world TUM poses without extrapolation."""

    def __init__(self, path):
        rows = [
            s.split()
            for s in path.read_text().splitlines()
            if s.strip() and not s.lstrip().startswith("#")
        ]
        ns = np.array([int(Decimal(r[0]) * 10**9) for r in rows], dtype=np.int64)
        self.epoch_ns = int(ns[0])
        self.t = (ns - ns[0]) * 1e-9
        self.p = np.array([[float(x) for x in r[1:4]] for r in rows])
        q = np.array([[float(x) for x in r[4:8]] for r in rows])
        if (
            not np.all(np.diff(ns) > 0)
            or not np.isfinite(self.p).all()
            or not np.isfinite(q).all()
            or np.max(abs(np.linalg.norm(q, axis=1) - 1)) > 1e-3
        ):
            raise ValueError("invalid trajectory")
        self.r = Rotation.from_quat(q)
        self.slerp = Slerp(self.t, self.r)
        self.gaps = np.flatnonzero(np.diff(self.t) > 0.02)

    def pose(self, t):
        return np.column_stack(
            [np.interp(t, self.t, self.p[:, i]) for i in range(3)]
        ), self.slerp(t)

    def covered(self, a, b):
        return (
            a >= self.t[0]
            and b <= self.t[-1]
            and not np.any((self.t[self.gaps] < b) & (self.t[self.gaps + 1] > a))
        )

    def sampled(self, t, step, interpolate):
        a = np.floor(t / step) * step
        p, r = self.pose(a)
        if not interpolate:
            return p, r
        b = a + step
        pp, rr = self.pose(b)
        f = (t - a) / step
        return p + (pp - p) * f[:, None], r * Rotation.from_rotvec(
            (r.inv() * rr).as_rotvec() * f[:, None]
        )


def store():
    """Register the standard Livox Driver 2 point and cloud layouts."""
    s = get_typestore(Stores.ROS2_HUMBLE)
    s.register(
        get_types_from_msg(
            "uint32 offset_time\nfloat32 x\nfloat32 y\nfloat32 z\nuint8 reflectivity\nuint8 tag\nuint8 line\n",
            "livox_ros_driver2/msg/CustomPoint",
        )
    )
    s.register(
        get_types_from_msg(
            "std_msgs/Header header\nuint64 timebase\nuint32 point_num\nuint8 lidar_id\nuint8[3] rsvd\nlivox_ros_driver2/CustomPoint[] points\n",
            "livox_ros_driver2/msg/CustomMsg",
        )
    )
    return s


def fit_plane(points):
    """Huber IRLS plane with 30 mm tuning, from training points only."""
    weights = np.ones(len(points))
    for _ in range(12):
        center = np.average(points, axis=0, weights=weights)
        _, vectors = np.linalg.eigh(
            np.einsum("ni,nj,n->ij", points - center, points - center, weights)
        )
        n = vectors[:, 0]
        distance = np.einsum("ni,i->n", points - center, n)
        weights = np.minimum(1, 0.03 / np.maximum(abs(distance), 1e-12))
    if n[2] < 0:
        n = -n
    return n, float(np.sum(n * center))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag", type=Path)
    ap.add_argument("trajectory", type=Path)
    ap.add_argument("--output", required=True, type=Path)
    args = ap.parse_args()
    if args.output.exists():
        raise ValueError("output must be a new directory")
    gt = Trajectory(args.trajectory)
    decoder = store()
    con = sqlite3.connect(f"file:{args.bag.resolve()}?mode=ro", uri=True)
    topics = con.execute(
        "select id,type from topics where name='/livox/lidar'"
    ).fetchall()
    if len(topics) != 1:
        raise ValueError("need one /livox/lidar topic")
    tid, typ = topics[0]
    ids = []
    local = []
    times = []
    starts = []
    middles = []
    rates = []
    counts = Counter()
    margin = 1 / 60 + 0.000001
    for i, (raw,) in enumerate(
        con.execute(
            "select data from messages where topic_id=? order by timestamp", (tid,)
        )
    ):
        counts["clouds_total"] += 1
        m = decoder.deserialize_cdr(raw, typ)
        if len(m.points) != m.point_num:
            raise ValueError("inconsistent point count")
        off = np.array([p.offset_time for p in m.points], dtype=np.int64)
        begin = (int(m.timebase) - gt.epoch_ns + int(off.min())) * 1e-9
        end = (int(m.timebase) - gt.epoch_ns + int(off.max())) * 1e-9
        if not gt.covered(begin - margin, end + margin):
            counts["clouds_rejected_coverage_or_gap"] += 1
            continue
        # Odd decimation retains all four lanes and original timing anomalies.
        selected = m.points[::11]
        ts = (
            np.array([int(m.timebase) - gt.epoch_ns + p.offset_time for p in selected])
            * 1e-9
        )
        xyz = np.array([[p.x, p.y, p.z] for p in selected])
        keep = (
            np.isfinite(xyz).all(axis=1)
            & (np.linalg.norm(xyz, axis=1) > 0.5)
            & (np.linalg.norm(xyz, axis=1) < 15)
        )
        xyz, ts = xyz[keep], ts[keep]
        counts["clouds_retained"] += 1
        rot = gt.slerp(np.array([begin, end]))
        rate = (rot[0].inv() * rot[1]).magnitude() / (end - begin)
        n = len(ts)
        ids.extend([i] * n)
        local.extend(xyz)
        times.extend(ts)
        starts.extend([begin] * n)
        middles.extend([(begin + end) / 2] * n)
        rates.extend([rate] * n)
    ids = np.array(ids)
    local = np.array(local)
    times = np.array(times)
    starts = np.array(starts)
    middles = np.array(middles)
    rates = np.array(rates)
    p, r = gt.pose(times)
    reference = r.apply(local) + p
    unique, first = np.unique(ids, return_index=True)
    train_rates = rates[first][unique % 5 == 0]
    train_limit = float(np.quantile(train_rates, 1 / 3))
    train = (ids % 5 == 0) & (rates <= train_limit)
    heldout = ids % 5 != 0
    high_limit = float(np.quantile(rates[first][unique % 5 != 0], 2 / 3))
    high = heldout & (rates >= high_limit)
    # Broad predeclared floor ROI; separate training scan IDs and low-motion data.
    floor_train = (
        train & (abs(reference[:, 2]) < 0.15) & (np.linalg.norm(local, axis=1) < 8)
    )
    n, d = fit_plane(reference[floor_train])
    if abs(n[2]) < 0.95:
        raise ValueError("floor fit is not near horizontal")
    # Label floor candidates with ONE midpoint pose per cloud, independently
    # of the per-point GT/interpolation policies under test. The broad ROI may
    # still contain clutter; this is not independently surveyed floor truth.
    policies = {"gt_per_point": reference}
    for name, tt in [("snapshot_cloud_start", starts), ("snapshot_cloud_mid", middles)]:
        pp, rr = gt.pose(tt)
        policies[name] = rr.apply(local) + pp
    coarse_residual = np.einsum("ni,i->n", policies["snapshot_cloud_mid"], n) - d
    floor = (
        heldout & (abs(coarse_residual) < 0.20) & (np.linalg.norm(local, axis=1) < 8)
    )
    for hz in [60, 128, 240, 1000]:
        for interpolate in ([False, True] if hz == 1000 else [True]):
            name = f'{"interpolate" if interpolate else "hold"}_{hz}hz'
            pp, rr = gt.sampled(times, 1 / hz, interpolate)
            policies[name] = rr.apply(local) + pp
    report = {
        "scope": "Recorded-return deskew/pose-policy comparison against interpolated MoCap. Not native engine execution or calibrated scan/range error.",
        "counts": dict(counts),
        "sampled_points": len(local),
        "point_stride": 11,
        "gt_gap_rejection_s": 0.02,
        "gt_margin_s": margin,
        "training_scan_policy": "original cloud index modulo 5 == 0, lower third net angular speed",
        "training_rate_max_deg_s": float(np.rad2deg(train_limit)),
        "high_motion_rate_min_deg_s": float(np.rad2deg(high_limit)),
        "floor_plane": {
            "normal": n.tolist(),
            "offset_m": d,
            "training_points": int(floor_train.sum()),
            "evaluation_label": "held-out points within 0.20 m of plane under ONE cloud midpoint pose and range <8 m; independent of per-point reference but may favor midpoint control and include clutter",
        },
        "groups": {},
    }
    for group, mask in [("heldout_all", heldout), ("heldout_high_motion", high)]:
        section = {
            "clouds": len(np.unique(ids[mask])),
            "points": int(mask.sum()),
            "policies": {},
        }
        for name, world in policies.items():
            error = np.linalg.norm(world - reference, axis=1)
            normal_error = np.einsum("ni,i->n", world, n) - d
            section["policies"][name] = {
                "point_displacement_m": metrics(error[mask]),
                "floor_residual_m": metrics(normal_error[floor & mask]),
            }
        report["groups"][group] = section
    # Per-cloud floor RMS for a paired bootstrap, rather than treating rays as independent.
    paired = []
    for idx in np.unique(ids[floor & high]):
        mask = floor & high & (ids == idx)
        if mask.sum() < 20:
            continue
        paired.append(
            [
                int(idx),
                int(mask.sum()),
                *[
                    float(
                        np.sqrt(
                            np.mean((np.einsum("ni,i->n", world[mask], n) - d) ** 2)
                        )
                    )
                    for world in policies.values()
                ],
            ]
        )
    # Shift poses only; original point labels/times and training plane stay fixed.
    sensitivity = {}
    for shift in [-0.01, -0.005, -0.001, 0, 0.001, 0.005, 0.01]:
        pp, rr = gt.pose(times + shift)
        shifted = rr.apply(local) + pp
        sensitivity[str(shift)] = metrics(
            (np.einsum("ni,i->n", shifted, n) - d)[floor & high]
        )
    report["pose_time_offset_floor_high_motion_m"] = sensitivity
    paired_array = np.array(paired)
    if len(paired_array) < 10:
        raise ValueError("insufficient paired floor scans")
    rng = np.random.default_rng(20261022)
    sample = rng.integers(len(paired_array), size=(2000, len(paired_array)))
    weight = paired_array[sample, 1]
    gt_mse = paired_array[sample, 2] ** 2
    raw_mse = paired_array[sample, 3] ** 2
    delta = np.sqrt(np.sum(weight * raw_mse, axis=1) / weight.sum(axis=1)) - np.sqrt(
        np.sum(weight * gt_mse, axis=1) / weight.sum(axis=1)
    )
    report["paired_cloud_bootstrap_floor_rms_reduction_m"] = {
        "clouds": len(paired_array),
        "resamples": 2000,
        "p025": float(np.percentile(delta, 2.5)),
        "p975": float(np.percentile(delta, 97.5)),
        "caveat": "cloud resampling; temporal correlation remains",
    }
    report["paired_cloud_columns"] = ["id", "points", *policies]
    report["paired_high_motion_floor_rms"] = paired
    report["sha256"] = {
        f.name: hashlib.sha256(f.read_bytes()).hexdigest()
        for f in [args.bag, args.trajectory]
    }
    args.output.mkdir(parents=True)
    (args.output / "report.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n"
    )
    np.savez_compressed(
        args.output / "evaluation.npz",
        times=times,
        ids=ids,
        local=local,
        reference=reference,
        heldout=heldout,
        high=high,
        floor=floor,
        normal=n,
        offset=d,
        **{k: v for k, v in policies.items() if k != "gt_per_point"},
    )
    concise = {
        k: v
        for k, v in report.items()
        if k not in ["paired_high_motion_floor_rms", "paired_cloud_columns"]
    }
    print(json.dumps(concise, indent=2))


if __name__ == "__main__":
    main()
