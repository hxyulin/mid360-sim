#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Audit ROS2 Livox CustomMsg timing against a TUM LiDAR-frame trajectory.

Read-only SQLite decoding; no ROS runtime, retiming, deskew or model fitting.
Requires the existing analysis NumPy/SciPy/rosbags environment. Gyro alignment
is a finite-difference diagnostic, not calibrated sensor synchronization.
"""

import argparse
import hashlib, json, sqlite3
from collections import Counter
from pathlib import Path
from decimal import Decimal
import numpy as np
from scipy.spatial.transform import Rotation
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("bag", type=Path)
parser.add_argument("trajectory", type=Path)
parser.add_argument("--output", required=True, type=Path)
args = parser.parse_args()
store = get_typestore(Stores.ROS2_HUMBLE)
store.register(
    get_types_from_msg(
        "uint32 offset_time\nfloat32 x\nfloat32 y\nfloat32 z\nuint8 reflectivity\nuint8 tag\nuint8 line\n",
        "livox_ros_driver2/msg/CustomPoint",
    )
)
store.register(
    get_types_from_msg(
        "std_msgs/Header header\nuint64 timebase\nuint32 point_num\nuint8 lidar_id\nuint8[3] rsvd\nlivox_ros_driver2/CustomPoint[] points\n",
        "livox_ros_driver2/msg/CustomMsg",
    )
)
rows = [
    r.split()
    for r in args.trajectory.read_text().splitlines()
    if r and not r.startswith("#")
]
tns = np.array([int(Decimal(r[0]) * 10**9) for r in rows], dtype=np.int64)
t = (tns - tns[0]) * 1e-9
p = np.array([[float(x) for x in r[1:4]] for r in rows])
q = np.array([[float(x) for x in r[4:8]] for r in rows])
assert np.all(np.diff(tns) > 0) and np.isfinite(p).all() and np.isfinite(q).all()
assert len(t) > 2 and np.max(abs(np.linalg.norm(q, axis=1) - 1)) < 1e-3
rot = Rotation.from_quat(q)
dt = np.diff(t)
w = (rot[:-1].inv() * rot[1:]).as_rotvec() / dt[:, None]
v = np.diff(p, axis=0) / dt[:, None]


def stats(a):
    return dict(
        min=float(np.min(a)),
        median=float(np.median(a)),
        p95=float(np.percentile(a, 95)),
        max=float(np.max(a)),
    )


con = sqlite3.connect(f"file:{args.bag.resolve()}?mode=ro", uri=True)
topics = list(con.execute("select id,name,type from topics"))
report = {
    "gt_samples": len(t),
    "gt_duration_s": float(t[-1]),
    "gt_interval_s": stats(dt),
    "gt_gap_count_over_20ms": int(sum(dt > 0.02)),
    "quaternion_norm": stats(np.linalg.norm(q, axis=1)),
    "translation_speed_m_s": stats(np.linalg.norm(v, axis=1)),
    "angular_speed_deg_s": stats(np.rad2deg(np.linalg.norm(w, axis=1))),
    "topics": topics,
}
imu = []
scans = []
fields = set()
ids = set()
hdiff = []
monotonic = []
offset_steps = []
point_total = 0
step_counts = Counter()
negative_at_packet_boundary = 0
negative_inside_packet = 0
nonmonotonic_scans = 0
packet_start_deltas = []
for tid, name, typ in topics:
    if name not in ["/livox/lidar", "/livox/imu"]:
        continue
    for stamp, raw in con.execute(
        "select timestamp,data from messages where topic_id=? order by timestamp",
        (tid,),
    ):
        m = store.deserialize_cdr(raw, typ)
        hn = int(m.header.stamp.sec) * 10**9 + int(m.header.stamp.nanosec)
        if name == "/livox/imu":
            g = m.angular_velocity
            imu.append([hn, g.x, g.y, g.z])
            continue
        assert m.point_num == len(m.points)
        offsets = np.array([p.offset_time for p in m.points], dtype=np.int64)
        assert len(offsets) > 0
        for pt in m.points:
            fields.add(int(pt.line))
        scans.append(
            [
                int(m.timebase) + int(offsets.min()),
                int(m.timebase) + int(offsets.max()),
                len(offsets),
            ]
        )
        diffs = np.diff(offsets)
        step_counts.update(map(int, diffs))
        ix = np.flatnonzero(diffs < 0) + 1
        negative_at_packet_boundary += int(sum(ix % 96 == 0))
        negative_inside_packet += int(sum(ix % 96 != 0))
        nonmonotonic_scans += int(len(ix) > 0)
        packet_start_deltas.extend(np.diff(offsets[::96]).tolist())
        hdiff.append(hn - int(m.timebase))
        ids.add(m.header.frame_id)
        monotonic.append(bool(np.all(np.diff(offsets) >= 0)))
        offset_steps.extend(np.unique(np.diff(offsets)).tolist())
        point_total += len(offsets)
scans = np.array(scans, dtype=np.int64)
imu = np.array(imu, dtype=np.float64)
it = (imu[:, 0] - tns[0]) * 1e-9
# integer timestamps restored separately below only span/seconds diagnostics;
# float conversion here has sub-microsecond epoch rounding, well below 100 Hz GT.
mt = (t[:-1] + t[1:]) / 2
keep = (mt > it[0] + 0.2) & (mt < it[-1] - 0.2) & (dt < 0.02)
offsets = np.arange(-0.1, 0.10001, 0.001)
scores = []
for shift in offsets:
    sampled = np.column_stack(
        [np.interp(mt[keep] + shift, it, imu[:, k]) for k in range(1, 4)]
    )
    err = w[keep] - sampled
    bias = np.median(err, axis=0)
    scores.append(np.sqrt(np.mean(np.sum((err - bias) ** 2, axis=1))))
idx = int(np.argmin(scores))
zero = int(np.argmin(abs(offsets)))
report.update(
    {
        "lidar_scans": len(scans),
        "lidar_points": point_total,
        "lidar_time_span_s": float((scans[-1, 1] - scans[0, 0]) * 1e-9),
        "lidar_scan_duration_s": stats((scans[:, 1] - scans[:, 0]) * 1e-9),
        "lidar_scan_start_interval_s": stats(np.diff(scans[:, 0]) * 1e-9),
        "point_counts": stats(scans[:, 2]),
        "lane_values": sorted(fields),
        "offset_step_ns": {
            "unique_count": len(set(offset_steps)),
            "most_common": step_counts.most_common(8),
            "negative_at_96_point_boundary": negative_at_packet_boundary,
            "negative_elsewhere": negative_inside_packet,
            "nonmonotonic_scan_count": nonmonotonic_scans,
            "min": min(offset_steps),
            "max": max(offset_steps),
        },
        "packet_start_delta_ns": stats(packet_start_deltas),
        "all_point_offsets_nondecreasing": all(monotonic),
        "frame_ids": sorted(ids),
        "header_minus_timebase_ns": stats(hdiff),
        "scans_fully_gt_bracketed": int(
            np.sum((scans[:, 0] >= tns[0]) & (scans[:, 1] <= tns[-1]))
        ),
        "gt_missing_initial_lidar_s": float(max(0, tns[0] - scans[0, 0]) * 1e-9),
        "gt_missing_final_lidar_s": float(max(0, scans[-1, 1] - tns[-1]) * 1e-9),
        "imu_samples": len(imu),
        "imu_interval_s": stats(np.diff(it)),
        "gyro_gt_diagnostic": {
            "model": "GT finite-difference body angular velocity versus identity-extrinsic IMU; constant median bias removed per offset; no smoothing",
            "best_query_imu_at_gt_time_plus_s": float(offsets[idx]),
            "best_vector_rms_rad_s": float(scores[idx]),
            "zero_shift_vector_rms_rad_s": float(scores[zero]),
            "not_a_calibration": True,
        },
    }
)
for f in [args.bag, args.trajectory]:
    report.setdefault("sha256", {})[f.name] = hashlib.sha256(f.read_bytes()).hexdigest()
args.output.write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2))
