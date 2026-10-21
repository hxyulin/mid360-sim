# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Evaluate inferred MID-360 coverage; requires NumPy and SciPy.

Percentages measure visited one-degree angular cells, without scene occlusion
or beam footprints. Gap probes are spaced 0.25 degrees. Extrapolation beyond
the original four seconds is unverified.
"""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    import numpy as np
    from scipy.spatial import cKDTree

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fit_report", type=Path)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if (
        hashlib.sha256(args.csv.read_bytes()).hexdigest()
        != "aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a"
    ):
        parser.error("unexpected source CSV checksum")
    fit = json.loads(args.fit_report.read_text())
    coef = np.array(fit["coefficients_rad"])
    terms = fit["terms"]

    def model(start, end):
        idx = np.arange(start, end)
        t = (idx // 4) / 50000
        lane = idx % 4
        spin = 2 * np.pi * (2189 / 12) * t
        sweep = 2 * np.pi * 10 * t
        a = coef[0, lane] - spin
        e = coef[0, lane + 4].copy()
        for j, (u, v) in enumerate(terms):
            p = u * spin + v * sweep
            c = np.cos(p)
            s = np.sin(p)
            a += coef[1 + j, lane] * c + coef[23 + j, lane] * s
            e += coef[1 + j, lane + 4] * c + coef[23 + j, lane + 4] * s
        return np.stack(
            [np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)], -1
        ), np.column_stack([np.rad2deg(a) % 360, np.rad2deg(e)])

    vec = []
    ang = []
    for first in range(0, 2400000, 100000):
        v, a = model(first, first + 100000)
        vec.append(v)
        ang.append(a)
    vec = np.concatenate(vec)
    ang = np.concatenate(ang)
    az, el = np.meshgrid(np.arange(0.125, 360, 0.25), np.arange(-6.875, 52, 0.25))
    probe = np.stack(
        [
            np.cos(np.deg2rad(el)) * np.cos(np.deg2rad(az)),
            np.cos(np.deg2rad(el)) * np.sin(np.deg2rad(az)),
            np.sin(np.deg2rad(el)),
        ],
        -1,
    ).reshape(-1, 3)
    weights = np.cos(np.deg2rad(el)).ravel()

    def counts(a):
        valid = (a[:, 1] >= -7) & (a[:, 1] < 52)
        return np.histogram2d(
            a[valid, 1], a[valid, 0], bins=(np.arange(-7, 53), np.arange(361))
        )[0].astype(int)

    def metrics(v, a):
        c = counts(a)
        dist = cKDTree(v).query(probe, workers=1)[0]
        d = np.rad2deg(2 * np.arcsin(np.clip(dist / 2, 0, 1)))
        cellweights = np.cos(np.deg2rad(np.arange(-6.5, 52)))[:, None]
        return dict(
            occupied_cells_pct=100 * float(np.mean(c > 0)),
            solid_angle_weighted_occupied_cells_pct=100
            * float(np.sum((c > 0) * cellweights) / (360 * np.sum(cellweights))),
            nearest_direction_p95_deg=float(np.percentile(d, 95)),
            largest_probe_gap_deg=float(d.max()),
            counts=c.tolist(),
        )

    report = {
        "cell_grid": "360 x 59 one-degree azimuth/elevation cells; declared -7..52 degree FOV",
        "gap_probes": "0.25 degree cell centers; max is sampled estimate, not continuous worst-case bound",
        "windows": [],
    }
    for secs in [0.1, 0.2, 0.5, 1, 2, 4, 8, 12]:
        n = round(secs * 200000)
        m = metrics(vec[:n], ang[:n])
        m.update(duration_s=secs, points=n, extrapolated=secs > 4)
        report["windows"].append(m)
        print(secs, {k: v for k, v in m.items() if k != "counts"}, flush=True)
    # Verify inferred coverage against the original angle table, independent of fit metric.
    raw = np.loadtxt(args.csv, delimiter=",", skiprows=1)
    rawang = np.column_stack([raw[:, 1] % 360, 90 - raw[:, 2]])
    for w in report["windows"]:
        if w["duration_s"] <= 4:
            w["csv_occupied_cells_pct"] = 100 * float(
                np.mean(counts(rawang[: w["points"]]) > 0)
            )
    framecoverage = []
    for start in np.arange(0, 12, 0.1):
        n = round(start * 200000)
        a = ang[n : n + 20000]
        framecoverage.append(100 * float(np.mean(counts(a) > 0)))
    report["individual_100ms_frame_occupied_cells_pct_range"] = [
        min(framecoverage),
        max(framecoverage),
    ]
    # One slow sweep advances the fast phase by this many turns modulo 1.
    report["fast_turns_per_vertical_cycle"] = (2189 / 12) / 10
    report["intercycle_azimuth_shift_deg"] = (
        (-(2189 / 12) / 10 * 360 + 180) % 360
    ) - 180
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print("frame range", report["individual_100ms_frame_occupied_cells_pct_range"])
    # Largest residual gaps and elevation dependence.
    dist = cKDTree(vec).query(probe, workers=1)[0]
    d = np.rad2deg(2 * np.arcsin(np.clip(dist / 2, 0, 1)))
    ids = np.argsort(d)[-8:][::-1]
    print(
        "largest gaps",
        [(float(az.ravel()[i]), float(el.ravel()[i]), float(d[i])) for i in ids],
    )
    c = counts(ang)
    print(
        "occupied elevation bands",
        [
            (float(e), round(100 * float((c[k] > 0).mean()), 2))
            for k, e in enumerate(np.arange(-6.5, 52))
            if (c[k] > 0).mean() < 0.99
        ],
    )
    print(
        "csv coverage",
        [(w["duration_s"], w.get("csv_occupied_cells_pct")) for w in report["windows"]],
    )
    # Include coarse-grid gap field for the visual.
    report["largest_gap_location_az_el_deg"] = [
        float(az.ravel()[ids[0]]),
        float(el.ravel()[ids[0]]),
    ]
    report["full_cycle_elevation_band_occupied_pct"] = [
        100 * float((row > 0).mean()) for row in c
    ]
    args.output.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
