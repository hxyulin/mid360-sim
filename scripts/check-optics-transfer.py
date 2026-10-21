# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Test frozen lane optics on an independent capture with disjoint clock fitting.

Only two local carrier phases/rates are fitted in each selected 0.2 s window.
No Fourier coefficients, mount transforms, or range-dependent corrections change.
Clock calibration and evaluation use disjoint 0.48 ms acquisition blocks.
This measures conditional shape transfer, not future clock prediction.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares, minimize_scalar


def summarize(errors):
    return dict(
        count=len(errors),
        median_deg=float(np.median(errors)),
        rms_deg=float(np.sqrt(np.mean(errors**2))),
        p95_deg=float(np.percentile(errors, 95)),
        max_deg=float(np.max(errors)),
    )


def check(analysis, model, output, window_s=0.2):
    output.mkdir(parents=True, exist_ok=False)
    profile = json.loads(model.read_text())
    coefficients = np.array(profile["coefficients"])
    terms = np.array(profile["terms"])
    data = np.load(analysis / "samples.npz")
    xyz = data["xyz_mm"].astype(float)
    lanes = (
        data["lane"].astype(int) if "lane" in data else (data["index"] % 4).astype(int)
    )
    # The learned model evaluates all four nominal 5 us slots at the first
    # slot's group phase. Keep the original per-point times for data provenance.
    t = data["time_ns"] / 1e9 - lanes * 5e-6
    blocks = data["block"] if "block" in data else data["index"] // 96
    radius = np.linalg.norm(xyz, axis=1)
    good = (radius >= 2000) & (data["tag"] == 0) & (lanes < 4)
    if not good.any():
        raise ValueError("no unflagged >=2 m returns")
    xyz = xyz / np.maximum(radius[:, None], 1)
    az = np.arctan2(xyz[:, 1], xyz[:, 0])
    el = np.arctan2(xyz[:, 2], np.hypot(xyz[:, 0], xyz[:, 1]))
    cal_mask = good & (blocks % 7 < 3)
    eval_mask = good & (blocks % 7 >= 3)
    # Coarse carrier acquisition uses calibration rays only, not fixed source
    # rates. The initial FFT-free search permits hardware-to-hardware variation.
    first_ids = np.flatnonzero(cal_mask & (t < min(t[-1], 2)))
    first_ids = first_ids[:: max(1, len(first_ids) // 12000)]
    if len(first_ids) < 100:
        raise ValueError("insufficient initial calibration data")

    def peak(freq, values):
        return abs(np.mean(values * np.exp(-2j * np.pi * freq * t[first_ids])))

    grid = np.arange(175, 190, 0.1)
    values = np.exp(1j * az[first_ids])
    fs = grid[np.argmax([peak(f, values) for f in grid])]
    fs = minimize_scalar(
        lambda f: -peak(f, values), bounds=(fs - 0.1, fs + 0.1), method="bounded"
    ).x
    grid = np.arange(8, 12, 0.02)
    values = el[first_ids] - np.mean(el[first_ids])
    fw = grid[np.argmax([peak(f, values) for f in grid])]
    fw = minimize_scalar(
        lambda f: -peak(f, values), bounds=(fw - 0.03, fw + 0.03), method="bounded"
    ).x
    n = len(terms)

    def predict(ids, center, p, mapping):
        dt = t[ids] - center
        sp = p[0] + 2 * np.pi * (fs + p[2]) * dt
        sw = p[1] + 2 * np.pi * (fw + p[3]) * dt
        phase = sp[:, None] * terms[:, 0] + sw[:, None] * terms[:, 1]
        c = coefficients[mapping[lanes[ids]]]
        angles = c[:, 0] + np.sum(
            np.cos(phase)[:, :, None] * c[:, 1 : 1 + n]
            + np.sin(phase)[:, :, None] * c[:, 1 + n :],
            axis=1,
        )
        angles[:, 0] += sp
        return angles

    def residual(p, ids, center, mapping):
        q = predict(ids, center, p, mapping)
        return np.column_stack(
            [
                np.angle(np.exp(1j * (q[:, 0] - az[ids]))) * np.cos(el[ids]),
                q[:, 1] - el[ids],
            ]
        ).ravel()

    # Select a cyclic/reversed lane convention on calibration observations in
    # the first populated window only. Subsequent windows keep that mapping.
    begin = max(0, float(t[first_ids[0]]))
    window_ids = np.flatnonzero(cal_mask & (t >= begin) & (t < begin + window_s))
    window_ids = window_ids[:: max(1, len(window_ids) // 600)]
    center = begin + window_s / 2
    initial_spin = np.angle(
        np.mean(
            np.exp(1j * (az[window_ids] - 2 * np.pi * fs * (t[window_ids] - center)))
        )
    ) - np.mean(coefficients[:, 0, 0])
    choices = []
    for sense in [1, -1]:
        for shift in range(4):
            mapping = (sense * np.arange(4) + shift) % 4
            for slow_phase in np.linspace(-np.pi, np.pi, 8, endpoint=False):
                fit = least_squares(
                    residual,
                    [initial_spin, slow_phase, 0, 0],
                    args=(window_ids, center, mapping),
                    bounds=([-np.inf, -np.inf, -3, -1], [np.inf, np.inf, 3, 1]),
                    max_nfev=45,
                )
                choices.append((np.mean(fit.fun**2), mapping, fit.x))
    best, mapping, seed = min(choices, key=lambda v: v[0])
    print(
        "carriers",
        fs,
        fw,
        "lane mapping",
        mapping.tolist(),
        "initial calibration RMS deg",
        np.rad2deg(np.sqrt(best)),
        flush=True,
    )
    records = []
    lane_errors = {l: [] for l in range(4)}
    errors = []
    for begin in np.arange(
        begin, max(begin + 0.001, t[-1] - window_s), max(window_s, (t[-1] - begin) / 60)
    ):
        center = begin + window_s / 2
        train = np.flatnonzero(cal_mask & (t >= begin) & (t < begin + window_s))
        test = np.flatnonzero(eval_mask & (t >= begin) & (t < begin + window_s))
        train = train[:: max(1, len(train) // 1600)]
        test = test[:: max(1, len(test) // 4000)]
        if len(train) < 100 or len(test) < 100:
            continue
        spin = np.angle(
            np.mean(np.exp(1j * (az[train] - 2 * np.pi * fs * (t[train] - center))))
        ) - np.mean(coefficients[:, 0, 0])
        fits = []
        for sw in np.linspace(-np.pi, np.pi, 8, endpoint=False):
            fit = least_squares(
                residual,
                [spin, sw, 0, 0],
                args=(train, center, mapping),
                bounds=([-np.inf, -np.inf, -3, -1], [np.inf, np.inf, 3, 1]),
                max_nfev=45,
            )
            fits.append(fit)
        fit = min(fits, key=lambda f: np.mean(f.fun**2))
        q = predict(test, center, fit.x, mapping)
        pred = np.column_stack(
            [
                np.cos(q[:, 1]) * np.cos(q[:, 0]),
                np.cos(q[:, 1]) * np.sin(q[:, 0]),
                np.sin(q[:, 1]),
            ]
        )
        er = np.rad2deg(
            np.arctan2(
                np.linalg.norm(np.cross(pred, xyz[test]), axis=1),
                np.sum(pred * xyz[test], axis=1),
            )
        )
        errors.extend(er)
        for l in range(4):
            lane_errors[l].extend(er[lanes[test] == l])
        rec = dict(
            start_s=float(begin),
            calibration_count=len(train),
            calibration_rms_deg=float(np.rad2deg(np.sqrt(np.mean(fit.fun**2)))),
            spin_hz=float(fs + fit.x[2]),
            sweep_hz=float(fw + fit.x[3]),
            **summarize(er)
        )
        records.append(rec)
        print(
            "window",
            round(begin, 2),
            "withheld RMS",
            round(rec["rms_deg"], 4),
            flush=True,
        )
    if not errors:
        raise ValueError("no evaluable disjoint windows")
    obj = dict(
        optics_source_sha256=profile["source_capture_sha256"],
        target=json.loads((analysis / "decode-report.json").read_text()),
        method="frozen Fourier coefficients; per-window two phases/two rates; selected windows; 0.48ms blocks modulo 7 <3 calibrate, >=3 evaluate; calibration-only cyclic/reversed lane selection",
        filters="range>=2m, raw tag zero",
        coarse_spin_hz=float(fs),
        coarse_sweep_hz=float(fw),
        lane_mapping=mapping.tolist(),
        caveat="conditional reconstruction using observed clock calibration, not blind phase prediction; finite multistart search, not certified global optimum",
        window_s=window_s,
        withheld=summarize(np.array(errors)),
        by_lane={str(l): summarize(np.array(e)) for l, e in lane_errors.items() if e},
        windows=records,
    )
    (output / "transfer-report.json").write_text(json.dumps(obj, indent=2) + "\n")
    print(json.dumps(obj["withheld"], indent=2), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("analysis", type=Path)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--window-s", type=float, default=0.2)
    args = parser.parse_args()
    if not 0.04 <= args.window_s <= 1.0:
        parser.error("window-s must be in 0.04..1.0")
    check(args.analysis, args.model, args.output, args.window_s)
