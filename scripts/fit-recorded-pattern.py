# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Fit lane optics and sparse capture-clock reconstruction with disjoint packets.

Requires NumPy/SciPy and samples + decode metadata from analyze-stationary-capture.py.
The measured phase trace reconstructs this session; it is not a motor predictor.
Output contains derived model parameters and statistics, never raw returns.
"""

from pathlib import Path
import numpy as np, json, time
from scipy.sparse import diags, bmat
from scipy.sparse.linalg import spsolve
import argparse

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "analysis", type=Path, help="Output directory from analyze-stationary-capture.py"
)
parser.add_argument(
    "--output", type=Path, required=True, help="New output directory; never overwritten"
)
parser.add_argument("--iterations", type=int, default=12)
args = parser.parse_args()
if not 1 <= args.iterations <= 100:
    parser.error("iterations must be in 1..100")
r = args.analysis
metadata = json.loads((r / "decode-report.json").read_text())
d = np.load(r / "samples.npz")
idx = d["index"]
x = d["xyz_mm"].astype(float)
ranges = np.linalg.norm(x, axis=1) / 1000
t = (idx // 4) * 20e-6
lane = (idx % 4).astype(int)
az = np.arctan2(x[:, 1], x[:, 0])
el = np.arctan2(x[:, 2], np.hypot(x[:, 0], x[:, 1]))
good = (ranges >= 2) & (d["tag"] == 0)
# Phase calibration uses one source packet per 80 packets, and is the only
# data allowed to update clock knots, including initialization. Geometry starts
# from non-withheld packets in [0,2), then refits calibration packets in [0,30).
# Early evaluation excludes those seed packets. All other evaluation packets
# are excluded from coefficient/phase initialization and refinement.
if len(idx) < 1000 or np.any(np.diff(idx.astype(np.int64)) <= 0):
    raise ValueError(
        "need ordered sampled emissions with dense initial calibration data"
    )
if not np.array_equal(d["time_ns"], idx * 5000):
    raise ValueError("requires a continuous nominal 5 us emission clock")
if metadata["emissions"] < 6_000_000:
    raise ValueError("need at least 30 seconds for this fit policy")
if metadata.get("non_480us_point_intervals") != 0:
    raise ValueError("capture has discontinuous point timing")
if any(
    np.sum(good & (lane == l) & (t < 2) & ((idx // 96) % 5 != 1)) < 100
    for l in range(4)
):
    raise ValueError("insufficient first-two-second geometry calibration samples")
args.output.mkdir(parents=True, exist_ok=False)
cal = np.flatnonzero(good & ((idx // 96) % 80 == 0))
packet_ids, group = np.unique(idx[cal] // 96, return_inverse=True)
knot_t = (packet_ids * 96 + 47.5) * 5e-6
base_s = 2 * np.pi * 181.004 * t
base_v = 2 * np.pi * 9.8079 * t
terms = np.array([(j, k) for j in range(2) for k in range(-7, 8) if j > 0 or k > 0])
n = len(terms)
cs = np.zeros((4, 1 + 2 * n, 2))
# Variable-projection seed fit uses only calibration packets in first 2 s.
from scipy.optimize import minimize

seed_ids = []
for l in range(4):
    ii = np.flatnonzero(good & (lane == l) & (t < 2) & ((idx // 96) % 5 != 1))
    seed_ids.append(ii[:: max(1, len(ii) // 2500)])
seed_a0 = [
    np.angle(np.mean(np.exp(1j * (az[ii] - 2 * np.pi * 181 * t[ii]))))
    for ii in seed_ids
]


def seed_solve(params, export=False):
    fs, fw, ds, dw = params
    total = 0.0
    number = 0
    coefs = []
    for l, ii in enumerate(seed_ids):
        sp = 2 * np.pi * (fs * t[ii] + 0.5 * ds * t[ii] ** 2)
        vp = 2 * np.pi * (fw * t[ii] + 0.5 * dw * t[ii] ** 2)
        phase = sp[:, None] * terms[:, 0] + vp[:, None] * terms[:, 1]
        mat = np.column_stack([np.ones(len(ii)), np.cos(phase), np.sin(phase)])
        target = np.column_stack(
            [np.angle(np.exp(1j * (az[ii] - sp - seed_a0[l]))) + seed_a0[l], el[ii]]
        )
        c = np.linalg.lstsq(mat, target, rcond=None)[0]
        res = np.einsum("ij,jk->ik", mat, c) - target
        total += np.sum(res[:, 0] ** 2 * np.cos(el[ii]) ** 2 + res[:, 1] ** 2)
        number += len(ii)
        coefs.append(c)
    return np.array(coefs) if export else total / number


seed = minimize(
    seed_solve,
    [181.002, 9.804, 0.0, 0.0],
    method="Nelder-Mead",
    options={
        "initial_simplex": [
            [181.002, 9.804, 0, 0],
            [181.003, 9.804, 0, 0],
            [181.002, 9.805, 0, 0],
            [181.002, 9.804, 0.001, 0],
            [181.002, 9.804, 0, 0.001],
        ],
        "xatol": 1e-9,
        "fatol": 1e-13,
        "maxiter": 350,
    },
)
cs = seed_solve(seed.x, True)
print(
    "calibration-only seed rates/drift",
    seed.x,
    "rms deg",
    np.rad2deg(np.sqrt(seed.fun)),
    flush=True,
)
# Approximate initial phase using packet-averaged demodulation. Slow modulation
# and scene bias are then corrected by joint optical registration below.
anchor_s = []
anchor_v = []
for g in range(len(packet_ids)):
    ii = cal[group == g]
    anchor_s.append(np.angle(np.mean(np.exp(1j * (az[ii] - base_s[ii])))))
    # Very short packets cannot identify slow phase from a sinusoidal envelope.
    # Use broader one-second windows ONLY for initial acquisition, not final fitting.
    m = good & ((idx // 96) % 80 == 0) & (t >= knot_t[g] - 0.5) & (t < knot_t[g] + 0.5)
    m &= (t < 30) if knot_t[g] < 30 else (t >= 30)
    if g % 10 == 0:
        ids = np.flatnonzero(m)[:: max(1, m.sum() // 1000)]
        vv = np.mean((el[ids] - el[ids].mean()) * np.exp(-1j * base_v[ids]))
        anchor_v.append((g, np.angle(vv)))
# Spin observation includes a constant coefficient offset and optical modulation.
ps = np.unwrap(anchor_s) - np.mean(cs[:, 0, 0])
vref = np.angle(np.mean(cs[:, 1, 1] - 1j * cs[:, 1 + n, 1]))
vv = np.unwrap([a[1] for a in anchor_v]) - vref
pv = np.empty(len(packet_ids))
anchor_ids = np.array([a[0] for a in anchor_v])
for training in [True, False]:
    target = (knot_t < 30) == training
    support = (knot_t[anchor_ids] < 30) == training
    if support.sum() < 2:
        raise ValueError("insufficient separate training/validation clock anchors")
    pv[target] = np.interp(np.flatnonzero(target), anchor_ids[support], vv[support])
N = len(ps)
D = diags(
    [np.ones(N - 2), -2 * np.ones(N - 2), np.ones(N - 2)], [0, 1, 2], shape=(N - 2, N)
).tocsr()
D = D[(knot_t[:-2] >= 30) | (knot_t[2:] < 30)]
P = D.T @ D


# vref term index0 corresponds (0,1).
def eval_all(ids, phase_s, phase_v, deriv=False):
    ss = base_s[ids] + phase_s
    vv = base_v[ids] + phase_v
    phase = ss[:, None] * terms[:, 0] + vv[:, None] * terms[:, 1]
    c = np.cos(phase)
    s = np.sin(phase)
    ll = lane[ids]
    cp = cs[ll, 1 : 1 + n, :]
    sp = cs[ll, 1 + n :, :]
    q = cs[ll, 0, :] + np.sum(c[:, :, None] * cp + s[:, :, None] * sp, axis=1)
    q[:, 0] += ss
    if not deriv:
        return q
    derivative = -s[:, :, None] * cp + c[:, :, None] * sp
    js = np.sum(derivative * terms[None, :, 0, None], axis=1)
    js[:, 0] += 1.0
    jv = np.sum(derivative * terms[None, :, 1, None], axis=1)
    return q, js, jv


# Joint alternating shape/clock fit. Per-packet phase is constant correction to
# base clocks over 0.48ms; slow drift during a packet is negligible here.
for iteration in range(args.iterations):
    for _ in range(5):
        q, js, jv = eval_all(cal, ps[group], pv[group], True)
        weight = np.cos(el[cal])
        res = q - np.column_stack([az[cal], el[cal]])
        res[:, 0] = np.angle(np.exp(1j * res[:, 0]))
        res[:, 0] *= weight
        js[:, 0] *= weight
        jv[:, 0] *= weight
        aa = np.bincount(group, weights=np.sum(js * js, axis=1))
        bb = np.bincount(group, weights=np.sum(js * jv, axis=1))
        cc = np.bincount(group, weights=np.sum(jv * jv, axis=1))
        ar = np.bincount(group, weights=np.sum(js * res, axis=1))
        br = np.bincount(group, weights=np.sum(jv * res, axis=1))
        det = aa * cc - bb * bb + 1e-12
        regularity = 1.0
        H = bmat(
            [
                [diags(aa) + regularity * P, diags(bb)],
                [diags(bb), diags(cc) + regularity * P],
            ],
            format="csc",
        )
        delta = spsolve(H, np.r_[ar + regularity * P @ ps, br + regularity * P @ pv])
        ds = delta[:N]
        dv = delta[N:]
        ps -= np.clip(ds, -0.2, 0.2)
        pv -= np.clip(dv, -0.2, 0.2)
    # Refit shared optics on the first 30s only. Corrections are interpolated at
    # actual point time to use the same clock model as withheld evaluation.
    train = cal[t[cal] < 30]
    ss = base_s[train] + np.interp(t[train], knot_t[knot_t < 30], ps[knot_t < 30])
    vs = base_v[train] + np.interp(t[train], knot_t[knot_t < 30], pv[knot_t < 30])
    phase = ss[:, None] * terms[:, 0] + vs[:, None] * terms[:, 1]
    mat = np.column_stack([np.ones(len(train)), np.cos(phase), np.sin(phase)])
    for l in range(4):
        m = lane[train] == l
        ii = train[m]
        target = np.column_stack(
            [
                np.angle(np.exp(1j * (az[ii] - ss[m] - cs[l, 0, 0]))) + cs[l, 0, 0],
                el[ii],
            ]
        )
        cs[l] = np.linalg.lstsq(mat[m], target, rcond=None)[0]
    print(
        "iteration",
        iteration,
        "calibration r.m.s deg",
        np.rad2deg(np.sqrt(np.mean(np.sum(res**2, axis=1)))),
        "knots",
        len(ps),
        flush=True,
    )


# Evaluate with only calibration-packet phases, linearly interpolated to unseen
# packets. Geometry/phase leakage from target packets is disallowed.
def stats(ids):
    out = []
    for batch in np.array_split(ids, max(1, len(ids) // 20000)):
        q = eval_all(
            batch, np.interp(t[batch], knot_t, ps), np.interp(t[batch], knot_t, pv)
        )
        aa = q[:, 0]
        ee = q[:, 1]
        pred = np.column_stack(
            [np.cos(ee) * np.cos(aa), np.cos(ee) * np.sin(aa), np.sin(ee)]
        )
        act = x[batch] / np.linalg.norm(x[batch], axis=1)[:, None]
        er = np.rad2deg(
            np.arctan2(
                np.linalg.norm(np.cross(pred, act), axis=1), np.sum(pred * act, axis=1)
            )
        )
        out.extend(er)
    out = np.array(out)
    return dict(
        count=len(out),
        rms_deg=float(np.sqrt(np.mean(out**2))),
        median_deg=float(np.median(out)),
        p95_deg=float(np.percentile(out, 95)),
        p99_deg=float(np.percentile(out, 99)),
        max_deg=float(out.max()),
    )


report = {}
for lo, hi in [
    (0, 2),
    (2, 4),
    (4, 30),
    (30, 60),
    (60, 120),
    (120, min(300, metadata["emissions"] / 200000)),
]:
    ids = np.flatnonzero(
        good
        & ((idx // 96) % 80 != 0)
        & (t >= lo)
        & (t < hi)
        & ((t >= 2) | ((idx // 96) % 5 == 1))
    )
    report[f"withheld_{lo}_{hi}s"] = stats(ids)
print(json.dumps(report, indent=2), flush=True)
obj = {
    "base_spin_hz": 181.004,
    "base_sweep_hz": 9.8079,
    "terms": terms.tolist(),
    "coefficients": cs.tolist(),
    "phase_time_s": knot_t.tolist(),
    "spin_phase_rad": ps.tolist(),
    "sweep_phase_rad": pv.tolist(),
    "withheld": report,
    "last_sample_index": metadata["emissions"] - 1,
    "source_capture_sha256": metadata["capture_sha256"],
    "decoder_sha256": metadata["decoder_sha256"],
    "policy": {
        "iterations": args.iterations,
        "minimum_range_m": 2,
        "tag_policy": "raw tag equals zero",
        "geometry_seed_s": [0, 2],
        "geometry_seed_packets": "packet index modulo 5 != 1, deterministic thinning to about 2500 per lane",
        "geometry_refinement_s": [0, 30],
        "phase_calibration_packets": "source packet index modulo 80 == 0 (96 returns per packet)",
        "phase_regularization": "unit-weight knot-index second difference, disconnected at geometry training boundary 30 s",
        "evaluation": "non-phase-calibration packets; before 2 s also require packet index modulo 5 == 1",
        "phase_model": "recorded corrections interpolated between measured anchors; not forecasting withheld time without anchors",
    },
    "clock_knot_count": len(knot_t),
    "profile_f64_storage_bytes": int(cs.size * 8 + len(knot_t) * 3 * 8),
}
(args.output / "hardware-profile.json").write_text(json.dumps(obj, indent=2) + "\n")

# Export the exact same numerical model for the dependency-free Rust runtime.
assert np.isfinite(cs).all() and np.isfinite(ps).all() and np.isfinite(pv).all()
assert np.all(np.diff(knot_t) > 0)
mean_s = np.polyfit(knot_t, ps, 1)
mean_v = np.polyfit(knot_t, pv, 1)
lines = [
    "// SPDX-License-Identifier: MIT OR Apache-2.0",
    "// Fitted parameters from an owner-provided stationary MID-360 recording.",
    "// Original measurements retain their ownership; no raw returns are stored here.",
    "// Generated by scripts/fit-recorded-pattern.py; do not edit by hand.",
    "// Capture SHA-256: " + metadata["capture_sha256"],
    "pub(super) const CAPTURE_LAST_SAMPLE: u64 = "
    + str(metadata["emissions"] - 1)
    + ";",
    "pub(super) const SPIN_REFERENCE_HZ: f64 = 181.004;",
    "pub(super) const SWEEP_REFERENCE_HZ: f64 = 9.8079;",
    "pub(super) const MEAN_SPIN_HZ: f64 = "
    + repr(float(181.004 + mean_s[0] / (2 * np.pi)))
    + ";",
    "pub(super) const MEAN_SWEEP_HZ: f64 = "
    + repr(float(9.8079 + mean_v[0] / (2 * np.pi)))
    + ";",
    "pub(super) const MEAN_SPIN_OFFSET_RAD: f64 = " + repr(float(mean_s[1])) + ";",
    "pub(super) const MEAN_SWEEP_OFFSET_RAD: f64 = " + repr(float(mean_v[1])) + ";",
    "pub(super) const HARDWARE_COEFFICIENTS: [[[f64; 2]; 45]; 4] = [",
]
for lane_coefficients in cs:
    lines.append("    [")
    for row in lane_coefficients:
        lines.append("        [" + ", ".join(repr(float(x)) for x in row) + "],")
    lines.append("    ],")
lines.extend(["];", f"pub(super) static CLOCK_KNOTS: [[f64; 3]; {len(knot_t)}] = ["])
for row in zip(knot_t, ps, pv):
    lines.append("    [" + ", ".join(repr(float(x)) for x in row) + "],")
lines.extend(["];", ""])
(args.output / "hardware_data.rs").write_text("\n".join(lines))
