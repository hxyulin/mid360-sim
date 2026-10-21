# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
from pathlib import Path
import json, numpy as np
from scipy.optimize import minimize_scalar
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser(
    description="Report carrier phase and same-index recurrence from sampled Livox returns."
)
parser.add_argument("analysis", type=Path)
args = parser.parse_args()
r = args.analysis
d = np.load(r / "samples.npz")
idx = d["index"]
t = d["time_ns"] / 1e9
x = d["xyz_mm"].astype(float)
ranges = np.linalg.norm(x, axis=1) / 1000
a = np.arctan2(x[:, 1], x[:, 0])
e = np.arctan2(x[:, 2], np.hypot(x[:, 0], x[:, 1]))
valid = ranges >= 2
# Demodulate independently in 1-second bins, with the source frequencies only as
# a centre for phase tracking (not a fitted per-point optical model).
centers = []
sp = []
sw = []
for k in range(300):
    m = valid & (t >= k) & (t < k + 1)
    tt = t[m]
    aa = a[m]
    ee = e[m]
    if len(tt) < 100:
        continue
    centers.append(k + 0.5)
    sp.append(np.angle(np.mean(np.exp(1j * (aa - 2 * np.pi * 181.0 * tt)))))
    sw.append(np.angle(np.mean((ee - ee.mean()) * np.exp(-2j * np.pi * 9.8 * tt))))
centers = np.array(centers)
sp = np.unwrap(sp)
sw = np.unwrap(sw)
slope_s = np.polyfit(centers, sp, 1)
slope_w = np.polyfit(centers, sw, 1)
fs = 181 + slope_s[0] / (2 * np.pi)
fw = 9.8 + slope_w[0] / (2 * np.pi)
# Same-index recurrence, no fitted initial phases needed. Compare only observed
# pairs with >=2m range in both members; missing returns cannot supply a ray.
rec = {}
for dt in [0.1, 1.0, 4.0, 5.0, 12.0, 30.0, 60.0]:
    target = idx + int(round(dt * 200000))
    j = np.searchsorted(idx, target)
    keep = j < len(idx)
    ii = np.flatnonzero(keep)
    jj = j[keep]
    ok = (idx[jj] == target[ii]) & valid[ii] & valid[jj]
    ii = ii[ok]
    jj = jj[ok]
    if len(ii) > 200000:
        ii = ii[:: len(ii) // 200000]
        jj = jj[:: len(jj) // 200000]
    u = x[ii] / np.linalg.norm(x[ii], axis=1)[:, None]
    v = x[jj] / np.linalg.norm(x[jj], axis=1)[:, None]
    err = np.rad2deg(
        np.arctan2(np.linalg.norm(np.cross(u, v), axis=1), np.sum(u * v, axis=1))
    )
    rec[str(dt)] = {
        "observed_valid_pairs": len(ii),
        "median_separation_deg": float(np.median(err)),
        "p95_separation_deg": float(np.percentile(err, 95)),
    }
acc = d["imu"][:, 3:]
mu = acc.mean(0)
tilt = np.rad2deg(np.arctan2(np.linalg.norm(mu[:2]), mu[2]))
report = {
    "phase_regression_spin_hz": float(fs),
    "phase_regression_sweep_hz": float(fw),
    "spin_phase_detrended_p95_abs_deg": float(
        np.percentile(abs(np.rad2deg(sp - np.polyval(slope_s, centers))), 95)
    ),
    "sweep_phase_detrended_p95_abs_deg": float(
        np.percentile(abs(np.rad2deg(sw - np.polyval(slope_w, centers))), 95)
    ),
    "imu_mean_specific_force_g": mu.tolist(),
    "apparent_gravity_tilt_deg": float(tilt),
    "recurrence": rec,
}
(r / "phase-report.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2))
fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
axes[0, 0].plot(centers, np.rad2deg(sp), label="measured residual to +181 Hz")
axes[0, 0].plot(
    centers, np.rad2deg(np.polyval(slope_s, centers)), "--", label="linear trend"
)
axes[0, 0].set(
    title="Azimuth carrier phase (1 s bins)", xlabel="Time [s]", ylabel="Phase [deg]"
)
axes[0, 0].legend(fontsize=8)
axes[0, 1].plot(centers, np.rad2deg(sw), label="measured residual to 9.8 Hz")
axes[0, 1].plot(
    centers, np.rad2deg(np.polyval(slope_w, centers)), "--", label="linear trend"
)
axes[0, 1].set(
    title="Elevation fundamental phase (1 s bins)",
    xlabel="Time [s]",
    ylabel="Phase [deg]",
)
axes[0, 1].legend(fontsize=8)
m = valid & (t < 0.1)
axes[1, 0].scatter(np.rad2deg(a[m][::5]), np.rad2deg(e[m][::5]), s=1, alpha=0.5)
axes[1, 0].set(
    title="First 100 ms: measured valid rays only",
    xlabel="Sensor azimuth [deg]",
    ylabel="Sensor elevation [deg]",
)
ks = list(rec)
axes[1, 1].bar(ks, [rec[k]["median_separation_deg"] for k in ks])
axes[1, 1].set(
    title="Measured same-index recurrence test",
    xlabel="Time separation [s]",
    ylabel="Median angular separation [deg]",
)
fig.suptitle(
    "Stationary MID-360 hardware capture: 300 s / see decode-report.json for packet continuity\nSensor-frame rays; returns <2 m excluded; phase estimates are not calibrated motor encoders",
    fontsize=12,
)
fig.savefig(r / "hardware-validation.png", dpi=160)
