# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Plot the disjoint-packet fit and measured clock reconstruction, not forecasting."""

import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("analysis", type=Path)
parser.add_argument("profile", type=Path)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
data = np.load(args.analysis / "samples.npz")
model = json.loads(args.profile.read_text())
terms = np.asarray(model["terms"])
coefficients = np.asarray(model["coefficients"])
knots = np.asarray(model["phase_time_s"])
ps = np.asarray(model["spin_phase_rad"])
pv = np.asarray(model["sweep_phase_rad"])
idx = data["index"]
time_s = (idx // 4) * 20e-6
xyz = data["xyz_mm"].astype(float)
ranges = np.linalg.norm(xyz, axis=1)
valid = (ranges >= 2000) & (data["tag"] == 0)
held = valid & ((idx // 96) % 80 != 0) & ((time_s >= 2) | ((idx // 96) % 5 == 1))


def predict(ids):
    t = time_s[ids]
    spin = 2 * np.pi * model["base_spin_hz"] * t + np.interp(t, knots, ps)
    sweep = 2 * np.pi * model["base_sweep_hz"] * t + np.interp(t, knots, pv)
    phase = spin[:, None] * terms[:, 0] + sweep[:, None] * terms[:, 1]
    c = coefficients[(idx[ids] % 4).astype(int)]
    n = len(terms)
    q = c[:, 0] + np.sum(
        np.cos(phase)[:, :, None] * c[:, 1 : 1 + n]
        + np.sin(phase)[:, :, None] * c[:, 1 + n :],
        axis=1,
    )
    q[:, 0] += spin
    a, e = q.T
    return np.column_stack([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])


ids = np.flatnonzero(held & (time_s >= 30))
errors = []
for batch in np.array_split(ids, max(1, len(ids) // 20000)):
    pred = predict(batch)
    measured = xyz[batch] / ranges[batch, None]
    errors.extend(
        np.rad2deg(
            np.arctan2(
                np.linalg.norm(np.cross(pred, measured), axis=1),
                np.sum(pred * measured, axis=1),
            )
        )
    )
errors = np.sort(errors)
fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
axes[0, 0].plot(knots, np.rad2deg(ps - ps[0]), label="spin correction")
axes[0, 0].plot(knots, np.rad2deg(pv - pv[0]), label="sweep correction")
axes[0, 0].set(
    title="Sparse measured clock corrections",
    xlabel="Time [s]",
    ylabel="Phase change [deg]",
)
axes[0, 0].legend()
axes[0, 1].plot(errors, np.arange(1, len(errors) + 1) / len(errors))
axes[0, 1].set(
    xlabel="Angular error [deg]",
    ylabel="Fraction",
    title="Unseen packets, 30–300 s",
    xlim=(0, 1),
    ylim=(0, 1),
)
axes[0, 1].grid(alpha=0.3)
ii = np.flatnonzero(held & (time_s < 0.1))[::3]
measured = xyz[ii] / ranges[ii, None]
pred = predict(ii)
for rays, label, marker in [(measured, "measured", "o"), (pred, "fit", "x")]:
    axes[1, 0].scatter(
        np.rad2deg(np.arctan2(rays[:, 1], rays[:, 0])),
        np.rad2deg(np.arctan2(rays[:, 2], np.hypot(rays[:, 0], rays[:, 1]))),
        s=8,
        marker=marker,
        alpha=0.6,
        label=label,
    )
axes[1, 0].set(
    title="First 100 ms: withheld rays",
    xlabel="Sensor azimuth [deg]",
    ylabel="Sensor elevation [deg]",
)
axes[1, 0].legend()
labels = list(model["withheld"])
z = np.arange(len(labels))
width = 0.25
for delta, key, name in [
    (-width, "median_deg", "median"),
    (0, "rms_deg", "RMS"),
    (width, "p95_deg", "95th percentile"),
]:
    axes[1, 1].bar(
        z + delta, [model["withheld"][v][key] for v in labels], width, label=name
    )
axes[1, 1].set(
    xticks=z,
    xticklabels=[
        "–".join(
            str(round(float(k.rstrip("s"))))
            for k in v.replace("withheld_", "").split("_")
        )
        for v in labels
    ],
    ylabel="Angular error [deg]",
    xlabel="Withheld interval [s]",
    title="Optics learned from first 30 s only",
)
axes[1, 1].legend(fontsize=8)
fig.suptitle(
    "Hardware-derived optics + sparse measured phase reconstruction\nClock anchors are observed across this session; these errors do not measure future-phase prediction",
    fontsize=12,
)
args.output.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(args.output, dpi=160)
print(
    json.dumps(
        {
            "later_withheld_count": len(errors),
            "rms_deg": float(np.sqrt(np.mean(errors**2))),
            "median_deg": float(np.median(errors)),
            "p95_deg": float(np.percentile(errors, 95)),
        },
        indent=2,
    )
)
