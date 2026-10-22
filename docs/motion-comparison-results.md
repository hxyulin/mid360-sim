<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Recorded-motion comparison: JEPLO EIL_Mask2 (2026-10-22)

Our linear-position/quaternion-SLERP acquisition policy closely follows the
interpolated independent motion-capture trace in this recording: **1.86 mm RMS
point displacement at 128 Hz**, versus **118.75 mm** for a single pose per
approximately 50 ms cloud. This validates the interpolation policy against a
sampled trajectory; it is not a calibrated hardware error bound or an end-to-end
Isaac/Gazebo execution test.

Source, immutable revision, file hashes, observed clock anomalies and frame
conventions are recorded in the [ground-truth investigation](motion-ground-truth.md).
The dataset is [JEPLO](https://huggingface.co/datasets/ASIG-Lab/JEPLO-Dataset),
with independent indoor motion capture expressed in the LiDAR frame.

## Protocol and sample

- Preserve original `timebase + offset_time`, including nonmonotonic offsets.
  Do not fit optics, adjust clocks or perform per-cloud registration.
- Use one point in 11, retaining all four lanes. Reject nonfinite points,
  ranges outside 0.5–15 m, and complete clouds without sufficient GT coverage.
- Reference motion is position interpolation and quaternion SLERP between the
  original MoCap poses (median approximately 10 ms apart). Reject any GT gap
  over 20 ms intersecting the cloud plus a 16.667 ms interpolation margin.
- Retain 467 of 670 clouds. Every fifth original cloud index is reserved for
  training; the remaining 371 clouds provide 149,986 evaluation points.
- Report higher motion separately: the top third of held-out clouds by net
  start-to-end angular speed, at least 54.27 deg/s. This contains 124 clouds
  and 50,810 sampled points. It is not an instantaneous gyro-speed threshold.
- Sample the reference trajectory at 60/128/240/1000 Hz on a fixed grid whose
  origin is the first GT timestamp. Interpolate those poses at each point time.
  These are policy evaluations, not timings or live simulator outputs.

## Added point error relative to the MoCap-based reference

Transform the same recorded local point using each candidate pose and compare
it with its transform under the reference per-point pose. This measures pose
approximation's effect on measured returns; it does not compare independent
nominal rays point-for-point or raycast a reconstructed room.

| Pose policy | All held-out RMS | Higher-motion RMS | Higher-motion p95 |
|---|---:|---:|---:|
| Frozen pose at cloud start | 81.00 mm | 118.75 mm | 244.45 mm |
| Frozen pose at cloud midpoint | 43.10 mm | 62.15 mm | 129.84 mm |
| Interpolation at 60 Hz | 5.27 mm | 6.02 mm | 12.43 mm |
| Interpolation at 128 Hz | 1.79 mm | 1.86 mm | 3.64 mm |
| Interpolation at 240 Hz | 0.62 mm | 0.68 mm | 1.27 mm |
| Hold pose per 1 ms | 1.71 mm | 2.37 mm | 4.87 mm |
| Interpolation at 1000 Hz | 0.078 mm | 0.084 mm | 0.037 mm |

The submillimetre numbers describe numerical agreement with an interpolated
100 Hz trace, **not independently measured physical motion at that resolution**.
The 1000 Hz p95 is smaller than RMS because most intervals follow one reference
segment exactly; errors concentrate near reference knots. Raw MoCap noise and
interpolation uncertainty remain in the reference.

## Check against real scene structure

Fit one fixed floor plane using 5,872 points from separate training clouds in
the lowest third of training angular speeds. Use Huber IRLS with 30 mm tuning;
the initial training ROI is world height within 150 mm of zero, range below 8 m.
The fitted plane is approximately `-0.00799*x -0.00961*y +0.99992*z = 0.00627 m`.
It is an estimated scene reference, not an independently surveyed plane.

Label evaluation floor candidates using **one cloud-midpoint pose**, range
below 8 m and signed plane distance within 200 mm. This selection does not use
the per-point transform being tested. It can still contain low clutter and
favors the midpoint control. All policies evaluate exactly the same 18,728
higher-motion candidate returns, without refitting the plane.

| Transform of real returns | Higher-motion plane residual RMS |
|---|---:|
| Single start pose / no within-cloud deskew | 91.33 mm |
| Single midpoint pose | 63.01 mm |
| Per-point MoCap reference | 61.67 mm |
| Our 60 Hz interpolation | 61.35 mm |
| Our 128 Hz interpolation | 61.57 mm |

Per-point deskew reduces residual RMS by **32.5%** versus the start-pose control.
A paired cloud bootstrap (2,000 resamples, fixed seed) gives a 24.8–34.9 mm
95% interval for the reduction; temporal dependence between clouds remains.
The tiny improvement of 60/128 Hz over the MoCap reference is not evidence that
our model is more accurate; smoothing/noise and the estimated plane can do this.

An earlier diagnostic selected floor points using per-point GT and a 150 mm
band. It produced 87.8 → 44.4 mm, but that selection favors the GT result.
The more conservative midpoint-selected result above supersedes it.

## Uncertainty and practical conclusion

- Moving the GT query time by +/-10 ms changes the reference floor RMS from
  61.67 mm to 69.82 / 62.30 mm. A +5 ms diagnostic gives 60.70 mm. We do not
  select that offset as calibration or rewrite timestamps.
- The approximately 62 mm real floor residual includes surface selection,
  scene nonplanarity, range error, pose/extrinsic uncertainty and recorded time
  anomalies. It must not be attributed entirely to our simulation.
- A correct mounted callback, coherent robot self-geometry, moving obstacles,
  synchronized IMU and range/material response remain unvalidated by this test.

At 128 Hz, interpolation error is small relative to this recording's residual
uncertainty. Prioritize coherent moving scene captures and timestamp validation
before making the harmonic scan pattern more exact. There is no evidence here
that a substantially faster interpolation rate is needed for this trajectory.

## Reproduce

Use `requirements-analysis.txt` and the unchanged files identified in the
source investigation. The comparison writes only a new output directory:

```sh
python scripts/compare-motion-ground-truth.py /path/EIL_Mask2_0.db3 \
  /path/EIL_Mask2.txt --output local-assets/motion-ground-truth/comparison-new
python -W error scripts/test-motion-comparison.py
```

`report.json` contains hashes, masks/counts, thresholds, per-policy metrics,
time-offset sensitivity, bootstrap interval and paired per-cloud statistics.
`evaluation.npz` retains sampled points, masks and candidate transforms for
plotting. Final local results and PNG/SVG plots are in ignored
`local-assets/motion-ground-truth/comparison-v3/`.

## Cheap interpolation follow-up

A decomposition of the same higher-motion sample attributes 1.858 mm RMS to
orientation approximation alone and 0.135 mm to translation alone. These are
separate counterfactuals, not independent variances to add. Baseline RMS grows
with return range: 0.804 mm at 0.5–2 m, 1.957 mm at 2–5 m and 2.949 mm at 5–15 m.

All baseline squared error is in acquisition intervals that cross a reference
MoCap knot (77.7% of these points). Away from those knots, the reference itself
is already linear/SLERP, so the baseline reproduces it exactly. This makes the
1.9 mm strongly dependent on the chosen reference interpolation and sampled
tracking noise; it is not a measured simulator numerical defect.

A causal three-pose quadratic orientation estimate needs one additional
historical pose and the already available interval endpoints. In start-local
rotation-log coordinates, blend half its curvature correction into SLERP. An
alignment guard falls back to SLERP if previous/current angular increments do
not have a positive dot product. Translation stays linear.

| Exploratory 128 Hz policy | Higher-motion RMS | p95 | Maximum | Real-floor RMS |
|---|---:|---:|---:|---:|
| Baseline | 1.860 mm | 3.637 mm | 29.184 mm | 61.565 mm |
| Half orientation curvature | 1.636 mm | 3.182 mm | 24.004 mm | 61.628 mm |
| Guarded half curvature | 1.620 mm | 3.104 mm | 24.004 mm | 61.620 mm |

The guarded candidate lowers RMS by 12.9% (0.240 mm), improving 110 of 124
higher-motion clouds. This is exploratory reuse of an already inspected
held-out set; it is not new independent validation. Four fixed grid phases
are also evaluated rather than optimizing an alignment to the GT clock.
No callback scheduling, input timestamps, optical model or simulator runtime
has been changed. No native GPU throughput result is claimed for the candidate.

Controlled fixed-axis acceleration at 720 deg/s² halves the interpolation
error with the half correction (0.700 → 0.350 mm RMS at 10 m). But a stop exactly
at the interval boundary reveals unguarded overshoot: 15.34 mm peak versus zero
baseline error. The guard removes that particular stop artifact; it is not a
proof of safety under arbitrary impacts, reversals or changing rotation axes.

Keep the release runtime default. The numerical gain is small compared with
recording uncertainty, and real-floor residual does not improve. A future
opt-in implementation should validate abrupt dynamics, coherent moving scene
geometry and actual per-ray GPU cost before becoming a default. Preserving
existing capture knots can avoid unnecessary resampling, but aligning to this
recording's approximately 100 Hz GT is not a general physical improvement.

Reproduce without downloading or decoding the bag again:

```sh
python scripts/investigate-motion-interpolation.py \
  local-assets/motion-ground-truth/comparison-v3/evaluation.npz \
  /path/EIL_Mask2.txt --output local-assets/motion-ground-truth/new-interpolation.json
python -W error scripts/test-motion-comparison.py
```

The report includes decomposition, range bins, inherited train/evaluation masks,
per-cloud improvement counts, grid phases, floor residuals and controlled cases.
The added math regressions cover stop fallback, smooth acceleration, endpoints
and quaternion-sign invariance. The default runtime continues to use SLERP.

The later [native backend replay](backend-ground-truth.md) runs all three actual
adapters on a shared measured-motion segment inside an exact synthetic enclosure.
