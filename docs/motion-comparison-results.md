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
