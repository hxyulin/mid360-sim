<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Realism assessment and roadmap

## What the public data establishes

[Livox specifications](https://www.livoxtech.com/mid-360/specs) describe 200,000
first-return points/s, typical 10 Hz output, 360 degree horizontal coverage,
-7 to +52 degree vertical coverage and a 0.1 m blind zone. Range precision is
specified as at most 2 cm (1 sigma) at 10 m on an 80% target at 25 C; at 0.2 m
it is at most 3 cm. Angular precision is below 0.15 degrees (1 sigma).
These are test conditions/bounds, not a complete probability distribution.
In particular 0.15 degrees must not be treated as measured Gaussian sigma.

Detection range is specified at 100 klx as 40 m on a 10% target and 70 m on an
80% target. This does not uniquely identify an inverse-square detection law,
incidence curve or stochastic dropout model. Low-reflectivity/tiny surfaces
within 0.1-1 m have additional detection caveats. The large embedded-laser
emission angle in the safety footnote must not be used as the effective
per-return angular footprint of a simulated ray without optical evidence.

The official simulation at revision 1cce1073633a062b92e30243a4c2920e45551bb5
replays an 800,000-row CSV modulo its size. It provides no analytical optics
model or per-ray calibrated clock. The table spans four nominal seconds at
200,000/s; its xacro defaults instead to 24,000 samples at 10 Hz (3.33 s to wrap).
The source has no cycle-dependent quarter reconstruction or phase offset.

Our fit uses four interleaved lanes and two rational-clock hypotheses,
2189/12 and 10 Hz, with 22 Fourier pairs per lane/angle. It stores 2,880 bytes
of coefficients. Training uses 10,812 points from the first two seconds;
held-out validation covers all 400,000 remaining points. Lane RMS errors are
0.05594, 0.01312, 0.01312 and 0.05595 degrees; maximum 0.15549 degrees.
The inferred model repeats after 12 s; that extrapolation is not measured.
Fit error and physical angular noise are separate and must not be conflated.

Reproduce the fit (NumPy required; CSV must match the pinned checksum):

```sh
mkdir -p local-assets
curl -fL https://raw.githubusercontent.com/Livox-SDK/livox_laser_simulation/1cce1073633a062b92e30243a4c2920e45551bb5/scan_mode/mid360.csv -o local-assets/mid360.csv
python scripts/fit-mid360-pattern.py local-assets/mid360.csv --rust-output local-assets/coefficients.rs --report local-assets/fit.json
```

## Hardware check

A [300-second stationary capture](hardware-validation.md) does not confirm the
fixed official-table trajectory as a calibrated hardware model. It shows
approximately +181.004 Hz azimuth and 9.80787 Hz elevation carriers, a convention
mismatch, and no exact 12-second recurrence. The CSV validation above must not
be presented as hardware validation. The original profile remains selectable as `official-table`; the default is
now the simplified representative `nominal` hardware profile.
The separate [hardware-derived profile](hardware-fit.md) fits optics with disjoint packets and exposes both
mean clocks and a bounded recorded-clock reconstruction. Its measured-phase
reconstruction accuracy must not be treated as future-phase prediction.

## Implemented here

- Per-emission scene queries with explicit times and local FLU rays. A backend
  can supply interpolation of real robot/obstacle motion and self-occlusion.
  The standalone demo performs continuous analytic sensor motion; arbitrary
  physics interpolation is deliberately not implemented a second time here.
- Seeded Gaussian range noise, bias and independent dropout, invariant to
  batching/decimation. Below-minimum surfaces occlude farther surfaces.
- Optional independent tangent-plane angular perturbations, normalized and
  applied BEFORE intersection. Noise therefore affects edge occlusion as well
  as reported XYZ. Zero by default; this is a generic emitted-ray model,
  not a model of measured encoder uncertainty or correlated hardware optics.
- Backend-provided reflectance/incidence records. An optional diffuse dropout
  stress model uses their product as survival probability. It is not calibrated,
  defaults off, and changes detection only; ROS reflectivity remains synthetic.
- Checked packet timing, explicit misses, first-return tags and bounded buffers.

## Highest-value next work

1. **Live sensor geometry backend.** Use optical meshes independent of gameplay
   collision proxies, including chassis self-shadowing and moving mechanisms.
   Game collision simplification can otherwise erase rails or close openings.
   Avoid a second dynamics world: query timestamped scene captures.
2. **IMU + synchronization.** Navigation pipelines such as LiDAR-inertial odometry
   need timestamped gyro/accelerometer signals with noise, bias and drift, correct
   extrinsics, gravity and clock relationships. Do not publish perfect poses as
   sensor measurements. This matters more than adding more Fourier coefficients.
3. **Calibrated materials.** Measure white/black surfaces, metal, grazing angles
   and near-range detection before tuning response/intensity. Keep RGB separate
   from 905 nm reflectance. Replace the opt-in heuristic with data, not guessed
   hard cutoffs derived from two maximum-range specifications.
4. **Beam/edge behaviour.** A small ray bundle can model partial hits/edge mixing,
   at a multiplication of raycast cost. Validate the footprint and return-selection
   policy against recordings before claiming MID-360 fidelity.
5. **Long recordings.** Capture more than 12 s to assess motor drift, exact lane
   order/timing, long-period recurrence and correlation. The official four-second
   table cannot validate these. Offer reference CSV replay for comparison.
6. **Transport effects.** Optional latency/jitter, loss and reordering belong in
   publication/transport scheduling without changing acquisition timestamps.

Prioritize geometry + IMU + real recordings for navigation. Weather, multipath,
interference and false alarms are lower priority for an indoor arena. Neither
this package nor the original simulator currently provides these calibrated
hardware effects.

See [motion distortion](motion-distortion.md) for the per-backend acquisition
comparison, reproducible rolling-motion errors and moving-hardware validation gaps.
