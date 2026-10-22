<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Realism and scope

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

The historical `official-table` fit uses four interleaved lanes and two rational-clock hypotheses,
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

The following effects belong to the Rust core unless a simulator adapter is
named explicitly. Adapter Livox fields do not imply that all core noise and
material options are implemented in that backend.

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
  The reusable `effects::AngularNoise` module exposes the same model for
  backends that already own acquisitions and their own range effects.
- Backend-provided reflectance/incidence records. An optional diffuse dropout
  stress model uses their product as survival probability. It is not calibrated,
  defaults off, and changes detection only; ROS reflectivity remains synthetic.
- Checked packet timing, explicit misses, first-return tags and bounded buffers.

## Current scope and optional research

The [capability summary](guide/status.md) tracks implemented interfaces and
validation gaps. Native motion tests use each backend's stated reference;
downstream LIO checks establish consumption by an estimator. Neither establishes
calibrated hardware returns. See [use with a navigation stack](guide/navigation.md)
for the caller/consumer boundary.

The caller owns robot dynamics, IMU synthesis, extrinsics and clocks. The
navigation stack owns deskewing, state estimation, place recognition and recovery.
The [synchronized examples](guide/synchronized-sensors.md) demonstrate this
boundary with an analytic fixture and an external RM host. Their IMU trajectory
is caller-owned example code; the core provides only an inertial value contract
and optional SI ROS publishing.

Keep estimator changes in the navigation project and use this package's
measurement contract at the boundary.

Further work can extend realism without changing that scope:

- **Articulated scenes:** the [synthetic jointed-scene regression](guide/acquisition.md#articulated-scene-regression)
  checks moving links, mounts and self-occlusion against independent triangles.
  It does not establish native full-robot fidelity; within-step moving-object
  motion remains approximate in the documented adapter modes.
- **Materials and edges:** measure infrared reflectance, grazing/near-range
  behavior and beam footprint before adding calibrated intensity or mixed-return
  policies. The [known-target validation workflow](guide/material-validation.md)
  audits complete emissions and held-out sessions; no suitable physical dataset
  is currently available here. The existing diffuse dropout option remains an
  uncalibrated stress model.
- **Devices and operating conditions:** additional synchronized captures can
  separate temperature, device clocks and correlated noise. The 18.2-hour nominal
  cycle is a simulation choice, not measured hardware recurrence or future phase.
- **Transport stress:** the [deterministic packet adapter](guide/packet-faults.md)
  adds latency, jitter, whole-packet loss and reordering on an explicit delivery
  timeline without rewriting acquisition timestamps. It is a synthetic test
  policy, not a measured network model.

Weather, multipath, interference and calibrated false returns are outside the
current indoor-navigation approximation. Neither this package nor the original
simulator establishes those hardware effects.

See the [acquisition contract](guide/acquisition.md) for current backend modes,
[validation guide](guide/validation.md) for accuracy definitions, and
[runtime/accuracy report](performance-tradeoffs.md) for measured native results.
The [controlled-motion experiment](motion-distortion.md) retains the earlier
analytic policy checks; it is a separate metric from native replay.
