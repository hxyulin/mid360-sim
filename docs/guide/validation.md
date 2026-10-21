<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Validation and limitations

The model is useful for navigation development, acquisition timing and deskew
experiments. It is not a calibrated MID-360 return emulator.

## What has been checked

| Check | Evidence | What it does not establish |
|---|---|---|
| Real scan timing and optics | [Stationary capture](../hardware-validation.md), [fit](../hardware-fit.md) | Motion behavior or a universal device calibration |
| Independent optical transfer | [Two public recordings](../external-captures.md) | Exact future phase or isolated manufacturing variation |
| Rolling pose policy | [MoCap comparison](../motion-comparison-results.md) | Actual simulator callback/geometry fidelity |
| Ideal acquisition semantics | [Controlled motion test](../motion-distortion.md) | Real range noise or dynamics |
| Official simulator | [Source audit](../official-reference.md) | Native Gazebo 9 execution or benchmark |

Native adapter checks use analytic geometry: Isaac on Linux/RTX 5090, MuJoCo on
macOS arm64 and Gazebo Harmonic in an isolated Linux container. Their timing
numbers use different hardware and scopes; installation guides retain the
conditions rather than presenting a CPU/GPU ranking.

## Open realism work

Moving robot self-geometry and obstacles must follow each acquisition time.
Synchronized IMU output, time/extrinsic calibration, material return behavior,
vibration and correlated noise still need work. Visual RGB is not 905 nm
reflectance. Synthetic tag/reflectivity fields are not measured material signals.

Our 128 Hz interpolation policy added about 1.9 mm RMS point displacement
relative to the sampled MoCap reference in one moving recording. That is not a
1.9 mm total real-sensor error bound. The dataset has packet timestamp anomalies
and tracking uncertainty; see the [GT audit](../motion-ground-truth.md).
