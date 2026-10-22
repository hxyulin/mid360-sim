<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Validation and limitations

The model supports navigation development, acquisition timing and deskew
experiments. It is an empirical scan approximation, not a calibrated MID-360
return emulator. Start with the [current runtime/accuracy comparison](../performance-tradeoffs.md)
for simulator measurements; use the evidence below for the question you need answered.

## What each accuracy number means

| Question | Evidence | Measured quantity | Limit |
|---|---|---|---|
| Does the scan resemble hardware? | [Stationary capture](../hardware-validation.md), [fit](../hardware-fit.md), [independent captures](../external-captures.md) | Angular fit, clocks and coverage | Not universal calibration or future phase prediction |
| Does a pose policy preserve recorded motion? | [Recorded-return comparison](../motion-comparison-results.md) | Point displacement caused by pose assignment | An offline policy test, not native simulator acquisition |
| Do the actual adapters preserve motion? | [Native MoCap replay](../backend-ground-truth.md) | Range difference against analytic room intersections at interpolated MoCap poses | Real motion, synthetic scene; no real-range accuracy bound |
| Does native RTX preserve motion? | [RTX experiments](../rtx-reference.md) | Range error, completeness and timestamp behavior | Custom repeating profile; not the MID-360 scan pattern |
| How does the official plugin behave? | [Official audit and native execution](../official-reference.md) | Snapshot agreement and an explicitly assigned rolling correspondence | Official messages lack per-point timestamps |
| Do ideal acquisition semantics work? | [Controlled motion](../motion-distortion.md) | Numerical geometry, time and deskew checks | No calibrated hardware noise or dynamics |
| Can I turn real scans into optical geometry? | [ETH3D Office worked example](scanned-geometry.md) | Held-out Faro range agreement, missed rays and native MuJoCo replay | Two training viewpoints; excluded-point reconstruction, not unseen-view or real MID-360 accuracy |

The native replay uses a two-second high-motion segment of independent 100 Hz
MoCap in an exact synthetic room. Its reference interpolates between measured
poses. The **0.251 mm Warp RMS and 0.294 mm MuJoCo rolling RMS describe agreement
with that reference**, not physical sensor precision. Nor do they measure the
nominal scan pattern's angular fit error: each direction is evaluated against
its own geometric intersection. This is a stress sample, not an average over
the full recording. See the [methodology](../backend-ground-truth.md).

The earlier approximately 1.9 mm result is a different quantity: extra point
displacement from a 128 Hz pose policy applied to recorded returns. It is not
an older estimate of the same native range metric. Dataset clock anomalies
and tracking uncertainty are covered in the [GT audit](../motion-ground-truth.md).

## Runtime comparisons need their scope

The [current comparison](../performance-tradeoffs.md) includes actual native
execution of Livox's pinned ROS Melodic / Gazebo 9 scan callback. Our headless
Warp bulk-history path is faster in that fixture, with finer motion sampling.
That configuration needs static optical geometry, caller-owned poses and
disabled USD writeback. It is not the default USD-mounted extension.

MuJoCo timings omit dynamics integration; RTX includes rendering and a longer
warmup/drain loop; the quoted Harmonic throughput uses a separate warmed scene.
These are documented acquisition-policy measurements, not equivalent full-app
loads or hard real-time guarantees. Older [Isaac architecture probes](../isaac-performance.md)
remain useful evidence, but are labeled separately from the current comparison.

## What remains open

Isaac includes tested explicit static/dynamic optical partitions and rigid
transform histories. Those tests cover moving geometry and self-occlusion in
synthetic and CAD scenes; they do not establish articulated full-robot fidelity.
Within-step moving-object interpolation remains approximate. MuJoCo's optional
rolling API interpolates the sensor, while geometry is current for the call;
Gazebo holds sensor and geometry per physics step.

Synchronized IMU output, time/extrinsic calibration, calibrated material returns,
vibration and correlated noise remain open. Visual RGB is not 905 nm reflectance.
Synthetic tags and reflectivity fields are transport fixtures, not measured
material signals. Full dynamic robot scenes and publication/consumer latency
need evaluation before extending the simple-room performance claim.
