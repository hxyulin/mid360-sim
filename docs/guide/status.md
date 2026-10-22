<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Capabilities and status

The current LiDAR scope is implemented: scan generation, scene acquisition,
per-emission timing and the documented outputs. This is an empirical MID-360
approximation, with backend-specific acquisition policies. It does not establish
hardware-equivalent returns or lossless delivery under every workload.

This page describes the current source checkout. For pinned deployments, match
[release artifacts](../distribution.md) to their source revision; a shared package
version does not imply that an older archive contains every current API.

## Implemented

| Capability | Where to use it | Boundary |
|---|---|---|
| Compact scan generation and checked ray/return timing | [Rust core](../install/rust.md), [patterns](patterns.md) | No default dependencies; callers own the scene and simulation clock |
| Indexed range noise, dropout and angular perturbations | [Realism and effects](../realism.md#implemented-here) | Generic stress models, not calibrated device distributions; adapter support differs |
| Motion-aware scene acquisition, including dynamic mounts | [Acquisition policies](acquisition.md) | Per-ray, held-pose and geometry-history semantics differ by backend |
| GPU optical mesh tracing | [Isaac Warp adapter](../install/isaac-sim.md) | Runs in the caller's simulator; no Rust GPU runtime in the core |
| MuJoCo and Gazebo scene queries | [MuJoCo](../install/mujoco.md), [Gazebo](../install/gazebo.md) | Use each backend's documented geometry and time policy |
| Livox CustomMsg, timed PointCloud2, SI IMU and physical TF | [ROS installation](../install/ros2.md), [navigation demo](navigation-demo.md) | Optional rclrs publishers; callers supply IMU values, transforms and stamps |
| Synchronized LiDAR/IMU examples and external RM consumption | [Synchronized sensors](synchronized-sensors.md) | Example-owned kinematics or the existing external world; no RM dependency |
| Deterministic packet delay, jitter and loss | [Packet fault adapter](packet-faults.md) | Caller-owned delivery timeline and bounded queue; unchanged sensor payloads |
| Known-target measurement auditing | [Material/edge workflow](material-validation.md) | Synthetic tooling checks until physical measurements are supplied; no calibrated material law |
| Motion, downstream-consumer and transport checks | [Validation](validation.md), [transport benchmarks](transport-benchmarks.md) | Each report states its reference, workload and tested platform |

Raw local points retain their emission times, including motion skew. Callers own
robot dynamics, optical geometry, pose interpolation, acquisition scheduling,
IMU synthesis and extrinsics. Navigation consumers own deskewing, odometry,
localization and recovery; see [connect a navigation stack](navigation.md).

## Validated scope and open work

**Motion and geometry:** controlled tests, recorded-motion comparisons and native
backend replays pass against their stated references. Dynamic rotations and
explicit moving-geometry partitions are supported. These tests do not establish
full articulated-robot fidelity; within-step moving-object motion remains
approximate in the documented modes. The [articulated-scene regression](acquisition.md#articulated-scene-regression)
checks a synthetic mast/two-link mechanism against an independent triangle
reference; native full-robot validation remains broader work.

**ROS and consumers:** native delivery and the runnable external LIO regression
have been exercised on macOS arm64. The cross-platform lockfile does not establish
Linux DDS runtime validation. Long-duration drift, global localization and Nav2
obstacle-layer behavior are outside the current short compatibility checks.
See [navigation results](../navigation-evaluation.md) for the tested estimator,
source revisions and limits.

**Transport:** normal delivery has passed in the recorded configurations, but
loaded runs have also exposed missing deliveries and bounded-queue losses.
External RM stage telemetry is now implemented; it diagnoses where loss occurs,
not a fix for overload. Sustained tests with the intended native consumer under
controlled load, followed by Linux runtime checks, are the main validation
follow-ups. Connection/codec changes still need measured A/B comparisons;
[transport evidence](../transport-evaluation.md#current-follow-up-status) does
not yet justify a replacement middleware.

**Hardware realism:** optics and timing fits have explicit data provenance;
fit residuals, measured specifications and generic noise are different quantities.
Material response, beam footprint, correlated noise and device/temperature
variation need additional measurements before calibrated models can be claimed.
Visual RGB is not 905 nm reflectance. See [realism and research scope](../realism.md).

The [packet fault adapter](packet-faults.md) now supplies deterministic delay,
loss and reordering without changing acquisition. Calibrated material effects
remain pending physical measurements; the [validation workflow](material-validation.md)
defines the required evidence and held-out-session checks.

## Read the evidence for the question

Use [validation and limitations](validation.md) to distinguish hardware-fit,
geometry, motion and consumer metrics. Historical results retain their original
source identities and conditions; newer implementations do not retroactively
change those measurements. Installation commands live in the
[environment guides](getting-started.md), with research reports linked separately.
