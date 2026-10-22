<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# mid360-sim 0.1.0

These are draft notes for the first multi-simulator release candidate, not a
record of an uploaded release. They describe current source; review them with
the exact tag/revision before publishing. See [distribution](distribution.md)
for availability and artifact verification.

The candidate packages a representative MID-360 scan law, explicit acquisition
times and optional scene adapters.

- Dependency-free Rust core with seeded, batching-invariant sensor effects.
- rclrs / ROS 2 Jazzy room demo publishing Livox Driver 2 messages and /clock.
- Isaac Sim 6.2 Warp extension, MuJoCo 3.7 Python adapter and Gazebo Harmonic plugin.
- Compact four-lane nominal optics with a representative 18.2-hour cycle.
- Isaac deferred readback, static bulk histories and explicit moving partitions.
- Optional MuJoCo rolling sensor poses with bounded origin approximation.
- Separate installation guides, acquisition contracts and native backend comparisons.

Read [installation](https://hxyulin.github.io/mid360-sim/guide/getting-started.html)
and [artifact contents](https://hxyulin.github.io/mid360-sim/distribution.html).
Verify downloads against SHA256SUMS. Registry publication is not part of this release.

This is an empirical approximation, not a calibrated Livox emulator. Geometry
motion differs by adapter; use short acquisition batches. IMU output, coherent
per-emission moving self-geometry and calibrated material response remain open.
Offline pose-policy tests and native simulator replay are separate evidence;
the latter uses measured motion with exact synthetic geometry, not a calibrated
hardware range reference. See [current runtime/accuracy results](performance-tradeoffs.md). See [validation](https://hxyulin.github.io/mid360-sim/guide/validation.html).
