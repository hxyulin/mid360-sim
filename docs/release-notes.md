<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# mid360-sim 0.1.0

The first multi-simulator release packages a representative MID-360 scan law,
explicit rolling acquisition times and optional scene adapters.

- Dependency-free Rust core with seeded, batching-invariant sensor effects.
- rclrs / ROS 2 Jazzy room demo publishing Livox Driver 2 messages and /clock.
- Isaac Sim 6.2 Warp extension, MuJoCo 3.7 Python adapter and Gazebo Harmonic plugin.
- Compact four-lane nominal optics with a representative 18.2-hour cycle.
- Separate installation guides, acquisition contracts and reproducible validation.

Read [installation](https://hxyulin.github.io/mid360-sim/guide/getting-started.html)
and [artifact contents](https://hxyulin.github.io/mid360-sim/distribution.html).
Verify downloads against SHA256SUMS. Registry publication is not part of this release.

This is an empirical approximation, not a calibrated Livox emulator. Geometry
motion differs by adapter; use short acquisition batches. IMU output, coherent
per-emission moving self-geometry and calibrated material response remain open.
The MoCap comparison validates a pose-interpolation policy, not complete runtime
hardware equivalence. See [validation](https://hxyulin.github.io/mid360-sim/guide/validation.html).
