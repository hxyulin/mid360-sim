<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Getting started

Choose the adapter that already has access to your scene. Each one shares the
nominal scan law, but its acquisition and output capabilities differ. See
[capabilities and status](status.md) for the overall scope and validation gaps.

| Environment | Install guide | Motion within an acquisition batch | ROS output |
|---|---|---|---|
| Rust | [Core library](../install/rust.md) | Scene implementation owns each emission | Optional separate publisher |
| ROS 2 | [Room demo](../install/ros2.md) | Analytic per-emission sensor pose | CustomMsg + /clock |
| Isaac | [Kit extension](../install/isaac-sim.md) | Per-ray sensor interpolation; per-step geometry or explicit history/partition modes | Caller bridge from Livox fields |
| MuJoCo | [Python package](../install/mujoco.md) | Held poses by default; optional rolling sensor poses; geometry current per call | Caller bridge from Livox fields |
| Gazebo | [System plugin](../install/gazebo.md) | Physics-step scene snapshots | Native Gazebo messages |

## Acquisition comes before publishing

At full rate, 200 rays span 1 ms and 20,000 rays span 100 ms. To simulate motion,
query small batches after physics steps and aggregate for publication. A single
100 ms frozen query does not become a rolling scan by adding timestamps.

Keep global sample indices continuous, including misses. Publish local XYZ with
its original acquisition time; do not pre-deskew it if the consumer is testing
motion compensation. See the [acquisition contract](acquisition.md).

## Choose a motion/performance policy

Default settings favor straightforward scene integration. Isaac can defer
readback without coarsening acquisition, or batch tracing with explicit static
geometry and moving partitions. MuJoCo's `scan_rolling()` trades extra CPU work
for per-emission sensor rotation and bounded origin approximation. Gazebo's
physics-step snapshots improve with shorter steps. See the
[acquisition contract](acquisition.md) and [measured tradeoffs](../performance-tradeoffs.md)
before applying a fast static-scene configuration to a moving robot.

## Connect navigation

Publish raw local returns with their acquisition times; let the navigation stack
deskew them. For LiDAR–inertial odometry, the caller also supplies IMU samples,
clock alignment and mounting extrinsics. Start with
[use with a navigation stack](navigation.md) before changing scan or physics rates.

## Source or release

Install from a checkout while developing adapters. Use the checksummed
[release artifacts](../distribution.md) for pinned deployments. The Python
package is not currently published on PyPI and the Rust package is not currently
published on crates.io. Avoid an unqualified registry install of either name.
Guides track `main`; advanced history/partition/rolling APIs describe the current
checkout and may be newer than a downloaded archive. Match an archive to its
`release.json` source revision rather than assuming the shared 0.1.0 package
version contains every current feature.
