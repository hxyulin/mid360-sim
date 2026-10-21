<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Getting started

Choose the adapter that already has access to your scene. Each one shares the
nominal scan law, but its acquisition and output capabilities differ.

| Environment | Install guide | Motion within an acquisition batch | ROS output |
|---|---|---|---|
| Rust | [Core library](../install/rust.md) | Scene implementation owns each emission | Optional separate publisher |
| ROS 2 | [Room demo](../install/ros2.md) | Analytic per-emission sensor pose | CustomMsg + /clock |
| Isaac | [Kit extension](../install/isaac-sim.md) | Sensor interpolation; scene frozen | Caller bridge from Livox fields |
| MuJoCo | [Python package](../install/mujoco.md) | Pose and geometry frozen per call | Caller bridge from Livox fields |
| Gazebo | [System plugin](../install/gazebo.md) | Physics-step scene snapshots | Native Gazebo messages |

## Acquisition comes before publishing

At full rate, 200 rays span 1 ms and 20,000 rays span 100 ms. To simulate motion,
query small batches after physics steps and aggregate for publication. A single
100 ms frozen query does not become a rolling scan by adding timestamps.

Keep global sample indices continuous, including misses. Publish local XYZ with
its original acquisition time; do not pre-deskew it if the consumer is testing
motion compensation. See the [acquisition contract](acquisition.md).

## Source or release

Install from a checkout while developing adapters. Use the checksummed
[release artifacts](../distribution.md) for pinned deployments. The Python
package is not currently published on PyPI and the Rust package is not currently
published on crates.io. Avoid an unqualified registry install of either name.
