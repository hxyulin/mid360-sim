<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
## Start with your environment

| Environment | First step | Tested runtime |
|---|---|---|
| Rust | [Embed the core](install/rust.md) | Stable Rust, no default dependencies |
| ROS 2 | [Run the publisher](install/ros2.md) | Jazzy + rclrs 0.8; native DDS on macOS arm64 |
| Isaac Sim | [Enable the extension](install/isaac-sim.md) | 6.2.0 + Warp 1.17 on Linux / RTX 5090 |
| MuJoCo | [Install the adapter](install/mujoco.md) | 3.7.0 on macOS arm64 |
| Gazebo | [Build the plugin](install/gazebo.md) | Harmonic 8.15 + DART/Bullet on Linux |

The current LiDAR scope is feature-complete for scan generation, motion-aware
scene acquisition and the documented outputs. Your simulator owns dynamics and
IMU measurements; your navigation stack owns deskewing and localization. See
[use with a navigation stack](guide/navigation.md) for the integration contract.

Read [validation and limitations](guide/validation.md) before treating any
approximation as a hardware measurement. See [distribution](distribution.md)
for artifact contents and platform requirements.
