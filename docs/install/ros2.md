<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# ROS 2 Jazzy

The optional Rust publisher uses rclrs 0.8 and Livox Driver 2 `CustomMsg`.
Pixi/RoboStack supplies ROS and message generators without a system ROS install.
Rust comes from rustup. Locks cover macOS arm64, Linux x86_64 and Linux arm64;
native DDS delivery has been tested on macOS arm64.

## Install and run

Install [Pixi](https://pixi.sh/) and [Rust](https://rustup.rs/), then from the
complete source checkout or extracted source archive:

```sh
pixi install --locked
pixi run smoke
pixi run run --frames 100
```

The build generates Livox interfaces in `.ros/` and compiles the Rust publisher.
The demo owns an analytic 20 × 20 × 4 m room. Its sensor translates and rotates
at each emission. It is not a bridge from an Isaac, MuJoCo or Gazebo scene.

## Consume messages

Subscribe to `/livox/lidar` with best-effort-compatible QoS and set
`use_sim_time=true`. Only one publisher should own `/clock`. Frame header and
`timebase` are the first emission time, even if it missed. `offset_time` stays
relative to that time; valid points are sensor-local at their own acquisition.

The demo defaults to 200,000 emissions/s, 10 Hz publication, 2 cm generic range
noise and 1% independent dropout. These are simulation choices, not calibrated
hardware parameters. `line = index % 4`, `tag = 0x10` and reflectivity 100 are
explicit synthetic policies. IMU, odometry and TF are not fabricated.

```sh
pixi run run --help
pixi run run --frames 100 --pattern nominal
pixi run run --frames 20 --angular-noise-deg 0.05 --range-noise-m 0.02
pixi run smoke --stride 3
```

## Embed the publisher

Enable the `ros2` Cargo feature and use `publisher::LivoxPublisher` outside the
physics thread. DDS publication may block; use a bounded queue. Interface and
runtime libraries must be sourced through `scripts/with-ros.sh` or an equivalent
ROS environment. Release source includes the pinned interface definitions;
there is no portable, ROS-free publisher executable.
