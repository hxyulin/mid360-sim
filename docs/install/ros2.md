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
explicit synthetic policies. The ordinary room demo does not publish IMU,
odometry or TF. The separate
[synchronized example](../guide/synchronized-sensors.md) demonstrates
caller-provided IMU measurements and optional native ROS publication.
For LIO, provide coherent IMU measurements from your simulator and configure
the consumer's acceleration units explicitly; see
[use with a navigation stack](../guide/navigation.md).

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

### Optional typed Livox endpoint

Enable `ros2-typed` to use `publisher::TypedLivoxPublisher`. It publishes the same
Driver 2 `CustomMsg` with fresh owned RMW-native storage, avoiding per-point
dynamic field lookup. It supports all-miss, singleton and larger packets, while
preserving first-emission timebases, retained offsets, local coordinates, device
IDs and synthetic reflectivity/tag/lane fields. It validates headers, bounded
counts, finite coordinates and increasing offsets before encoding. Use
`LivoxPacket::from_scan` to retain the acquisition validation contract.

This feature requires generated Livox Rust interfaces exported through `ros-env`.
The Pixi interface build supplies them in the ignored `.ros/install` tree. A
C-introspection-only installation can continue using `ros2` and
`publisher::LivoxPublisher`. Neither feature changes the dependency-free default
core; generated bindings are not checked into source. Only macOS arm64 DDS has
been validated locally; the platform lockfile does not establish Linux runtime
or generated-interface validation.

```sh
pixi run typed-test
pixi run typed-smoke
pixi run typed-smoke --stride 3
pixi run bash scripts/with-ros.sh cargo run --release --features ros2-typed --locked -- --frames 100
```

The room demo, synchronized example and transport example select the typed
Livox endpoint when compiled with `ros2-typed`. Builds with `ros2` alone retain
their dynamic endpoint. PointCloud2, IMU, TF and clock schemas/QoS remain the
same. Only one publisher should own `/clock`, including multi-device hosts.
Publishing can still block; use the same caller-owned bounded worker/queue
policy. See the [one/two-sensor evaluation](../single-sensor-performance.md).

### Dynamic singleton limitation

With the pinned rclrs 0.8 dynamic sequence implementation, `LivoxPublisher`
returns a descriptive error for packets containing exactly one retained point,
before entering the upstream sequence operation that would panic. Empty
(all-miss) packets and packets with two or more points remain supported. Use
`TypedLivoxPublisher` with `ros2-typed`, `PointCloudPublisher` (PointCloud2), or aggregate
acquisitions in the caller while preserving the packet timing contract. The
publisher never pads or duplicates measurements to bypass this limitation.

For local Cyclone DDS trials (the profile intentionally excludes remote hosts):

```sh
CYCLONEDDS_URI="file://$PWD/config/cyclonedds-loopback.xml" \
  pixi run smoke --rmw rmw_cyclonedds_cpp --discovery-range SYSTEM_DEFAULT
pixi run synchronized-smoke
```

The native smoke tests bound shutdown and validate dense packets after reception.
Fast DDS remains the ordinary publisher default. See the synchronized guide and
its evaluation for the observed automatic-interface failures on this Mac.
