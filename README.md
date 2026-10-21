<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# mid360-sim

Standalone deterministic MID-360 scan approximation in Rust, with optional
**rclrs 0.8 / ROS 2 Jazzy** publishing of Livox Driver 2 `CustomMsg` and `/clock`.
The core has no dependencies, ROS, Bevy, Rapier or rm-simulator requirement.
This is an empirical simulator, not a calibrated Livox hardware emulator.

Extracted from rm-simulator; copied implementation and measurement provenance
are retained in [NOTICE.md](NOTICE.md). No CAD or official 16.8 MB scan table is
redistributed. Source: [hxyulin/mid360-sim](https://github.com/hxyulin/mid360-sim).

```sh
cargo test --locked
cargo clippy --all-targets --locked -- -D warnings
pixi install --locked
pixi run smoke
pixi run run --frames 100
pixi run run --frames 20 --angular-noise-deg 0.05 --range-noise-m 0.02
```

Pixi/RoboStack supplies ROS and interface generators without a system ROS install.
Rust comes from rustup. Supported environment locks: macOS arm64, Linux x86_64
and Linux arm64. Native delivery is tested on macOS arm64 only. Generated files
stay in `.ros/`, `.pixi/` and `target/`. Fast DDS is the default RMW.
The smoke subscriber uses rclpy for independent DDS validation; the simulator
and publisher are Rust/rclrs. No hardware driver or SDK is needed.

`pixi run run` publishes a moving sensor inside an analytic 20 x 20 x 4 m room.
The sensor moves and rotates at each emission time. It is a demonstration scene,
not the RoboMaster arena or a robot dynamics simulation. Use `--fast` to disable
wall-time pacing; timestamps always come from the explicit simulation clock.
Defaults: 200,000 emissions/s, 10 Hz packets, 2 cm generic range noise, 1% generic
independent dropout, no angular noise. The library itself defaults to ideal
measurements. These defaults are simulation choices, not calibrated sensor data.

To embed, implement `sensor::Scene::nearest_hit(time_ns, direction, max_range_m)`.
It receives each emission's local sensor direction and timestamp. Use the actual
sensor pose and geometry at that time and return the nearest surface, even if
below the minimum range. This preserves motion distortion and self-occlusion.
The core never creates or steps a second physics world, reads wall time, or
publishes privileged poses. Mesh BVHs, sensor mounts and moving geometry belong
to the caller/backend. A `None` return means a miss; invalid surface records are
also suppressed. Geometry discontinuities must be handled by the backend.

`Sensor::sample_into` consumes global sample indices. Keep them continuous across
packets and checkpoint the epoch, next index, configuration and seed. Counter-based
noise remains identical under chunking and decimation. Odd strides retain all four
inferred lanes. `RollingFrame` preserves misses and timestamps. `LivoxPacket`
converts valid returns to sensor-local float32 XYZ with original nanosecond offsets;
`publisher::LivoxPublisher` supplies real rclrs dynamic DDS messages when `ros2`
is enabled. Subscriber QoS must accept best-effort delivery. Publishing may block;
use a bounded worker queue outside authoritative game/physics threads.

`CustomMsg` header/timebase use the first emission time, including a missed first
ray. Points use sensor FLU at their own emission, requiring consumer deskewing.
`line = index % 4`, `tag = 0x10`, constant configurable reflectivity (default 100)
and zero reserved bytes are explicit simulation policies. There is no fabricated
IMU, odometry or TF. Only one publisher should own `/clock`; consumers use
`use_sim_time=true`. This repository does not yet stream live rm-simulator scenes.
The existing simulator adapter/control bridge is retained there for compatibility;
this standalone package has no path/git dependency on that repository.

Read [realism and limitations](docs/realism.md) before selecting noise or material
settings. The optional `diffuse_dropout` model is a deliberately uncalibrated
stress-test heuristic and is disabled by default. Physical reflectance is not
inferred from visual RGB. Reported ROS reflectivity remains a placeholder even
when that detection heuristic is enabled.

For native checks: `pixi run test`, `pixi run smoke --stride 3`, and
`pixi run interfaces` followed by `pixi run bash scripts/with-ros.sh cargo clippy --all-targets --features ros2 --locked -- -D warnings`.
Dependency policy: `cargo deny --config deny.toml check`. The rclrs async-std
maintenance exception is scoped in `deny.toml` and described in `NOTICE.md`.
