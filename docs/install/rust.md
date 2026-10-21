<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Rust

The default Rust library has no dependencies. ROS publishing is behind `ros2`;
the core needs no ROS, Bevy or simulator runtime. Use a current stable Rust
supporting edition 2024.

## Install from source

From a checkout or extracted complete source bundle:

```sh
cargo test --locked
cargo doc --no-deps --open
```

Use a local path dependency in your application:

```toml
[dependencies]
mid360-sim = { path = "../mid360-sim" }
```

For a remote deployment, pin a reviewed Git commit. There is no crates.io
release yet; the release `.crate` is a source package, not a binary.

## Supply scene queries

Implement `sensor::Scene::nearest_hit(time_ns, direction, max_range_m)`.
The query receives a local FLU unit direction and an explicit emission time.
Evaluate your mounted sensor pose and nearest scene surface at that time.
Return the nearest opaque hit even if it is inside the minimum range, so it
still occludes farther objects.

`Sensor::sample_into` writes caller-owned rolling frames. Preserve the epoch,
next emission index and seeded noise state across batches. See the runnable
examples in Rust API docs for `Sensor` and `NominalPattern`.

Generate local API documentation with `cargo doc --no-deps`. The core remains
independent of all optional simulator adapters; it does not advance physics.
