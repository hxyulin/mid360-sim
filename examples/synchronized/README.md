<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Caller-owned synchronized sensors

Run `cargo run --release --example synchronized -- --evaluate` from the checkout
root. The [guide](../../docs/guide/synchronized-sensors.md) documents the kinematic
fixture, SI IMU contract and optional native ROS smoke test. The fixture is
example code, not a calibrated hardware model or a second dynamics world.

See the [navigation demo](../../docs/guide/navigation-demo.md) for optional
physical TF, PointCloud2 and evaluation with an external LIO node.
