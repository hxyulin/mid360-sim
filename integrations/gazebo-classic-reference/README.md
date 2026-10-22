<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Native upstream comparison harness

This executes Livox's actual `OnNewLaserScans()` callback and ROS publication
in Gazebo 9 / ROS Melodic. It is a controlled synchronous callback harness,
not an unmodified roslaunch/sensor-thread throughput test. The subclass only
exposes the protected callback; the vendor acquisition implementation is unchanged.
See [results and limitations](../../docs/performance-tradeoffs.md).

Supply an external checkout of
[Livox-SDK/livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation/tree/1cce1073633a062b92e30243a4c2920e45551bb5)
at **`1cce1073633a062b92e30243a4c2920e45551bb5`**, including its MID-360 CSV.
No vendor source, scan table or measured trajectory is redistributed here.
The source retains its upstream license. Verify at least the acquisition source:

```
93ee98d13fb67e248574d0d77f633c64b54900dbee4a84385efa329fccad1c02  src/livox_points_plugin.cpp
ebff472657913be2c4a649c9958921809d8a0ff8d71851a47c0a52b28eb7286f  src/livox_ode_multiray_shape.cpp
```

Generate the [MoCap fixture](../../docs/backend-ground-truth.md) at 1 ms.
Its generated CSV has 2,001 whitespace-separated rows:
`time_ns x_m y_m z_m qx qy qz qw`. Then, on Linux with Docker:

```sh
integrations/gazebo-classic-reference/run.sh /path/to/upstream /tmp/official /path/to/mocap-fixture.csv
```

The pinned ROS image must be available locally (`docker pull` the digest shown
in `run.sh` first). Builds run without network access; all dependencies already
exist in that image. A copied source tree replaces the obsolete hard-coded
`libprotobuf.so.9` linker spelling with `protobuf`; no acquisition C++ changes.
Build and source-loading time are excluded. The world advances 2,000 native
1 ms steps after warmup, calling upstream every 100 steps. All 20×20,000 XYZ
points must arrive. Publication and subscriber XYZ writes are exercised; a
bounded final ROS drain is outside the loop timer. No renderer runs.

We favor the official implementation with `range.min=0`, removing the measured
100 mm shortfall at its original `0.1` setting in this enclosure. Change
`prepare.py --min-range .1` to reproduce that configuration separately.
Minimum range zero here is a benchmark policy, not a recommended sensor model.
The room has no near surfaces. The callback supplies no per-point timestamps;
`scripts/evaluate-official-native.py` explicitly assigns nominal 5 µs offsets
only for a rolling-motion correspondence diagnostic.
