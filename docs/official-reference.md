<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Official Livox reference: motion audit (2026-10-22)

This audit and the native execution below pin Livox-SDK/livox_laser_simulation
at `1cce1073633a062b92e30243a4c2920e45551bb5`. They do not describe every
upstream revision or fork.

The initial source/proxy audit below is retained as historical evidence.
We subsequently compiled and executed this exact upstream scan callback in
ROS Melodic / Gazebo 9. See [native runtime and motion comparison](performance-tradeoffs.md)
for the measured results, minimum-range configuration and harness limitations.

## Native execution: current result

The pinned upstream acquisition C++ was compiled and executed in ROS Melodic /
Gazebo 9. A synchronous harness invokes the actual callback every 100 native
1 ms steps, collecting 20 × 20,000 points over two simulated seconds. This is
not a stock roslaunch/sensor-thread throughput test. Only an obsolete protobuf
linker spelling is adapted; acquisition code remains unchanged.

The clean three-run median is **134.17 ms wall time (14.91× realtime)**. With
`range.min=0`, native snapshot ranges agree with analytic geometry to
**0.00032 mm RMS**. The original 0.1 m setting produced a uniform 100 mm
shortfall in this room; that bias is excluded from our claimed motion advantage.

A diagnostic assigns nominal 5 µs offsets to each point, producing **376.34 mm
RMS** against rolling MoCap poses. Those offsets are **not present in upstream
messages**. This measures the mismatch between snapshot and rolling acquisition,
not random range noise or decoded per-point timing. See the
[current comparison](performance-tradeoffs.md) for scope and the
[native harness](https://github.com/hxyulin/mid360-sim/tree/main/integrations/gazebo-classic-reference)
for source/image pins and reproduction instructions.

## Source findings

The [plugin](https://github.com/Livox-SDK/livox_laser_simulation/blob/1cce1073633a062b92e30243a4c2920e45551bb5/src/livox_points_plugin.cpp)
loads CSV times but advances ray indices by the configured sample count, modulo
table length. Times do not drive historical sensor poses or acquisitions.
`OnNewLaserScans` initializes the batch and calls `rayShape->Update()` once.
The [ODE implementation](https://github.com/Livox-SDK/livox_laser_simulation/blob/1cce1073633a062b92e30243a4c2920e45551bb5/src/livox_ode_multiray_shape.cpp#L50-L67)
locks the current physics scene for collision queries; it has no motion history.
There is no explicit per-emission pose interpolation.

The ROS output is `sensor_msgs/PointCloud` with one `ros::Time::now()` header.
No time channels, per-point offsets, lane or tag fields are populated. With
`use_sim_time` enabled that header can be simulation time; it is not inherently
wall time. It still cannot support ordinary per-point Livox deskew without an
adapter inventing timestamps. Misses become zero-range/zero-XYZ entries.

The [MID-360 xacro](https://github.com/Livox-SDK/livox_laser_simulation/blob/1cce1073633a062b92e30243a4c2920e45551bb5/urdf/livox_mid360.xacro)
defaults to 24,000 samples, downsample 1 and 10 Hz: 240,000 queried rays/s. The
800,000-row reference table therefore wraps after 3.33 s of simulated callback
time, rather than its nominal four seconds at 200,000/s. It also declares a
200 m maximum range; that is a configuration default, not MID-360 validation.
The launch file defaults to the Avia xacro; explicitly select MID-360 when
attempting a native comparison. The repository targets ROS Melodic/Gazebo 9.

These findings explain why using the official table alone cannot repair motion
distortion. The `official-table` profile here approximates directions from that
table; it does not deliberately reproduce these plugin acquisition limitations.

## Earlier recorded-motion policy comparison

The initial experiment below **did not execute the native plugin**. It evaluated
the source-derived snapshot policy using the same recorded local returns and independent MoCap
reference as the [previous experiment](motion-comparison-results.md). This is
point displacement from pose assignment, not actual official raycast/range error.
No claim is made about actual Gazebo performance, callback jitter or optical fit.

Use a 100 ms grid (10 Hz), with phases 0/25/50/75 ms relative to the first GT
pose. Evaluate both start and end pose assignments because upstream supplies no
per-point times to define a rolling correspondence. Retain the exact same points
across all eight assignments, excluding whole grid windows crossing GT gaps.
Our 128 Hz policy is recomputed on that common subset.

| Evaluation subset | Points | Clouds | Snapshot RMS across assignments | Our 128 Hz RMS |
|---|---:|---:|---:|---:|
| All held-out | 143,217 | 365 | 128.22–162.91 mm | 1.739 mm |
| Higher-motion | 47,353 | 120 | 185.12–230.85 mm | 1.834 mm |

Higher-motion snapshot p95 ranges from 377.23 to 484.50 mm; our policy's p95 is
3.540 mm. These are not the same sample counts as the prior 50 ms-cloud test:
the 100 ms windows require stronger common GT coverage. The reference remains
interpolated approximately 100 Hz MoCap, not exact continuous physical truth.

A snapshot simulator can produce unrealistically clean static walls during
motion because all rays share a scene pose. It should not be described as simply
adding 20 cm of range noise. The problem is that it omits rolling distortion;
feeding invented per-ray timestamps to deskew can then deform that snapshot.
Our displacement proxy quantifies the pose mismatch, not an observed 20 cm
range error from running the official simulator.

## Reproduce the earlier offline policy test

```sh
python scripts/compare-official-reference.py \
  local-assets/motion-ground-truth/comparison-v3/evaluation.npz \
  /path/EIL_Mask2.txt --output local-assets/official-reference/new-policy.json
```

The referenced evaluation NPZ is generated by `compare-motion-ground-truth.py`.
The output records the revision, phase sweep, exact sample counts, all RMS/p95
metrics and limitations. Upstream source downloads and measured results remain
in ignored `local-assets/official-reference/`; no third-party code is vendored.
The subsequent native run above supplies direct execution evidence. Keep the
metrics separate: this older test measures point displacement on recorded
returns, while native replay measures range differences in a synthetic room.
