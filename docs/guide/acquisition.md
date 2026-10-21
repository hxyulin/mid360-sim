<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Acquisition and motion

World and sensor coordinates are metres in forward/left/up (FLU). For emission
`i`, evaluate the sensor pose and scene at `epoch_ns + i * 5000`, cast its local
unit direction, and report `range * direction` in that instantaneous sensor frame.
The consumer transforms those points to a common pose when deskewing.

## Keep time explicit

The caller owns physics steps, capture history and acquisition scheduling.
Neither the core nor the adapters creates another dynamics world. Preserve
emission indices across batches and misses; reset the epoch/index together.
Odd decimation strides retain all four lanes. Counter-based Rust noise is
invariant to batching and decimation.

`RollingFrame` retains misses and timestamps. `LivoxPacket` filters invalid
returns without renumbering emission times. Its header/timebase includes a
missed first emission. Python/Isaac `livox_fields()` exposes equivalent fields;
Gazebo publishes its own native point schema. Simulator adapters do not
implicitly start a ROS publisher.

## Motion limits

Isaac interpolates sensor translation and quaternion orientation per ray but
captures geometry once per batch. Moving self-geometry can therefore become
inconsistent with the interpolated sensor. MuJoCo freezes the site and scene
per call; Gazebo samples the physics scene per step. Use short batches and
retain real acquisition offsets when aggregating packets.

The [recorded-motion comparison](../motion-comparison-results.md) tests the
interpolation policy against approximately 100 Hz MoCap; it is not an end-to-end
validation of simulator callbacks, moving obstacles or robot self-occlusion.
See [realism](../realism.md) for uncalibrated range/material effects.
