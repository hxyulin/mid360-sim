<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Acquisition and motion

World and sensor coordinates are metres in forward/left/up (FLU). For emission
`i`, the intended acquisition time is `epoch_ns + i * 5000`. Query its local
unit direction at the sensor/scene state for that time, and report
`range * direction` in the instantaneous sensor frame. The consumer owns deskew.

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

## Choose how motion is represented

| API / mode | Sensor pose | Optical geometry | Main tradeoff |
|---|---|---|---|
| Rust scene query | Caller evaluates each emission | Caller supplies time-aware queries | Caller must provide coherent capture history |
| Isaac default | Per-ray translation and quaternion interpolation | Captured/refit per acquisition call or physics step | More queries/refits for moving scenes |
| Isaac deferred readback | Same per-step acquisition | Same per-step scene | Lower host overhead, later delivery |
| Isaac static tracing / bulk history | Per-ray interpolation within retained boundary segments | Fixed for the pending batch | Fewer launches; rejects moving selected geometry |
| Isaac hybrid | Same per-ray sensor interpolation | Static partition batched; dynamic partition captured per step | Explicit disjoint partition required |
| Isaac rigid history | Same per-ray sensor interpolation | Fixed local meshes, per-step object transforms | Avoids refits; moving mesh count affects cost |
| MuJoCo `scan()` | Held site pose | Current `mjData` for the call | Fast, but loses within-call motion |
| MuJoCo `scan_rolling()` | SLERP rotation per emission; grouped translation origins | Current `mjData` for the call | More CPU work; origin tolerance is not a range-error bound |
| Gazebo | Held per physics step | Current native collision scene | Shorter steps improve motion fidelity at engine cost |

See the [Isaac](../install/isaac-sim.md), [MuJoCo](../install/mujoco.md) and
[Gazebo](../install/gazebo.md) guides for setup and geometry limits. Isaac's
partition modes retain moving geometry per step, not continuous within-step
object motion. Physically moving meshes must not be put in a static batch.
Changing topology/deforming meshes requires rebuilding the Isaac backend.

## Acquisition interval is not publication interval

At full rate, 200 rays span 1 ms and 20,000 rays span 100 ms. Acquiring against
each 1 ms scene and publishing every 100 ms preserves much more motion history
than one frozen query every 100 ms. Increasing the host batch reduces readback
or transport overhead but adds delivery latency; retain original point stamps.

Delayed static tracing can evaluate known sensor history at flush time without
changing acquisition times. It cannot recover obstacle history that was never
captured. Caller-provided boundary poses must describe the same world/time
convention as the geometry. Disabling Isaac USD writeback is appropriate only
with a synchronized native/caller-owned pose source; it is not a safe blanket
optimization for USD-mounted sensors or rendering.

## What the tests establish

The [recorded-return comparison](../motion-comparison-results.md) isolates pose
assignment. The [native MoCap replay](../backend-ground-truth.md) exercises actual
adapter intersections against an analytic scene. The
[runtime/accuracy comparison](../performance-tradeoffs.md) includes the official
plugin and RTX with their different time/profile assumptions. None establishes
calibrated physical range accuracy. See [validation](validation.md) for the
metric definitions and remaining work.
