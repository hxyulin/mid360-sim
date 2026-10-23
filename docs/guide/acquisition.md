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

## Share aligned optical windows in Rust

If several sensors use the same scan profile, global indices and stride,
prepare their unperturbed local directions once. Use a separate `Sensor`, scene
query context and `RollingFrame` for each device. The scene contexts below are
provided by your host and must evaluate each physical mount at the query time:

```rust
use mid360_sim::{pattern::ScanPattern, sensor::{Effects, PreparedRays, RollingFrame, Sensor}};

let sensor_a = Sensor::new(0.1, 40., 0, Effects { seed: 7, ..Default::default() })?;
let sensor_b = Sensor::new(0.1, 40., 0, Effects { seed: 17, ..Default::default() })?;
let window = PreparedRays::new(ScanPattern::Nominal, 7, 200, 1)?;
let (mut scan_a, mut scan_b) = (RollingFrame::default(), RollingFrame::default());
sensor_a.sample_prepared_into(&scene_a, &window, &mut scan_a)?;
sensor_b.sample_prepared_into(&scene_b, &window, &mut scan_b)?;
```

Sharing does not imply equal world poses or shared measurements. Angular noise
is applied separately before each scene query; range noise, dropout, misses and
occlusion remain device-specific. Each timestamp uses that sensor's epoch and
global emission index. Prepare a new window when indices/profile/stride change;
retain the caller's acquisition schedule and geometry history.

A window owns 24 bytes of direction payload per retained emission, at most
6 MiB plus container overhead. It contains no world rays, surface hits or pose
history. Borrow it across sensors instead of cloning the vector. There is no
global cache, worker, simulator world or implicit clock. For one sensor, keep
using `sample_into` or `sample_pattern_into` unless measurement justifies the
extra preparation/allocation. See the [architecture measurements](../architecture-performance.md).

## Reuse angular effects in a backend

`effects::AngularNoise` is independent of the scan generator and `Sensor`
acquisition loop. A backend with its own time-aware geometry queries or range
effects can use it directly:

```rust
use mid360_sim::{effects::AngularNoise, hardware::NominalPattern};

let angular = AngularNoise::new(0.001, 7)?; // radians per tangent axis, device seed
let index = 42; // global emission index, never a packet-local index
let emitted = angular.perturb(index, NominalPattern.direction(index))?;
```

Transform `emitted` using the sensor's actual pose at this emission's time,
including any dynamic carrier rotation and mounting offset. Query the nearest
physical surface with that ray before range filtering and dropout. Report local
XYZ as `emitted * measured_range`. Perturbing only the reported point after
intersection would leave edge occlusion unchanged and is a different model.

Construction validates the angular standard deviation; `perturb` validates a
finite unit input and returns a normalized direction. Zero noise returns the
input bits unchanged. The immutable seed and global emission index determine
each perturbation, independently of call order, chunking, decimation and misses.
This is the same angular model and random streams used by `Sensor`.

The module does not own a simulation clock, scene history, pose interpolation or
range/dropout policy. For example, rm-simulator can retain its existing mounted
capture interpolation and range-noise streams while adopting this angular model.
Its current dependency pin must include this API before using it. No hardware
calibration, encoder uncertainty or finite beam footprint is implied by the
chosen standard deviation; the default is zero.

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

### Explicit shared Isaac rigid captures

`RigidHybridSensor.scan()` self-captures by default, using bounded contiguous
storage. For sensors with the same stage and **ordered** moving mesh paths,
callers can explicitly share captures after their actual physics-to-USD writeback:

```python
from mid360.isaac import SceneTransformHistory

history = SceneTransformHistory(stage, sensor_a.dynamic.paths, generation=batch_id)
# After this step's physics writeback, before either sensor callback:
snapshot = history.capture(step_id)
sensor_a.scan(first_a, count_a, pose_a0, pose_a1, scene_snapshot=snapshot)
sensor_b.scan(first_b, count_b, pose_b0, pose_b1, scene_snapshot=snapshot)
```

A generation holds at most 100 distinct captures and 128 moving meshes. Create a
new history for the next generation; retain old snapshots for consumers still
using that generation. Reusing a capture identity returns its original immutable
matrices even after later pose edits: identities must describe actual scene
snapshots, never just the latest pose or USD notice revision. The caller must
serialize writes/capture and supply coherent static geometry and sensor poses.
A pending sensor batch may span history generations within its 100-segment bound;
keep sharing or self-capture consistent throughout that batch. Sensors
keep their own emission times, pose fractions, SLERP, flush schedules, device
uploads and owned outputs; sharing does not require a common flush.

Removal, replacement, resync, mesh edits and singular/nonfinite/non-affine
transforms fail closed; rebuild sensors/history after structural edits. Nested scale, reflection
and shear retain the existing Gf affine inversion. Capture sharing does not add
within-step object interpolation or change the extension's production defaults.

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

## High speed, acceleration and mounted sensors

We interpolate **poses**, then intersect rays at their emission times; we do not
SLERP point clouds or deskew the output in advance. Linear position plus
shortest-arc quaternion SLERP is exact for constant translational velocity and
constant rotation about a fixed axis within a segment. High speed alone does
not require a higher-order interpolator; acceleration, curved mount motion,
collisions and inconsistent timestamps are the harder cases.

For smooth translation with acceleration magnitude bounded by `a`, the maximum
linear interpolation position error is bounded by `a * dt² / 8`. A sensor at
lever arm `l` from a fixed rotation center follows an arc: interpolating only
its world endpoint positions introduces a midpoint chord error
`l * (1 - cos(omega * dt / 2))`, approximately `l * omega² * dt² / 8`.
Interpolating the body pose and then applying the fixed sensor extrinsic avoids
that chord approximation for constant-axis rotation. The current APIs take
sensor endpoint poses; they do not automatically perform body/extrinsic-aware
interpolation.

A timestamp offset introduces first-order position error approximately
`speed * offset` and angular error approximately `angular_speed * offset`.
At 10 m, 10 rad/s and a 100 µs offset, the angular term alone corresponds to
about 10 mm lateral ray displacement. This is not a range-error bound: grazing
surfaces and changing nearest hits can amplify the range difference.

Prefer, in order:

1. Coherent pose/geometry timestamps and correct mounted extrinsics.
2. Denser sensor pose segments where acceleration or rotation demands them;
   keep acquisition resolution independent of output/readback batching.
3. Shorter dynamic-geometry capture intervals when object motion matters.
4. Velocity-aware translation interpolation when trusted endpoint velocities
   are available. Cubic Hermite can improve smooth accelerated motion, but
   estimating derivatives from sparse noisy poses can overshoot; it is not
   currently a selectable adapter mode. Quaternion SLERP also needs shorter
   segments when angular velocity changes.

MuJoCo's `origin_tolerance_m` controls a separate translation-origin grouping
approximation. Tightening it can increase native ray-call count without fixing
pose-sample timing or changing geometry history. No interpolation recovers
unobserved motion between sparse ground-truth samples, or missing scene geometry.

## What the tests establish

The [recorded-return comparison](../motion-comparison-results.md) isolates pose
assignment. The [native MoCap replay](../backend-ground-truth.md) exercises actual
adapter intersections against an analytic scene. The
[runtime/accuracy comparison](../performance-tradeoffs.md) includes the official
plugin and RTX with their different time/profile assumptions. None establishes
calibrated physical range accuracy. See [validation](validation.md) for the
metric definitions and scope boundaries.

## Articulated-scene regression

Run `cargo test --locked sensor::articulated_tests` without a simulator, ROS or
GPU. The caller-owned test scene prescribes chassis/head rotations, a mounting
lever arm, an opaque mast and a shoulder/elbow mechanism. It adds no dynamics
world or public robot model. Each emission queries the actual kinematic state.

The adapter intersects inverse-transformed oriented boxes; its independent
reference uses explicit matrix chains and forward-transformed triangles. Across
8,000 emissions spanning changing joint poses, every link and the background are
exercised and ideal ranges agree within the 1e-9 m numerical test tolerance.
A frozen-geometry negative control differs on more than 100 emissions, preventing
a passing test that accidentally freezes articulation. Separate tests retain
bit-identical seeded noise/dropout under chunking and strides 3/7/31, and ensure
a mounted surface below minimum range occludes the background at multiple poses.

This is a regression for the scene-query contract and transform/time conventions.
The tolerance is numerical agreement in synthetic geometry, not hardware range
precision, native backend validation or full articulated-robot fidelity.
