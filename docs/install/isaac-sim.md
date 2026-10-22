<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Isaac Sim MID-360 extension

`hxyulin.mid360` is a Kit extension and in-process Python adapter for the shared
nominal MID-360 pattern. It casts rays on the GPU with NVIDIA Warp over actual
USD optical triangle meshes and cubes, rather than creating another dynamics
world or loading a giant repeated ray table. Verified on Talos's Isaac Sim 6.2.0
container and RTX 5090, using bundled Warp 1.17. No pip install or Rust FFI is
required in Isaac. ROS dependencies remain optional in the existing Rust core.

## Install from a release

Extract `mid360-sim-0.1.0-isaac.zip` into a writable folder. Its top-level
`hxyulin.mid360/` directory is the extension. Add the extracted parent as a Kit
extension search path. For a source checkout, use `integrations/isaac-sim/exts/`
as shown below. No pip install into Isaac is required.

## Enable and mount

Add this directory's `exts/` to Isaac's extension search paths, then enable
`hxyulin.mid360`. In a running Isaac Python console:

```python
import omni.kit.app
import omni.usd
manager = omni.kit.app.get_app().get_extension_manager()
manager.add_path("/absolute/path/to/mid360-sim/integrations/isaac-sim/exts")
manager.set_extension_enabled_immediate("hxyulin.mid360", True)

from mid360.isaac.extension import create_sensor, latest_frame
stage = omni.usd.get_context().get_stage()
create_sensor(stage, "/World/Robot/Mid360", roots=("/World",))
# Set the sensor Xform relative to the robot, then Play the timeline.
# Include the robot's visible meshes for self-occlusion, or explicitly exclude
# chosen subtrees. Sensor mounts must have unit scale.

frame = latest_frame("/World/Robot/Mid360")
if frame is not None:
    print(frame.timestamps_ns, frame.ranges_m)
    fields = frame.livox_fields()
```

The marker exposes `mid360:enabled`, `mid360:roots`, `mid360:exclude`,
`mid360:minRangeM`, and `mid360:maxRangeM` as custom USD attributes. Configure
before Play; Stop then Play rebuilds the captures. Acquisition runs on PhysX
post-step callbacks and owns only the newest frame per sensor. Pause preserves
sensor time and frame identity; Stop and stage replacement clear state.

Multiple mounted sensors are supported, with independent geometry captures.
Their shared extension clock starts at zero on Stop/reset. Callback timestamps
come from accumulated simulation step durations rounded to nanoseconds, never
wall time. Ensure physics writes poses into USD; Fabric-only transforms are not
automatically read by this adapter. The extension's first step initializes the
sensor pose without inventing an earlier pose or velocity.

## Explicit stepping API

After starting `SimulationApp` and enabling `omni.warp.core`:

```python
from mid360.isaac import Mid360Sensor
sensor = Mid360Sensor(stage, roots=("/World",), exclude=(), device="cuda:0")
frame = sensor.scan(
    first_index=0, count=3334,
    pose_start=((0, 0, 0.8), (0, 0, 0, 1)),
    pose_end=((0.01, 0, 0.8), (0, 0, 0, 1)),
    epoch_ns=0, stride=1,
)
```

Poses are translation in metres and quaternion **xyzw**, corresponding to the
first/last selected emissions. This differs from rm-simulator's wxyz pose
convention: convert explicitly. Local rays are +X forward, +Y left, +Z up.
Only metre, Z-up stages are admitted. Full rate is 200,000 emissions/s, with
original nanosecond timestamps and four lane labels. Queries are bounded to
200,000 rays per call and reject invalid/overflowing windows before scanning.
Odd decimation strides retain all four lanes.

`ScanFrame` contains owned NumPy arrays of local directions and ranges; misses
have range -1. `livox_fields()` returns a transport-free record with `timebase`,
`offset_time`, `xyz_m`, `line`, synthetic `tag=0x10` and reflectivity 100. Misses
are filtered without shifting acquisition offsets. The timebase includes a
missed first emission. No privileged world pose is published. This release
provides data for a caller's ROS/other bridge, not an embedded rclrs publisher
inside Kit. The separate Rust ROS publisher remains available.

## GPU architecture and limits

- Ten Fourier harmonics with quarter-degree integer coefficients, exported from
  the Rust source. Clocks use integer modular arithmetic: 181 Hz spin and
  642697/65521 Hz sweep, common cycle 65,521 s. The rates are representative
  simulation choices. They do not predict a particular hardware device's phase.
- One combined GPU mesh BVH per sensor, so each ray performs one scene query
  rather than querying every object. Source local points and object transforms
  are retained. On a rigid transform change, a GPU kernel updates world vertices
  and refits the BVH; unchanged captures skip this work. Output buffers grow and
  are reused, with synchronized host copies for caller-owned frames.
- Per-ray sensor translation and quaternion interpolation preserve sensor
  motion distortion. Scene geometry is captured once per batch, so moving
  obstacle motion within that batch is approximate. Use physics-step-sized
  batches; do not treat a 100 ms batch as a fully rolling dynamic scene.
- Visible triangle `UsdGeom.Mesh` and `UsdGeom.Cube` are supported, including
  instance proxies and rigid/nonuniform affine transforms. Quads, concave
  polygons, subdivision surfaces, mesh holes and other Gprims fail explicitly:
  triangulate/bake them upstream.
  Deformation, skinning, vertex/topology/visibility changes require rebuilding
  the sensor. USD point instancers are rejected until expanded upstream.
- Nearest surfaces are queried from the true origin, then minimum range is
  applied. A near surface blocks distant surfaces even when too close to report.
  Surfaces are two-sided. This is geometric ray casting, not the proprietary
  RTX lidar material/return engine. Calibrated reflectance, multipath, noise,
  dropout and IMU data are not implemented in the Isaac adapter.
- GPU float32 geometry/rays are checked against Rust f64 reference directions;
  they are not promised to be bit-identical across CPU/GPU implementations.
  Partitioning/stride within the tested GPU implementation is deterministic.

The official [Physics Raycast Sensor documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_physics_raycast.html)
states that ray attributes are read at simulation start and time offsets repeat
with a sweep period. This adapter instead generates each explicit batch from
its global index. Warp's [ray-query API](https://nvidia.github.io/warp/v1.17/language_reference/_generated/warp.mesh_query_ray.html)
supplies mesh BVH queries. No NVIDIA implementation source is vendored.

## Reproduce and verify

Generate the shared model and cross-language fixtures from the Rust source:

```sh
cargo run --quiet --locked --example export_isaac_model > integrations/isaac-sim/exts/hxyulin.mid360/mid360/isaac/nominal.json
cargo run --release --locked --example pattern_bench
```

Run the real GPU and Kit lifecycle test using Isaac's own interpreter:

```sh
/path/to/isaac-sim/python.sh integrations/isaac-sim/tests/headless.py \
  --output /writable/path/headless.json
```

The test creates an enclosure without network assets, checks ray intersections
against analytic distances, compares Rust fixtures across the 18.2 h boundary,
checks partition/stride invariance, moving/scaled objects, a triangulated wall,
near-hit occlusion, sensor-motion interpolation and extension Play/Pause/Stop.
A success JSON containing `checks` must exist. Isaac's fast shutdown can return
exit code zero even after a Python exception; do not infer success from process
status alone. The test writes a `.failure.txt` before shutdown on failure.

Axiom's release direction-only benchmark measured 9.10 ms scalar versus
5.31 ms grouped per 200,000 rays (30 measured repetitions, 5 warmups). This is
about 1.71x for generating rays, excluding geometry/noise/transport. Grouped
Rust results remain bit-identical to scalar/chunked/strided generation.

Initial Talos GPU enclosure measurements were around 0.2 ms per 20,000 rays,
including transform synchronization and synchronized host output copies,
after 5 warmup batches. See the final local validation report for measured
median/p95. These are a simple enclosure, not arena-scale or full-app FPS.
BVH construction, initial shader/JIT startup, dynamic refits and ROS are excluded
from that static-scene timing. A subsequent [Gazebo CPU adapter](gazebo.md) now uses the same compact
model; its collision geometry and per-step motion approximations differ from
this optical-mesh GPU backend.

Final validation on Talos measured **0.183 ms median / 0.192 ms p95** per
20,000 rays in the static enclosure, over 30 measured batches after 5 warmups.
Maximum sampled GPU/Rust direction vector difference was 8.05e-7. The final
report/log are retained in ignored `local-assets/isaac-validation/` in the
source checkout and in the remote `results/headless-validated.json` / host
`workspace/mid360-sim-headless-validated.log`.

### Physics-step acquisition benchmark

Run the actual extension callback at 1 kHz, rather than only timing large GPU batches:

```sh
./python.sh /path/mid360-sim/integrations/isaac-sim/tests/realtime.py \
  --steps 2000 --output /writable/path/realtime.json
```

The headless test steps PhysX at 1 ms with a translating/rotating USD rigid-body
mount, first without sensing and then with the existing extension's post-step
callback. Sleeping is disabled for the mount. Each callback acquires 200 rays;
transport-free output arrays are combined into 100 ms frames. Warmup, initial
BVH construction and shader startup are excluded. It verifies mount motion,
callback emission continuity, per-step counts and completed-frame offsets, and
reports real-time factor, step median/p95 and steps exceeding 1 ms. Rendering,
ROS and network publication are excluded. The two runs continue the same body
trajectory through a simple static enclosure; they are not identical scene
captures or a detailed-arena benchmark. Require a report containing `checks`;
Isaac fast shutdown can mask failures in the process exit status.

On Talos (Isaac Sim 6.2.0, RTX 5090), a 10-second simulated run processed
2,000,000 rays into 100 frames in 9.315 s wall time: **1.074× real time**.
The physics-only baseline took 3.390 s. Sensor-enabled steps had a 0.920 ms
median and 1.044 ms p95; 1,357 of 10,000 steps exceeded 1 ms. Average throughput
kept up in this simple enclosure, but individual deadlines were not guaranteed.
The result includes callback/USD synchronization and host array costs, unlike
the large-batch GPU microbenchmark. It excludes rendering and DDS/network
publication. Raw reports/logs are retained in ignored `local-assets/realtime/`.

### Batched readback without coarser acquisition

Use `create_sensor(..., batch_steps=10)` to return a host frame every ten
physics callbacks, or `batch_steps=100` for a 100 ms frame at 1 kHz. The
corresponding USD setting is `mid360:batchSteps` (1–100, default 1); configure
it before starting. `latest_frame()` retains the last completed frame between
readbacks. STOP/reset discards incomplete batches. This adds output latency,
but every physics callback still queries its own captured scene and interpolated
sensor poses. It does not freeze the scene for the whole publication interval.

Direct backend callers can use `sensor.scan(..., defer_readback=True)` followed
by `sensor.flush()`. Emissions must be contiguous with matching stride/epoch;
flush before changing those or switching to immediate scans. Pending storage
is bounded to 200,000 rays. Queries/refits stay on the same Warp stream, followed
by synchronized owned host output. Concurrent scene mutation on another stream
is unsupported. The host reads only the populated output slice.

USD change notices invalidate the cached geometry transforms when source prims
or their ancestors change; unchanged geometry avoids repeated transform walks.
Scenes containing instance proxies or physics-driven geometry retain conservative
per-query refreshes.
Topology/deformation changes still require rebuilding the backend. Per-step
pose validation avoids NumPy array allocations. These changes add no threads
or second physics world.

Compare the available output intervals using the same 1 ms acquisition test:

```sh
./python.sh /path/mid360-sim/integrations/isaac-sim/tests/realtime.py \
  --steps 10000 --batch-steps 10 --output /writable/path/batch10.json
```

`--batch-steps` accepts 1, 10 or 100. Original point offsets and motion remain
per emission. Larger batches amortize host synchronization; GPU query kernels
still run every physics step so moving geometry is captured at that resolution.

Final optimized Talos runs (10 simulated seconds / two million rays each):

| Host readback interval | Wall time | Real-time factor | Step p95 | Steps >1 ms |
|---|---:|---:|---:|---:|
| 1 ms | 9.131 s | 1.095× | 1.040 ms | 923 / 10,000 |
| 10 ms | 6.717 s | 1.489× | 0.969 ms | 363 / 10,000 |
| 100 ms | 6.187 s | 1.616× | 0.719 ms | 132 / 10,000 |

The pre-optimization 1 ms-readback run took 9.315 s (1.074×). The 100 ms option
therefore reduced total wall time by 33.6%, or increased throughput by 50.5%, in
this scene. Its physics-only baseline took 3.364 s; sensor-enabled excess wall
time was 2.823 s. These are single runs and include host scheduling variance;
the intervals also trade output latency. Every case retains 1 ms scene capture
and per-ray sensor-pose interpolation. The regression suite checks retained
frames, buffer growth, queued queries across moving geometry, ancestor transform
invalidation, and playback pause/reset. No claim is made for detailed-arena
performance or a maximum deadline. Use 10 ms readback for lower output latency,
or 100 ms when publishing at 10 Hz. Raw reports are `local-assets/realtime/final-*.json`.

### Alternative execution: segmented tracing of static geometry

`create_sensor(..., batch_steps=100, static_trace=True)` opts into **static
optical geometry**. Instead of launching 100 small tracing kernels, the adapter
records each step's sensor boundary poses and emission fractions, then traces
all 100 segments in one GPU launch. Each segment retains its own translation
and rotation, so turns and changes of velocity between steps remain represented.
This is not a single interpolation across the whole 100 ms publication window.

All selected optical geometry must stay fixed while a batch is pending. Source
or ancestor USD edits cause an error before tracing; there is no automatic
fallback that traces historical poses against newly moved objects. Instance
proxies and physics-driven geometry/ancestors are rejected for this mode. Use the default per-step tracing for scenes
including moving robot geometry, self-occlusion or moving obstacles. See the explicit static/dynamic partition modes below for retaining moving
bodies while batching fixed arena queries.
New topology/deformations still require rebuilding the backend in either mode.

Direct callers can wrap an idle backend in `StaticSceneBatch(sensor)`, record
contiguous `scan(...)` segments and call `flush()`. Storage is bounded to 100
segments and 200,000 rays. Static batches own copies of their input poses.

Both tracing architectures now accept `pose_fraction=(first, last)`: the GPU
maps emissions into that subinterval of the supplied pose pair. The extension
passes raw physics-step boundary poses, avoiding two CPU interpolations followed
by another GPU interpolation. Defaults preserve the direct API's first/last
emission convention. Tests compare subinterval tracing against explicit CPU
endpoint interpolation and compare nonlinear, variable-sized, strided segments
against ordinary per-step queries.

```sh
./python.sh /path/mid360-sim/integrations/isaac-sim/tests/realtime.py \
  --steps 10000 --batch-steps 100 --static-trace \
  --output /writable/path/static-trace.json
```

Trusted in-process adapters may also construct `Extension(pose_reader=reader)`.
The reader receives `(stage, sensor_path)` and returns a world pose in metres
with an xyzw unit quaternion. It owns extrinsic transforms and synchronization
with the callback's physics step; the default reader uses USD. The benchmark's
`--pose-source physics` probes the native PhysX rigid-body pose interface for its
specific identity-mounted sensor. It is not a general robot mount resolver.

Measured architecture probes on Talos, each with 1 ms physics, 100 ms output,
10 simulated seconds and two million rays:

| Tracing architecture / pose reader | Wall time | Real-time factor | Step p95 |
|---|---:|---:|---:|
| Prior per-step tracing, CPU endpoint interpolation / USD | 6.187 s | 1.616× | 0.719 ms |
| Per-step tracing, GPU subinterval interpolation / USD | 5.855 s | 1.708× | 0.686 ms |
| Segmented static tracing, GPU subinterval interpolation / USD | 4.751 s | 2.105× | 0.560 ms |
| Per-step tracing, GPU subinterval interpolation / native PhysX probe | 6.004 s | 1.666× | 0.691 ms |
| Segmented static tracing, GPU subinterval interpolation / native PhysX probe | 4.959 s | 2.017× | 0.572 ms |

The static/USD case's physics-only baseline was 3.425 s (2.919×). Native pose
access did not win in these single runs; USD remains the default. Results are
simple-room throughput probes, not controlled hardware-wide benchmarks or hard
real-time guarantees. The static case still had 117 of 10,000 steps over 1 ms.
It cannot replace moving-geometry acquisition without explicit scene partitioning.
Reports are retained in ignored `local-assets/realtime/architecture-*.json`
and `native2-pose-*.json`. The native diagnostic additionally samples
position and sign-invariant quaternion differences against USD during callbacks;
its identity-mount mapping is not a production extrinsics implementation.

The native diagnostic sampled 101 callbacks. Maximum native-versus-USD position
difference was 0.1000 mm, consistent with one 1 ms translation step at the
benchmark's 0.1 m/s velocity. Maximum sign-invariant normalized quaternion
Euclidean difference was 0.000151. After `sim.step()` returned, the body poses
agreed within the test tolerances. These callbacks therefore must not be assumed
to expose interchangeable snapshots; native throughput numbers are architecture
probes, not proof of equivalent acquisition timing. USD remains the default.
Physics-driven optical geometry also bypasses notice-only transform caching:
PhysX writeback need not emit ordinary USD authoring notices.

### Static/dynamic scene partitions

Set `dynamic_roots=("/World/Robots", "/World/MovingTargets")` on
`create_sensor(...)` to opt into a split scene. Dynamic roots must lie within
`roots`; they are removed from the static partition and loaded into a separate
optical BVH. Each partition must contain geometry. The dynamic partition retains
per-step capture/tracing; the static partition uses retained sensor poses and
batched tracing. This can retain self-occlusion by including the robot's meshes
in the dynamic roots. No bodies or second physics world are created.

Both partitions query raw nearest hits down to zero distance. Their ranges are
merged on the GPU **before** applying the configured minimum range, so a near
opaque surface in either partition blocks farther geometry. One owned host
frame is read back after merging. Overlapping partitions and changed static
geometry are rejected. The direct API is `HybridSensor(static_backend,
dynamic_backend, min_range_m=0.1)`; both underlying backends must have zero
minimum range, matching maximum range, stage and device, and disjoint source
paths. They are exclusively owned by the hybrid acquisition while in use.

A second opt-in mode, `rigid_trace=True` with `dynamic_roots`, records the moving
objects' inverse transforms each step and queries fixed local-space BVHs at
publication time. Each mesh must keep its original local geometry/topology;
rebuild after edits/deformations. Sensor motion remains interpolated per ray;
moving-object geometry stays frozen per step, matching the ordinary backend.
Nonuniform affine scale is handled by converting local hit distances back to
world metres before nearest-hit selection. This avoids per-step GPU updates,
launches and BVH refits, but loops over the moving meshes per ray. It is capped
at 128 moving source meshes, 100 step captures and 200,000 rays per batch. This
is an experimental explicit partition, not automatic scene classification.

The direct API for this variant is `RigidHybridSensor(static_backend,
dynamic_backend, min_range_m=0.1)`. A failed acquisition must be discarded;
partial partition results are never published. Static-only tracing and hybrid
configuration are mutually exclusive. The default execution mode is unchanged.

Reproduce matched moving-obstacle tests with either the combined-scene default,
`--hybrid`, or `--hybrid --rigid-trace`:

```sh
./python.sh /path/mid360-sim/integrations/isaac-sim/tests/realtime.py \
  --steps 2000 --batch-steps 100 --moving-obstacle --static-boxes 64 \
  --hybrid --output /writable/path/hybrid.json
./python.sh /path/mid360-sim/integrations/isaac-sim/tests/hybrid.py \
  --output /writable/path/hybrid-check.json
```

Add `--rigid-trace` to the correctness test to check that architecture. The
correctness test compares union hits, local directions and original timestamps
against ordinary combined-scene queries across moving objects and turning
sensors. It separately checks near-hit shadowing from each partition, owned
outputs and failure after static edits. Benchmarks include one translating/
rotating rigid obstacle, a translating/rotating sensor, and optional independent
static cubes, with physics-only baselines. They exclude rendering and ROS/network
publication. Part counts are synthetic source prims, not detailed-arena results.

Matched Talos results (1 kHz physics, 200,000 rays/s, 100 ms output, USD poses),
with a moving sensor and one translating/rotating obstacle:

| Optical scene / mode | Simulated duration | Wall time | Real-time factor | Step p95 |
|---|---:|---:|---:|---:|
| Enclosure + obstacle, combined | 2 s | 1.663 s | 1.203× | 0.954 ms |
| Enclosure + obstacle, split per-step moving BVH | 2 s | 1.715 s | 1.166× | 0.980 ms |
| +64 static parts, combined | 10 s | 10.624 s | 0.941× | 1.353 ms |
| +64 static parts, split per-step moving BVH | 10 s | 8.333 s | 1.200× | 0.930 ms |
| +64 static parts, batched rigid histories | 10 s | 5.677 s | 1.761× | 0.630 ms |

The 64-part scene has 66 source cubes / 792 optical triangles in total, including
the enclosure and moving obstacle. The longer runs each processed two million
rays into 100 frames. Batched rigid histories reduced total wall time by 46.6%
versus the combined scene, increasing throughput by 87.1%. It still had 122 of
10,000 steps above 1 ms. Its physics-only baseline was 3.651 s. The split
per-step variant adds overhead in the nearly empty scene; explicit partitioning
is not automatically faster. These are single synthetic-scene runs, not
arena-scale or full robot-dynamics benchmarks. The rigid-history path remains
opt-in; moving mesh count, topology and scene complexity need separate profiling.

Both partition correctness tests cover two nested moving meshes, nonuniform and
reflected scales, rotation/translation, a turning sensor, odd segment sizes and
stride 3. Across 4,849 sampled rays, per-step splitting matched the combined
ranges exactly; batched local-space queries differed by at most 0.00000525 m
(approximately 5 micrometres). This is numerical agreement on the test scene,
not a hardware precision estimate. Both retained all emission timestamps and
passed near-hit shadowing in each partition, output ownership and static-edit
failure checks. Raw reports are in ignored `local-assets/realtime/partition-*`.

### Caller-owned arena / CAD robot benchmark

`tests/prepare_arena.py` reads a caller-provided RMUC 2027 runtime field package
and bound neutral robot GLB, verifies manifest checksums, applies glTF graph
transforms and manifest placements, and writes ignored triangle arrays. No
source geometry is included in this repository or release. Field ownership
notices and robot attribution remain attached to the caller's inputs/metadata.
It uses visual triangles, not collision proxies. Equipment remains in rest pose.

```sh
python integrations/isaac-sim/tests/prepare_arena.py \
  --field /path/to/field-2027 --robot /path/to/bound-tunnel-sentry/neutral.glb \
  --output local-assets/realtime/arena-input.npz
./python.sh /path/mid360-sim/integrations/isaac-sim/tests/arena.py \
  --input /writable/path/arena-input.npz --robots 4 --mode rigid --steps 2000 \
  --output /writable/path/arena-rigid4.json
```

The fixed field components and each robot are merged into mesh groups without
removing triangles, preserving placements. Robot GLB X-left/Y-up/Z-forward is
mapped to FLU; its root is shifted vertically to place the lowest vertex at zero.
Neutral robot poses remain fixed internally. Robot roots translate and rotate
as unconstrained zero-gravity PhysX bodies, including the sensor's own robot;
this is not a wheel, suspension, folding or contact-dynamics benchmark. The
sensor uses a 0.45 m mount offset. Static geometry stays in the same USD scene.

`--mode combined`, `--mode hybrid` and `--mode rigid` select the three acquisition
architectures against identical geometry and root motion. `--robots` accepts
1, 4 or 8 and `--steps` accepts multiples of 100 through 10,000. Each run includes
a physics-only baseline and 100 warmup steps; startup, mesh/BVH construction and
shader compilation are excluded. One sensor emits 200 rays every 1 ms and reads
100 ms frames. Reports separate scheduled rays from valid returns; misses into
the open sky are expected. Rendering and ROS/network output are excluded.

`--verify` additionally compares 5,000 sampled rays against ordinary combined
queries on the same moving scene, outside the timed run. Hit/miss masks and
emission timestamps must agree, and range differences must remain below 1 mm.
The CPU-only GLB preparation regression checks shared mesh instances, interleaved
vertex buffers, column-major matrices, TRS composition and reflected transforms.

Measured Talos arena results use the supplied RM2027 runtime Low visual package
(355,542 triangles, complete equipment at rest) and neutral EGA tunnel-Sentry
CAD (222,900 triangles per robot), at 1 kHz and 200,000 scheduled rays/s:

| Robots | Total triangles | Acquisition mode | Simulated duration | Wall time | Real-time factor |
|---|---:|---|---:|---:|---:|
| 4 | 1,247,142 | Combined BVH | 2 s | 6.799 s | 0.294× |
| 4 | 1,247,142 | Split / per-step moving BVH | 2 s | 1.850 s | 1.081× |
| 4 | 1,247,142 | Batched rigid histories | 2 s | 1.362 s | 1.468× |
| 1 | 578,442 | Batched rigid histories | 2 s | 1.082 s | 1.848× |
| 8 | 2,138,742 | Batched rigid histories | 2 s | 1.760 s | 1.136× |
| 8 | 2,138,742 | Batched rigid histories | 10 s | 8.788 s | 1.138× |
| 8 | 2,138,742 | Batched rigid histories, repeat | 10 s | 8.448 s | 1.184× |

Four-robot modes produced identical valid-return counts (88,421 / 400,000
scheduled emissions). The ten-second eight-robot runs both returned 408,438
valid points from two million emissions. The repeat's p95 step was 0.935 ms;
219 of 10,000 steps exceeded 1 ms. It has modest average throughput margin,
not hard real-time deadlines. The matched four-robot comparison is approximately
5× the combined-scene throughput; do not compare these timings directly with
other engines/hardware or the earlier synthetic-room scene.

The eight-robot verification sampled 5,000 rays over 100 ms, including six
robot-occluded returns identified against a static-field-only query. All hit/
miss masks and emission times matched the combined scene; maximum sampled range
difference was 0.000000954 m. This is numerical agreement on a subset, not
physical sensor accuracy. Raw reports are ignored `local-assets/realtime/arena-*.json`.
Only triangle grouping was changed for the benchmark; static groups preserve
manifest placements and robot internals remain in neutral pose. Full robot
articulation, wheel/contact physics, moving field mechanisms, rendering and
ROS/network publishing remain outside these measurements.

Optimization coverage across the simulator adapters currently differs:

| Adapter | Work completed | Remaining performance work |
|---|---|---|
| Isaac | Deferred readback, GPU pose interpolation, static/dynamic partitions and rigid history tracing; tested with arena + moving CAD roots | Articulated source-part scaling, full robot dynamics, rendering/publication budgets |
| MuJoCo | Native multi-ray queries, reusable scratch buffers, consecutive lane-group fast path (~5.2% lower runtime in alternating room tests) | Equivalent full-arena measurement and further architecture profiling |
| Gazebo | Native step-aware rays, shared four-lane optical phases, reusable output capacity and warmed sensor-disabled/CPU phase profiling | Detailed arena scaling and native engine ray-query acceleration |

For NVIDIA’s built-in RTX LiDAR, see the [native RTX comparison](../rtx-reference.md).
It tests an explicitly repeating custom profile with Motion BVH enabled and disabled.

## Bulk caller-owned pose history

For static optical scenes, `StaticSceneBatch.scan_history(first_index, counts,
poses, epoch_ns=0, stride=1)` accepts one owned NumPy history. `counts` contains
1–100 positive segment sizes (at most 200,000 total rays); `poses` is
`(len(counts)+1, 7)`, with metre XYZ followed by unit XYZW quaternions.
Each consecutive pair bounds a segment; its final emission precedes the end
boundary by one emission interval. Call `flush()` explicitly before another
history. Inputs are copied, outputs are owned, and geometry changes fail closed.

```python
import numpy as np
from mid360.isaac import Mid360Sensor, StaticSceneBatch

batch = StaticSceneBatch(Mid360Sensor(stage))
poses = np.array([[0, 0, .8, 0, 0, 0, 1],
                  [.001, 0, .8, 0, 0, 0, 1]], dtype=float)
batch.scan_history(0, np.array([200]), poses)
frame = batch.flush()
```

This avoids per-step Python transform construction while preserving GPU pose
interpolation. It does not supply moving-obstacle history. Keep the existing
step-wise or explicit hybrid partition APIs for changing optical geometry.
[Runtime/accuracy measurements](../performance-tradeoffs.md) distinguish the
ordinary USD-driven extension from an explicitly headless caller-pose setup.
Disabling USD writeback is a harness option, not an extension default.
