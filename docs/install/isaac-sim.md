<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Isaac Sim MID-360 extension

`hxyulin.mid360` is a Kit extension and in-process Python adapter for the shared
nominal MID-360 pattern. It casts rays on the GPU with NVIDIA Warp over actual
USD optical triangle meshes and cubes, rather than creating another dynamics
world or loading a giant repeated ray table. Verified with Isaac Sim 6.2.0
in a Linux container on an RTX 5090, using bundled Warp 1.17. No pip install or Rust FFI is
required in Isaac. ROS dependencies remain optional in the existing Rust core.

Guides track the current `main` checkout. Bulk histories, rolling adapters and
partition modes may postdate a downloaded 0.1.0 artifact; verify its `release.json`
source revision. No new release is implied by the package version alone.

## Install from source or an available release

For current source, use `integrations/isaac-sim/exts/` below. Once a release
archive is published, extract `mid360-sim-0.1.0-isaac.zip` into a writable folder. Its top-level
`hxyulin.mid360/` directory is the extension. Add the extracted parent as a Kit
extension search path. For a source checkout, use `integrations/isaac-sim/exts/`
as shown below. No pip install into Isaac is required.

## Enable and mount

Add `integrations/isaac-sim/exts/` to Isaac's extension search paths, then enable
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

## Choose an execution mode

| Workload | Configuration | Geometry contract |
|---|---|---|
| General scene | Default `create_sensor(...)` | Capture/refit the combined scene per physics step |
| Same acquisition, fewer host copies | `batch_steps=10` or `100` | Still capture/query per step; output arrives later |
| Fixed arena, moving sensor | `static_trace=True`, with `batch_steps` | All selected optical geometry fixed while pending |
| Arena plus moving objects | `dynamic_roots=(...)` | Explicit disjoint static and per-step moving partitions |
| Many fixed-shape moving objects | `dynamic_roots=(...)`, `rigid_trace=True` | Fixed local meshes and retained per-step transforms |
| Caller already owns pose history | Direct `StaticSceneBatch.scan_history()` | Static selected geometry, owned boundary history |

The modes below are opt-in; default scene acquisition and USD writeback remain.
Headless tuning in the [runtime comparison](../performance-tradeoffs.md) is a
separate caller-pose configuration, not a production default. Historical
[architecture/scaling results](../isaac-performance.md) include moving CAD scenes.

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

## Batched readback without coarser acquisition

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

## Alternative execution: segmented tracing of static geometry

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

Native pose readers must synchronize extrinsics and callback timing explicitly.
The diagnostic native reader and USD can differ by a physics step during callbacks;
see the [pose-reader probe](../isaac-performance.md#segmented-tracing-and-native-pose-reader-probe).
Physics-driven geometry refreshes conservatively because PhysX writeback need
not emit ordinary USD authoring notices.

## Static/dynamic scene partitions

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

## Headless use and benchmark scope

The fast bulk-history benchmark uses the public simulation manager, synchronous
PhysX and disabled USD writeback while obtaining poses from caller-owned history.
Real physics steps still run. Do not disable writeback for the ordinary
USD-mounted extension or assume renderer transforms stay current. A real native
pose bridge must handle mounted extrinsics, coherent geometry and callback timing.

Use the [current tradeoff report](../performance-tradeoffs.md) for the measured
room comparison; [architecture experiments](../isaac-performance.md) retain older
microbenchmarks, moving partitions, CAD scaling and their reproduction commands.
A high RTF in the six-wall room does not imply the same budget with a full arena,
articulated robots, rendered cameras and ROS publication.
