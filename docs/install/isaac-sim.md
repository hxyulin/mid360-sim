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
