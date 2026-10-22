<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Isaac architecture and scaling experiments

These are **earlier, separately scoped architecture experiments** retained for
reproduction and scaling analysis. They are not a single performance ladder:
scene complexity, physics driver, pose source, output schedule and measured
windows differ. For the current measured-motion comparison against native
official/RTX implementations, start with [runtime/accuracy tradeoffs](performance-tradeoffs.md).
For current API setup and limits, use the [Isaac install guide](install/isaac-sim.md).

GPU and native Isaac measurements below use a Linux system with a Ryzen 9
9950X3D and RTX 5090, running Isaac Sim 6.2.0 with Warp 1.17.
The separately identified Apple CPU direction benchmark has a different scope.

The numbers below are numerical checks and throughput probes, not calibrated
sensor accuracy or guaranteed application deadlines. Dataset/CAD assets and
ignored local reports are not shipped in release archives.

## Initial microbenchmarks and live callback baseline

Axiom's release direction-only benchmark measured 9.10 ms scalar versus
5.31 ms grouped per 200,000 rays (30 measured repetitions, 5 warmups). This is
about 1.71x for generating rays, excluding geometry/noise/transport. Grouped
Rust results remain bit-identical to scalar/chunked/strided generation.

Initial GPU enclosure measurements were around 0.2 ms per 20,000 rays,
including transform synchronization and synchronized host output copies,
after 5 warmup batches. See the final local validation report for measured
median/p95. These are a simple enclosure, not arena-scale or full-app FPS.
BVH construction, initial shader/JIT startup, dynamic refits and ROS are excluded
from that static-scene timing. A subsequent [Gazebo CPU adapter](install/gazebo.md) now uses the same compact
model; its collision geometry and per-step motion approximations differ from
this optical-mesh GPU backend.

The earlier GPU validation measured **0.183 ms median / 0.192 ms p95** per
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

With Isaac Sim 6.2.0 on the RTX 5090, a 10-second simulated run processed
2,000,000 rays into 100 frames in 9.315 s wall time: **1.074× real time**.
The physics-only baseline took 3.390 s. Sensor-enabled steps had a 0.920 ms
median and 1.044 ms p95; 1,357 of 10,000 steps exceeded 1 ms. Average throughput
kept up in this simple enclosure, but individual deadlines were not guaranteed.
The result includes callback/USD synchronization and host array costs, unlike
the large-batch GPU microbenchmark. It excludes rendering and DDS/network
publication. Raw reports/logs are retained in ignored `local-assets/realtime/`.

## Deferred readback comparison

Earlier optimized runs (10 simulated seconds / two million rays each):

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

## Segmented tracing and native pose-reader probe

Measured architecture probes, each with 1 ms physics, 100 ms output,
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

## Moving partition comparison

Matched results on the same system (1 kHz physics, 200,000 rays/s, 100 ms output, USD poses),
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

## Caller-owned arena / CAD robot benchmark

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

Measured arena results on the same system use the supplied RM2027 runtime Low visual package
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

