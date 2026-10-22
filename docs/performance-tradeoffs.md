<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Runtime and motion accuracy tradeoffs

**A bounded win, not a universal one:** the opt-in headless Isaac/Warp bulk
history path beats the actual official Livox Gazebo callback on runtime and
rolling-motion agreement in this static-room test. Ordinary USD writeback,
MuJoCo rolling acquisition and our current Gazebo Harmonic integration do not
beat that official runtime. No production defaults were changed to hide this.

For current API capabilities and the definitions of each accuracy metric, see
the [validation guide](guide/validation.md). Historical Isaac timing/scaling
experiments are retained [separately](isaac-performance.md); their settings differ
from the comparison below.

## What was measured

Benchmark hardware: Ryzen 9 9950X3D, RTX 5090, NVIDIA driver 615.78; Isaac Sim 6.2.0 / Warp
1.17, MuJoCo 3.7.0 / NumPy 2.5.4, Gazebo Classic 9 / ROS Melodic, and Gazebo
Harmonic 8.15. A [two-second high-motion MoCap segment](backend-ground-truth.md)
drives the sensor inside an exact six-wall enclosure. All nominal paths emit
400,000 rays at 200,000/s, with no range noise. Startup, compilation and offline
analysis are excluded. This is neither a full robot/arena benchmark nor an
estimate of real MID-360 range accuracy. The reference interpolates measured
100 Hz poses; it cannot establish true motion between those measurements.

RMS below measures ranges against per-emission analytic intersections using
that reference. Ours uses the nominal fitted pattern. Official uses its native
CSV directions; RTX uses a custom repeating 32-elevation rotary profile, **not
an NVIDIA MID-360 preset or radar**. Each native direction is evaluated against
its own corresponding reference intersection. Different distributions make
these policy comparisons, not identical-ray microbenchmarks.

![Original static-room full-loop and marginal sensor runtimes; measurement boundaries differ.](/figures/runtime.svg)

[Figure data and context](guide/results.md).

## Cross-backend baseline results

These are the original matched-room measurements, before the allocation-focused
follow-up below. That follow-up remeasures selected paths; the entire
cross-backend matrix has not been rerun after those changes.

RTF is simulated seconds / wall seconds; higher is faster. Wall time is the
median of three trials except RTX (one complete native replay per rate).

| Path / policy | Wall time | RTF | Range RMS | Runtime scope |
|---|---:|---:|---:|---|
| Official Livox / Gazebo 9, 10 Hz snapshots¹ | 134.17 ms / 2 s | 14.91× | 376.34 mm¹ | Native physics + callback + ROS XYZ sink |
| Ours Isaac/Warp, bulk history, headless² | **110.70 ms / 2 s** | **18.07×** | **0.251 mm** | Native physics + query + XYZ sink |
| Ours Isaac/Warp, bulk history, USD writeback on | 299.52 ms / 2 s | 6.68× | 0.251 mm | Native physics + query + XYZ sink |
| Ours MuJoCo, 1 ms held poses | 155.03 ms / 2 s | 12.90× | 5.666 mm | Pose updates + queries + owned arrays; no dynamics |
| Ours MuJoCo, rolling poses, 0.5 mm origin bound | 245.64 ms / 2 s | 8.14× | **0.294 mm** | Pose updates + queries + owned arrays; no dynamics |
| Ours Gazebo Harmonic, 1 ms held poses³ | 389.9 ms / 2 s | 5.13× | 5.666 mm | Warmed native engine + query + publication |
| Native RTX Motion BVH, 10 Hz updates | 414 ms / 3.4 s | 8.22× | 152.24 mm | Native physics + rendering + writer |
| Native RTX Motion BVH, 100 Hz updates | 2.215 s / 3.4 s | 1.53× | 9.400 mm | Native physics + rendering + writer |
| Native RTX Motion BVH, 1000 Hz updates | 16.484 s / 3.4 s | 0.206× | 2.788 mm | Native physics + rendering + writer |

¹ Official messages have **no per-point timestamps**. Its rolling RMS is a
**harness-assigned uniform 5 µs correspondence diagnostic**, not an error
decoded from native acquisition times. After setting `range.min=0`, its
snapshot geometry agrees to 0.00032 mm RMS. With its original 0.1 m setting,
this fixture instead produces a uniform 100 mm shortfall. We use the favorable
zero setting and do not count that bias as our accuracy advantage. The room
contains no surfaces within minimum range.

² Static optical geometry, caller-owned boundary pose history, PhysX thread
count zero, `SimulationManager.step()` and `/physics/updateToUsd=false`.
All 2,000 native 1 ms integration steps still execute. An independent dynamic
probe moves 0.200 m in each trial, verifying that dynamics were not skipped.
This switch is appropriate only when pose consumers use an explicit/native
pose source. A USD-mounted sensor or renderer cannot assume updated transforms
with USD writeback disabled. Existing USD-driven extension defaults remain.

³ Harmonic runtime is the separate warmed constant-motion six-box benchmark,
not the MoCap replay timer; its error comes from MoCap replay. It is shown as
context, not a paired speed/accuracy claim. Reducing the native physics step to
200 µs lowers RMS to 1.166 mm, but requires five times as many engine steps.
The earlier approximately 1.77 s 200 µs run includes cold startup and is not
comparable to the warmed timings above.

RTX replay includes 1 s stationary warmup and 0.4 s drain in its 3.4 s loop;
only the middle 2 s / 20 scans enter accuracy analysis. It advances 1 ms
physics at every update rate. The writer captures all 20 scans; valid returns
are 399,969 / 399,972 / 399,979 respectively. RMS excludes explicitly invalid
returns. No extra dynamic probe body is present in RTX, unlike the Warp test.
These are controlled engine harnesses, not equivalent full application loads.

## What the optimization changes

`StaticSceneBatch.scan_history()` queues up to 100 segments / 200,000 rays
with one owned NumPy pose history, instead of constructing per-step Python
transforms and transferring them individually. GPU interpolation and nearest
hits are unchanged. It rejects moving optical sources and changing scene
revisions; it does not invent obstacle motion. Legacy per-step APIs remain.

Bulk Warp sensor work (query, acquisition API, readback and XYZ sink, excluding
physics) is **13.50 ms** for 400,000 rays, versus **122.67 ms** in the official
callback/publication interval: approximately **9× lower marginal sensor cost**.
The whole-loop gain is smaller: **17.5% less wall time / 1.21× throughput**.
Trial ranges were 109.66–110.76 ms for headless Warp and 131.94–134.71 ms for
official in the clean reproduction (the preceding three-run official median
was 136.75 ms). This modest full-loop margin needs rechecking on actual robot scenes.
Thread count zero avoids worker scheduling in this tiny workload; it is not
a recommendation for large multi-body simulations. NVIDIA documents these
[workload-dependent PhysX tradeoffs](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/reference_material/sim_performance_optimization_handbook.html).

`MujocoSensor.scan_rolling()` uses vectorized quaternion rotation per ray and
native multi-ray calls grouped by ray-origin displacement. Its 0.5 mm setting
bounds origin approximation, **not range error** at grazing incidence. It
reduces this sample's RMS by about 19× for 58% more runtime than held acquisition.
Moving obstacle geometry is still the current `mjData` for that call.

## Follow-up: allocation cost and denser poses

The next pass preserves the existing sensor motion model and acquisition times:

- MuJoCo expands quaternion vector rotation component-wise, avoiding temporary
  `np.cross` arrays and axis-handling overhead. Five alternating before/after
  replays on the Ryzen 9 9950X3D reduced median wall time from **237.19 to
  217.51 ms (8.3%)** for the same 400,000 emissions. Hit masks were identical;
  maximum range difference was 1.07e-14 m. A separate Apple CPU run improved
  by 8.0%. These include native pose updates, queries and owned range arrays,
  not integrated dynamics, rendering, XYZ publication or transport.
- Isaac static tracing now reuses bounded GPU output allocations. Seven
  alternating RTX 5090 microbenchmark replays measured **7.067 to 6.954 ms
  (1.6%)** per 400,000 rays, with identical output. This includes bulk history
  acquisition and owned range readback, but excludes physics, XYZ sinks,
  rendering and publication. The small gain is not a whole-simulation claim.
  One full replay per mode warmed the GPU paths before timing. Growing and
  shrinking batches retain owned host frames and expose no capacity tail.

For accuracy, shorter pose intervals reduce interpolation error in the same
MoCap reference; a tighter MuJoCo origin tolerance reduces a separate
approximation. The following uses the optimized rolling code, identical
400,000 emission times, and the reference's original measured-pose interpolation:

| Pose interval | Origin tolerance | Range RMS | Wall time / 2 simulated seconds |
|---|---:|---:|---:|
| 1 ms | 0.5 mm | 0.294 mm | 217.51 ms, five-run median |
| 1 ms | 0.05 mm | 0.252 mm | 300.44 ms, single run |
| 0.2 ms | 0.5 mm | 0.0454 mm | 692.10 ms, single run |
| 0.2 ms | 0.05 mm | 0.0276 mm | 734.50 ms, single run |

The denser modes execute five times as many pose-update/query calls; native
robot dynamics are not integrated in this runner. These tiny residuals are
**agreement with an interpolated reference**, not evidence of corresponding
hardware precision or new 5 kHz ground-truth measurements. The chosen recording
is a changing-motion stress sample, not a general high-speed accuracy bound.
For the larger accuracy factors—timestamp alignment, acceleration and mounted
extrinsics—see [high-speed acquisition](guide/acquisition.md#high-speed-acceleration-and-mounted-sensors).

Reproduce accuracy settings with `check-backend-ground-truth.py prepare
--step-us 200`, then `mujoco --rolling --origin-tolerance-m 0.00005` using the
matching fixture. Compare each output with `evaluate --backend mujoco`; keep
step duration and original per-emission reference paired. The before/after
allocation probes alternate current code against source revision `11a4673`.
Ignored evidence is `local-assets/perf-followup/linux-mujoco-results.json`,
`gpu-results.json` and `accuracy-*.evaluation.json`. Regression checks include
native per-ray rotation at small/near-half turns, output ownership and both
Isaac hybrid union paths. No scanning or physics-rate defaults were changed.

## Further pass: packed metadata and the actual arena loop

Profiling the static bulk-history path found five small metadata allocations/
uploads per flush: counts, offsets, endpoint poses and interpolation fractions.
A [Warp structured record](https://nvidia.github.io/warp/v1.17/user_guide/runtime.html#structured-arrays)
now carries these in one upload. The rigid-object path
shares that record and fuses static/moving nearest-hit queries into one kernel,
so each emission's sensor pose is interpolated once. Ray counts, per-point times,
scene capture intervals and minimum-range occlusion remain unchanged.

On the same Linux Ryzen 9 9950X3D / RTX 5090 system, seven alternating trials
against the previous buffer-reuse implementation gave:

| Acquisition workload, 400,000 rays | Before | After | Wall-time reduction |
|---|---:|---:|---:|
| Static room, bulk history and owned range readback | 6.860 ms | 5.953 ms | 13.2% |
| Room with eight moving cubes, per-step USD edits/captures and readback | 1.3156 s | 1.2952 s | 1.6% |

Static outputs were identical. Moving outputs retained identical hit/miss masks;
maximum range difference was 2.86e-6 m (float32 rounding). Neither timer includes
physics integration, rendering, XYZ packaging or transport. CPU pose authoring/
capture dominates the moving-cube workload; the static gain does not transfer
as a fixed percentage to moving scenes.

A full-arena code-only pair with eight moving CAD roots was essentially flat:
1.674 versus 1.685 s for two simulated seconds. This is not evidence of a
whole-simulation speedup from the kernel changes. Physics-only stepping in those
runs already cost approximately 1.15 s. A separate CPU profile also exposed
repeated SDK timestep/scene lookup and USD event-processing overhead.

### Opt-in caller-owned headless stepping

The arena harness now compares public SDK stepping choices with explicit worker
counts. The tested scene has 355,542 static triangles and eight neutral CAD
robots with 222,900 triangles each. These have free moving roots, not wheel,
contact or articulated dynamics. USD writeback stays enabled in every row:

| Physics workers / SDK call pattern | Wall time / 2 simulated seconds | RTF |
|---|---:|---:|
| 8 workers, `SimulationContext.step` once per tick | 1.636 s | 1.223× |
| 0 workers, `SimulationContext.step` once per tick | 1.111 s | 1.800× |
| 0 workers, `SimulationManager.step(steps=100, callback=...)` | 0.990 s | 2.021× |

These are **single native runs per configuration**, not statistical guarantees.
The tuned row used 39.5% less wall time than the explicit eight-worker row.
Worker count zero executes PhysX synchronously; it is workload-specific and
may be worse for complex contact/joint worlds.

`steps=100` means **100 separate 1 ms integrations**, not one 100 ms physics
step. Every PhysX/LiDAR callback still runs, and the supplied SDK callback runs
after each individual step. All three configurations produced 20 frames,
400,000 scheduled ranges, 88,668 valid returns and **the same complete range
SHA-256**. A separate 5,000-ray check retained identical hit/miss masks against
combined geometry, with maximum range difference 1.43e-6 m and 24 robot-occluded
returns. The shared MoCap room replay also retains 0.251334 mm RMS against its
original reference.

This is an opt-in headless caller loop, not a change to sensor or Isaac defaults.
Controllers/input processing must stay in per-step callbacks if they require
1 kHz interaction; processing commands only between 100-step calls changes
control latency. Render scheduling, Fabric updates and graphical timeline tasks
need their own integration. Do not advance physics from inside a sensor callback.
The sensor adapter remains passive and owns no dynamics loop.

### Reproduce these probes

Run the source comparison with an external earlier `backend.py` and adjacent
`nominal.json`; the test alternates both implementations in the same process
and warms each with one replay before timing:

```sh
./python.sh /path/mid360-sim/integrations/isaac-sim/tests/history_benchmark.py \
  --input /tmp/mocap-fixture.npz --baseline /tmp/baseline/backend.py \
  --mode static --output /tmp/history-static.json
```

Use `--mode rigid --objects 8` for moving sources. For the arena, prepare the
caller-owned geometry using the [CAD benchmark instructions](isaac-performance.md#caller-owned-arena-cad-robot-benchmark), then run:

```sh
./python.sh /path/mid360-sim/integrations/isaac-sim/tests/arena.py \
  --input /tmp/arena.npz --mode rigid --robots 8 --steps 2000 --verify \
  --physics-threads 0 --physics-driver manager --physics-batch-steps 100 \
  --output /tmp/arena-manager.json
```

Compare `--physics-driver context --physics-batch-steps 1` with explicit
`--physics-threads 8` or `0`. `--backend-file` selects an external acquisition
implementation while preserving the scene/extension workload. `--profile-cpu`
writes a separate profile and labels its timings as profiled; do not mix them
with ordinary throughput numbers. The reports record backend checksums.
Ignored evidence is under `local-assets/perf-pass2/`: `pass2-history-*.json`,
`pass2-arena-final-*.json`, `pass2-arena-profile.profile.txt` and
`ground-truth-evaluation.json`.

The next likely gains are coherent native/bulk pose capture and reducing SDK/
USD bookkeeping. A GPU-resident downstream consumer could avoid readback, but
that alone does not speed up a ROS pipeline that needs CPU message data. No
public GPU-frame API, CUDA-graph mode or new physics-rate default is introduced.

## Reproduce and inspect

Prepare the [shared fixture](backend-ground-truth.md), preserving its source
checksum. For the new MuJoCo option:

```sh
python scripts/check-backend-ground-truth.py mujoco \
  --input /tmp/mocap-fixture.npz --output /tmp/rolling.npy --rolling
```

Run Isaac with its bundled Python; the existing simulation supplies the poses:

```sh
./python.sh /path/mid360-sim/integrations/isaac-sim/tests/ground_truth.py \
  --input /tmp/mocap-fixture.npz --output /tmp/warp.npy \
  --modes history --repeats 3 --physics-driver manager \
  --physics-threads 0 --no-usd-writeback
```

Omit `--no-usd-writeback` for the slower USD-updated control. Use
`rtx_ground_truth.py --render-steps 100` / `10` / `1` with `--physics-threads 0`
for the native RTX runs. See the [official native harness](https://github.com/hxyulin/mid360-sim/blob/main/integrations/gazebo-classic-reference/README.md)
for pinned upstream source/image, synchronous callback scheduling, ROS drain
scope and minimum-range configuration. Only its obsolete protobuf link name
is adapted; acquisition C++ is unchanged.

Local ignored evidence: `local-assets/realtime/pareto-warp-native.timing.json`,
`pareto-warp-native-accuracy.json`, `pareto-warp.timing.json`,
`final-{held,rolling}-{1,2,3}.timing.json`, `pareto-rtx-*.json`,
`pareto-rtx-evaluation.json`, and `official-native/` logs, source hashes and
`reproduction-gt.json`. Source/algorithm tests are committed with the harness code;
generated captures and installed simulator runtimes remain outside releases.

Further proof needs articulated/contact-driven robots with coherent native pose
access, self-occlusion and optical partition updates, including transport costs.
The CAD-root stepping probes above do not cover those dynamics.
Current results support the headless static-scene path; they do not establish
that every backend beats official on both fronts.
