<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Runtime and motion accuracy tradeoffs

**A bounded win, not a universal one:** the opt-in headless Isaac/Warp bulk
history path beats the actual official Livox Gazebo callback on runtime and
rolling-motion agreement in this static-room test. Ordinary USD writeback,
MuJoCo rolling acquisition and our current Gazebo Harmonic integration do not
beat that official runtime. No production defaults were changed to hide this.

## What was measured

Talos: Ryzen 9 9950X3D, RTX 5090, NVIDIA driver 615.78; Isaac Sim 6.2.0 / Warp
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

## Results

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

The next useful proof is a matched moving-robot scene with native pose access,
self-occlusion and optical partition updates, including publication costs.
Current results support the headless static-scene path; they do not establish
that every backend beats official on both fronts.
