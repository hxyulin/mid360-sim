<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# One- and two-sensor optimization targets

The optional endpoint is now implemented as `publisher::TypedLivoxPublisher`,
behind `ros2-typed`; see [installation and usage](install/ros2.md#optional-typed-livox-endpoint).
The construction and native-query measurements below retain the original
pre-implementation experiment and its source hashes. Later changes to the
benchmark package/lockfile do not rewrite that historical timing provenance.

For typical one- or two-device use, this investigation selected an
**optional typed Livox publisher**, now available with `ros2-typed`. A broad new
Rust batch-query API has no
demonstrated consumer benefit yet: the simulator adapters already batch through
their own native interfaces, and batching must retain motion and first hits.
The prepared-window API remains optional; its larger multi-device gains do not
establish a common single-device speedup.

These are bounded investigations; the original experiment did not change
production acquisition or publishing. [Raw paired timings, source hashes and generated-interface hashes](/benchmarks/architecture-targets-m5-max.json)
record the 2026-10-23 Apple M5 Max/macOS arm64 experiments. The workstation was
shared, without CPU pinning. Small differences and absolute times need a repeat
on the deployment machine.

## Implemented endpoint validation

A separate paired run measures the actual `LivoxPublisher::publish` and
`TypedLivoxPublisher::publish` methods, including the typed packet validation.
Both paths use fresh owned messages and cached endpoints. Each timing averages
ten packet sets, after two warmups and across ten alternating-order pairs.
Two-device sets publish serially to two separate endpoints. Subscription counts
are checked zero before and after each timing. Acquisition, packet conversion,
clock publication and subscriber latency are excluded.

| Retained points per packet | Devices | Dynamic publish | Typed publish |
|---|---:|---:|---:|
| 6,667 | 1 | 0.723 ms | 0.409 ms |
| 6,667 | 2 | 1.547 ms | 0.899 ms |
| 20,000 | 1 | 2.223 ms | 1.222 ms |
| 20,000 | 2 | 4.457 ms | 2.428 ms |

The observed full-rate saving is about 1 ms per packet per sensor (roughly
10 ms/s of wall work at 10 Hz). This shared-host result supports the optional
path; it does not establish matched-subscriber latency or whole-simulator speed.
[Raw production timings and source hashes](/benchmarks/typed-livox-m5-max.json)
are separate from the original construction probe below.

```sh
pixi run bash scripts/with-ros.sh target/release/mid360-ros-construction-bench --production
```

Build the isolated package using the commands below before this invocation.
The production native regression also checks unchanged headers, misses, offsets,
lanes, clocks, empty/singleton/20,000-point transitions, rejected malformed packet
headers and recovery. Typed unit tests cover owned buffers and invalid coordinate,
count, frame and offset inputs. CI runs typed tests and both smoke strides.
Field regressions use an isolated ROS domain and acknowledge each format before
sending the next, without retrying missing measured packets. A back-to-back
20,000-point PointCloud2/typed-Livox burst lost PointCloud2 in the initial local
Fast DDS test while Livox and clock arrived. Paced field checks do not establish
dual-format delivery under load; use the separate transport benchmarks for that.

## ROS construction

The isolated `examples/ros-construction` package compares the current cached
dynamic construction strategy with generated RMW-native Livox structs, using
rclrs 0.8.0, ros-env 0.3.0, RoboStack Jazzy and Fast DDS. Both paths fill identical
existing records into fresh owned messages. Two-device cases use two independent
endpoints, called serially. No acquisition, packet conversion or subscriber runs
inside timing. There are two warmups and ten alternating-order pairs per case.

The generated path avoids seven named dynamic field accesses per point and the
dynamic sequence's proxy vector. In the pinned implementation,
`MessageStructure::get_field_info` performs a linear field-name search. Its safe
public sequence API does not offer indexed field access or contiguous typed point
storage; the consuming field iterator cannot be applied to borrowed sequence
elements. Keeping dynamic messages while caching field offsets is therefore not
a straightforward local safe change. No raw casts or local unsafe code are used.

| Retained points per packet | Devices | Dynamic build + publish | Typed build + publish |
|---|---:|---:|---:|
| 20,000 | 1 | 2.599 ms | 1.329 ms |
| 20,000 | 2 | 5.354 ms | 2.743 ms |
| 6,667 | 1 | 0.876 ms | 0.461 ms |
| 6,667 | 2 | 1.819 ms | 1.000 ms |

At 20,000 points for one device, dynamic point filling alone takes about
1.203 ms, versus 0.069 ms for the entire typed build. Both
publication/drop stages remain about 1.29 ms. The publication timers
include rclrs/RMW work and message destruction **with no matched subscriber**;
they are not DDS delivery latency. The input models existing full/decimated
packets; no acquisition or downstream deserialization is timed.

At 10 Hz, the observed total saving corresponds to roughly 13 ms of wall work
per second for one full-rate sensor, or 26 ms for two. That is useful headroom,
but modest compared with a complete simulator or a slow subscriber. Typed
publishing retains CustomMsg's per-point representation and cannot remove its
Python object construction cost. The older
[end-to-end transport results](transport-evaluation.md) have a different timing
boundary and should not be compared as a before/after measurement here.

An independent PointCloud2 byte-encoding control costs about 0.056 ms per
20,000-point packet. It excludes ROS sequence copying/publication and uses one
packet even in two-device rows. Removing that copy remains a lower-priority
candidate than Livox construction.

### Correctness and adoption cost

Before timing, every measured size compares generated structs field-for-field
against the dynamic result, including headers, reserved bytes, offsets and lanes.
A separate native rclpy subscriber checks all fields and changing packet sizes
`0/1/2/20000/3/0/1`. Typed singleton delivery passes; the production dynamic
publisher's documented rclrs singleton rejection remains unchanged. Runtime
validation covers macOS arm64 only.
Dependency-policy checks retain the existing scoped async-std maintenance
exception; no dependency policy or production lockfile is changed.

The [upstream message-generation workflow](https://github.com/ros2-rust/ros2_rust/blob/main/docs/message-generation.md)
supports typed interfaces. This local environment already exports generated
Livox Rust code through `ros-env`, so the prototype builds without checking any
generated bindings into the repository. Other C-introspection-only installs may
not export those types. A production path should be an explicit optional feature
or companion ROS crate, retaining the dynamic fallback and the dependency-free
core. Validate generation and packaging on Linux before claiming support there.
Use RMW-native types directly to avoid an extra idiomatic-Vec-to-C-sequence copy;
zero-copy middleware is a separate question and is not demonstrated here.

Reproduce after [ROS setup](install/ros2.md), from the repository root:

```sh
pixi run interfaces
pixi run bash scripts/with-ros.sh cargo build --release --locked --manifest-path examples/ros-construction/Cargo.toml
pixi run bash scripts/with-ros.sh target/release/mid360-ros-construction-bench --validate
pixi run bash scripts/with-ros.sh target/release/mid360-ros-construction-bench
pixi run bash scripts/with-ros.sh cargo clippy --locked --manifest-path examples/ros-construction/Cargo.toml -- -D warnings
```

The package is isolated because its generated-interface requirement should not
silently become a requirement of the existing `ros2` feature. Its own lockfile
pins the experiment; generated code remains in `.ros/install`, not in source.

## Native query batching

MuJoCo already calls `mj_multiRay`; Isaac launches Warp query kernels and Gazebo
submits a ray array to the physics backend. They do not route each native query
through Rust's `Scene::nearest_hit`. Adding a core batch trait alone would not
accelerate those adapters. The standalone analytic Rust scene has no Python/FFI
boundary to amortize. Its motion still needs evaluation at every emission.

The inspected clean RM checkout at `318ee089626492fa6e2eb7ccb07c442801e80b15`
also has `StaticGeometry::cast_rays_into`, which loops over validated metric rays
and calls the same scalar traversal used internally by the LiDAR. A batch-shaped
API alone does not establish shared BVH traversal or a GPU query. No RM source
was changed or timed in this investigation.

### MuJoCo query-only experiment

MuJoCo 3.7.0, NumPy 2.5.4. The script tests 67-ray stride-3 and 200-ray
stride-1 windows for one/two devices in six- and 134-geom scenes. Each ray has its
own yaw rotation; translations use the adapter's 0.5 mm midpoint-origin policy.
Direction and pose-array generation are outside timing; owned output copies are
inside both scalar and batch timings. Two warmups precede eight alternating
pairs, twenty calls each. A moving blocker is captured at a new coherent pose
before each pair; geometry stays current throughout the call, as in the adapter.
There is no physics stepping, GPU or ROS.

The timed scalar path uses **exactly the batch's grouped origins**. Distances
agree within 1e-12 m and geom IDs match at all ten snapshots in every case. A
separate per-emission-origin scalar reference measures translation approximation;
these two comparisons must not be conflated. Warm repeated early-window rays
and synthetic geometry limit extrapolation to full scans or native robot scenes.

Selected one-device, 200-ray query times:

| Geoms | Translation across window | Origin groups | Scalar calls | Existing native batch |
|---:|---:|---:|---:|---:|
| 6 | 0 mm | 1 | 0.148 ms | 0.015 ms |
| 6 | 2 mm | 2 | 0.145 ms | 0.016 ms |
| 6 | 20 mm | 20 | 0.145 ms | 0.049 ms |
| 134 | 0 mm | 1 | 0.271 ms | 0.086 ms |
| 134 | 2 mm | 2 | 0.264 ms | 0.101 ms |
| 134 | 20 mm | 20 | 0.268 ms | 0.400 ms |

With per-ray rotation and no translation, batching is exact in this experiment
and already pays off. As translation forces more groups, repeated native setup
can dominate: the 134-geom/20-group case is slower than scalar Python calls.
The [native API](https://mujoco.readthedocs.io/en/3.7.0/APIreference/APIfunctions.html#mj-multiray)
shares one origin per batch. Concatenating two displaced sensor origins into one
batch would violate that contract. Separate calls are needed for the two mounts.

The 2 mm translation cases had no changed geom IDs over these snapshots, with
maximum exact-origin range differences around 0.65 mm. The 20 mm clutter stress
case changed the nearest geom on 10 of 2,000 one-device emissions and produced
a maximum range difference of 4.10 m, despite sub-millimetre origin displacement.
A positional bound cannot bound range discontinuities at edges. At a 1 ms window,
2/20 mm correspond to 2/20 m/s: the latter is a deliberate stress case, not a
recommended acquisition interval or motion model. The benchmark retains physical
first-hit results, including near surfaces; it does not apply range/noise effects.

Run in the [MuJoCo environment](install/mujoco.md):

```sh
python scripts/benchmark-query-batches.py > local-assets/query-batches.json
```

## Matched delivery controls

The follow-up uses the actual publishers and one rclpy subscriber, two devices,
20,000 points per device at 10 Hz, three alternating pairs of 30 frames.
Each control changes one setting from best-effort depth 5. Measured messages
are neither acknowledged nor retried by the application; a 300 ms drain follows
each run. [Raw sends, arrivals, source hashes and protocol](/benchmarks/delivery-controls-m5-max.json)
record the shared, unpinned macOS arm64/Fast DDS host. Control blocks run in a
fixed order, so small timing differences are not causal evidence.

| Control | Dynamic received / sent | Typed received / sent |
|---|---:|---:|
| Decoded subscriber, default QoS | 129 / 180 | 114 / 180 |
| Serialized-byte subscriber | 125 / 180 | 105 / 180 |
| Depth 20 | 128 / 180 | 105 / 180 |
| Reliable QoS | 180 / 180 | 180 / 180 |
| Separate device publications by 5 ms | 134 / 180 | 114 / 180 |

Reliable publishing took median 4.804/3.379 ms per dynamic/typed device set;
maximum publication completion lateness stayed below 17 ms. Received-packet
callback latency was 32.2/35.3 ms, versus 24.6/25.7 ms at default QoS. Missing
packets are absent from latency statistics, and the raw callback excludes
deserialization, so those latency populations are not equivalent. These short
runs support caller-selected reliable QoS for this workload, not a delivery
guarantee or a middleware-wide diagnosis. Raw reception and deeper queues did
not remove losses; faster construction alone did not improve delivery.
A separate precommit repeat reversed the control block order: reliable again
delivered 180/180 for both paths, while best effort delivered 123/180 dynamic
and 110/180 typed. Reliable publish sets took 5.261/2.910 ms and received-packet
latency was 38.7/36.7 ms. The repeat is retained in the same evidence file.

## Supported exact MuJoCo queries

`scan_rolling(query_mode="exact")` now uses public Python `mj_ray` calls at each
emission's interpolated origin. Grouped queries remain the default. Complete
acquisition measurements include physics, refreshed geometry, optics, queries,
owned output and 100 ms cloud assembly, excluding transport. With three
interleaved pairs, exact/grouped wall-time ratios were 0.42–0.83 in the fast
cluttered cases and 1.37–2.37 near stationary motion. These are synthetic scenes
on a shared host, not universal crossover thresholds.
[Paired measurements and source provenance](/benchmarks/mujoco-exact-rolling-m5-max.json)
describe the workload. The fused C-loop prototype remains separate: the supported
mode uses no private pointer bridge or external native extension.
A fresh precommit repeat of the complete matrix gave exact/grouped ratios of
0.41–0.76 for fast clutter and 1.43–2.50 near stationary motion; simple fast-motion
cases ranged from 0.92 to 1.64. Explicit selection remains appropriate.

## Shared Isaac history in acquisition

`SceneTransformHistory` now provides explicit shared captures for rigid-history
consumers; self-capture uses bounded contiguous storage. Sensor poses, times and
flush schedules remain independent. Native regressions include actual PhysX
writeback, near occlusion, nested scale/reflection, removal/recreation and retained
snapshot ownership. The union-reference comparison differed by at most 5.25 µm.

Final precommit measurements used five alternating warmed acquisition trials on
an RTX 5090, eight objects and 2,000 distinct 1 ms captures. One-sensor self-capture
measured 1.352 → 1.379 s (+1.98%); two sensors sharing captures measured
1.738 → 1.788 s (+2.88%). Baseline and candidate ranges matched exactly. Review
also fixed invalid source-transform acceptance and stale mesh histories after
hole/subdivision edits; all final native checks passed.

The replay includes USD edits, capture, tracing and owned range readback, excluding
physics, rendering, XYZ packaging and transport. Small acquisition regressions
remain; earlier capture-only savings did not carry through this workload. Trials
were not CPU-pinned, have no confidence intervals, and include one two-sensor
baseline outlier. GPU upload sharing and AABB gating remain unimplemented.
[Paired trials, native checks and source provenance](/benchmarks/isaac-shared-history-rtx5090.json)
record the scope. See [shared capture usage](guide/acquisition.md#explicit-shared-isaac-rigid-captures).

## Adoption and remaining evaluation

1. The optional **typed Livox endpoint** now uses the existing owned packet
   contract. Matched delivery controls above retain existing defaults and expose
   caller-selected QoS. Compare complete simulator/consumer CPU on deployment
   hosts before selecting defaults. No middleware switch is justified yet.
2. Keep current adapter batching and short coherent scene captures. Investigate
   a supported fused per-ray-origin binding for fast translation; the public
   scalar exact mode is available now. Do not relax the tolerance or freeze poses.
3. Add a bounded Rust batch interface only alongside a backend that can exploit
   it. Require explicit per-emission times, perturbed directions, exact result
   ordering and first-surface occlusion. Compare complete one/two-device
   acquisition before exposing another public API.
