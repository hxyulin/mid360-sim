<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Gazebo Harmonic adapter

Native `gz-sim8` system plugin for the shared nominal MID-360 pattern. This
first version uses **DART with the Bullet collision detector** and Gazebo's
`RaycastData` component, not the regular-grid GPU lidar sensor. It uses the
existing physics world, emits native `gz.msgs.PointCloudPacked`, and adds no
ROS or Gazebo dependency to the Rust crate. Gazebo Classic is not targeted.

## Build and test

Use an installed Harmonic SDK or the isolated Ubuntu 24.04 container:

```sh
cmake -S integrations/gazebo -B local-assets/gazebo-build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build local-assets/gazebo-build -j4
ctest --test-dir local-assets/gazebo-build --output-on-failure
cmake --install local-assets/gazebo-build --prefix "$HOME/.local"
```

The supplied Dockerfile includes the SDK and tools required for a headless test:

```sh
docker build -t mid360-gazebo:harmonic integrations/gazebo
mkdir -p local-assets/gazebo-out
docker run --rm \
  -v "$PWD/integrations/gazebo:/workspace/source:ro" \
  -v "$PWD/local-assets/gazebo-out:/workspace/out" \
  mid360-gazebo:harmonic bash /workspace/source/tests/run.sh
```

This compiles the plugin with warnings denied, runs C++/Rust optical fixtures,
and starts a **real Gazebo server and native transport subscriber** for each
stride 1, 3 and 1001. The subscriber validates actual nonempty cloud layouts,
per-point XYZ against analytic wall intersections, timestamps and lane IDs.
It rejects empty clouds, so an unsupported collision backend cannot pass by
publishing empty data. Native log and frame-count journals remain in the
writable output directory. No GPU, GUI or downloaded scene is required.

## World configuration

Keep the world's physics engine explicit. The standalone Bullet Featherstone
engine does not provide the required ray feature in the tested release.
DART's default ODE detector also does not implement these rays. This plugin
refuses an absent/non-Bullet detector configuration and fails closed if the
engine returns no ray results.

```xml
<physics name="step" type="ignored">
  <max_step_size>0.001</max_step_size>
  <dart><collision_detector>bullet</collision_detector></dart>
</physics>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
  <engine><filename>gz-physics-dartsim-plugin</filename></engine>
</plugin>
```

Attach the plugin to a robot model and name its mounted link:

```xml
<model name="robot">
  <!-- Robot links, inertias and geometry go here. -->
  <link name="lidar"><pose>0 0 0.8 0 0 0</pose></link>
  <plugin filename="mid360-gazebo" name="mid360::Mid360">
    <link_name>lidar</link_name>
    <topic>/mid360/points</topic>
    <frame_id>livox_frame</frame_id>
    <min_range_m>0.1</min_range_m>
    <max_range_m>70</max_range_m>
    <stride>1</stride>
  </plugin>
</model>
```

Set `GZ_SIM_SYSTEM_PLUGIN_PATH` to the directory containing
`libmid360-gazebo.so`, then start the world with `gz sim -s -r world.sdf`.
The native test's [enclosure.sdf](https://github.com/hxyulin/mid360-sim/blob/main/integrations/gazebo/tests/enclosure.sdf) is a complete minimal
example. Its optional `report_file` is for automated diagnostics; omit it in
normal use. The parent directory must already be writable.

## Data contract

- Same 181 Hz / 642697÷65521 Hz clocks, ten quarter-degree optical harmonics,
  four interleaved lanes and exact 65,521-second common cycle as Rust/Isaac.
  The small generated header comes from `export_gazebo_model`; it is not a
  stored scan table or a manufacturer calibration.
- PreUpdate submits explicit rays, Gazebo physics computes them, and PostUpdate
  consumes results. Pausing freezes acquisition. Reset/rewind clears pending
  batches and restarts the explicit clock. At most 200,000 queries are accepted
  per step; larger steps fail rather than silently dropping emissions.
- Per-step geometry and mount pose are frozen; per-ray motion interpolation is
  not implemented here. Use short physics steps. Geoms are **physics collision
  geometry**, so native convex proxies can differ from render/optical meshes.
- Rays start at the actual origin. A surface below minimum range occludes
  farther geometry and produces a miss. Self-occlusion is retained. Ray geometry
  follows the collision engine; no optical material model or IMU is fabricated.
- Output groups approximately 100 ms of nominal emissions. Odd strides reduce
  count but retain original time and lane progression. Batches do not reset the
  scan phase. The latest incomplete publication is discarded on reset.
- Each valid point is a 20-byte little-endian record: float32 x/y/z at offsets
  0/4/8, uint32 `offset_time` in ns at 12, uint8 reflectivity/tag/line at 16/17/18,
  and one padding byte. Reflectivity 100 and tag 0x10 are explicit placeholders.
  Misses are filtered without renumbering emissions. Frame header time is the
  first scheduled emission even if it missed; metadata includes `frame_id`,
  `first_sample_index` and `stride`. XYZ is in mounted-link FLU coordinates.
- `PointCloudPacked` is suitable for a caller's native or ROS PointCloud2 bridge.
  This is not a Livox CustomMsg publisher; its equivalent timing/lane fields
  can be converted by an adapter. No ROS runtime is required by the plugin.

The [RaycastData contract](https://gazebosim.org/api/sim/8/structgz_1_1sim_1_1components_1_1RaycastDataInfo.html)
and [physics feature](https://gazebosim.org/api/physics/7/GetRayIntersection_8hh_source.html)
are supplied by Gazebo. No Gazebo implementation code is vendored. A future GPU
adapter should preserve this time/point contract; the standard regular-grid
GPU lidar cannot simply be substituted for the irregular four-lane pattern.

Regenerate the C++ optical table and cross-language fixtures with:

```sh
cargo run --quiet --locked --example export_gazebo_model > integrations/gazebo/include/mid360/model.hh
```

Validated on Talos with gz-sim 8.15.0 and gz-physics DART 7.8.0. At stride 1,
the final test validated 400,000 points in 20 native messages over 2,000
1 ms physics steps (two simulated seconds); the server run took 1.945 s wall
time. Stride 3 validated 126,673 points in 19 complete messages, and stride
1001 validated 400 points in 20 messages. Partial final publications are not
flushed just to complete a test. These full-server timings include physics,
scheduling and publication; do not compare them directly to Isaac's isolated
GPU query timing. Simple-room throughput is not evidence for a detailed arena.
Local evidence is under ignored `local-assets/gazebo-validation/`.

## Performance profiling

The adapter now shares each Fourier basis across a four-lane optical group and
reserves its 400 kB publication buffer. It retains per-step native intersections,
original emission identities and odd-stride behavior. No geometry is reused
across motion steps, and no points are discarded for speed.

Run `bash /workspace/source/tests/benchmark.sh` inside the development container
after `tests/run.sh`. An optional first argument selects a previously built
plugin directory for an alternating original/optimized comparison. The harness
warms 100 real steps before timing 2,000 more, verifies the actual server
iteration count, and separately measures a valid sensor-disabled SDF world.
It omits expensive subscriber-side per-point validation during timing, while
ordinary native tests still validate points against analytic enclosure hits.

Three final Talos trials gave these median times for two simulated seconds:

| Case | Wall time | Real-time factor |
|---|---:|---:|
| Sensor disabled | 13.6 ms | 146.7× |
| Original plugin | 401.5 ms | 4.98× |
| Shared optical groups | 389.9 ms | 5.13× |

An earlier three-trial run measured 377.3 → 350.3 ms (7.2% less runtime);
the final run measured 2.9% less runtime. Treat this as a modest improvement,
with run-to-run variability, rather than a fixed percentage. Both full-rate
plugins produced 400,000 returns in 20 messages. Cold startup costs roughly
1.3–1.6 seconds in this installation and explains most of the earlier apparent
near-real-time limitation. This remains a six-box enclosure, not an arena test.

Add `<profile_file>/absolute/writable/report.json</profile_file>` to the plugin
for optional monotonic host-time diagnostics. This does not affect simulation
stamps. The report is written on plugin destruction and aggregates PreUpdate,
the intervening physics/scheduler interval, and PostUpdate. Omit it during
ordinary use; no profiling clocks run by default. A final 2,100-step capture
(including warmup) measured 26.1 ms in PreUpdate, 362.3 ms between callbacks and
7.3 ms in PostUpdate: about 92% of the profiled interval is in native physics/
scheduling, not pattern generation or serialization. That interval includes
other engine work and is not a measurement of isolated raycast calls.

The next substantial optimization would need to address native engine ray
queries or use a separately designed optical GPU adapter. Merely delaying
publication does not eliminate the motion-aware per-step intersections.
