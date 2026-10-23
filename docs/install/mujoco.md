<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# MuJoCo adapter and shared NumPy scan

Install into a Python 3.10+ virtual environment. From the source root:

```sh
python -m pip install './integrations/python[mujoco]'
python -m unittest discover -s integrations/python/tests -v
python integrations/python/benchmark.py
```

If a checksummed release wheel is available, install it with the command below.
The current candidate has not been published as a tagged release; use the source
installation above. Consult [distribution](../distribution.md) for availability.

```sh
python -m pip install 'mid360_sim_adapters-0.1.0-py3-none-any.whl[mujoco]'
```

The package is not published on PyPI. Dependencies are installed separately;
no MuJoCo or NumPy runtime is bundled in the wheel.

Validated with MuJoCo 3.7.0 and NumPy 2.5.4 on macOS arm64; the release-wheel
installation also passes native checks with MuJoCo 3.16.0. Python >=3.10,
NumPy >=2 and MuJoCo >=3.7,<4 are declared; other versions are not individually
validated.
No display, GPU or OpenGL context is required for these CPU ray queries.

```python
import mujoco
from mid360_sim.mujoco import MujocoSensor

# Add <site name="lidar" pos="0 0 .8"/> at the desired body/mount in MJCF.
model = mujoco.MjModel.from_xml_path("robot.xml")
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
sensor = MujocoSensor(model, data, "lidar", min_range_m=.1, max_range_m=70.)
frame = sensor.scan(first_index=0, count=200, epoch_ns=0)  # 1 ms at full rate
print(frame.timestamps_ns, frame.ranges_m, frame.geom_ids)
packet = frame.livox_fields()
```

The adapter calls native `mj_multiRay` against the caller's **existing** matched
`mjModel`/`mjData`. It never calls `mj_step`, creates another physics world or
reads wall time. Call `mj_forward`/`mj_kinematics` after editing qpos. The caller
owns scheduling, reset, synchronization and global sample indices. MuJoCo may
use its ray scratch fields; do not concurrently mutate/query the same mjData.
Queries preserve dynamics time/qpos, not an assertion that every mjData byte is
unchanged. Local rays are FLU (+X,+Y,+Z), using the site orientation in the live
model. No quaternion conversion is needed for the default site-mounted API.

Default `scan()` holds the mount and geometry during its batch. Keep these
batches short (e.g. 200 rays per millisecond). For finer sensor motion, use:

```python
# Caller-owned boundary poses: metres, xyzw unit quaternion.
a = ((0, 0, .8), (0, 0, 0, 1))
b = ((.001, 0, .8), (0, 0, .0005, .999999875))
frame = sensor.scan_rolling(0, 200, a, b,
    pose_fraction=(0, 199 / 200), origin_tolerance_m=.0005)
```

`scan_rolling()` interpolates orientation per emission using shortest-arc SLERP.
Native `mj_multiRay` shares an origin, so translation is divided into groups
with midpoint origins. The tolerance bounds **origin displacement**, not range
error at grazing surfaces. Smaller tolerances can increase query overhead.
The scene remains the current `mjData` throughout each call: this API supplies
sensor history, not obstacle history. It never advances physics. Returned
arrays own their data. See the [runtime/accuracy comparison](../performance-tradeoffs.md)
for measured benefits and costs; the rolling option is not automatically faster.

For exact per-emission translation, explicitly select the public scalar ray path:

```python
frame = sensor.scan_rolling(0, 200, a, b,
    pose_fraction=(0, 199 / 200), query_mode="exact")
```

`query_mode="grouped"` remains the default. `"exact"` uses the same per-emission
rotations and translations with one public Python `mj_ray` call per emission;
`origin_tolerance_m` does not control its origins. It can change edge hits relative
to midpoint grouping and incurs Python call overhead. It is not a bit-preserving
optimization of grouped results. Both modes support 1–200,000 rays per call,
existing group/body filters, range limits, owned output and the current geometry
snapshot. Exact queries require a live `MjModel` and its matching `MjData`.

MuJoCo ray tests use its geoms, not an independent render scene. Include
suitable optical mesh geoms if collision proxies erase visual detail. The
six-entry `geom_groups` filter is explicit; `contype`/`conaffinity` do not select
optical geometry. MuJoCo's transparency/visibility rules apply. The default
`body_exclude=-1` includes robot self-occlusion; excluding a body is an explicit
caller policy. Ray origins are not advanced by min range: a too-close opaque
surface still blocks the wall behind it. Max range and invalid hits become -1.

`frame.livox_fields()` returns transport-free `timebase`, `offset_time`, local
XYZ, lane, and synthetic tag/reflectivity fields. Misses preserve timestamps;
packet offsets are relative to the first scheduled ray even when it missed.
ROS publishing/noise/IMU/material response are not implemented by this adapter.
Callers can transport the fields; no ROS dependency is imported here.

`mid360_sim.directions(first_index, count, stride)` provides vectorized NumPy
scan geometry using the same quarter-degree coefficients and 18.2-hour clock
as Rust/Isaac/Gazebo. It shares harmonics over groups and is checked against
Rust-generated fixtures, including large indices and the long-cycle boundary.
It does not load any capture replay table. The bundled tiny `nominal.json` is
regenerated from `cargo run --example export_isaac_model`; keep it byte-identical
to the Isaac model after updating the Rust source.

Nine native regressions cover optical fixtures, analytic distances, moving
mounted poses, chunk/stride consistency, timing, group filtering, near
occlusion, validation and dynamics-time/qpos preservation. The direction-only
cross-language comparisons tolerate floating-point differences, not hardware
angular error. Sampling is deterministic for the tested NumPy implementation.

Axiom's benchmark measured **4.21 ms median / 4.77 ms p95 per 20,000 rays**
(30 measurements after 5 warmups) for NumPy generation, native queries and owned
output arrays in a static six-wall room. It excludes physics stepping,
rendering and transport, and is not an arena-scale result. It is not a
controlled CPU/GPU comparison with the separately measured Linux/RTX 5090 Isaac benchmark.

See [MuJoCo's native ray API](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-multiray)
for engine semantics. MuJoCo/NumPy remain separately installed dependencies;
no engine implementation code or recorded points are vendored.

### Physics-step acquisition benchmark

To exercise motion-aware 1 ms acquisition instead of the large-batch microbenchmark:

```sh
PYTHONPATH=integrations/python python integrations/python/benchmark_realtime.py \
  --steps 2000 --output local-assets/realtime/mujoco.json
```

This runs the existing sensor against a translating/rotating free body in a
six-wall room, queries 200 rays after each 1 ms physics step, refreshes mounted
transforms after integration, and assembles transport-free 100 ms output arrays.
It compares a physics/capture-only baseline against sensor-enabled stepping.
Warmup is excluded; original emission indices, offsets, point counts and mount
motion are checked. Reports include real-time factor, step median/p95 and the
number of steps exceeding 1 ms. There is no sleeping, rendering, ROS or network
publication. These are throughput/deadline diagnostics, not a hard real-time
scheduler guarantee or a detailed-arena benchmark.

On Axiom (Apple M5 Max, MuJoCo 3.7.0), a 10-second simulated run processed
2,000,000 rays into 100 frames in 0.982 s wall time: **10.19× real time**.
The physics-only baseline took 0.0262 s. Sensor-enabled steps had a 0.0963 ms
median and 0.1119 ms p95, with no measured step exceeding 1 ms. This includes
Python checks and array aggregation, but not DDS/network publication.

Consecutive emissions now derive their four-lane group mapping directly instead
of using NumPy's general sorting/inverse operation. In three alternating
10-second before/after runs on the same machine/process, median wall time fell
from 0.8230 s to 0.7804 s (5.2% less runtime). Odd starts/counts, cycle boundaries
and decimated queries are checked for identical directions. Timing remains a
simple-room result, not an estimate for mesh-heavy scenes.
