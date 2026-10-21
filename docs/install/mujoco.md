<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# MuJoCo adapter and shared NumPy scan

Install into a Python 3.10+ virtual environment. From the source root:

```sh
python -m pip install './integrations/python[mujoco]'
python -m unittest discover -s integrations/python/tests -v
python integrations/python/benchmark.py
```

Alternatively, install the wheel from a checksummed GitHub release:

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

Geometry and mount pose are **frozen during a scan**. Original per-emission
timestamps are retained, but this version does not interpolate sensor or
obstacle motion inside that scan. Prefer physics-step-sized batches (e.g. 200
rays per millisecond). This differs from Isaac's optional per-ray sensor-pose
interpolation. Do not claim equivalent rolling motion fidelity for a 100 ms
MuJoCo batch. All returned arrays own their data and survive later queries.

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

Five native regressions cover optical fixtures, analytic distances, moving
mounted poses, chunk/stride consistency, timing, group filtering, near
occlusion, validation and dynamics-time/qpos preservation. The direction-only
cross-language comparisons tolerate floating-point differences, not hardware
angular error. Sampling is deterministic for the tested NumPy implementation.

Axiom's benchmark measured **4.21 ms median / 4.77 ms p95 per 20,000 rays**
(30 measurements after 5 warmups) for NumPy generation, native queries and owned
output arrays in a static six-wall room. It excludes physics stepping,
rendering and transport, and is not an arena-scale result. It is not a
controlled CPU/GPU comparison with the different Talos Isaac benchmark.

See [MuJoCo's native ray API](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-multiray)
for engine semantics. MuJoCo/NumPy remain separately installed dependencies;
no engine implementation code or recorded points are vendored.
