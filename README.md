<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# mid360-sim

A rolling MID-360 LiDAR approximation for robotics simulators.

[Documentation](https://hxyulin.github.io/mid360-sim/) ·
[Installation](docs/guide/getting-started.md) ·
[Validation](docs/guide/validation.md) · [Changelog](CHANGELOG.md)

Generate a compact, nonrepeating scan, query your simulator's scene, and keep
each return's acquisition time. The default model emits 200,000 rays/s across
four interleaved lanes, with a representative 18.2-hour cycle. It is an empirical
approximation, not a calibrated Livox hardware emulator.

| Use it with | Integration | Output |
|---|---|---|
| Your Rust simulator | [Dependency-free core](docs/install/rust.md) | Local points, ranges and timestamps |
| ROS 2 Jazzy | [rclrs + Pixi/RoboStack](docs/install/ros2.md) | Livox Driver 2 CustomMsg and /clock |
| Isaac Sim | [Warp GPU extension](docs/install/isaac-sim.md) | Optical mesh queries and Livox fields |
| MuJoCo | [Python adapter](docs/install/mujoco.md) | Native CPU rays and Livox fields |
| Gazebo Harmonic | [C++ system plugin](docs/install/gazebo.md) | Native PointCloudPacked |

## Quickstart

Use Python 3.10+ in a virtual environment. From this checkout:

```sh
python -m pip install './integrations/python[mujoco]'
python -m unittest discover -s integrations/python/tests -v
```

Add a `lidar` site to your MJCF model, then query your existing scene:

```python
import mujoco
from mid360_sim.mujoco import MujocoSensor

model = mujoco.MjModel.from_xml_path("robot.xml")
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
sensor = MujocoSensor(model, data, "lidar")
frame = sensor.scan(first_index=0, count=200)  # 1 ms at full rate
print(frame.livox_fields())
```

The caller advances physics and the emission index. For ROS, start with
`pixi run smoke`; for other backends, follow the installation links above.
[Release artifacts](docs/distribution.md) include a Python wheel, an Isaac
extension ZIP and source bundles. No simulator runtime or recorded dataset is bundled.

## Motion and realism

Raw points stay in the sensor frame at their own emission time; consumers own
deskewing. Isaac interpolates sensor poses per ray and supports per-step scene
capture, static tracing and explicit moving-geometry partitions. MuJoCo offers
held-pose queries or optional rolling sensor poses; its geometry stays current
per call. Gazebo samples the physics scene per step. Acquisition and publication
intervals are independent; batching output does not require freezing acquisition.

The [validation guide](docs/guide/validation.md) distinguishes hardware-derived
scan fits, controlled tests and motion-capture comparisons. The
[runtime/accuracy report](docs/performance-tradeoffs.md) compares actual native
backends, including the official plugin, with explicit measurement scopes.
Full articulated-robot validation, synchronized IMU output and calibrated
material response remain open work. Guides track the current source; check a
release archive's source revision before relying on newer APIs.

## Development

See [CONTRIBUTING.md](CONTRIBUTING.md) for checks, source boundaries and release
preparation. `cargo test --locked` runs the Rust core without ROS or a simulator.

## License

MIT OR Apache-2.0. [NOTICE.md](NOTICE.md) records Livox provenance, derived model
parameters and optional runtime dependencies.
