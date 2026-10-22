<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Synchronized LiDAR/IMU examples

Use the same acquisition clock for distorted LiDAR points and externally supplied
IMU measurements. The examples here make that boundary runnable without adding
robot dynamics, a physics engine or an rm-simulator dependency to this package.

## Run the self-contained example

Only Rust is required:

```sh
cargo run --release --locked --example synchronized
cargo run --release --locked --example synchronized -- --evaluate --frames 10
cargo test --locked --example synchronized
```

The caller-owned fixture in `examples/synchronized/fixture.rs` defines motion in
an analytic 20 × 20 × 4 m enclosure. It supplies a moving carrier, a rotating head,
a 0.25 m carrier-to-LiDAR lever arm, and a distinct IMU extrinsic: translation
`[0.04, -0.02, 0.015]` m in LiDAR FLU and pitch +0.1 rad. These are **example
choices**, not MID-360 factory extrinsics or calibrated robot dynamics.

The caller differentiates its kinematic trajectory analytically using value,
first-derivative and second-derivative arithmetic. It computes IMU-frame angular
rate from `Rᵀ Ṙ` and specific force from `Rᵀ(a - g)`, including the IMU lever arm.
Five midpoint samples approximate each preceding 5 ms mean. Gravity is explicitly
9.80665 m/s² in this fixture. RM's actual solver uses 9.81 m/s²; the publisher
preserves either caller's SI values rather than changing gravity conventions.

LiDAR uses `Sensor` and per-emission `Scene` queries. The caller aggregates
100 ms packets across 7.8125 ms acquisition chunks. IMU samples end every 5 ms,
with exactly 20 samples in `(start,end]` paired with LiDAR `[start,end)`.
Neither sensor's stamps depend on evaluation runtime or publication wall time.
`--motion stationary|yaw|articulated`, `--stride` and `--chunk-ns` select other
bounded runs. The default is 20 articulated packets at full emission rate. Reported `elapsed_s`
includes fixture ray queries, pose consistency checks, hashing and output callbacks
(including pacing for ROS); it is not a core-only raycasting benchmark.

`--evaluate` checks stationary, constant-yaw and articulated motion. It compares
all selected point bits across 100 ms and 7.8125 ms acquisition chunks, checks
stride-3 retention, and repeats chunking checks with explicit uncalibrated 2 cm
range noise, 0.001 rad angular noise and 1% dropout. Separate tests compare IMU
values with the independent stationary-gravity and constant-spin lever-arm
formulas. This is a kinematic sensor-contract check, not a dynamics simulation,
a hardware measurement or a navigation estimator.

## Publish the fixture through native ROS

Follow [ROS installation](../install/ros2.md), then:

```sh
pixi run synchronized-smoke
pixi run synchronized-smoke --stride 3
```

For an interactive consumer, subscribe before starting the publisher:

```sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
export CYCLONEDDS_URI="file://$PWD/config/cyclonedds-loopback.xml"
pixi run synchronized --frames 100 --stride 3
```

The example waits up to five seconds for **both** subscribers. It publishes
`/synchronized/lidar` as Livox `CustomMsg`, `/synchronized/imu` as
`sensor_msgs/Imu`, and one simulation `/clock`. Use `example_lidar` and
`example_imu` as the physical frame IDs and configure the example extrinsic
explicitly in the consumer. No TF or odometry is published. Use simulation time
and best-effort-compatible QoS; IMU subscriptions should have depth at least 200
because each 100 ms bundle releases 20 IMU messages before its LiDAR packet.
Cross-topic callback order is not guaranteed; consumers must buffer by stamp.

Angular velocity is **rad/s** and acceleration is specific force in **m/s²**.
Orientation is unavailable (`orientation_covariance[0] = -1`). Remaining zero
covariances mean unknown uncertainty. No perfect orientation is injected. Some
Livox consumers expect acceleration in g: configure them for SI or perform one
explicit conversion. See [navigation integration](navigation.md).

`imu::ImuSample` is a dependency-free value/validation contract for caller data;
it synthesizes nothing. With `ros2`, `publisher::ImuPublisher` sends that data
through rclrs. Publication can block: real hosts should use bounded delivery
workers, as RM does, rather than publishing on their physics thread.

The supplied Cyclone profile is **local-only**. The ordinary demo still defaults
to Fast DDS; its smoke test now accepts `--rmw`, `--discovery-range`,
`--domain-id` and `--frames`, and bounds native shutdown with a process watchdog.
The receive callback collects bounded packets and validates points afterwards,
avoiding validation-induced loss on dense 20,000-point packets. Middleware
configuration is a transport choice, not a change to acquisition timestamps.

## Consume an actual RM match

`examples/rm_sensors/live.py` uses Python's standard library to consume the
read-only measured stream from an **already running** RM app/server. It does not
create, reset, move or pause that match. RM must include synchronized attachments
from revision `c3784e168cf21167f15c8728ba8e8edb8d1d80a9` or a compatible later
version. This is an optional external process, not a Cargo dependency.

Enable the RM operator HTTP API, choose an existing chassis ID, then from this
checkout:

```sh
python3 examples/rm_sensors/live.py --host http://127.0.0.1:7780 \
  --robot 0 --frames 20 --stride 3
```

`--config FILE.json` supplies the host's full sensor configuration instead of the
robot/stride flags, including `"carrier": "Gimbal"`, explicit mounts and sensor
effects. `--allow-drops` reports gaps without rejecting them. Default checks fail
on unexpected loss. The consumer checks every retained emission and miss,
absolute stamps, local metric ranges, complete IMU windows and packet ordering.
It detaches on completion; a vanished caller's attachment expires in the host.

For ROS publication from a real match, use RM's own native rclrs bridge:

```sh
# From the external RM checkout's integrations/ros2/:
pixi run run --host 127.0.0.1:7780 --robot 0 --frames 600 \
  --topic /robot0/livox/lidar --imu-topic /robot0/livox/imu --publish-clock
```

Use one `/clock` authority and the same RMW configuration for the bridge and
consumers. This command attaches to the existing host, not the separate
LiDAR-only probe. The [RM repository](https://github.com/trident-rm/rm-simulator)
contains the actual dynamics, substep IMU model and bounded acquisition workers.
The sample library here does not copy those systems or depend on them.

## Evaluate the external integration

The evaluator below starts **its own fresh external CAD host** and changes only
that host. It does not attach to an existing match. Supply your own installed
CAD package; no CAD or runtime is bundled. Build the external host first:

```sh
# In the RM checkout containing synchronized attachments:
nix develop -c cargo build -p rm-simulator-server --locked
# In this checkout:
python3 examples/rm_sensors/evaluate.py \
  --rm-checkout /path/to/rm-simulator \
  --cad-assets /path/to/field-2027 --frames 20 \
  --report local-assets/synchronized-rm-evaluation.json
python3 -m unittest discover -s examples/rm_sensors -v
```

This checks stationary and spinning chassis, full rate and stride 3, the actual
world-stabilized gimbal carrier, an actively moving head, pause/resume, slow-reader
backpressure, topology-change recovery, explicit detach and expiry while paused.
A stopped reader must leave at most two queued output bundles, report drops and
allow the same physical world to keep advancing. A geometry discontinuity must
discard partial pairs and resume on the original clock, not retimestamp old data.
The five-second expiry is host liveness time, not sensor acquisition time.
Reports distinguish each collection's `collected_frames` from cumulative `frames`
when one validator spans pause/resume or a topology event.

See the [recorded evaluation](../synchronized-evaluation.md) for results and their
limits. These checks establish measurement-contract and transport behavior, not
calibrated IMU noise, SLAM accuracy or a complete navigation stack.
