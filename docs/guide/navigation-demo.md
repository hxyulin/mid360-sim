<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Physical frames, PointCloud2 and a LIO regression

The optional ROS publishers now support physical TF and standard PointCloud2 in
addition to Livox CustomMsg and SI IMU. No estimator or simulator is a dependency
of the Rust core. The runnable LIO harness uses a separately installed TriLIO
node; the RM case starts one fresh external CAD host and changes only that host.

## Preserve the physical frame tree

`frames::Transform` maps child coordinates to parent coordinates:
`p_parent = R_parent_child p_child + translation_m`. Quaternions use **xyzw**;
RM's JSON poses use **wxyz**. Neither convention should be guessed from values.
`FrameTransform` adds physical frame names and a caller-provided nanosecond stamp.
`publisher::TransformPublisher` publishes explicit batches to `/tf` or
`/tf_static`. Publish the complete static set owned by one publisher together,
since its transient-local history retains one batch. Callers own the tree,
parent uniqueness, joint history, authority and interpolation.

For the analytic example, the physical tree is:

```text
example_body --moving head + lever arm--> example_lidar --fixed mount--> example_imu
```

The moving edge carries the head's actual relative rotation at 200 Hz, including
the acquisition's initial endpoint. It is never placed on `/tf_static`.
The fixed edge is `[0.04,-0.02,0.015]` m with pitch +0.1 rad. These are example
mounts, not factory calibration. A LiDAR-to-IMU estimator transform maps LiDAR
points **into IMU coordinates** and is the inverse of that forward TF edge:
pitch -0.1 rad and translation approximately
`[-0.0383026654,0.02,-0.0189183991]` m. Both sensors remain rigidly related even
while their shared head rotates. If they are on different moving links, a
fixed-extrinsic estimator needs additional joint-aware compensation; these demos
do not pretend its fixed mount model applies.

The example publishes physical relative transforms only. The estimator owns its
world/odometry frames. No ground-truth world-to-body transform or orientation is
sent to it. Consumers must buffer by acquisition stamp: publication in 100 ms
bundles does not guarantee callback order across topics. TF interpolation at
200 Hz is an approximation to continuous motion, not exact per-emission truth.

## Publish standard clouds

```sh
pixi run synchronized --frames 100 --pointcloud --tf
pixi run synchronized-smoke
pixi run synchronized-smoke --stride 3
```

Subscribe before starting the example. With `--pointcloud`, startup waits for
PointCloud2 and IMU subscribers; otherwise it waits for CustomMsg and IMU.
Both cloud formats are published when enabled. `/synchronized/points` has
`frame_id=example_lidar`, height 1, little-endian records of 20 bytes:

| Field | Offset, bytes | ROS datatype | Units / meaning |
|---|---:|---|---|
| x / y / z | 0 / 4 / 8 | FLOAT32 | metres, instantaneous LiDAR FLU |
| offset_time | 12 | UINT32 | nanoseconds after header stamp |
| reflectivity | 16 | UINT8 | synthetic constant, not calibrated intensity |
| tag | 17 | UINT8 | Livox first-return policy |
| line | 18 | UINT8 | retained original emission lane |
| padding | 19 | — | zero |

Header time equals the first emission's time, including a leading miss. Misses
are omitted while original time gaps remain. Empty all-miss packets are valid.
`is_dense=true` describes the finite retained points; it does not assert all rays
hit. Raw cloud motion distortion remains intact. A consumer that ignores timing
can display the cloud but cannot deskew it correctly. Field names beyond XYZ
are a documented adapter contract, not a universal PointCloud2 timing convention.

`pointcloud::encode` exposes the same byte layout without ROS dependencies.
`publisher::PointCloudPublisher` uses native rclrs dynamic introspection.
PointCloud and IMU publishers default to best-effort; `new_with_qos` allows caller
selection. `--reliable` selects reliable delivery for those two example endpoints
when required by a consumer. Publishing can block and belongs off a physics owner.

The standard [PointCloud2 schema](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html)
uses its [PointField descriptors](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointField.html)
to describe this explicit layout.

## Run an actual estimator

The harness supports the external TriLIO `trident-lio-ros` node contract:
PointCloud2/IMU inputs, SI scaling, configurable fixed extrinsics,
`/lio/odometry`, and `/trident_lio_rust/flush` with `exit_after_flush`.
The evaluated source is TriLIO commit
`87f2c51468b71054e3049d31acca5bfb58719292`, a distinct older implementation
from the rewrite snapshot in the historical ETH3D experiment. Install/build it
in its own checkout; no estimator source, binary or configuration is bundled.
From that checkout, its native workflow is:

```sh
pixi run -e ros2 build-ros
```

Back here, build the fixture and use the external binary:

```sh
pixi run synchronized-build
pixi run bash scripts/with-ros.sh python examples/rm_sensors/lio.py \
  --estimator /path/to/trilio/target/release/trident-lio-ros \
  --output local-assets/lio-fixture
```

The ideal fixture stays still for three seconds, then smoothly enters articulated
motion, including translation, body rotation, head rotation and IMU lever-arm
acceleration. The harness uses full native ROS paths at wall-paced 1× speed,
120 packets by default, stride 3, and all 2,400 IMU samples. It verifies estimator
input counts and nonzero corrections, finite monotonic output, run coverage,
clean flush and bounded process-group shutdown. Separate truth files are written
only for post-hoc scoring; no truth file is passed to the estimator.

Error uses one SE3 transform from the first output pose, without a trajectory fit,
scale adjustment or time shift. The scored `/lio/odometry` is the node's fused
state output at up to 200 Hz, including propagation between corrections; it is
not a stream consisting solely of LiDAR correction instants. Default smoke
limits are position RMSE 0.15 m and rotation RMSE 3 degrees; override explicitly
with `--max-position-rmse` and `--max-rotation-rmse`. These are regression choices,
not hardware specifications. Failed runs exit nonzero and keep diagnostic logs.

## Evaluate real RM dynamics

Build a compatible RM host as described in the
[synchronized guide](synchronized-sensors.md), and supply an extracted CAD package
rather than Git LFS pointer files:

```sh
pixi run bash scripts/with-ros.sh python examples/rm_sensors/lio.py \
  --source rm --rm-checkout /path/to/rm-simulator \
  --cad-assets /path/to/extracted/field-2027 \
  --estimator /path/to/trilio/target/release/trident-lio-ros \
  --output local-assets/lio-rm-chassis
# Repeat with a new output directory and --carrier Gimbal.
# Add --forward-m-s 0.2 for a translating-and-turning chassis case.
```

This launches a fresh host, spawns/stops a Sentry, allows a quiet initialization
interval, then requests +0.5 rad/s chassis yaw. The host supplies every measured
LiDAR/IMU bundle. `examples/rm_sensors/ros.py` is a reusable optional Python ROS
adapter; the normal no-ROS consumer remains standard-library-only. There is no
second dynamics world. Both RM mounts use the host's explicit default
`[0,0,0.4]` m in the selected carrier, with identity relative rotation.

For a gimbal, the adapter derives **only the relative body-to-gimbal edge** from
actual body and turret poses in the same operator snapshot. It uses that
snapshot's simulation timestamp, never requested aim or receipt wall time.
This is an ideal joint encoder surrogate, not a measured/calibrated encoder.
State polling does not reproduce RM's internal substep joint history; the report
records snapshot spacing. The LIO sensor pair is on the same carrier and the
estimator uses its fixed relative extrinsic, not this dynamic TF, for fusion.

Privileged snapshots also produce a separate IMU-origin truth trajectory for
scoring, with linear position and shortest-arc quaternion interpolation and no
extrapolation. Snapshot polling error is part of this score; these are not
substep-exact motion-capture measurements. Reports retain measurement gaps,
counts, delivery age, estimator binary hash and external RM revision. Only this
fresh test host is controlled; existing matches are never modified.

See [recorded results](../navigation-evaluation.md) for tested platforms, source
identities, timing and accuracy limits. These short integration checks do not
establish Nav2 behavior, global localization, calibrated IMU effects, or Linux
DDS runtime behavior.

The [transport benchmark](transport-benchmarks.md) separates native publisher
and HTTP/JSON costs without changing acquisition timestamps.
