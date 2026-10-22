<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Use with a navigation stack

The simulator supplies LiDAR measurements. Your scene/physics application
supplies sensor motion, obstacle history and IMU measurements; the navigation
stack supplies deskewing, odometry, localization and recovery. See
[capabilities and status](status.md) for supported interfaces and validation gaps.

## Connect the measurements

1. **Choose optical geometry.** Include surfaces the laser should see, including
   robot self-occlusion where appropriate. Simplified collision proxies can
   remove visible detail; missing source coverage cannot be fixed by raycasting.
2. **Acquire against coherent poses and geometry.** Preserve global emission
   indices and their timestamps across batches and misses. Select a backend
   policy from [acquisition and motion](acquisition.md); one 100 ms held-pose
   query is not a motion-distorted scan just because it has point offsets.
3. **Publish raw local returns.** Each XYZ is in the instantaneous LiDAR frame
   at its own emission time. Do not transform the whole cloud into a reference
   frame or pre-deskew it when testing the estimator's motion compensation.
4. **Supply IMU and extrinsics.** Use the same simulation clock and a documented
   LiDAR-to-IMU transform. The LiDAR adapters and standalone ROS demo do not
   synthesize IMU samples. A caller can derive angular rate and specific force
   from its simulated rigid-body motion, with separately chosen sensor effects.
5. **Tune publication independently.** Aggregate acquisitions without changing
   their timestamps. Longer output intervals add consumer latency even when
   acquisition is accurate. Check the estimator's accepted frame rate and QoS.

The [synchronized sensor examples](synchronized-sensors.md) demonstrate a
caller-supplied kinematic IMU and consumption of an actual RM host stream,
including rotating mounts and bounded transport. The core does not synthesize IMU.

Use [ROS 2 Jazzy](../install/ros2.md) for optional native publishers; the
[navigation demo](navigation-demo.md) demonstrates timed PointCloud2, SI IMU and TF.
The MuJoCo and Isaac adapters return Livox-shaped fields for a caller-owned
bridge; they do not automatically publish ROS messages or TF. Gazebo publishes
its own native schema. Only one component should publish `/clock`.

## Check units before tuning the estimator

For a colocated IMU with body-to-world rotation `R`, world acceleration `a`
and world gravity vector `g`, ideal accelerometer specific force is
`Rᵀ (a - g)`. A stationary, upright sensor reports approximately +9.80665 m/s²
on its up axis, not zero. Gyroscope measurements are body-frame angular rate
in rad/s. An offset IMU also needs its own rotational/lever-arm acceleration.
A perfect pose is ground truth for scoring, not an IMU measurement.

Make acceleration scaling explicit at the bridge and consumer. Some Livox
consumers expect acceleration in g and multiply by 9.80665; a bridge publishing
SI m/s² must use a matching consumer setting. Do the conversion exactly once.
Check the quiet stationary interval before changing filter noise parameters.

Use one epoch for LiDAR and IMU, preserve per-return integer offsets, and
configure frame transforms explicitly. Neither timestamps alone nor a more
accurate scan-pattern fit compensates for wrong units, extrinsics or clock
alignment. Motion skew should remain in the raw scan until the consumer deskews it.

## Runnable ROS regression

The [navigation demo](navigation-demo.md) adds caller-owned dynamic/static TF,
PointCloud2 with integer per-point timing, and an external TriLIO regression
using the analytic fixture or an isolated actual RM CAD host.
See its [recorded evaluation](../navigation-evaluation.md) for current results.

## Historical downstream compatibility checks

Earlier ETH3D/TriLIO and local ICP experiments use a different estimator snapshot
and scoring setup. Their [results, source identity and limits](../navigation-evaluation.md#historical-eth3d-compatibility-checks-2026-10-22)
are retained with the navigation evaluation; they are not the runnable regression
above or a global-localization claim.

## Keep the evidence separate

The LIO checks establish downstream consumption of moving simulated data.
They do not improve the simulator's scan law, validate real MID-360 pose
precision, or measure the combined raycasting/ROS/navigation loop. Localization
algorithm improvements should be evaluated in the estimator project.

Motion distortion is checked independently by the
[native MoCap replay](../backend-ground-truth.md): measured poses drive native
adapters against exact synthetic geometry, with an explicit interpolated
reference. Stationary reconstruction, hardware angular fits and backend runtime
have different references and metrics; see [validation and limitations](validation.md).
