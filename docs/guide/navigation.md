<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Use with a navigation stack

The simulator supplies LiDAR measurements. Your scene/physics application
supplies sensor motion, obstacle history and IMU measurements; the navigation
stack supplies deskewing, odometry, localization and recovery. The current
LiDAR scope is implemented. Integrating a particular estimator does not require
adding that estimator or a second dynamics world to this package.

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

Use [ROS 2 Jazzy](../install/ros2.md) for the optional `CustomMsg` publisher.
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

A 2026-10-22 experiment fed native MuJoCo rolling scans of the public
[ETH3D Office mesh](scanned-geometry.md) and caller-generated 200 Hz IMU through
TriLIO's actual ROS publisher/node at 1× speed. It used a small ~3.2 m loop,
a full turn, and peak angular rate ~2 rad/s in the fast case. All input packets
and valid points reached the node, with no input queue drops.

| Input | Duration | Corrected position RMSE | Maximum error |
|---|---:|---:|---:|
| Ideal, slow motion | 24 s | 8.1 mm | 19.6 mm |
| Ideal, faster motion | 12 s | 8.4 mm | 19.6 mm |
| Faster motion, synthetic noise/dropout | 12 s | 6.3 mm | 20.2 mm |

Scoring uses starting-pose alignment only, without a whole-trajectory fit.
Noise/dropout are uncalibrated stress choices; their smaller error in one run
is not a general improvement claim. The live corrected stream ends one 25 ms
window before recording end because the node waits for more input; IMU-rate
output reaches the final sample. Configure `accel_scale=1.0` for these SI inputs;
the node's Livox-in-g default initially prevented initialization.

A separate offline consumer of the recorded ROS odometry deskewed noisy scans
and matched them to a fixed original-survey PCD. It recovered both 0.30 m / 15°
and 0.75 m / 30° starting errors through all 110 frames; 1.50 m / 90° and
4 m / 180° starts failed throughout. This is local ICP recovery, not global
relocalization. Global pose proposals, place recognition and recovery logic
belong to the localization stack.

These are compatibility results from a TriLIO development snapshot based on
`57604a28450bcd62732fdcadde3ce93808b882ca` plus existing uncommitted changes,
not a released integration or performance promise. The estimator and that historical
end-to-end replay harness are not bundled here. The newer runnable regression
above uses a different estimator implementation and is scored separately. Reproducing them requires a
compatible estimator and caller-owned IMU/ROS bridge. The source mesh example
is reproducible separately using the [public workflow](scanned-geometry.md).

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
