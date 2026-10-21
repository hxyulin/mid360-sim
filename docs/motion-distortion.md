<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Motion distortion versus real Livox acquisition

Real Livox returns are acquired successively, not from one pose per published
cloud. [Livox's distortion tutorial](https://www.livoxtech.com/showcase/211220)
explains that each point has its own instantaneous coordinate frame and time;
deskew transforms these points to a common reference frame. A correct simulator
must therefore intersect at acquisition time and preserve raw local coordinates.
Adding plausible timestamps to a snapshot cloud cannot reproduce this behavior.

For emission i, use the sensor pose T(t_i) and scene geometry G(t_i). Cast the
local ray d_i transformed by T(t_i), obtain range r_i, and publish r_i*d_i with
its original t_i. Do not transform all raw points to one pose before publishing.
A consumer's deskew should reconstruct stationary walls; withholding deskew
should retain the distortion. Publication frequency is independent of acquisition.

## Comparison with the upstream simulator

The pinned [official Gazebo plugin](https://github.com/Livox-SDK/livox_laser_simulation/blob/1cce1073633a062b92e30243a4c2920e45551bb5/src/livox_points_plugin.cpp#L101-L159)
updates all rays in one `rayShape->Update()` and publishes `sensor_msgs/PointCloud`
with one header timestamp and no per-point time channels. The loaded CSV times
are not used to sample historical sensor poses. This is a snapshot acquisition
limitation, independently of its repeated scan table. This assessment concerns
that revision, not every Livox simulator or fork.

| Backend | Sensor pose during acquisition | Scene geometry during acquisition |
|---|---|---|
| Real hardware | Continuous physical motion | Continuous physical motion |
| Rust core / analytic ROS demo | Caller query per emission; demo evaluates continuous pose | Caller query per emission; demo static room |
| Isaac Warp adapter | Linear position + quaternion SLERP per emission | One rigid scene capture per batch |
| MuJoCo adapter | One current site pose per call | Current mjData per call |
| Gazebo adapter | Current link pose per physics step | Current physics geometry per step |

Isaac's first extension callback has no earlier pose and initializes history.
Subsequent callbacks require current physics poses written to USD, not Fabric-only
transforms. A rotating off-center mount follows a curved translation path;
linear endpoint interpolation only approximates that curve. Sharp turns, contacts,
and geometry motion require shorter batches or time-resolved rigid captures.
In particular, interpolating the sensor while freezing the moving robot body can
produce self-occlusion artifacts. Excluding all robot geometry conceals this
error but also removes real self-occlusion.

MuJoCo's whole 100 ms call is not rolling acquisition. Querying 200 emissions
after each 1 ms physics update, while retaining original times and aggregating
for publication, approximates it. Gazebo already accumulates physics-step
queries into approximately 100 ms publications. Neither currently interpolates
poses or geometry within a physics step.

## Reproducible controlled comparison

```sh
python scripts/check-motion-distortion.py --output local-assets/motion-distortion/report.json
# Optional real MuJoCo check, using the installed Python integration environment:
python scripts/check-motion-distortion.py --mujoco --output local-assets/motion-distortion/report.json
```

The experiment uses 20,000 nominal rays in 100 ms inside a static 20 m cube,
with sensor height 0.8 m, translation 3 m/s and yaw rate 180 deg/s. A second
trajectory adds 10 m/s² translation acceleration and 720 deg/s² yaw acceleration.
No noise, dropout, IMU error, obstacle motion or scan calibration error is added.

The metric is RMS 3D point displacement versus exact per-emission raycasting,
after applying exact ground-truth deskew to both clouds. Since local directions
are identical, it equals RMS range difference. Corner hit changes are included.
It is not range precision or a measured real-sensor residual.

| Acquisition policy | Constant motion RMS | Accelerating motion RMS |
|---|---:|---:|
| Frozen start pose for 100 ms | 924.824 mm | 1,025.250 mm |
| Frozen start pose per 1 ms | 10.384 mm | 12.841 mm |
| Endpoint interpolation per 16.667 ms | < 0.000001 mm | 1.901 mm |
| Endpoint interpolation per 1 ms | < 0.000001 mm | 0.006617 mm |

Constant-motion interpolation is analytically exact for this trajectory, not
for arbitrary real motion. The numerical results compare acquisition policies,
not timed end-to-end runs of every simulator. The optional native MuJoCo check
confirmed its 100 successive 200-ray calls match the predicted 1 ms hold ranges
within 5.4e-15 m in this synthetic room. The sign/phase of step hold can differ
between adapters; endpoint rather than start hold changes individual residuals.

The exact rolling cloud, mistakenly treated as a single frame without deskew,
has 0.762 m RMS nearest-wall residual in the constant-motion case. This differs
from the displacement metric in the table and demonstrates intentional raw
motion distortion rather than simulator error.

## What is and is not hardware-validated

The owner recording is stationary. It establishes emission timing and scan
shape, not behavior under motion. Public moving scans used in the optical-fit
checks also have no independently verified trajectory in those checks. We have
not established hardware-equivalent motion distortion from these data.
The current Isaac native test covers rolling translation and a separate fixed
rotated mount; it does not validate the full moving-robot callback path against
real hardware or time-resolved moving obstacles.

For navigation/LIO, prioritize coherent per-emission sensor poses, scene motion
and sensor/IMU timestamps over reproducing an individual device's exact scan
phase. Next validation should use a controlled translating/rotating rig or a
moving dataset with independent ground truth and known LiDAR/IMU extrinsics:
compare raw wall bending, then residual after identical deskew. Publish the same
trajectory's accelerometer/gyro data with explicit gravity and bias conventions.
IMU extrinsics, time offsets, missing returns, range/material response and
vibration remain separate realism issues. A roughly one-degree pattern-transfer
error is not a one-degree angular-noise setting; directions used to intersect
and publish must remain coherent.

A subsequent [ground-truth investigation](motion-ground-truth.md) downloads and
audits an independent MoCap recording, with explicit timing limitations.

The [official-reference source audit](official-reference.md) checks current
upstream configuration and compares its snapshot policy on the recorded motion.
