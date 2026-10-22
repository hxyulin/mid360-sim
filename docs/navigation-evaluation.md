<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Navigation integration evaluation

2026-10-23, macOS arm64 / Apple M3 Pro. This evaluates the optional
[navigation demo](guide/navigation-demo.md), not hardware accuracy or a released
navigation stack. The actual external TriLIO ROS node receives native ROS
PointCloud2 and SI IMU. It initializes and builds its own map without receiving
world pose truth. Physical sensor extrinsics are explicit.

## Source and method

TriLIO `87f2c51468b71054e3049d31acca5bfb58719292`, clean source checkout, was built
using its locked native `build-ros` workflow. Executable SHA-256:
`20c29fa17d75b297670ac222a7501df293915b8ab4324dc1efb626eca83e0acc`.
This is the older `trident-lio-ros` point-wise estimator, distinct from the
rewrite snapshot in the historical ETH3D benchmark. The source, external runtime
and CAD are not bundled. Initial linking against this repository's minimal ROS
environment lacked the older rclrs example-interface libraries; building in the
estimator's own ROS environment resolved that prerequisite without changing the
sensor package dependencies.

RM uses `b9d0a2c752dfc8d4f247349a2d01e89664b59484`, the same native server and
CAD field used by the preceding synchronized evaluation. One fresh host is
started per run; existing matches are untouched. Only sensor adapters execute
in this repository. The initial attempt pointed at an unexpanded Git LFS
manifest and was rejected at host startup; valid runs use the extracted package.

Each run spans 12 acquisition seconds: 120 clouds at 10 Hz, stride 3, and
2,400 IMU samples at 200 Hz. The fixture uses a three-second quiet interval then
a smooth onset of articulated motion. RM stays quiet for three sensor seconds
before requesting +0.5 rad/s chassis rotation. Gravity is 9.80665 m/s² in the
fixture and 9.81 m/s² in RM. Consumer acceleration scale is exactly 1.0.

Scoring uses all 2,201 fused `/lio/odometry` poses, spanning 11 seconds after
initialization, including propagation between LiDAR updates. It aligns only the
first output pose, with no global trajectory fit, scale fit or time shift.
Fixture truth spacing is 5 ms; RM truth comes from independent operator snapshots
with maximum observed spacing 39.0625 ms in the two rotation-only runs. Position
is interpolated linearly and quaternion orientation by the shortest arc.
The harness rejects output beyond truth rather than extrapolating: an early
RM attempt exposed a final truth-bracket omission, which was fixed by recording
an additional post-measurement snapshot and rerunning.

## Recorded results

| Input | Position RMSE | Position maximum | Rotation RMSE | Rotation maximum |
|---|---:|---:|---:|---:|
| Analytic articulated fixture | 0.215 mm | 2.561 mm | 0.00347° | 0.03686° |
| RM chassis-mounted, body spinning | 11.272 mm | 12.322 mm | 0.04359° | 0.11899° |
| RM gimbal-mounted, body spinning | 11.111 mm | 11.815 mm | 0.02321° | 0.14917° |
| RM chassis-mounted, driving + turning | 11.121 mm | 12.324 mm | 0.03522° | 0.18446° |

The estimator consumed all 120 clouds / 2,400 IMU samples in every successful
run. The fixture accepted 537,435 point corrections; the RM chassis accepted
178,405 and the gimbal 166,473. Counts reflect different geometry and spatial
selection and are not directly comparable measures of quality.

The chassis achieved peak measured angular rate 0.49623 rad/s. The stabilized
gimbal IMU had zero measured angular rate while its body-relative TF changed;
61 received joint TF samples were checked as changing rather than frozen.
All three successful RM runs had zero packet gaps and no reported interval drops;
maximum packet-end simulation delivery age was 35.94 / 39.06 ms. These exclude
the 100 ms acquisition span and are not end-to-end latency bounds.

The driving case adds a +0.2 m/s body-forward request after initialization and
reached 0.483 m maximum IMU-origin displacement (the harness requires at least
0.1 m). It accepted 177,876 corrections,
reached peak measured gyro 0.51712 rad/s and had maximum packet-end delivery age
31.25 ms; truth spacing remained at most 39.0625 ms.

A final driving repeat after adding an explicit subscriber-discovery guard also
passed: 11.167 mm position RMSE, 0.03941° rotation RMSE, 177,434 accepted
corrections, all 120 clouds / 2,400 IMUs, no interval drops, and 0.483 m maximum
displacement. These small run-to-run differences are retained, not averaged.

One gimbal run overlapping other local validation reported incomplete/lost
intervals and was rejected. An isolated repeat passed without loss. Local
contention is a possible explanation, not a diagnosed cause. The regression
continues to reject such runs rather than relaxing loss checks.

The native synchronized ROS smoke also compares every PointCloud2 byte-decoded
field to CustomMsg, including stride-3 timing/lane retention, and checks dynamic
head edges, fixed IMU mounts, shared stamps and clean teardown. Independent
Rust/Python checks verify transform directions, rotated lever arms, inverses,
quaternion interpolation and that first-pose alignment does not hide later drift.

## Limits

These are short, ideal-input compatibility checks in a small analytic room and
one CAD field. They do not establish calibrated sensor noise, physical precision,
long-duration drift, global localization or Nav2 obstacle-layer behavior.
RM snapshot interpolation contributes scoring error, particularly during fast
motion; the gimbal TF is an ideal state-derived encoder surrogate, not a physical
encoder model. A fixed-extrinsic LIO pair remains on the same carrier in all runs.
No Linux DDS runtime test was performed. Loopback Cyclone configuration and the
outer watchdog bound the tested macOS process lifecycle; they do not resolve the
previously observed native middleware defects in every configuration.

Raw reports, truth/estimate trajectories and logs remain ignored under
`local-assets/lio-integration-20261023/`. Use new output directories when
reproducing; the harness refuses to overwrite prior results.
