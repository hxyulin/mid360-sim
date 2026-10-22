<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Synchronized sensor evaluation

Evaluation on 2026-10-23, Apple Silicon M5 Max / macOS 27.0.1. The
[worked examples](guide/synchronized-sensors.md) contain the runnable fixture,
wire validator and isolated external-host evaluation. Runtime libraries and
source CAD are not included. The RM source was committed as
`c3784e168cf21167f15c8728ba8e8edb8d1d80a9`; its sensor stream used the actual
live world, not a restored or separately advanced sensor world.

## Analytic fixture

One simulated second per motion, nominal 200,000 emissions/s, 200 Hz IMU,
100 ms packet windows and 7.8125 ms acquisition chunks. The default sensor is
ideal for the geometric comparisons. Both full rate and stride 3 preserve all
20 IMU samples per packet. The fixture's explicit gravity is 9.80665 m/s².

| Motion | Full-rate emissions / IMU | Correct per-emission wall RMS | Frozen packet-start pose wall RMS |
|---|---:|---:|---:|
| Stationary | 200,000 / 200 | < 1e-10 m | < 1e-10 m |
| Constant yaw | 200,000 / 200 | < 1e-10 m | 0.182401 m |
| Moving carrier and rotating head | 200,000 / 200 | < 1e-10 m | 0.131369 m |

Stride 3 retained 66,667 emissions and all 200 IMU samples. Its point bits matched
those retained from the full-rate run; 100 ms and 7.8125 ms acquisition chunks
also matched bit-for-bit. Explicit synthetic range/angular noise and dropout
preserved the same chunking invariance. Unit tests separately checked stationary
gravity and constant-yaw IMU against the closed-form extrinsic/lever-arm result.

Wall RMS is distance to the closest enclosure plane after transforming local
points with the supplied pose. The tiny correct-pose error establishes internal
geometric consistency, not physical LiDAR precision. The frozen-pose comparison
shows that raw scans contain motion distortion; it is not estimator accuracy.
The point query and geometric check share the supplied trajectory. The independent
stationary/spin IMU references cover derivative signs, units and lever arms;
full articulated dynamics and physical calibration are not established.

## Actual live RM CAD host

The external evaluator started a fresh headless CAD host containing the normal
field and an operator-spawned Sentry. Its scene contained 187,661 ground,
94,274 fixture and 165,950 equipment collision triangles. Tests used actual
128 Hz physics, recorded solver substeps and a 5 ms IMU period. RM's gravity is
9.81 m/s². Geometry remains the game's collision proxy, not calibrated optical
surface/material geometry.

Each main case collected 20 complete 100 ms packets:

| Carrier / motion | Retained emissions | Valid returns | IMU samples | Peak angular rate | Median specific force | Packet gaps / reported drops |
|---|---:|---:|---:|---:|---:|---:|
| Stationary chassis, stride 1 | 400,000 | 148,728 | 400 | 0.00116 rad/s | 9.81000 m/s² | 0 / 0 |
| Spinning chassis, stride 1 | 400,000 | 148,873 | 400 | 0.49623 rad/s | 9.80993 m/s² | 0 / 0 |
| Spinning chassis, stride 3 | 133,333 | 49,625 | 400 | 0.49623 rad/s | 9.81009 m/s² | 0 / 0 |
| Stabilized actual gimbal, stride 3 | 133,333 | 47,314 | 400 | 0 rad/s | 9.81009 m/s² | 0 / 0 |

The nominal requested body spin was +0.5 rad/s; the IMU reported the achieved
physical motion. The world-stabilized gimbal did not inherit the chassis spin.
Changing the head's target then produced 120 complete IMU samples in six packets,
with a peak angular rate of 12 rad/s from the model's prescribed, motor-limited
head motion. That value is model behavior, not a measured robot actuator limit.
No commanded angle was substituted for an actual carrier pose.

The validator checked every global retained LiDAR index, explicit miss,
acquisition stamp, local point/range pair, complete 20-sample IMU interval and
packet epoch. The first incomplete derivative warm-up bundle was discarded as
documented; all main cases reported one startup input discontinuity. No missing
IMU interval was filled or assigned a newer timestamp.

Maximum measured host-clock age **after a packet's interval end** ranged from
31.25 to 35.94 ms across the four main cases. This metric excludes the packet's
100 ms acquisition span and is not wall-clock end-to-end latency. An early IMU
sample in a bundle is correspondingly older. Maximum HTTP poll wall duration
ranged from 22.80 to 58.50 ms in this run; it includes request handling, acquisition
backlog when present, JSON transport and decoding, not ROS or estimator work.
A repeat with the final validator, during concurrent build/native ROS activity,
still reported zero queue drops in the main cases but reached **95.31 ms** of
packet-end host-clock age and **94.58 ms** for an HTTP poll. The repeat preserved
the same epoch across pause/resume and exposed exactly one sequence gap and one
dropped interval after the topology change. This variability makes a binary or
native delivery adapter a useful later optimization; it does not identify JSON
serialization as the only cause. These short local runs are not latency bounds
or multi-robot throughput claims.

Additional controlled checks passed:

- Pausing and draining completed output held host time fixed and produced no
  newly advancing packets. Resume delivered six complete pairs with unchanged
  5 ms IMU cadence and no reported loss.
- Deliberately stopping polls for 0.85 s left **two** queued bundles and reported
  **six** dropped output intervals. The host continued to advance. The queue did
  not grow and the consumer did not receive renamed old timestamps.
- Spawning another robot changed geometry topology. The affected interval was
  counted as dropped; input discontinuities increased from one to two. Six
  subsequent pairs were complete and retained the original acquisition epoch.
  The validator retained its pre-event state and observed the missing sequence,
  rather than starting a new clock after the event.
- Explicit detach made further polls return 404. An attachment abandoned for
  5.3 host seconds while the match was paused also expired with 404.

The normal validation path rejects unexpected queue loss; deliberate overload
and topology tests allow and inspect the corresponding counters. JSON results
are written only to ignored `local-assets/` by the evaluator. The committed
report records this run rather than bundling runtime logs or CAD.

## Native ROS delivery

The standalone articulated fixture was published through rclrs 0.8.0 / ROS 2
Jazzy with explicit Cyclone DDS 0.10.5 IPv4 loopback configuration. The full-rate
native check delivered six Livox packets / 120,000 valid local points and
120 IMU messages. It checked shared stamps, exact 5 ms IMU cadence, SI acceleration
scale, unavailable orientation, final `/clock` and clean publisher/subscriber
shutdown. The smoke runner bounds the entire process group lifecycle.

Stride 3 also passed six packets / 40,000 valid points and 120 IMU samples.
The ordinary Fast DDS LiDAR-only smoke tests passed at strides 1 and 3 after
deferring point validation until after reception. This does not establish that
the separate automatic-interface problem has been fixed.

This establishes local schema, timing and delivery compatibility. It does not
measure an estimator's drift or demonstrate Linux runtime compatibility. ROS
publication releases IMU in 100 ms bundles and best-effort QoS can still drop
samples if consumers cannot keep up. The DDS profile intentionally excludes
remote hosts; qualify a network deployment separately.

The earlier automatic-interface failures remain relevant: changing the middleware
alone did not establish their native root cause. The working local configuration
pins loopback. Separately, dense-packet inline Python field validation starved
reception; smoke validation is now performed after bounded collection. Both
changes are kept distinct from sensor-model accuracy.
