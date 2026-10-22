<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Native Isaac RTX comparison

The installed Isaac Sim 6.2.0 `SUPPORTED_LIDAR_CONFIGS` catalog contains no Livox
or MID-360 entry. This comparison uses NVIDIA's native **RTX LiDAR**, not RTX
Radar, with a custom repeating rotary profile. It is not a manufacturer preset.

The profile matches the nominal 200,000 emissions/s, 10 Hz scans, 360° horizontal
coverage, −7° to +52° vertical coverage and 0.1–70 m range. It has 32 uniformly
spaced, fixed elevation channels, 6,250 firing groups/s and 5 µs emitter offsets.
This preserves acquisition rate and timestamps but **does not reproduce the
MID-360 pattern or its coverage distribution**. Our ordinary Warp adapter uses
its unchanged four-lane, evolving nominal optics. A rotary scan repeats every
100 ms; our representative nominal model has an 18.2-hour common cycle.

## Which experiment to use

This page preserves the earlier **constant-velocity room experiment**. Its small
Motion BVH residual is not representative of changing-velocity MoCap replay.
The later [measured-motion experiment](backend-ground-truth.md#rtx-with-motion-bvh)
checks 10/100/1000 Hz scene updates: RMS is 152.24 / 9.400 / 2.788 mm respectively.
Use the [current runtime/accuracy report](performance-tradeoffs.md) for those
whole-loop runtimes and the optimized Warp comparison.

The replay harness now defaults to a render writer so it captures all 20 scans;
its earlier polling mode missed two at 1 kHz. The constant-velocity polling
results below remain evidence of that harness's scheduling, not a latency or
precision guarantee for every RTX integration.

## Motion matters more than the backend name

Current NVIDIA documentation describes [Motion BVH](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_rtx.html)
and [custom RTX profiles](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_rtx_custom.html).
The experiment sets `outputMotionCompensationState=NONCOMPENSATED`; raw output
should retain rolling acquisition, leaving deskew to the consumer. Motion BVH
is enabled explicitly through `SimulationApp({"enable_motion_bvh": True})`.
A timestamp alone does not establish that an intersection used that pose.

The controlled test uses six enclosing boxes with inner planes at ±10 m,
a sensor 0.8 m above the origin, translation at 1 m/s and yaw at 0.5 rad/s.
Both backends use the same scene and 1 ms physics steps, render at 100 ms
intervals, and copy owned host results. Geometry is static; no robot dynamics,
ROS publication, material calibration or arena load is measured. Noise is
zero, including RTX angular/range error settings. RTX retains invalid slots;
**20,000 scheduled emissions is not a guarantee of 20,000 valid returns**.

For each retained return, reconstruct its local unit vector and compare its
range with an analytic box intersection at its timestamp and known continuous
sensor pose. These are numerical simulator checks, not physical sensor accuracy.
Two simulated seconds after a one-second warmup produced:

| Acquisition | Continuous-pose range RMS | Frozen scan-start pose RMS | Valid returns |
|---|---:|---:|---:|
| RTX, stationary, Motion BVH on | 0.0013 mm | 0.0013 mm | 400,000 / 400,000 |
| RTX, moving, Motion BVH off | 165.4 mm | 6.27 mm | At least 19,987 per 20,000-slot scan |
| RTX, moving, Motion BVH on | 0.643 mm | 170.5 mm | At least 19,986 per 20,000-slot scan |
| Warp, moving, segmented static tracing | 0.0017 mm | 168.1 mm | 400,000 / 400,000 |

The frozen-pose residual helps distinguish a point cloud that actually rolls
from one whose timestamp labels merely advance. The RTX Motion BVH result is
close enough for many navigation experiments. Its small residual and a few
missing returns are reported, not replaced with synthetic hits. The first
retained RTX frame's motion history begins at 902 ms although emission stamps
begin at 900 ms; this boundary discrepancy is visible in the saved GMO poses.
The harness does not attribute every residual to that discrepancy.

In short room runs on the RTX 5090 system, RTX with Motion BVH achieved roughly
6.7–7.4× real time; our static segmented Warp mode achieved roughly 6.0–6.1×.
These are whole measured loops including physics, rendering and host copies,
but exclude initialization, warmup, offline analysis and report writing.
They use different direction distributions, different retained time windows
and separate processes; this is throughput context, not a controlled claim
about one ray-query kernel being faster. Rendering of a full arena, moving
geometry and articulations can change the comparison substantially.

The direct `LidarSensor.get_data` loop also exposed a timing difference:
the completed RTX scan was about 102 ms old in simulation time at readback;
the Warp batch was available at its completion boundary (sub-microsecond
floating-point clock drift). This is **observed harness/annotator scheduling**,
not a proven lower bound for RTX. A navigation adapter must retain the actual
acquisition stamps and account for this age; a different render/annotator
schedule may reduce it. The reported throughput does not subtract that age.

## Reproduce

Run with Isaac's Python; no asset server or recorded dataset is needed:

```sh
./python.sh /path/to/mid360-sim/integrations/isaac-sim/tests/rtx_reference.py \
  --moving --motion-bvh --output /tmp/rtx-moving.json
./python.sh /path/to/mid360-sim/integrations/isaac-sim/tests/rtx_reference.py \
  --moving --output /tmp/rtx-without-motion-bvh.json
./python.sh /path/to/mid360-sim/integrations/isaac-sim/tests/rtx_reference.py \
  --backend warp --moving --output /tmp/warp-moving.json
```

Reports include source timestamps, every scheduled/valid count, timing, actual
GMO endpoint poses and completed-scan readback lag in simulation time. NPZ
companions retain raw spherical coordinates, emission offsets and emitter IDs.
The Warp epoch is taken from the caller's actual post-initialization simulation
time; Isaac initialization has already advanced it by 2 ms in this installation.
Assuming a zero epoch would introduce a false millimetre-scale motion error.

An experimental `--profile nominal` option instead authors the first 100 ms
of our nominal pattern as 20,000 virtual solid-state emitters, repeated at 10 Hz.
Its Motion BVH initialization was too slow in a bounded exploratory run and was
stopped; it has **no validated performance or correctness result** here. It is
not the profile behind the table. The conventional rotary profile remains the
default. The comparison does not switch the public Warp extension's backend.

Local evidence: ignored `local-assets/realtime/rtx-*.json` and `.npz` files.

The later [measured-motion replay](backend-ground-truth.md#rtx-with-motion-bvh)
tests changing velocity/rotation at 10, 100 and 1000 Hz scene updates. Its errors
are substantially larger than this constant-velocity example, and dense RTX
updates cost much more than the earlier 10 Hz rendering throughput figures.
