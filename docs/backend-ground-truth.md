<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Backend replay against measured motion

This runs our actual MuJoCo, Gazebo and Isaac/Warp acquisition code using the
same independently measured motion. The reference combines **real MoCap poses
with exact synthetic scene geometry**. It does not compare generated returns
against a surveyed reconstruction of the real recording environment.

## Current versus baseline results

The protocol and original held-pose/ordinary-Warp results below are retained
for reproduction. Subsequent implementations add MuJoCo `scan_rolling()` and
Isaac bulk history, without changing the reference:

| Current addition | Range RMS | Absolute p95 | Maximum |
|---|---:|---:|---:|
| MuJoCo rolling sensor pose, 0.5 mm origin bound | 0.294 mm | 0.381 mm | 8.697 mm |
| Isaac bulk static pose history | 0.251 mm | 0.106 mm | 8.692 mm |

MuJoCo interpolates rotation per emission and groups translation origins; its
tolerance is an origin-displacement bound, not a range-error guarantee.
Geometry stays current for the call. Isaac bulk tracing changes scheduling and
input packing, retaining the same boundary-pose interpolation and static scene.
The [current tradeoff report](performance-tradeoffs.md) adds runtimes, actual
native official-plugin execution, RTX rates and all cross-backend scope limits.

![Original MoCap range residuals for held MuJoCo, rolling MuJoCo and Warp bulk history.](/figures/motion-accuracy.svg)

[Figure data and context](guide/results.md).

## Reference and protocol

The [JEPLO EIL_Mask2 audit](motion-ground-truth.md) records source provenance,
clock limitations and the independent 100 Hz Qualisys trajectory. Its TUM file
SHA-256 is `3a7f5ae4db916c1003ceefbc8b2a91e568c56936708f6662095753991211155a`.
We choose a two-second segment starting 1.75 s after the first trajectory pose,
maximizing mean squared 10 ms rotation increments among gap-covered candidates.
No backend error enters selection. No GT gap exceeds 20 ms; the largest gap in
the chosen segment is 12.10 ms. This is a high-motion stress sample, not an
average or whole-recording error estimate.

Translate the trajectory so its first position is `(0, 0, 0.8)` m inside six
box walls whose inner faces are exactly ±10 m. Preserve all orientations and
relative positions. There is no ICP, clock fitting or optical refit. Generate
the same 400,000 nominal emissions at 5 µs intervals and the same 2,001 boundary
poses at 1 ms intervals. Independent double-precision slab intersections at
per-emission MoCap poses provide the reference ranges. Reference poses use
linear translation and quaternion SLERP between measured MoCap samples.

- **MuJoCo:** prescribe the existing world's mocap-mounted site at each step
  endpoint, run `mj_forward`, then call the actual `MujocoSensor.scan` for that
  step's 200 emissions. Geometry and sensor pose are held within each call.
- **Gazebo:** a test-only, high-priority replay system commands the native
  model pose before physics. The ordinary plugin submits DART/Bullet rays and
  publishes native messages. A subscriber saves all local XYZ, offsets and
  lane fields; evaluation checks complete emission coverage and identities.
- **Isaac/Warp:** step the existing simulation, supply the same boundary poses
  to the ordinary backend and segmented-static backend, and interpolate each
  emission on the GPU. Retain 100 ms output batching in both modes.

These exercise native intersections and acquisition paths, not the previously
reported offline pose-policy proxy. Prescribed input bypasses controllers and
robot dynamics, deliberately isolating sensor acquisition. They do not exercise
ROS transport, rendered camera imagery, material response, moving obstacles or
robot self-occlusion. All points hit the enclosure and noise is disabled.

## Original baseline results

Every path returned all 400,000 emissions. Distances below are range differences
relative to the same interpolated MoCap trajectory, in millimetres:

| Our adapter | RMS | Absolute p95 | Maximum |
|---|---:|---:|---:|
| MuJoCo, 1 ms held pose | 5.666 | 12.169 | 77.275 |
| Gazebo, 1 ms held pose | 5.666 | 12.169 | 77.276 |
| Isaac/Warp, ordinary per-step queries | 0.251 | 0.106 | 8.692 |
| Isaac/Warp, segmented static queries | 0.251 | 0.106 | 8.692 |

An additional diagnostic separates intersection error from acquisition policy:
MuJoCo agrees with the independent step-end held-pose prediction to floating-
point precision; Gazebo agrees within 0.00214 mm RMS. Isaac agrees with the
independent boundary-pose interpolation prediction within 0.00253 mm RMS.
Thus the larger MuJoCo/Gazebo discrepancy primarily comes from the documented
1 ms pose hold, rather than different box geometry or an extra callback delay.
Isaac's ordinary and optimized outputs agree to a maximum range difference
of less than 0.00001 m in this replay.

Isaac's residual against the per-emission trajectory concentrates around
original MoCap interpolation knots; p95 is consequently below RMS. A 100 Hz
sampled trajectory is not continuous physical truth at 5 µs resolution. These
submillimetre values describe agreement with the reference, not a hardware
accuracy bound. Real MID-360 range noise, synchronization and calibration would
add other errors. The [real-return floor residual](motion-comparison-results.md)
is a separate, substantially noisier metric and should not be compared directly.

The [native RTX comparison](rtx-reference.md) is a different experiment using a
custom repeating rotary pattern. The Isaac rows here are **our Warp adapter**.

## Reproduce

Fixture preparation needs `requirements-analysis.txt`; the native MuJoCo runner
needs its separate adapter environment. Recordings and generated fixtures stay
ignored and must not be redistributed in a release.

```sh
python scripts/check-backend-ground-truth.py prepare \
  --gt /path/to/jeplo-mask2-gt.txt --output /tmp/mocap-fixture.npz
python scripts/check-backend-ground-truth.py mujoco \
  --input /tmp/mocap-fixture.npz --output /tmp/mocap-mujoco.npy
python scripts/check-backend-ground-truth.py evaluate --backend mujoco \
  --input /tmp/mocap-fixture.npz --result /tmp/mocap-mujoco.npy \
  --output /tmp/mocap-mujoco.json
```

For Isaac, run `integrations/isaac-sim/tests/ground_truth.py` with its `python.sh`,
`--input /tmp/mocap-fixture.npz --output /tmp/mocap-isaac.npy`. It writes
`mocap-isaac-ordinary.npy` and `mocap-isaac-segmented.npy`; evaluate each with
`--backend isaac` and the same fixture.

For Gazebo, copy the generated sibling `mocap-fixture.csv` into the development
container's `/workspace/out/`. Run `tests/run.sh` to build and check the adapter,
then `bash /workspace/source/tests/ground_truth.sh`. Evaluate the resulting
`/workspace/out/mocap-gazebo.bin` with `--backend gazebo`. The ordinary enclosure
fixture has ±9.9 m inner faces; the replay script explicitly changes them to the
shared ±10 m reference. Reusing that unmodified test room would measure a scene
mismatch rather than sensor error.

Local evidence: ignored `local-assets/realtime/mocap-fixture.*`,
`mocap-mujoco.json`, `mocap-gazebo.json` and `mocap-isaac-*.json`.

## Follow-up: acquisition timing and native RTX Motion BVH

The larger MuJoCo/Gazebo values above are explained by their 1 ms pose hold.
Holding only orientation at the endpoint, while retaining exact per-emission
translation, gives 5.735 mm RMS. Holding only translation gives 0.401 mm.
These are separate diagnostics and do not add linearly. Derived MoCap angular
speed has median 70.0, p95 179.5 and maximum 262.3 degrees/s in this sample;
these finite differences may include tracking noise. Median translation speed
is 0.884 m/s. At 12 m and beyond, the held-pose RMS is 7.25 mm, versus 2.54 mm
for nearer hits. At incidence cosine below 0.7 it reaches 9.26 mm. Long ranges
and oblique planar intersections amplify small angular timing errors.

Only 1.66% of nominal rays exceed 20 mm held-pose error in this stress sample;
0.032% exceed 50 mm. The millimetre numbers are additional acquisition error
with noise disabled, not a statement that the whole simulated sensor is wrong
by that amount in every situation.

### Smaller batches, actual native adapters

Re-preparing the same motion at 200 us and running both native adapters gives:

| Adapter / acquisition step | RMS | Absolute p95 | Maximum |
|---|---:|---:|---:|
| MuJoCo, 1 ms | 5.666 mm | 12.169 mm | 77.275 mm |
| Gazebo, 1 ms | 5.666 mm | 12.169 mm | 77.276 mm |
| MuJoCo, 200 us | 1.166 mm | 2.489 mm | 17.177 mm |
| Gazebo, 200 us | 1.166 mm | 2.488 mm | 17.177 mm |

All runs return the same 400,000 nominal emissions. MuJoCo prescribes the site
and queries its existing scene at 5 kHz; this does not benchmark 5 kHz robot
dynamics. Gazebo runs the entire test world's physics at 5 kHz. Their small-step
improvement is measured, not merely a prediction from the reference script.
No public adapter default or production dynamics rate is changed.

Offline policy diagnostics predict 2.833 mm RMS for one pose at each 1 ms batch
midpoint, or 0.574 mm for a 200 us midpoint. These midpoint values have not been
implemented as native adapter modes. A midpoint is easy for delayed/replayed
acquisition but may require future poses or an explicit prediction policy in a
live controller. Sub-batching sensor queries or interpolating emitted sensor
poses independently of physics integration are the more general next changes.
They must retain consistent geometry and self-occlusion histories. The later
MuJoCo rolling sensor mode implements the sensor-pose option; it does not
supply moving-obstacle history. See the current-results section above.

### RTX with Motion BVH

Run the same measured trajectory in the same box through native NVIDIA RTX
LiDAR, using its custom repeating 32-channel rotary profile. This differs from
our nominal ray distribution, so there is no claim of point-for-point identity
with the MuJoCo/Gazebo/Warp rows. Each native return is compared against an
independent intersection using its reported direction and acquisition time.

| Native RTX configuration | RMS | Absolute p95 | Scheduled scans |
|---|---:|---:|---:|
| Motion BVH on, 10 Hz scene/sensor updates | 152.24 mm | 352.66 mm | 20 / 20 |
| Motion BVH on, 100 Hz updates | 9.40 mm | 15.44 mm | 20 / 20 |
| Motion BVH on, 1000 Hz updates | 2.79 mm | 6.22 mm | 20 / 20 |
| Motion BVH off, 100 Hz updates | 37.02 mm | 80.69 mm | 20 / 20 |

Each scan schedules 20,000 rays with preserved 5 us offsets; invalid returns
remain explicitly invalid rather than being filled in. The 1 kHz complete run
has 399,979 valid returns of 400,000, and maximum valid range error 26.97 mm.
A global +/-0.5 or +/-1 ms timestamp sensitivity check does not eliminate that
residual. We have not isolated its remaining cause inside the RTX acquisition/
scene-update path and do not identify it as a specific NVIDIA kernel defect.
The input itself uses the prepared 1 ms trajectory interpolation.

The constant-velocity room experiment was much easier: linear/SLERP endpoint
motion approximates it well even at low update rates. The measured trajectory
has changing velocity and rotation; Motion BVH cannot recover motion that the
scene-update schedule never supplies. Enabling it alone is insufficient.

At 1 kHz, polling `LidarSensor.get_data` captured 18 of the expected 20 scans in
this harness. A `GenericModelOutput` render writer captured all 20 in two runs;
this is now the replay harness default. Polling remains an explicit diagnostic
option and incomplete output is rejected unless `--allow-incomplete` is supplied.
That flag reports missing scans and does not imply a passing coverage test.
The earlier measured 1 kHz writer run spent about 14 s in rendering/readback for 3.4 s
of simulation, excluding pose setup and physics. Thus the earlier roughly 7x
real-time result at 10 Hz rendering does not apply to high-rate motion updates.
The [current runtime table](performance-tradeoffs.md) adds whole-loop measurements
with explicit synchronous PhysX configuration; do not mix those timers with
this earlier rendering/readback-only timing.

### Reproduce the follow-up

```sh
python scripts/check-backend-ground-truth.py prepare \
  --gt /path/to/jeplo-mask2-gt.txt --step-us 200 --output /tmp/mocap-200us.npz
python scripts/check-backend-ground-truth.py mujoco \
  --input /tmp/mocap-200us.npz --output /tmp/mujoco-200us.npy
```

Pass the resulting CSV path as the second argument to Gazebo's test script:
`bash /workspace/source/tests/ground_truth.sh 200 /workspace/out/mocap-200us.csv`.
It changes the test world's physics step, preserving exact decimal SDF duration
and the original two-second acquisition interval. Evaluate using the matching
200 us fixture, not the original 1 ms fixture.

For native RTX, use Isaac's Python to run
`integrations/isaac-sim/tests/rtx_ground_truth.py --input /tmp/mocap-fixture.npz
--render-steps 1 --output /tmp/mocap-rtx.json`. Allowed render intervals are 1,
10 and 100 physics steps; `--no-motion-bvh` supplies the control. The shared
fixture here remains the original 1 ms one. The output JSON and sibling NPZ
preserve complete scans and raw offsets for offline evaluation.

`scripts/investigate-backend-timing.py --gt /path/to/jeplo-mask2-gt.txt
--fixture /tmp/mocap-fixture.npz --rtx /tmp/mocap-rtx.json --output /tmp/timing.json`
reports range/incidence subsets, rotation/translation attribution, alternate
hold policies and diagnostic clock sensitivity without changing timestamps.
Evidence is under ignored `local-assets/realtime/mocap-timing-final.json`,
`mocap-mujoco-200us.json`, `mocap-gazebo-200us.json` and `mocap-rtx-*.json`.
