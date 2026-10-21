<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Independent capture transfer checks (2026-10-21)

Two public MID-360 recordings were downloaded and decoded successfully. Neither
is assumed stationary. The original measured hardware Fourier coefficients remain unchanged. A
subsequent accepted nominal simplification is described in [hardware fit](hardware-fit.md). This is an exploratory transfer check, not
a manufacturer calibration or proof of a globally optimal alignment.

## Sources and integrity

- [Livox official indoor sample](https://www.livoxtech.com/mid-360/downloads):
  `Indoor_sampledata.lvx2`, 222,540,611 bytes; SHA-256
  `f892732ff43882b56d1cebc683f6ea9374ab3d3ac688368c9d560f49dcd4d647`.
  Direct URL:
  <https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/Indoor_sampledata.lvx2>.
  Parsing follows Livox's **LVX2 Specifications v1.0 (2023-06), pp. 4-8**,
  linked from its [downloads center](https://www.livoxtech.com/downloads).
  One device, type 9 (MID-360), identity stored extrinsics, 1,534 frames,
  162,293 complete 96-point packets. Every packet begins 480,000 ns after its
  predecessor. LVX2 omits the UDP interval field: within-packet timestamps use
  the explicit nominal 5,000 ns assumption; lanes are inferred by slot modulo 4.
  Point data retain original millimetre coordinates, reflectivity and tags.
  Duration 77.900635 s; 15,580,128 emissions, 10,178,673 nonzero returns.
- [Elevator-LIO Office1](https://huggingface.co/datasets/xiaofan0100/Elevator-LIO-Dataset):
  `Office1.bag`, 365,145,569 bytes; SHA-256
  `570521eb09437403c5c12725694ffa675d7cd17e575a88de1261800249397e96`.
  This hash also matches the Hugging Face LFS object ID. Dataset card declares
  CC BY 4.0. Direct URL:
  <https://huggingface.co/datasets/xiaofan0100/Elevator-LIO-Dataset/resolve/main/Office1.bag>.
  ROS1 embedded definitions decode 4,706 `/livox/lidar` CustomMsg records and
  list 18,824 `/livox/imu` messages. Point analysis retains actual `timebase`,
  `offset_time`, `line`, tag and reflectivity in `livox_frame` for one lidar ID.
  Duration 94.119754195 s; 18,807,648 points, 15,235,312 nonzero returns.
  The ROS adapter validates message schemas/counts and reads every LiDAR
  message; it does not decode or validate every IMU measurement.

Hashes identify the downloaded bytes, not original acquisition integrity.
Neither container lets this analysis verify original UDP CRCs. No recording,
raw measurement buffer, third-party decoder implementation or new fitted
coefficients are redistributed. Downloads and generated reports are ignored
under `local-assets/external-datasets/`.

## Frozen-optics experiment

The model is the owner's [hardware-derived optics](hardware-fit.md), with
source capture SHA-256
`82ef4b05ad71132999e522aae7b6f927db58a5570876b865a407a9d0faaa5b69`.
No Fourier coefficient, mounting transform or per-lane correction is refitted.

Normalize nonzero XYZ in the recorded sensor frame, filter range >=2 m and
raw tag zero, and retain all valid points in the first four seconds. Later,
retain one packet in 20 for LVX2, or one nominal 20 us time group in 20 within
ROS scans. Missing returns never renumber lane labels or timestamps.

Test up to 60 uniformly spaced 0.2 s windows. Time blocks of 0.48 ms with index
modulo 7 <3 calibrate clocks; the other blocks evaluate. Modulo 7 prevents the
packet thinning stride from selecting only one side. Each window fits only two
phases and two local rates, using calibration observations and an eight-start
slow-phase search. Each side requires at least 100 eligible observations;
otherwise skip the window. Deterministic thinning bounds calibration/evaluation
work. A cyclic/reversed lane mapping is selected on the first calibration
window and then frozen; all tested captures selected identity order.

All four lanes use the learned convention of evaluating a group at its first
slot phase: subtract nominal `line * 5 us` from individual return time before
clock fitting. ROS source stamps remain preserved in extraction. Coarse rate
search uses calibration observations only. No target evaluation ray is used
for phase acquisition, convention selection or local optimization.

| Capture | Usable windows | Clock-withheld rays | Median | RMS | p95 |
|---|---:|---:|---:|---:|---:|
| Original capture, diagnostic control | 60 | 24,635 | 0.109° | 0.248° | 0.441° |
| Livox indoor | 60 | 61,473 | 0.916° | 1.079° | 1.751° |
| Office1, recorded offsets | 30 | 25,431 | 0.443° | 0.503° | 0.814° |
| Office1, interval reconstruction hypothesis | 29 | 24,823 | 0.439° | 0.497° | 0.801° |

The source control is a clock-fitting diagnostic, not independent optical
validation: its optics were learned from that capture. Its figures differ from
the earlier 0.128° result because this experiment uses local linear clocks
instead of the original dense interpolated measured clock trace. These samples
are selected eligible windows, not every point in either recording. Both new
recordings have errors across all four lanes, rather than one incorrect lane.

**The four-lane model structure transfers, but the source coefficients plus
local clock fit do not reproduce independent data at the original accuracy.**
The errors include local clock approximation, coordinate/timestamp conventions,
measurement errors and numerical alignment limits; they do not isolate optical
manufacturing differences. No blind future-clock prediction is demonstrated.
Moving data is usable only while XYZ remains in the instantaneous sensor frame.
The optical consistency and raw-style message format are supporting evidence;
this analysis cannot certify the full acquisition pipeline had no transforms.

A 0.1 s window sensitivity run reduced control RMS to 0.159° and Office1 to
0.494°, but Livox indoor became unstable (2.40° RMS), consistent with inadequate
local phase observability/search or sparse support. It is not evidence that
physical sensor accuracy changes with the window. The declared 0.2 s experiment
is the main result; do not cherry-pick the best result per capture.

## ROS timing observation

Office1 contains 4,947 ns steps inside many 96-point groups, with the next group
starting 480,000 ns later. This matches integer division `475000 / 96` rather
than endpoint interpolation `475000 / 95`. See the upstream
[driver interval correction proposal](https://github.com/Livox-SDK/livox_ros_driver2/pull/69/files).
This is a consistency finding, not a determination of the recording's driver
revision. Some groups have different source steps; do not replace all of them
by a constant 5,000 ns.

An explicit optional hypothesis preserves each 96-point group start, recovers
the unique 100 ns total interval consistent with integer division by 96, and
interpolates it over 95 gaps. It rejects incomplete, nonuniform or inconsistent
groups. The original file and the default extraction retain source offsets.
RMS changes only slightly (0.503° to 0.497°); this is not a paired causal estimate:
corrected time also changes retained groups and which evaluation blocks qualify.
The difference does not explain most of the overall transfer error.

## Reproduce

Use the Python environment from `requirements-analysis.txt`; no ROS installation
or private TriLIO decoder is required. Supply a previously generated
`hardware-profile.json` from `fit-recorded-pattern.py`. All output directories
must be new. After downloading the sources above into ignored storage:

```sh
python scripts/analyze-external-capture.py /path/Indoor_sampledata.lvx2 \
  --output local-assets/external-datasets/indoor-analysis
python scripts/analyze-external-capture.py /path/Office1.bag \
  --output local-assets/external-datasets/office-analysis
python scripts/check-optics-transfer.py local-assets/external-datasets/indoor-analysis \
  --model /path/hardware-profile.json --output local-assets/external-datasets/indoor-transfer
python scripts/check-optics-transfer.py local-assets/external-datasets/office-analysis \
  --model /path/hardware-profile.json --output local-assets/external-datasets/office-transfer
python scripts/analyze-external-capture.py /path/Office1.bag \
  --ros-reconstruct-interval --output local-assets/external-datasets/office-reconstructed
python scripts/check-optics-transfer.py local-assets/external-datasets/office-reconstructed \
  --model /path/hardware-profile.json --output local-assets/external-datasets/office-reconstructed-transfer
python scripts/test-external-capture.py
```

Extractors write checksums, counts, timing policies and bounded samples.
Transfer reports record per-window clocks, counts, lane convention, per-lane
errors and aggregate statistics. Five regression tests cover miss preservation,
malformed LVX2, extrinsics rejection and reversible driver timing hypotheses.

The next experiment should compare shared optics with small per-device
calibrations, retaining independent evaluation and measured clock uncertainty.
These two recordings do not establish a universal calibrated model. MineInsight and TIERS remain candidates for later checks;
they have not been downloaded or tested in this pass.
