<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Results at a glance

See the nominal scan, then the measured motion, runtime and delivery results.
Each figure has its own reference and workload; the numbers are not hardware
precision specifications or interchangeable full-application benchmarks.

## Compact scan, dense coverage

![Generated nominal directions over 100 ms and one second: 20,000 and 200,000 emissions, coloured by nominal lane.](/figures/scan-pattern.webp)

Both windows start at emission zero and show every scheduled direction. The
scan is generated from the committed nominal coefficients; these dots are not
hardware observations. No scene queries or material model enter this figure.
See [scan patterns](patterns.md) for the representative cycle and hardware-fit
limits, or [acquisition](acquisition.md) for per-emission scene queries.

## Keep motion in the acquisition

![Range RMS in the original MoCap replay: MuJoCo held poses 5.666 mm, MuJoCo rolling 0.294 mm, and Isaac Warp bulk history 0.251 mm.](/figures/motion-accuracy.svg)

In the original two-second stress segment, MuJoCo rolling reduces range RMS
about **19.3×** relative to its 1 ms held-pose mode. Warp's bulk-history path
preserves its finer motion policy while batching GPU work. These residuals
measure agreement with interpolated recorded motion in an exact static room,
not real MID-360 range accuracy. The comparison uses 400,000 emissions and
1 ms boundary poses; MuJoCo's 0.5 mm setting bounds origin approximation.

| Original replay policy | Range RMS | Reference / scope |
|---|---:|---|
| MuJoCo, held poses | 5.666 mm | Current geometry held per 1 ms acquisition |
| MuJoCo, rolling poses | 0.294 mm | Per-ray rotation; 0.5 mm origin grouping bound |
| Isaac/Warp, bulk history | 0.251 mm | Per-ray sensor interpolation; static geometry |

[Motion protocol and baseline](../backend-ground-truth.md) ·
[Later accuracy/runtime follow-ups](../performance-tradeoffs.md#follow-up-allocation-cost-and-denser-poses).
The later optimized MuJoCo timings do not replace this original full matrix.

## GPU tracing with measured runtime boundaries

![Original static-room runtime: official Gazebo loop 134.17 ms versus headless Warp 110.70 ms; sensor intervals 122.67 ms versus 13.50 ms.](/figures/runtime.svg)

The original headless Warp loop used **17.5% less wall time** than the native
official Livox/Gazebo 9 loop in this static-room test. That is about **1.21×
throughput** for the whole loop; its roughly 9× lower marginal sensor interval
is a separate measurement boundary. Both loops execute native physics.

This uses Ryzen 9 9950X3D / RTX 5090, 400,000 emissions over two simulated
seconds, caller-owned pose history, static optical geometry and USD writeback
off for Warp. Scan directions and timing boundaries differ. Ordinary USD-driven
integration has different costs, and this does not predict a full articulated
robot's runtime. [Full timing definitions and controls](../performance-tradeoffs.md).

## Choose the representation for the consumer

![Python ROS callback interval p50/p99: PointCloud2 0.410/0.629 ms and Livox CustomMsg 25.385/39.855 ms.](/figures/ros-callbacks.svg)

Each 30-second reference received **305/305 clouds and 6,100/6,100 IMU messages**,
including warmup. The interval starts at the native publisher call and ends in
rclpy's callback. The PointCloud2 buffer path avoids the per-point Python object
construction of CustomMsg; the measurement includes conversion and scheduling.

This is one full-rate stream per format with Cyclone on Apple M5 Max / macOS
arm64. It is not isolated DDS wire latency or a Rust/C++ estimator benchmark.
Other loaded runs exposed missing deliveries; use the [transport report](../transport-evaluation.md)
for the full matrix, failed cases, clock calibration and scope.

## Inspect or regenerate

[Figure data](/figures/measurements.json) records the plotted values, units,
conditions and source revision. [Asset manifest](/figures/manifest.json) records
input/figure hashes and plotting-library versions. SVGs retain selectable text
and can be downloaded directly; the pattern is a lossless WebP to avoid hundreds of
thousands of SVG markers.

From a checkout, with `requirements-analysis.txt` installed in an isolated
Python 3.12 environment:

```sh
python scripts/generate-doc-figures.py
python scripts/generate-doc-figures.py --check
```

The generator checks values against the named report rows, validates unit rays,
and uses fixed dimensions, colours and SVG IDs. It does not rerun benchmarks or
use raw captures, CAD or an image-generation model. Layout and text are original;
model provenance remains in [NOTICE.md](https://github.com/hxyulin/mid360-sim/blob/main/NOTICE.md).
The [validation guide](validation.md) explains the different accuracy references.
