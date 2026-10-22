<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Architecture and shared optical work

For typical one- or two-sensor workloads, see the follow-up
[native query and ROS construction investigation](single-sensor-performance.md).
It measures the next targets separately from the multi-device optical experiment.

The first useful architectural change is to separate reusable optical directions
from device-specific acquisition. `sensor::PreparedRays` owns an immutable,
bounded emission window. Multiple sensors can borrow it, apply their own effects
and query their own physical mount/scene history. Ordinary scalar acquisition
continues to generate directions on demand; no cache or executor is installed.

This is an additive Rust API. It does not batch native simulator queries, change
a wire format, deskew returns or introduce a second physics world. See the
[shared-window guide](guide/acquisition.md#share-aligned-optical-windows-in-rust)
and API doctests for usage.

## What can be shared

A direction is determined by scan profile and global emission index. Share only
when profile, first index, count and stride match. Each device still owns its
epoch and effects seed. Angular perturbation happens before that device's nearest
hit query; range noise/dropout and misses are evaluated separately. Rotations,
translations and moving objects must still be evaluated at each emission time.
The prepared window stores no world rays, poses, surface hits or measurements.

Direction payload occupies 24 bytes per ray, up to 6 MiB, plus container overhead.
There is no unbounded global cache or retained scan history. The caller chooses
window lifetimes and scheduling. Borrow one window across devices; cloning
copies its vector. For a single sensor, preparation usually adds memory and may
add cost, so the existing cursor remains the ordinary path.

## Local scaling experiment

```sh
cargo run --release --locked --example prepared_bench
```

Apple M5 Max / macOS arm64, rustc 1.99.0, default release flags. Every device
acquires four 20,000-ray windows, or 80,000 retained rays. The shared path prepares
and allocates each window inside the timed interval, then drops it after all
devices acquire it. Outputs are warmed reusable frames in both paths. Each device
has a separate analytic moving-pose plane query; no native physics engine, ROS,
RM host or GPU runs. Distinct seeds apply optional generic noise per device:
0.02 m range sigma and 0.001 rad tangent-axis angular sigma. These are stress
settings, not calibrated device noise.

Two warmups precede ten alternating-order timing pairs per case. All sample
values, baseline default-path comparisons and source hashes are available in
[the machine-readable record](/benchmarks/prepared-rays-m5-max.json).
Unrelated host work was present; small differences and absolute times should not
be generalized. Bitwise output checks and scene-query regressions independently
verify correctness; a throughput ratio alone cannot establish motion fidelity.

Eight-device results (640,000 total retained rays per timed case):

| Profile | Effects | Generate per device | Prepare once per window | Observed throughput ratio |
|---|---|---:|---:|---:|
| Nominal | Ideal | 55.81 ms | 12.59 ms | 4.43× |
| Nominal | Range + angular | 126.74 ms | 101.66 ms | 1.25× |
| HardwareCapture | Ideal | 56.34 ms | 11.95 ms | 4.72× |
| HardwareCapture | Range + angular | 88.68 ms | 54.53 ms | 1.63× |

Two-device results are mixed and appear in the raw record. Single-device
results vary with the case; preparation adds allocation and does not offer a
consistent advantage. Noise-heavy work saves less because angular/range effects remain per
device. Native geometry can dominate instead of optical generation, so these
ratios are not predictions for a full robot/arena or Isaac/MuJoCo adapters.

Six alternating baseline/current scalar-path cases found no regression after
factoring the shared measurement kernel. Small scalar differences are
inconclusive under host load. The baseline binary's runtime corresponds to
`489360a`; its checksum and complete measurements are recorded. The prepared
path has bitwise comparisons across all profiles, strides, capture boundaries,
epochs and effects, plus chunking, occlusion and overflow regressions.

## Architectural candidates still worth testing

### Batched scene queries

The scalar `Scene::nearest_hit` interface works well for native Rust queries,
but can force too many calls when the backend naturally traces vectors of rays.
A future optional batch contract could accept bounded tiles of already perturbed
local directions and explicit emission times, returning one nearest-hit slot per
emission. A scalar fallback can preserve existing backends.

This requires a real batched backend benchmark before adding an API. Every
query must retain its time/pose context, stable output association and first-hit
occlusion; a batch must not become one frozen scene snapshot. Temporary query/hit
storage also costs memory. Native ray-call overhead, geometry traversal and
readback should be timed separately. Prepared local directions are reusable
input for that design; the present implementation still uses scalar queries.

### Compact acquisition storage

`RollingFrame` stores indices, timestamps, ranges and local XYZ for convenience,
including misses. A regular acquisition could represent repeated timing with
first/stride/epoch metadata and expose ranges/directions separately, reconstructing
XYZ when requested. That could reduce writes and enable ranges-only consumers.
It also introduces ownership/lifetime and validation choices, especially when
angular noise changes directions or callers supply irregular acquisitions.
Measure complete acquisition/consumer pipelines before adding another frame
representation. Preserve the current public frame and all f64 measurement bits.

### Typed or bulk transport construction

Existing [native measurements](transport-evaluation.md) show CustomMsg publishing
can cost much more than PointCloud2. Metadata caching removes repeated schema
work but not seven named assignments per Livox point. A typed/bulk encoding path
could address that structural cost, with an optional ROS binding strategy and
native schema/ownership validation. The pinned dynamic API does not expose a
safe serialized-publish shortcut; raw RCL bindings would require unsafe code and
are outside this crate's contract. No middleware or generated bindings were
changed here. Use the existing PointCloud2 path when the consumer supports it.

A streaming-output prototype also bypassed the intermediate return vector, but
its exploratory comparison skipped some packet validation. Its small apparent
gain is not a validated speedup, and no callback/sink API was added. An efficient
encoding path must retain overflow/error behavior and complete timestamp/miss
semantics rather than obtaining speed by omitting checks.

The next substantial gains depend on the actual workload: shared optical work
for aligned sensors, batch queries for suitable geometry engines, or transport
construction for CustomMsg consumers. Larger rewrites should be driven by paired
native measurements at the relevant boundary.
