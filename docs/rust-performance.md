<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Performance review and Rust baseline

Local microbenchmarks on 2026-10-23 isolate acquisition effects, rolling fixture
geometry, capture profiles, encoding, caller IMU synthesis and packet scheduling.
They guide optimization priorities; they do not replace the
[native backend results](performance-tradeoffs.md) or
[ROS transport measurements](transport-evaluation.md).

## Reproduce

```sh
cargo run --release --locked --example core_bench
```

Host: Apple M5 Max, macOS arm64, rustc 1.99.0. Each case has three warmups and
15 timed repetitions in fixed order, with no CPU pinning. Acquisition reuses
its output allocation; packing allocates new output on every call. Each scan
case covers ten 100 ms packets at the nominal emission rate; stride 3 retains
66,670 rays over those same windows. Stride selection restarts at each packet,
changing the modulo-3 phase; synchronized acquisition instead keeps global
multiples of three (66,667 rays in one second). The spherical surface is an intentionally
trivial query; the articulated room uses the shared analytic fixture, including
per-emission dynamic mounting rotations. This is not native simulator physics.

[Machine-readable results and source checksums](/benchmarks/rust-core-m5-max.json)
include medians and observed minima/maxima. The source revision identifies the
library and fixture; the benchmark was added afterward and has its own checksum.
No speedup has been implemented or established by this baseline.

## Measurements

Acquisition times below cover 200,000 retained rays unless stated otherwise.
Optional effects remain uncalibrated stress models.

| Case | Median wall time |
|---|---:|
| Nominal, trivial surface, ideal | 6.11 ms |
| Same, range noise | 10.41 ms |
| Same, angular noise | 15.03 ms |
| Same, range/angular noise + independent/diffuse dropout | 18.44 ms |
| Articulated room, ideal | 32.44 ms |
| Articulated room, all effects | 44.71 ms |
| Articulated room, all effects, stride 3 (66,670 rays) | 17.58 ms |
| Hardware mean profile, trivial surface, ideal | 20.17 ms |
| Hardware capture profile, trivial surface, ideal | 23.68 ms |
| Livox conversion, 200,000 existing returns | 0.420 ms |
| PointCloud2 encoding, 200,000 existing returns | 0.358 ms |

Packing repeats one warm 20,000-return packet ten times; the count is aggregate
work, not one 200,000-point packet. These stages run separately, with already
acquired input for packing. Subtracting
cases indicates incremental workload cost, not a profiler attribution. In
particular, dropout changes how many returns reach subsequent range-noise work.

The caller-owned articulated IMU fixture takes 160.36 ms for 200,000 samples,
about **0.160 ms per simulated second at 200 Hz**. The raw trial spans 1,000
simulated seconds, unlike the one-second LiDAR cases. It averages five analytic
subsamples per measurement. That cost is specific to this example synthesizer,
not the core IMU value contract or a dynamics engine.

Fault decisions take 0.342 ms per 100,000 packet IDs. Queue submission and drain
with 1% configured loss cost 5.61 / 17.94 / 22.19 ms per 100,000 submissions at
capacities 8 / 256 / 4,096. The test fills in capacity-sized groups, then drains
with an offline maximum deadline; it isolates bookkeeping with scalar payloads,
including empty-queue construction per trial but not payload allocation,
realistic polling latency or DDS. Even the largest
case is about 0.222 microseconds per submission, so queue replacement is a low
priority at 10 Hz clouds and 200 Hz IMU.

## Review findings and implementation order

Three focused reviews covered patterns/acquisition, effects/kinematics, and
transport/native adapters, including older code. The parent review checked
relevant source and independently reran the unexpected-miss reproducer. No
production optimization or bug fix is included in this review. Operation counts
and compiler evidence identify work to remove; they are not measured speedups.

### Correctness first: disabled diffuse dropout can reject a valid hit

`effects::uniform` can round to exactly `1.0`. The sensor tests
`uniform(seed, index, 2) < response` even when diffuse dropout is disabled and
`response` is one. With seed **7849933349262882758**, index zero, default effects
and a valid 2 m surface, acquisition returns a miss. This was reproduced against
the current release build. It is an endpoint rounding error, not configured
random dropout.

A narrowly scoped disabled-diffuse bypass is a candidate fix. Probability-one
survival in enabled diffuse mode also needs an explicit endpoint decision.
Changing the uniform mapping globally affects Gaussian noise as well. Add the
reproducer and survival-probability 0/1 regressions before fixing this; record
that rare outputs change. Do not label it a bit-preserving optimization.

### First: share scan-profile work across retained lanes

`hardware.rs` repeats phases and 22 harmonic sin/cos evaluations per lane in
mean/capture profiles; `pattern.rs` does likewise for the older official-table
profile. Four-lane evaluation could reduce harmonic sets from 88 to 22 per full
group; final ray trig remains lane-specific. Nominal acquisition already shares
this work at strides one and two. Add equivalent paths for all profiles while
preserving each lane's accumulation order and phase reduction. Retain a scalar
path for sparse strides instead of always calculating four lanes.

Capture also searches 7,004 calibration knots per ray. After grouping, test a
packet-local cursor initialized by one binary search and advanced monotonically.
Keep the exact knot-boundary comparison, interpolation arithmetic and held
residuals outside the knot range. A local cursor must not couple separate or
out-of-order calls. Numeric capture-support checks could also avoid the current
two endpoint direction evaluations during every acquisition's validation.

Benchmark directions separately from acquisition, all profiles, counts
1/4/200/20,000, unaligned starts and strides 1/2/3/4/7. Include late capture
windows and knot crossings. Compare scalar/group results bitwise, test bounded
capture rejection before output mutation, and retain overflow checks.

Caching scan directions must not cache sensor poses or geometry: angular
perturbation and scene queries still run at every emission's own timestamp,
including dynamic rotations and moving obstacles. Nominal retained-lane grouping
at strides two/three is a further candidate once the simpler paths are measured.

### Second: remove fixture-only work from rolling geometry and scoring

Release LLVM IR retains a full 288-byte Jet pose and derivative calculations in
room intersection even though only pose values are consumed. Use one shared
trajectory expression supporting both `f64` and Jet, preserving value arithmetic
order; retain derivatives for caller IMU synthesis. This could improve the
analytic example, not native simulator physics. Evaluate translation and rotation
at every emission. Compare pose values and complete returns bitwise, including
LIO ramp boundaries at three/four seconds.

The synchronized evaluator also recalculates the same frozen packet-start pose
for every hit. Compute that diagnostic pose once per packet. Motion-correct
scoring still needs each return's pose. Benchmark scoring separately from sensor
acquisition, and compare metric accumulators and hashes. Buffer reservation,
reusing the small IMU vector and avoiding whole-packet chunk copies are lower
priority candidates; preserve owned outputs, miss records and IMU endpoints.

### Third: cache ROS metadata and measure native construction

The locked rclrs 0.8.0 supports safe `DynamicMessageMetadata::create()`. Current
publishers recreate metadata and parse type names on each publish. Retaining
metadata can avoid schema construction; upstream already caches loaded libraries.
This does not require unsafe code or a new dependency. Reusing whole messages is
a separate ownership problem because publishing consumes `DynamicMessage`.

Livox also performs seven named assignments per point. Existing native results
record **6.154 ms per CustomMsg publish call versus 0.275 ms for PointCloud2**;
this is context, not attribution or a promised metadata-cache saving. See the
[transport measurement boundaries](transport-evaluation.md).

Separate metadata creation, sequence initialization, population and DDS publish
timing. Alternate baseline/candidate runs at full rate and stride three, including
IMU. Verify subscriber field equality, changing sizes, all-miss packets, stamps
and schema errors with the native ROS checks. Local core packing excludes these
costs. Direct destination encoding could remove a temporary copy, but is lower
priority until native measurements justify additional APIs. Keep the current
fault queue; its bookkeeping cost is small at ordinary cloud/IMU rates.

### Native adapter opportunities, including established code

- **Isaac moving-transform capture:** use a bounded contiguous history buffer to
  reduce per-step inverse-matrix/list allocations and flush consolidation. Capture
  each segment's real transforms and preserve sensor SLERP. Caller bulk/native
  poses need coherent snapshot alignment; do not freeze poses or assume USD
  notices track physics. Measure CPU capture separately from GPU queries, then
  rerun the native rigid-history and arena workloads.
- **Isaac object scaling:** test conservative local-space bounds before querying
  every moving mesh. Add a hierarchy only if measurements justify it. Bounds must
  respect affine transforms, grazing/inside origins and nearest-surface occlusion;
  a below-minimum hit must continue blocking farther objects. Test sparse and
  overlapping scenes at 1/8/32/128 meshes against combined geometry.
- **MuJoCo rolling calls:** profile reusable scratch arrays and supported NumPy
  `out=` operations for phases, interpolation and rotations in short 200-ray
  calls. Preserve owned range/ID outputs and per-ray rotations. Do not enlarge
  acquisition windows to hide allocations: geometry is current per call. Validate
  native rotation, ownership, moving-geometry and near-occlusion tests.
- **Gazebo and ROS packing:** message schema reuse and safe payload-buffer exchange
  may remove allocations/copies. Check publisher ownership semantics and native
  subscribers/reset behavior before changing them.

These adapter savings remain hypotheses and require their native environments.
Existing reports already show CPU bookkeeping can dominate moving-scene work;
a previous GPU kernel gain did not materially improve the arena. GPU work should
therefore target measured geometry/scene-scaling costs and sufficiently large
batches with device-resident data. This local benchmark provides no GPU results.

### Angular noise: expensive, but no simple arithmetic win established

Angular noise adds about 8.9 ms per 200,000 rays in the trivial scene. On this
M5 release build the compiler already combines tangent-basis sin/cos into one
`___sincos_stret` call and shares common indexed-hash work. Merely rewriting source
to call `sin_cos()` is not evidence of a speedup or portable bit identity.

Algebraic tangent bases, reciprocal normalization and paired Box–Muller outputs
can change seeded bits, poles or signed zeros. The current axes use independent
uniform pairs 10/11 and 12/13, each taking the cosine result. Benchmark the effect
separately from checked-input validation and complete acquisition before changing
it. Keep the old implementation as a bitwise comparison oracle across seeds,
indices, poles and strides: chunk invariance alone cannot catch a consistently
changed seeded stream. An alternative noise policy would need explicit semantics.

## Before claiming an optimization win

Extend the baseline with official-table and direction-only cases, sparse hardware
profiles, tiny/unaligned packets, early/middle/late capture windows, global stride
alignment, isolated noise/dropout modes, and synchronized scoring/assembly.
Record build flags and allocation counts where relevant. The current min/max
ranges and fixed case order cannot establish significance for small gains.
Run old/new comparisons serially on an otherwise idle host, alternate their
order, and retain scalar/reference oracles plus complete output hashes.

For compiler inspection without timing another workload:

```sh
cargo rustc --release --locked --example core_bench -- --emit=llvm-ir,asm
```

Preserve the distinction between direction generation, scene acquisition,
example scoring, native physics, message construction, DDS delivery and consumer
conversion. macOS rclpy results do not establish Linux DDS runtime or native
estimator performance. No wall clock belongs in core acquisition timestamps.
