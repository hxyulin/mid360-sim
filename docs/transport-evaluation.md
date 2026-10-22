<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Sensor transport measurements

The [benchmark guide](guide/transport-benchmarks.md) defines timing boundaries,
clock calibration and reproduction. This report records local native ROS and
external RM HTTP/JSON measurements; neither is a hardware latency specification.

## Scope

2026-10-23, Apple M5 Max (18 logical CPUs), macOS 27.0.1 arm64, Python 3.12.15.
Cases run sequentially on a shared workstation; this is not an exclusive-core
lab measurement. Default warmup is five packets. Short cases then measure 60
packets per stream (6 simulated seconds). CPU totals include warmup; native
publisher CPU additionally includes discovery and teardown. Eight busy SHA-256
processes provide the explicitly labeled load cases; they are not a calibrated
CPU-utilization target. Per-run load averages are retained.

The native publisher uses the optimized release build of this repository's
actual rclrs publishers with the existing analytic room acquisition fixture.
The subscriber is rclpy, so the callback interval includes Python message-object
construction and executor scheduling. It is not the cost of delivery into a Rust
or C++ estimator. Both formats carry exactly the same full-rate local fixture
returns, timing and SI IMU, but their schemas and runtime representations differ.

RM uses the existing debug CAD host at external revision
`b9d0a2c752dfc8d4f247349a2d01e89664b59484`. The host binary SHA-256 is
`72f6cdd51ca01137f76c2d96f274d078b82fed521396f2c2d8a3010c0d6619f4`.
The actual articulated gimbal carrier is driven by a spinning chassis and
changing requested head yaw; measurements use actual solver/carrier state.
Multiple streams attach to the same robot in one world, with independent bounded
workers. HTTP and native fixture results use different scenes/payloads; their
latency numbers must not be treated as a transport-only A/B comparison.

For the baseline measurements below, no production middleware, queue bounds,
sensor stamps or RM source were changed. The later external telemetry work is
recorded separately under [current follow-up status](#current-follow-up-status).
Reports include source revision/dirty state and native binary hashes. The
benchmark source is initially based on `fcbf715` plus this change; ignored local
reports retain the individual raw samples and identities.

## Lower-load native ROS references

The 30-second, one-stream Cyclone references ran at one-minute OS load averages
3.4–5.3. They received every cloud and IMU sample, including five warmup packets:
305 clouds and 6,100 IMU messages each. Acquisition stamps were unchanged.
Clock-offset half-width uncertainties were below 5 microseconds in these runs.

| Full-rate format | Acquisition p50, ms | Cloud publish call p50, ms | Publish start → rclpy callback p50, ms | Callback interval p99, ms |
|---|---:|---:|---:|---:|
| PointCloud2 | 3.293 | 0.275 | 0.410 | 0.629 |
| Livox CustomMsg | 3.348 | 6.154 | 25.385 | 39.855 |

The large difference is a **representation/consumer-path result**, not evidence
that DDS wire time differs by that amount. CustomMsg constructs a Python object
per point; PointCloud2 retains a byte buffer. Native publisher calls also differ
because safe dynamic introspection writes individual Livox point fields. No pure
DDS or pure Python-deserialization timer was inserted, and a native Rust/C++ LIO
subscriber must be measured separately before transferring these numbers to it.

The short 6-second Cyclone matrix, all articulated, gives scaling controls:

| Format / streams / stride / busy workers | Callback interval p50, ms | p99, ms | Cloud delivery | IMU delivery |
|---|---:|---:|---:|---:|
| PointCloud2 / 1 / 3 / 0 | 0.186 | 0.340 | 65/65 | 1300/1300 |
| CustomMsg / 1 / 3 / 0 | 7.930 | 12.176 | 65/65 | 1300/1300 |
| PointCloud2 / 4 / 1 / 0 | 0.393 | 0.453 | 260/260 | 5200/5200 |
| CustomMsg / 4 / 1 / 0 | 40.090 | 57.682 | 260/260 | 5200/5200 |
| PointCloud2 / 4 / 3 / 0 | 0.181 | 0.297 | 260/260 | 5200/5200 |
| CustomMsg / 4 / 3 / 0 | 13.107 | 20.474 | 260/260 | 5200/5200 |
| PointCloud2 / 4 / 1 / 8 | 0.530 | 0.976 | 260/260 | 5200/5200 |
| CustomMsg / 4 / 1 / 8 | 205.201 | 351.221 | 260/260 | 1213/5200 |

In the four-stream CustomMsg/8-worker case, all clouds arrived but **3,987 IMU
samples were missing after the bounded drain**. This is failed end-to-end delivery,
not proof that packets were lost on a network link. Reader history, deserialization,
executor scheduling and receive deadlines all participate. The corresponding
PointCloud2 case delivered everything; its worst individual callback interval was
5.225 ms. Clean native process shutdown was required in all normally reported runs.

![One-stream native ROS Python callback interval p50 and p99 for PointCloud2 and CustomMsg.](/figures/ros-callbacks.svg)

[Figure data and context](guide/results.md).

## RM HTTP/JSON

These short cases use one sequential Python polling client, ten-millisecond
post-cycle delay and an actual moving gimbal. Their one-minute load averages
were roughly 5–8; "0 busy workers" does not mean an otherwise idle machine.
HTTP is the **headers + body-read wall interval**, excluding JSON decode.
Simulation age is the maximum of the per-stream p95 packet-end ages. It is not
wall latency and excludes the 100 ms acquisition span.

| Streams / stride / busy workers | Nonempty HTTP p50, ms | JSON decode p95, ms (all polls) | Simulation age p95, ms | Payload MiB/s | Reported drops |
|---|---:|---:|---:|---:|---:|
| 1 / 1 / 0 | 17.43 | 8.56 | 31.25 | 20.4 | 0 |
| 1 / 3 / 0 | 12.91 | 2.70 | 31.25 | 6.8 | 0 |
| 4 / 1 / 0 | 6.33 | 10.11 | 84.38 | 81.5 | 0 |
| 4 / 3 / 0 | 11.42 | 2.80 | 68.75 | 27.1 | 0 |
| 1 / 1 / 8 | 14.83 | 10.16 | 32.81 | 20.4 | 0 |
| 4 / 1 / 8 | 8.67 | 26.68 | 189.06 | 77.4 | 12 |

The final harness additionally separates empty/nonempty JSON decode distributions
and saves HTTP timing traces. The first matrix predates that additional recording;
its all-poll p95 is shown explicitly rather than relabeled as per-packet decode.
The full-rate single-stream client used about 0.10 CPU cores on average; the server
used about 0.65 cores including physics/acquisition, with coarse `ps` CPU timing.
Server time cannot be attributed to JSON serialization alone.

Changing only post-cycle polling delay from 1 / 10 / 50 ms produced single-stream
simulation-age p95 of 15.625 / 31.25 / 68.75 ms in the short controls. More frequent
polling also produced many empty requests. Inspection of the tested RM source
shows each response closes its connection (`http.rs::write_response`) and the
nonblocking accept loop waits 10 ms on `WouldBlock`. This is a plausible source
of the observed ~11–13 ms median empty-request interval. It is **an inference**,
not a controlled measurement of listener sleep or a justification for removing
lifetime/admission bounds.

The longer HTTP references show why short no-loss runs are insufficient:

| 30-second case | Worst stream simulation age p95, ms | Drops across streams | One-minute OS load before → after |
|---|---:|---:|---:|
| 1 stream, 0 injected workers | 31.250 | 0 | 5.3 → 9.7 |
| 4 streams, 0 injected workers | 137.500 | 3 | 9.7 → 10.9 |
| 4 streams, 8 injected workers | 215.625 | 62 | 10.9 → 15.0 |

Those drop counts matched sequence gaps, without additional input discontinuities
in the long runs. This is consistent with completed-output queue pressure; the
tested host revision did not expose separate timing/counters sufficient to assign
every loss unambiguously. Background load rose during the longer matrix, so
duration alone cannot be assigned as the cause of the difference from six-second
tests.

## Binary is smaller; this prototype is not a faster decoder

A complete full-rate measured RM bundle, with explicit misses and all f64 values,
was compared locally after stopping the server. The 15-iteration reference used
identical semantic data and required roundtrip equality, including integer times
and IMU sample indices. No live binary endpoint was added.

| Python codec, one complete frame | Size, MiB | Encode p50, ms | Decode p50, ms |
|---|---:|---:|---:|
| Compact JSON | 2.096 | 13.290 | 9.975 |
| M360BN01 prototype | 0.936 | 7.131 | 10.613 |

The binary payload was 44.7% of the JSON size (2.24× smaller).
However, reconstructing Python dictionaries and validating each binary value did
not beat the optimized JSON decoder in this prototype. This is not a native Rust
codec comparison, and the validation work differs. A binary adapter should be
benchmarked with its intended typed/array consumer before promising a CPU win.
Byte reduction alone does not address connection acceptance, polling or queue age.

## Later controls were confounded by unrelated host load

The additional four-thread HTTP controls began at one-minute OS load averages
around **59**, versus 3–8 in the first matrix, on an 18-logical-CPU workstation.
They reported 18 drops without injected workers and 141 with eight workers, and
an input discontinuity also appeared. A subsequent single-stream control reported
two drops. These runs exercise the concurrent code path and error accounting but
**cannot establish whether threads improve or worsen performance**. Thread wall
intervals include waiting for other work; they are not additive parse CPU time.
Repeat sequential/concurrent cases under matched host conditions before choosing
that implementation. No unrelated processes were stopped to obtain a cleaner run.

Fast DDS full-rate PointCloud2 also varied. Its first short SYSTEM_DEFAULT case
missed one cloud and four IMU messages despite modest OS load (~5). Three repeats
returned pass / missing data / pass while background load rose from ~15 to ~27.
Later LOCALHOST controls at load ~60 also missed deliveries. Stride-3 and Livox
short controls passed. This is evidence of configuration/workload sensitivity in
this setup, not proof of a specific DDS defect, a cross-platform ranking or that
switching discovery range fixes it. Both live DDS paths were tested only on macOS
arm64. Cyclone uses an explicit loopback XML; Fast DDS uses its own configuration.

## Recommendations from the baseline {#what-to-do-next}

These recommendations were made against the `b9d0a2c` host above. Their current
implementation status follows separately; the original measurements are unchanged.

1. Use the existing timed PointCloud2 path for Python consumers when their schema
   contract permits it. Keep CustomMsg for consumers that require it; measure the
   actual native LIO node rather than transferring rclpy callback costs to it.
2. Add **opt-in host-side stage telemetry** before changing RM middleware: acquisition
   completion, completed-output queue age, owner request wait and native encoding.
   Test independent native clients as well as a single Python fan-out client.
3. A/B-test bounded persistent delivery/connection acceptance against current polling
   on a controlled host. Retain queue bounds and original acquisition stamps.
4. Evaluate a binary adapter using typed buffers if bandwidth/object conversion
   remains material. The local codec is an experiment, not a production protocol.
   These measurements do not yet justify implementing a custom middleware.

All 40 workload runs (27 initial, 8 longer/repeat, 5 later controls) are retained,
including failures. The deliberately forced watchdog check is separate and records
`watchdog_timeout` rather than success. Raw reports, native timing/clock traces,
final receiver/HTTP traces, codec samples, logs and source manifests remain ignored
under `local-assets/transport-20261023/`. No datasets, ROS runtime or generated
interfaces were committed. Source revisions, binary hashes and load averages
identify conditions; repeat before asserting production p99 or capacity bounds.

## Current follow-up status

External RM commit `a6ae7223ecfcdc617979c34af79b07348b044db1` on integration
branch `feat/shared-angular-noise` implements the opt-in stage telemetry proposed
above. It retains ordinary endpoints and sensor stamps, and adds capture/ingest
work, input/output queue residence, owner-request wait, native JSON encoding and
separate stage loss counters. It is external source work, not a new dependency
or a published feature of mid360-sim. See the
[profiling workflow](guide/transport-benchmarks.md#optional-stage-profiling-in-rm).

Its separate six-trial CAD evaluation used Apple M5 Max / macOS arm64, an
optimized debug host and a **Python 3.9.6** consumer, at ambient one-minute load
averages around 18–24. The full-density single-stream profiled run recorded three
rejected input capture brackets, zero ingest errors and zero output drops.
The stride-3 and moving-gimbal runs had no stage losses. Four full-density
attachments produced six completed-output drops without input/ingest losses;
a deliberate 600 ms consumer stall produced four output drops. The default loss
policy correctly rejected the lossy single-stream runs; intentional pressure
trials allowed drops while recording them.

Those observations establish useful stage attribution, not a loss fix or a
controlled comparison with the Python 3.12 baseline above. Work-window timings
are not exact per-packet costs, and background load prevents an overhead or
capacity claim. Full source/binary/asset identities and attribution limits are
recorded in RM's `docs/sensor-transport-telemetry.md`; raw metadata remains local
to that evaluation checkout.

Remaining follow-ups are to measure the actual native consumer under sustained,
controlled load, validate Linux DDS runtime, and then A/B-test connection handling
or typed codecs if the measured bottleneck warrants it. The Python binary
prototype above remains an experiment; no replacement middleware has been
implemented or justified by these results.
