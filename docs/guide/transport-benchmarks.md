<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Benchmark sensor transport

Use the bounded harness in `examples/transport/benchmark.py` to measure existing
interfaces before choosing a new middleware. It starts only fresh test processes;
it does not attach to or control an existing match. Reports and logs are local
artifacts, not publication. No Cargo dependency on RM is added.

For repeatable synthetic delivery faults without ROS or an external host, use
[the packet fault adapter](packet-faults.md). It is separate from these actual
middleware measurements and preserves the original acquisition payloads.

## Native ROS

After [ROS installation](../install/ros2.md):

```sh
pixi run transport-benchmark --mode ros --frames 60 \
  --output local-assets/transport-pointcloud
pixi run transport-benchmark --mode ros --format livox --stride 3 \
  --output local-assets/transport-livox-stride3
```

This runs the actual native Rust `LivoxPublisher`, `PointCloudPublisher` and
`ImuPublisher` with a separate Python/rclpy subscriber. Each sensor independently
acquires 200,000 nominal emissions/s from the existing analytic room fixture,
aggregated into 100 ms cloud/20-sample IMU bundles. Misses retain original indices
and stamps; valid XYZ remain distorted sensor-local measurements. The default
`--motion articulated` includes a rotating body and head; use `stationary` or
`yaw` for controls. Multiple sensors replay the same physical trajectory and
scene, each acquiring independently; this is workload scaling, not a simulated
multi-robot experiment.

`--format pointcloud|livox` publishes only the selected cloud format and SI IMU,
with best-effort QoS. It creates both endpoint types but does not send redundant
cloud traffic. `--sensors 1..4`, `--stride 1..1001` and `--load-workers 0..8`
select load. Each load worker is a separate busy hashing process; this is explicit
CPU pressure, not a model of another robot application. It does not guarantee
exclusive cores or a fixed fraction of CPU capacity.

The default middleware is Cyclone with the repository's explicit local-only
loopback profile. `--rmw rmw_fastrtps_cpp` switches middleware without changing
sensor acquisition. `--discovery-range SYSTEM_DEFAULT|LOCALHOST` makes the ROS
discovery profile explicit; Cyclone still uses its explicit loopback XML. Each invocation runs in a fresh bounded process group,
including middleware teardown. A native failure/hang fails the run; a watchdog
kill is never counted as successful shutdown. Domain 82 is reserved by default;
run these tests sequentially so benchmark nodes do not interfere.

The Rust example exchanges twelve timestamped PING/PONG messages over pipes
before sending. For each reply, its `Instant` timestamp must fall between the
Python `monotonic_ns` request and response times. Intersecting these intervals
bounds the offset between the clocks; the report includes half-width uncertainty.
This is a same-host, short-run clock calibration, not network clock synchronization.
Wall-clock date/time is never substituted for an acquisition stamp.

Measured stages are:

- Native fixture acquisition (raycasting plus caller IMU synthesis).
- Conversion into checked Livox packet values.
- Publishing the 20-message IMU burst.
- Cloud publisher call, including dynamic message construction/encoding and RMW.
- Publisher-call start to Python subscriber callback, including receive-side
  deserialization and executor scheduling. This can overlap the publisher call.
- Acquisition completion to callback and per-IMU burst-start to callback.
- Acquisition-start lateness against the requested 10 Hz host pacing deadline.

These distributions are not additive: callback delivery can occur before the
publish call returns. The test does not isolate DDS wire time or internal RMW
queues. The fixture acquires a complete historical analytic packet at its nominal
packet-end pacing deadline; it is not a live physics acquisition worker. Its
acquisition duration is compute cost, not a measure of physical acquisition span.

Raw native timing events and clock samples are retained; the final harness also
records receiver cloud/IMU arrival stamps. The subscriber checks
all cloud counts/identities, exact expected IMU stamps, missing/duplicate messages,
and the last cloud's schema/sample points. It defers expensive validation/CDR
size serialization until after timing. Existing full
[synchronized smoke tests](synchronized-sensors.md) check complete field equality.

## External RM HTTP/JSON

Build a compatible external RM server and supply extracted CAD as described in
[the synchronized guide](synchronized-sensors.md). Only Python's standard library
is needed for this mode; ROS is not imported or run:

```sh
python3 examples/transport/benchmark.py --mode http --frames 60 \
  --rm-checkout /path/to/rm-simulator --cad-assets /path/to/field-2027 \
  --output local-assets/transport-http
python3 examples/transport/benchmark.py --mode http --stride 3 --sensors 4 \
  --rm-checkout /path/to/rm-simulator --cad-assets /path/to/field-2027 \
  --output local-assets/transport-http-four
```

This starts one actual CAD host, spawns/stops a Sentry and attaches one to four
measured streams to that robot. `--carrier Chassis|Gimbal` chooses the actual
carrier. With the default articulated motion, it requests chassis yaw +0.5 rad/s
and changes requested head yaw every 0.5 s. The host's actual dynamics and motor
response determine measurements; requested angles are not sensor poses.

Each stream has the host's existing four input-capture/two output-bundle queue
bounds. The consumer uses fresh HTTP requests just like the existing adapters;
RM closes each response connection. It reads up to 32 MiB, separates request to
response headers, response body read, JSON decode and lightweight metadata checks.
`--poll-ms` sets delay after a complete polling cycle, not a guaranteed request
period. It polls multiple attachments sequentially by default, exposing serial fan-out
cost. `--http-concurrency 4` overlaps requests with a bounded thread pool. Those
intervals include thread scheduling; concurrent wall durations must not be added
as CPU time. The control does not model independent native clients.
The metadata check inspects counts, epoch, sequence, all IMU times and three
retained emission indices/times; it does not add a full per-point validation pass
to this transport baseline.

This harness uses the ordinary RM attachment/poll endpoints. HTTP headers wait
includes connection acceptance, host request wait, JSON serialization and
scheduling; those parts cannot be separated by this test.
Packet-end **simulation age** is reported separately and is not wall latency.
Compare the native ROS and RM modes as different stage baselines, not as two
transports receiving identical scenes/payloads. No estimator is included.

### Optional stage profiling in RM

The external [rm-simulator repository](https://github.com/trident-rm/rm-simulator)
now has opt-in stage profiling in commit
`a6ae7223ecfcdc617979c34af79b07348b044db1` (integration branch
`feat/shared-angular-noise`). This is external source work, not a released
requirement of this package. In a checkout containing that change, see
`docs/sensor-transport-telemetry.md` and `scripts/benchmark-sensors.py`.

That evaluator uses an existing live robot and can select
`/api/sensors/attach-profiled` and `/api/sensors/poll-profiled`. It reports capture,
worker ingest, input/output queue residence, owner wait and off-owner JSON
encoding, with separate loss counters. It does not spawn or control the match;
this repository's harness above instead starts an isolated fresh host. Keep
these workflows and their artifacts distinct.

Host durations remain separate from simulation stamps. Acquisition work can
cross packet boundaries, so shared work batches must be deduplicated rather
than treated as exact per-packet compute. Profiling adds overhead. See the
[current follow-up status](../transport-evaluation.md#current-follow-up-status)
for diagnostic findings and remaining controlled experiments.

Reports contain client CPU (including warmup), approximate server CPU over the
same live interval from `ps`, and response payload bytes/rate after warmup.
Server CPU has coarse timing resolution and includes its physics/acquisition
workers, not just transport. It cannot identify the cost of JSON encoding alone.
The JSON pipeline measurements do not assume that the native server's Rust
serialization has the same cost as Python encoding.

## Binary codec experiment

After each HTTP run, the server is stopped before codec work. A captured complete
single-frame bundle is encoded/decoded repeatedly with compact JSON and the
benchmark-only `M360BN01` binary layout. It retains all measured fields: u64
indices and timestamps (including IMU indices), f64 range/XYZ/gyro/specific force,
explicit misses, sequence/window bounds and cumulative loss/discontinuity counts.
Roundtrip equality is required before timing. Frame/point/IMU counts and total
bytes are bounded; truncated/invalid inputs are rejected.

This is local Python object serialization, not a new server endpoint, network
protocol or evidence of a native Rust speedup. Binary decoding still constructs
Python dictionaries and checks finite values; JSON uses its optimized C decoder.
A smaller payload does not imply lower decode CPU. No compression or float32
conversion is applied. `--codec-repeats 15` controls the default short run.

## Interpret reports

Each invocation refuses to overwrite an output directory. It writes `report.json`,
worker logs and native traces where applicable. Loss/timeouts produce a nonzero
exit status and remain visible in the report. HTTP trace files retain request timings and bounded frame metadata, and the final
harness saves one binary codec sample for offline replay. Failure before a normal
report (including the outer watchdog) writes an explicit error report.

Five warmup packets are excluded
from latency distributions by default; CPU totals include warmup. Native publisher
CPU also includes startup/discovery and teardown. ROS delivery allows one second
of drain after the publisher exits, then counts missing messages.

Quantiles use nearest rank, retaining outliers. At 60 measured packets p99 is the
maximum for a single stream; it is not a reliable production-tail estimate.
Use longer runs and repeat cases before capacity or real-time guarantees. Record
host model, build profile, OS, unrelated load and source identity. Live validation
here covers macOS arm64 only; the cross-platform lockfile proves no Linux runtime
latency. A low load-worker count does not imply an idle workstation: inspect the recorded
OS load averages and repeat matched cases when background work changes.
See [recorded results](../transport-evaluation.md) for actual findings.
