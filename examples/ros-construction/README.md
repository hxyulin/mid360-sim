<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Native ROS construction experiment

Isolated benchmark package for cached dynamic messages and generated RMW-native
Livox structs. It does not change the production publisher or core dependencies.
Generated interfaces stay in the ignored ROS install tree.

See [measurement scope and reproduction](../../docs/single-sensor-performance.md)
and [ROS environment setup](../../docs/install/ros2.md).

For delivery controls, add `--points 20000 --sensors 2` to isolate the full-rate
two-sensor case. Change one of `--consumer raw`, `--depth 20`,
`--reliability reliable`, or `--spacing-ms 5` per comparison. Defaults are a
decoded Python subscriber, best-effort depth 5, and simultaneous sensor sets.
Both publishers and the subscriber use the selected QoS. Spacing changes only
publication scheduling; sensor timestamps and each sensor's 10 Hz rate stay fixed.

The raw consumer records serialized messages at callback entry and decodes
them after the measured interval for field checks. Its callback latency excludes
message decoding, so compare it as a delivery/receiver-cost control, not as
application-ready latency. Reliable publication can block; inspect publication
lateness and received counts alongside publish-call times. All measured
messages retain fixed-rate sending with no retries or receiver acknowledgements.

`matched.py` runs the production endpoints against the same native rclpy receiver
at 10 Hz (one/two sensors, 6,667/20,000 retained points). Build this package with
`mbx build --release --locked --manifest-path examples/ros-construction/Cargo.toml`.
In the activated native ROS environment, run:

```sh
python examples/ros-construction/matched.py \
  --binary "$PWD/target/release/mid360-ros-construction-bench" \
  --lock "$PWD/local-assets/perf-investigation.lock" \
  --pairs 3 --packets 30 > local-assets/matched-native.jsonl
```

The lock is mandatory and covers every local timing subprocess. When borrowing
another checkout's ROS environment, unset its `CARGO_TARGET_DIR` before building
and let `mbx` manage this checkout's own target directory. On macOS, the generated Python interfaces
may also require `DYLD_LIBRARY_PATH` containing the ROS install and Pixi `lib`
directories. The receiver requires the environment's existing `psutil` package;
no Rust or project dependency is added.

Best-effort/volatile/depth-5 QoS matches both production endpoints. Each fresh
process waits 300 ms after matching, sends one startup packet per sensor spaced
100 ms apart, waits for one acknowledgement after
full field verification, then sends without per-message acknowledgements or
retries. Three pairs alternate dynamic/typed order. A fixed 300 ms receiver drain
follows each run; losses and duplicate sequences remain visible. All received
headers and three points per measured packet are checked after timing; every
point of each startup packet is checked before timing. Coordinates, reflectivity,
lanes, offsets and simulation stamps change deterministically. Per-run raw sends
and arrivals retain RCL steady-clock nanoseconds separately from sensor stamps.
Each `sends` row is `[sequence, sensor, before_ns, after_ns, completion_lateness_ns]`;
each `arrivals` row is `[sequence, sensor, callback_entry_ns]`. Completion lateness
is relative to each sensor's scheduled deadline and includes preparation/publishing.

`publish_call_median_ms` times only each production `publish` call (construction,
validation, RMW publish and message drop). `publish_set_median_ms` sums the serial
sensor calls per frame. Latency starts immediately before publish and ends at
receiver callback entry, including native deserialization but excluding later
Python field assertions. CPU counters are process user+system totals over the
active send interval: publisher CPU includes caller packet preparation and DDS
threads; receiver CPU includes spinning, native deserialization and small field
samples. They do **not** isolate CPU inside publish. Their sum is transport-probe
CPU, not whole-simulator pipeline CPU. The receiver's fixed drain CPU is recorded
separately. CPU comparisons with different loss counts represent different
completed receiver work; latency excludes missing packets. Acquisition, physics,
packet conversion, IMU, clock publication and
production subscriber work are absent. Process counters have platform-dependent
resolution. Results on native macOS arm64/Fast DDS do not validate Linux runtime.
