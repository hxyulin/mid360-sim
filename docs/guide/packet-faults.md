<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Deterministic packet faults

Use `transport::FaultInjector<T>` in a caller-owned adapter to test packet delay,
jitter, loss and out-of-order delivery. It accepts a complete payload, so you can
fault a paired LiDAR/IMU bundle together or use separate stream identities for
independent topics. This is synthetic transport stress, not a calibrated network
or MID-360 failure model.

The dependency-free helper never reads a clock, sleeps, publishes messages or
changes a payload. Acquisition still runs against the original sensor/geometry
history. Delaying delivery does not move a robot or reacquire the scene later.

## Run the example

```sh
cargo run --release --locked --example packet_faults
cargo run --release --locked --example packet_faults -- \
  --seed 7 --loss 0 --delay-ns 0 --jitter-ns 0
cargo run --release --locked --example packet_faults -- \
  --loss 0 --delay-ns 500000000 --jitter-ns 0 --capacity 2
cargo test --locked transport::tests
```

The example acquires noisy rolling scans and caller-owned 200 Hz IMU from the
existing articulated room fixture, then schedules complete 100 ms bundles on an
explicit offline timeline. No ROS installation or external world is required.
It verifies the original LiDAR/IMU payload fingerprint on every delivery and
reports arrival sequences, injected losses, capacity drops and backward sequence
steps. The final drain releases all retained packets without real-time pacing.

For the default 30-packet, stride-3, seed-7 example, 27 packets were delivered,
three were deliberately lost and five backward sequence steps occurred, with no
capacity drops. Repeated runs produced identical reports. These are regression
outcomes for the synthetic settings, not measured hardware/network rates.

## Wrap an adapter

```rust
use mid360_sim::transport::{FaultConfig, FaultInjector, PacketId};

let config = FaultConfig {
    seed: 7,
    loss_probability: 0.02,
    delay_ns: 20_000_000,
    jitter_ns: 150_000_000,
};
let mut adapter = FaultInjector::new(config, 8)?;
let id = PacketId { stream: 0, sequence: 42 };
let original_sensor_stamp_ns = 1_000_000_123;
// Availability/release use a separate caller delivery timeline.
adapter.submit(id, 200_000_000, original_sensor_stamp_ns)?;
while let Some(packet) = adapter.pop_due(400_000_000) {
    assert_eq!(packet.payload, original_sensor_stamp_ns);
    // Publish the original payload/stamps here, including the original sequence.
}
# Ok::<(), &'static str>(())
```

For a simulation-controlled test, advance delivery time explicitly. A real-time
adapter can supply elapsed monotonic host time outside the core. Keep the chosen
delivery timeline consistent across submission, timers and draining. Use
`next_release_ns()` to schedule a caller timer and `pop_due(now_ns)` until empty;
drain already-due packets before submitting more if you want to avoid artificial
capacity pressure. Late polling adds latency beyond the scheduled deadline.

## Reproducibility and bounds

A stable `(stream, sequence)` and seed determine each loss/jitter decision.
Point decimation, acquisition chunking, other streams and submission order do
not consume random state. Keep stream identities stable across trials and use a
different seed for an independent experiment. Reusing an identity repeats its
fault decision; pending duplicate IDs are rejected. The caller must not reuse
sequence numbers within a stream after delivery or loss.

Jitter is a nonnegative uniform integer delay in `[0,jitter_ns]`, added to fixed
`delay_ns`. It can overtake another packet but never deliver before availability.
Equal deadlines sort by stream then sequence. Invalid probabilities, zero queue
capacity and overflowing deadlines are rejected; timestamps are never clamped.
Loss probabilities zero and one explicitly disable/drop all packets respectively.

The queue drops the newest submitted packet when full and separately counts
that overload. Configured loss is decided before queue capacity. Capacity drops
depend on admission/drain order, so use the same schedule when comparing overload
runs; only the configured fault decisions are independent of that schedule.
The queue bounds packets, not bytes: also bound the size of each payload.

A paired bundle preserves its internal LiDAR/IMU alignment under delay or loss.
Independent topic streams may arrive in different orders and must be buffered
by acquisition stamp by the consumer. This helper does not duplicate packets,
corrupt bytes, model burst loss or validate a DDS deployment. Use the
[transport benchmarks](transport-benchmarks.md) separately for actual middleware.
