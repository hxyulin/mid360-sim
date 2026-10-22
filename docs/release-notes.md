<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# mid360-sim 0.1.0

The first multi-simulator release of mid360-sim. Artifacts are built from tag
`v0.1.0`; each release's `release.json` records its exact source revision and
checksums. See [distribution](https://hxyulin.github.io/mid360-sim/distribution.html)
for artifact selection and verification.

## Included capabilities

- Dependency-free Rust scan generation and scene queries, with explicit
  per-emission times, misses and seeded effects invariant to chunking/decimation.
- Compact four-lane nominal optics with a representative 18.2-hour cycle,
  distinct official-table and hardware-fit profiles, and checksummed provenance.
- Optional rclrs / ROS 2 Jazzy publishers for Livox CustomMsg, timed PointCloud2,
  caller-provided SI IMU and physical static/dynamic TF.
- Isaac Sim 6.2 Warp extension with deferred readback, static bulk histories and
  explicit moving partitions/rigid histories; MuJoCo 3.7 adapter with optional
  rolling sensor poses; Gazebo Harmonic source plugin.
- Synchronized LiDAR/IMU and navigation examples with rotating mounts, an
  optional external RM host consumer and external TriLIO regression.
- Bounded deterministic packet delay, jitter, loss and reordering on a
  caller-owned delivery timeline, with separate injected/capacity-loss counts.
- Independent moving-link geometry regressions, native transport benchmarks,
  and an offline known-target material/edge validation workflow with explicit
  synthetic fixtures and held-out recording sessions.
- Task-oriented install guides, capability/status summary and separate evidence
  reports for hardware fits, motion, native backends, navigation and transport.

## Artifacts and installation

The release contains complete source, a Rust source crate, a Python adapter
wheel, an Isaac extension ZIP and a Gazebo source bundle, plus `release.json`
and `SHA256SUMS`. No simulator runtime, CAD, recorded dataset or private decoder
is bundled. Registry publication is separate; no PyPI/crates.io upload is made.

Start with [installation](https://hxyulin.github.io/mid360-sim/guide/getting-started.html), choose the
[appropriate artifact](https://hxyulin.github.io/mid360-sim/distribution.html#choose-an-artifact), and verify its hash
against `SHA256SUMS` from the same release. Source archives include the guides;
hosted documentation follows main and may advance after this release.

## Validation and limits

This is an empirical approximation, not a calibrated Livox emulator. Callers own
dynamics, optical geometry, pose interpolation, scheduling, IMU measurements and
extrinsics. Navigation consumers own deskewing, localization and recovery. Motion
semantics differ by adapter; see [capabilities and status](https://hxyulin.github.io/mid360-sim/guide/status.html).

Rust tests include deterministic fault scheduling, unchanged payloads and moving
mast/two-link scenes checked against independent triangles. Python tests check
calibration input integrity, miss retention and separation of training/validation.
Native ROS smoke has passed on macOS arm64 at full rate and stride 3; Linux DDS
runtime validation remains open. Loaded transport runs have also exposed missing
deliveries and bounded-queue losses, so no universal lossless-delivery or real-time
capacity guarantee is made.

Material/beam/noise calibration still needs independent physical measurements;
the new workflow's synthetic success is a tooling check. Native replay measures
agreement with its stated motion/geometry reference, not calibrated hardware
range precision. See [validation](https://hxyulin.github.io/mid360-sim/guide/validation.html),
[runtime/accuracy results](https://hxyulin.github.io/mid360-sim/performance-tradeoffs.html), and
[transport evidence](https://hxyulin.github.io/mid360-sim/transport-evaluation.html) for conditions and limits.
