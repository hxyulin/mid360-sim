<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Changelog

## Unreleased

### Dependencies

- Update Tokio to 1.54.1 to address RUSTSEC-2026-0384.

### Offline tools

- Add checksummed local edge fixtures and per-ray missing/false-foreground/
  false-background regression checks, with portable synthetic occlusion tests.

- Reconstruct the ETH3D Office example jointly from both scanner views, reserving
  validation points before fusion; refresh results and matched-ray panoramas.
  Retire the experimental layered repair example from the supported workflow.

### Documentation

- Add a reproducible public ETH3D Office → optical mesh → MuJoCo example,
  with held-out range/coverage results, attributed figures and explicit limits.
- Refresh capability/validation guides for native official-plugin execution,
  MuJoCo rolling scans and Isaac history/partition modes; separate accuracy metrics.
- Move historical Isaac benchmarks out of installation instructions, link the
  current tradeoff report, and clarify source/release and Pages deployment setup.

### Integrations

- Pack Isaac segment metadata into one upload and fuse static/rigid-object
  tracing while preserving per-emission poses and near-hit occlusion.
- Add controlled source-comparison benchmarks and headless SDK stepping/thread
  probes with per-step acquisition callbacks and complete range hashes.

- Reduce temporary arrays in MuJoCo per-emission quaternion rotation while
  retaining the same SLERP, timestamps and origin-displacement policy.
- Reuse bounded Isaac static-tracing device output buffers; preserve owned
  host frames across buffer growth and smaller subsequent batches.

- Add owned bulk pose histories for Isaac static-scene tracing, reducing
  Python/transfer overhead while retaining per-emission motion and bounds.
- Add opt-in MuJoCo rolling sensor poses with per-ray quaternion interpolation
  and a configurable bound on grouped ray-origin displacement.
- Benchmark the actual pinned Livox Gazebo Classic plugin against measured
  motion, with explicit timestamp assumptions, headless PhysX tuning and
  separate sensor versus engine-loop runtime/accuracy tradeoffs.

- Replay measured motion through native RTX Motion BVH at multiple update
  rates, collect complete scans through a render writer and diagnose polling
  coverage separately. Share the custom RTX profile between test harnesses.
- Attribute MuJoCo/Gazebo replay error to pose holding, validate 200 us native
  acquisition and add phase, range/incidence and timestamp sensitivity reports.

- Replay a common independent MoCap trajectory through native MuJoCo/Gazebo
  and both Isaac Warp paths, with analytic per-emission geometry references,
  timing-policy diagnostics and reproducible source checksums.

- Share Gazebo optical phases across each four-lane group, reserve output
  capacity and add opt-in CPU phase profiling and warmed native benchmarks.
- Compare Isaac's native RTX LiDAR with a custom repeating MID-360-spec rotary
  profile, explicit Motion BVH controls, analytic moving-room checks, preserved
  acquisition offsets and a matched-scene Warp run.

- Add caller-owned RM2027 arena / neutral CAD robot acquisition benchmarks with
  source checksum/provenance records, robot-count scaling and union-hit checks.

- Add explicit static/dynamic Isaac optical partitions with GPU nearest-hit
  merging before minimum-range filtering, preserving near-surface occlusion.
- Probe batched rigid-object transform histories against fixed local-space BVHs,
  and add matched moving-obstacle benchmarks and combined-scene equivalence tests.

- Add opt-in segmented GPU tracing for static optical geometry, retaining each
  physics step's sensor motion and rejecting scene changes during delayed tracing.
- Move pose-subinterval interpolation to the GPU and allow trusted in-process
  pose readers; benchmark USD versus native physics pose access separately.

- Batch Isaac GPU readback independently of physics-step acquisition, cache
  unchanged USD geometry transforms, and reduce per-query Python allocations.
  Keep immediate output as the default and bound deferred GPU storage.
- Avoid NumPy group sorting for consecutive MuJoCo emissions while retaining
  the same nominal directions and decimated-scan behavior.
- Quantize Isaac callback durations to integer nanoseconds before advancing
  emission boundaries, avoiding spurious boundary rays from float32 step times.

- Add live 1 kHz MuJoCo/Isaac acquisition benchmarks with moving mounts,
  physics-only baselines, 200 rays per step and 100 ms output aggregation.

- Extract the standalone Rust scene-query library from rm-simulator, retaining
  measurement and interface provenance with no default runtime dependencies.
- Add optional rclrs / ROS 2 Jazzy publishing, a Pixi/RoboStack environment and
  a moving-room demo with independent real DDS message and clock smoke checks.
- Add an Isaac Sim 6.2 Warp GPU extension with optical mesh queries, explicit
  acquisition times, rolling sensor interpolation and Kit lifecycle checks.
- Add the MuJoCo 3.7 Python adapter over the caller's live scene and a Gazebo
  Harmonic plugin using DART/Bullet rays, native point messages and headless tests.

### Acquisition

- Default to representative hardware-derived, quarter-degree nominal optics:
  ten harmonics, four lanes and an explicit 18.2-hour clock cycle.
- Share harmonic work across four-lane groups while preserving scalar/chunk/
  stride results. Retain official-table, hardware-mean and bounded capture profiles.
- Add generic seeded range/angular effects and an opt-in diffuse-dropout stress
  model. Keep noise batching-invariant and near-surface occlusion intact.
- Support explicit sensor mounting tilt in the analytic room demo.

### Evidence

- Investigate inexpensive orientation-curvature interpolation against recorded
  MoCap; document modest gains, knot dependence and abrupt-stop regressions
  while preserving the runtime default.

- Analyze a 300-second stationary capture, fit learned optics/clock profiles,
  and document withheld-data errors and measured-clock conditioning.
- Import independent LVX2/ROS captures and compare frozen optics across devices
  with explicit timestamp reconstruction hypotheses.
- Compare controlled rolling acquisitions and recorded MoCap trajectories;
  include read-only timing audits, held-out floor checks and selection uncertainty.
- Audit the official Livox source and its snapshot policy separately from optical
  references. Document remaining motion, geometry and hardware realism gaps.

### Distribution and documentation

- Add task-oriented VitePress guides for Rust, ROS 2, Isaac Sim, MuJoCo and Gazebo,
  with local search, production link/browser checks and a GitHub Pages workflow.
- Package a Python wheel, Kit extension ZIP, Rust crate and source tar.zst bundles
  with provenance, version checks, SHA-256 manifests and bounded source contents.
- Add reproducible release-tool locks, CI and a tag-triggered draft-release workflow.
