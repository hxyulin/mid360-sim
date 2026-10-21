<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Changelog

## Unreleased

### Integrations

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
