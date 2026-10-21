<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Contributing

Bug reports, simulator fixtures and acquisition improvements are welcome.
Include the adapter/runtime version, sample indices, timestamps and a minimal
scene. For motion issues, describe the physics step and acquisition batch size.

Read [development](docs/development.md) for setup and checks. Keep the Rust core
backend-independent and default-dependency-free. Adapters use the caller's
existing scene; do not introduce another dynamics world. Preserve local raw
points, misses and emission times rather than pre-deskewing output.

Distinguish hardware specifications, fitted errors and simulation choices.
Record source hashes and provenance for model changes; don't upload recordings
or private decoders. User-visible changes go under Unreleased in CHANGELOG.md.

Use `type(scope): summary` commit subjects of at most 72 characters. Validate
the affected runtime and documentation before proposing a change. Distribution
scripts prepare local files; tags, uploads and registry publication are separate
release actions. See [AGENTS.md](AGENTS.md) for the engineering contract.
