<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# mid360-sim engineering contract

The core is backend-independent Rust with no default dependencies. ROS support
is optional, implemented with rclrs, isolated behind the `ros2` Cargo feature.
Never add a path/git dependency on rm-simulator or a second dynamics world.
Callers own geometry, pose interpolation, acquisition scheduling and simulation
time. Never read wall time in the core or substitute wall time for sensor stamps.

Read README.md and docs/realism.md before changing sensor assumptions. Keep
measured specifications, empirical fit errors and uncalibrated heuristics distinct.
Do not derive 905 nm reflectance from visual RGB. Preserve Livox notices and
checksummed fit provenance; don't commit the CSV, CAD or generated ROS artifacts.
New public APIs need doc comments with units. Forbid unsafe code. Retained noise
must be invariant to chunking/decimation; every first hit occludes farther ones.

Verify with `cargo fmt --all -- --check`, `cargo test --locked`, and
`cargo clippy --all-targets --locked -- -D warnings`. ROS changes also require
`pixi run test`, `pixi run smoke`, `pixi run smoke --stride 3`, and native Clippy:
`pixi run bash scripts/with-ros.sh cargo clippy --features ros2 --all-targets --locked -- -D warnings`.
Use `cargo deny --config deny.toml check` for dependency policy. Live DDS tests
currently cover macOS arm64 only; don't claim Linux runtime validation from the
cross-platform lockfile. Keep the scoped async-std maintenance exception explicit.

Commits use `type(scope): summary` with an imperative subject <=72 characters.
Update CHANGELOG.md for user-visible changes. Do not publish without user intent.


## Documentation and distribution

User guides live in `docs/guide/` and `docs/install/`; integration READMEs link
there. The VitePress site uses `/mid360-sim/` and is built with `pnpm docs:build`
and checked with `pnpm docs:check`. Keep research evidence distinct from install
steps. `scripts/check-repository.py` validates versions, model exports and Python
license copies. Release tools use Python 3.12 and `requirements-release.txt`.
`scripts/package-release.py` writes local checksummed artifacts only; it requires
a clean tree unless `--allow-dirty` explicitly marks a local preview. Inspect with
`scripts/check-release.py`. Version tags create draft releases, not registry
publication. Never package ignored datasets, runtime installs or private decoders.
