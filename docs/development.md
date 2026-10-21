<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Development

Run commands from the repository root. Use Python 3.12 for repository/release
tooling (the adapter itself supports Python 3.10+). Simulator dependencies remain optional;
ordinary Rust checks do not require ROS, a GPU or another simulator.

## Rust and repository checks

```sh
cargo fmt --all -- --check
cargo test --locked
cargo clippy --all-targets --locked -- -D warnings
cargo deny --config deny.toml check
python3 scripts/check-repository.py
```

The repository check compares bundled nominal model copies against Rust exports,
checks distribution versions and verifies packaged licenses/notices. Keep source
provenance when changing derived coefficients. Do not commit captures, CAD,
build trees or private decoders; `local-assets/` is disposable ignored scratch.

## Optional runtime checks

```sh
pixi run test
pixi run smoke
pixi run smoke --stride 3
pixi run bash scripts/with-ros.sh cargo clippy --features ros2 --all-targets --locked -- -D warnings
python -m unittest discover -s integrations/python/tests -v
```

Python adapter tests require its MuJoCo extra. The Isaac and Gazebo install
guides explain native headless tests. Dataset-free motion math tests use the
NumPy/SciPy/rosbags analysis environment:

```sh
python -W error scripts/test-motion-comparison.py
python scripts/test-external-capture.py
```

## Documentation

The site uses VitePress 1.6.4, matching Layup's task-oriented guide structure.
Use Node 22+ and the pnpm version pinned in `package.json`:

```sh
pnpm install --frozen-lockfile
pnpm docs:dev
pnpm docs:build
pnpm docs:check
pnpm exec playwright install chromium
pnpm docs:test
pnpm docs:preview --host 127.0.0.1
```

The site base is `/mid360-sim/`; open that path when previewing. Local search,
sidebar navigation and installation guides are built from committed Markdown.
Do not disable dead-link checks to make the build pass. Keep research details
in the evidence pages and keep the README focused on the first useful task.

The documentation workflow builds on pull requests and deploys main through
GitHub Pages Actions. Configure **Settings → Pages → Source → GitHub Actions**
once when enabling hosting. The prepared site URL is
`https://hxyulin.github.io/mid360-sim/`; a local build is not a deployment.

## Distribution

Follow [distribution](distribution.md) for hash-locked packaging and inspection.
Version fields in Cargo, Python, Kit and CMake must agree. CI runs core checks,
Python adapter tests, a native Gazebo container and the documentation build.
Isaac's GPU tests and the documented ROS native checks require their runtime.
