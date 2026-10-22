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
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --locked
cargo deny --config deny.toml check
python scripts/check-repository.py
python scripts/test-model-checks.py
```

Deterministic adapter and articulated-scene regressions run in `cargo test`.
The same command runs public API doctests; `cargo test --doc --locked` runs
only those examples. Keep examples executable, with assertions for timing,
frames or validation behavior rather than construction alone. Public items
must have documentation, including units; rustdoc warnings should stay clean.
For the offline material/edge audit and synthetic fixtures:

```sh
python3 -m unittest discover -s examples/calibration -v
cargo run --release --locked --example packet_faults
```

The repository check compares bundled nominal model copies against Rust exports,
checks distribution versions and verifies packaged licenses/notices. Keep source
provenance when changing derived coefficients. Coefficients, clocks and emission
indices compare exactly; generated unit-vector reference components allow 1e-14
absolute cross-platform libm variation. This is a numeric export check, not an
angular fit/noise tolerance. Do not commit captures, CAD,
build trees or private decoders; `local-assets/` is disposable ignored scratch.

## Optional runtime checks

```sh
pixi run test
pixi run smoke
pixi run smoke --stride 3
pixi run bash scripts/with-ros.sh cargo clippy --features ros2 --all-targets --locked -- -D warnings
PYTHONPATH=integrations/python python -m unittest discover -s integrations/python/tests -v
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

Regenerate the result figures using `python scripts/generate-doc-figures.py`
with `requirements-analysis.txt`; `python scripts/generate-doc-figures.py --check`
checks evidence values and asset provenance without plotting dependencies.
See [the visual results guide](guide/results.md#inspect-or-regenerate).

The site base is `/mid360-sim/`; open that path when previewing. Local search,
sidebar navigation and installation guides are built from committed Markdown.
Do not disable dead-link checks to make the build pass. Keep research details
in the evidence pages and keep the README focused on the first useful task.

The documentation workflow builds on pull requests and deploys main through
GitHub Pages Actions. Configure **Settings → Pages → Source → GitHub Actions**
once when enabling hosting. If `configure-pages` fails with `Get Pages site
failed` / HTTP 404, check that setting: a successful build/artifact upload alone
does not create a site. The deploy job needs `pages: write` and `id-token: write`;
repository permissions may be needed for initial enablement. The prepared site URL is
`https://hxyulin.github.io/mid360-sim/`; a local build is not a deployment.

## Distribution

Follow [distribution](distribution.md) for hash-locked packaging and inspection.
Version fields in Cargo, Python, Kit and CMake must agree. CI runs core checks,
Python adapter tests, a native Gazebo container and the documentation build.
Isaac's GPU tests and the documented ROS native checks require their runtime.

## Documentation maintenance

Keep current API capabilities in the installation/acquisition guides, accuracy
metric definitions in the validation guide, and experiments in evidence pages.
The [runtime/accuracy report](performance-tradeoffs.md) is the current comparison;
[Isaac architecture experiments](isaac-performance.md) preserve earlier scopes.
When a native experiment supersedes an offline proxy, update the landing guides
and the original page's summary; label the old result rather than erasing it.
Separate physical specifications, optical fit errors, motion-policy displacement,
native synthetic-scene range errors and whole-application throughput.

Run the production build, link check and browser checks above after edits. These
verify navigation, local search and mobile layout. Also compare code examples
and mode limits against the current implementation; passing link checks cannot
establish semantic freshness. Document source versus release availability without
implying that a local candidate artifact has been uploaded.

Native Isaac performance investigations can use `tests/history_benchmark.py`
for alternating source comparisons and `tests/arena.py` for explicit worker/
SDK stepping choices. Both live under `integrations/isaac-sim/`. The latter
records complete range hashes so a faster caller loop can be checked against
the same acquired output. See the [measurement protocol](performance-tradeoffs.md#further-pass-packed-metadata-and-the-actual-arena-loop).
Keep profiler timings separate from unprofiled runs and retain runtime/geometry
scope when publishing results.
