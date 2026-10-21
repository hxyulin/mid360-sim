<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Distribution

Version 0.1.0 is the first multi-simulator release candidate. The repository is
public; release artifacts are prepared locally before a tag is published.
See [GitHub releases](https://github.com/hxyulin/mid360-sim/releases) for assets
that have actually been uploaded. There are no PyPI or crates.io packages yet.

## Choose an artifact

| Artifact | Contents | Requirements |
|---|---|---|
| `mid360-sim-0.1.0-source.tar.zst` | Complete source, guides, ROS interfaces and lockfiles | Rust; Pixi for ROS; Node/pnpm for docs |
| `mid360_sim_adapters-0.1.0-py3-none-any.whl` | NumPy pattern + optional MuJoCo adapter and model JSON | Python 3.10+; MuJoCo 3.7+ for that adapter |
| `mid360-sim-0.1.0-isaac.zip` | `hxyulin.mid360` Kit extension | Isaac Sim 6.2 with Warp and GPU support |
| `mid360-sim-0.1.0-gazebo-source.tar.zst` | C++ plugin, fixtures, Docker recipe and notices | Harmonic SDK; DART/Bullet |
| `mid360-sim-0.1.0.crate` | Rust source package | Stable Rust; ROS environment for optional `ros2` |
| `SHA256SUMS`, `release.json` | Artifact hashes, source revision, dirty-preview flag and sizes | SHA-256 tool |

Gazebo native binaries are tied to its ABI and runtime libraries. The source
bundle avoids implying that one Linux `.so` is portable across simulator setups.
The ROS publisher also depends on generated interfaces and dynamic ROS libraries;
use the complete source bundle and Pixi rather than copying a bare executable.
No simulator engine, CAD, raw capture, fitted analysis environment or dataset is
redistributed. Each package carries applicable licenses and provenance.

## Verify and install

Download artifacts and `SHA256SUMS` from the same release. On Linux use
`sha256sum --check SHA256SUMS`; on macOS use `shasum -a 256 --check SHA256SUMS`.
Only download a complete set if checking the complete list; otherwise check the
hash of your selected artifact against its entry.

Extract source with a Zstandard-capable tar (`tar --zstd -xf FILE` on GNU tar).
The complete archive has one `mid360-sim-0.1.0/` root. Run source commands from
there. The focused Gazebo archive retains `integrations/gazebo/` under that root.

For the Python wheel, in a virtual environment:

```sh
python -m pip install 'mid360_sim_adapters-0.1.0-py3-none-any.whl[mujoco]'
```

For Isaac, unzip into a writable extension folder and add the extracted parent
to Kit's extension search paths. Enable `hxyulin.mid360`; follow the
[Isaac guide](install/isaac-sim.md). Other install guides start from
[getting started](guide/getting-started.md).

## Prepare a release

Run release preparation in a Git checkout, using Python 3.12, Cargo and the
hash-locked release tools. Consumer source builds do not require Git:

```sh
uv venv --python 3.12 local-assets/release-venv
uv pip sync --python local-assets/release-venv/bin/python --require-hashes requirements-release.txt
local-assets/release-venv/bin/python scripts/check-repository.py
local-assets/release-venv/bin/python scripts/package-release.py --allow-dirty
local-assets/release-venv/bin/python scripts/check-release.py dist
```

`--allow-dirty` is for a local preview and records `dirty: true`; omit it for a
reviewed commit. Output must be a new directory. The packager never uploads,
tags or publishes. Source tarballs use Zstandard level 22; tar metadata and Kit
ZIP metadata are normalized. Wheel build dependencies are hash-locked, and
`SOURCE_DATE_EPOCH` comes from the source commit unless explicitly supplied.
This controls avoidable variability; it is not a cross-toolchain reproducible
build certification.

The release workflow runs on a `v*` tag, checks that its version matches the
source and creates a **draft** GitHub release. Review its artifacts and notes
before publishing. It never publishes to package registries.
