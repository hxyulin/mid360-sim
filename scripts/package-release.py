#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Build local release artifacts; never tag, upload or publish packages.

Requires Python 3.11+, Cargo and requirements-release.txt. Archives contain
allowlisted Git-visible source only; ignored data/builds and symlinks are excluded.
"""

import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import tomllib
import zipfile

import zstandard

ROOT = Path(__file__).resolve().parents[1]
LICENSES = ("LICENSE-MIT", "LICENSE-APACHE", "NOTICE.md")
DIRECTORIES = {
    ".github",
    "src",
    "examples",
    "scripts",
    "integrations",
    "interfaces",
    "docs",
}
ROOT_FILES = {
    "AGENTS.md",
    "README.md",
    "CHANGELOG.md",
    "CONTRIBUTING.md",
    "Cargo.toml",
    "Cargo.lock",
    "deny.toml",
    "pixi.toml",
    "pixi.lock",
    ".gitignore",
    "package.json",
    "pnpm-lock.yaml",
    "pnpm-workspace.yaml",
    "requirements-analysis.txt",
    "requirements-release.in",
    "requirements-release.txt",
    *LICENSES,
}


def git(*args):
    return subprocess.check_output(["git", *args], cwd=ROOT)


def source_files():
    """Use Git's ignore rules and an explicit distribution boundary."""
    names = (
        git("ls-files", "--cached", "--others", "--exclude-standard", "-z")
        .decode()
        .split("\0")
    )
    return sorted(
        {
            name
            for name in names
            if name
            and (ROOT / name).is_file()
            and not (ROOT / name).is_symlink()
            and (name in ROOT_FILES or Path(name).parts[0] in DIRECTORIES)
        }
    )


def archive(path, members, epoch):
    """Write a deterministic maximum-level Zstandard source tar."""
    with path.open("wb") as target, zstandard.ZstdCompressor(
        level=22, threads=0
    ).stream_writer(target) as compressed:
        with tarfile.open(
            fileobj=compressed, mode="w|", format=tarfile.PAX_FORMAT
        ) as tar:
            for name, data, executable in sorted(members):
                item = tarfile.TarInfo(name)
                item.size = len(data)
                item.mtime = epoch
                item.mode = 0o755 if executable else 0o644
                tar.addfile(item, io.BytesIO(data))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "dist")
    parser.add_argument(
        "--allow-dirty",
        action="store_true",
        help="local preview only; records dirty=true",
    )
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise SystemExit(
            "Output must be new; choose --output or remove only your previous generated artifacts."
        )
    dirty = bool(git("status", "--porcelain").strip())
    if dirty and not args.allow_dirty:
        raise SystemExit(
            "Commit reviewed source first, or use --allow-dirty for a local preview."
        )
    version = tomllib.loads((ROOT / "Cargo.toml").read_text())["package"]["version"]
    py = tomllib.loads((ROOT / "integrations/python/pyproject.toml").read_text())
    kit = tomllib.loads(
        (
            ROOT / "integrations/isaac-sim/exts/hxyulin.mid360/config/extension.toml"
        ).read_text()
    )
    if py["project"]["version"] != version or kit["package"]["version"] != version:
        raise SystemExit("Rust, Python and Kit versions must agree.")
    for name in LICENSES:
        if (ROOT / name).read_bytes() != (
            ROOT / "integrations/python" / name
        ).read_bytes():
            raise SystemExit(f"Python distribution notice differs: {name}")
    files = source_files()
    epoch = int(
        os.environ.get(
            "SOURCE_DATE_EPOCH",
            git("show", "-s", "--format=%ct", "HEAD").decode().strip(),
        )
    )
    env = {**os.environ, "SOURCE_DATE_EPOCH": str(epoch)}
    output.mkdir(parents=True)
    prefix = f"mid360-sim-{version}"
    members = [
        (
            f"{prefix}/{name}",
            (ROOT / name).read_bytes(),
            os.access(ROOT / name, os.X_OK),
        )
        for name in files
    ]
    archive(output / f"{prefix}-source.tar.zst", members, epoch)
    # The separate Gazebo archive retains the source-relative license paths.
    gazebo = [
        m
        for m in members
        if m[0].startswith(f"{prefix}/integrations/gazebo/")
        or m[0].split("/", 1)[1] in LICENSES
    ]
    archive(output / f"{prefix}-gazebo-source.tar.zst", gazebo, epoch)
    kit_prefix = "integrations/isaac-sim/exts/"
    with zipfile.ZipFile(
        output / f"{prefix}-isaac.zip",
        "w",
        compression=zipfile.ZIP_DEFLATED,
        compresslevel=9,
    ) as bundle:
        selected = {
            name[len(kit_prefix) :]: (ROOT / name).read_bytes()
            for name in files
            if name.startswith(kit_prefix)
        }
        for name in LICENSES:
            selected[f"hxyulin.mid360/{name}"] = (ROOT / name).read_bytes()
        selected["README.md"] = (
            b"Add the extracted directory to Isaac extension search paths. See https://hxyulin.github.io/mid360-sim/install/isaac-sim.html\n"
        )
        for name, data in sorted(selected.items()):
            info = zipfile.ZipInfo(name, date_time=(2026, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o100644 << 16
            bundle.writestr(info, data, compresslevel=9)
    # Stage only source so local setuptools caches cannot enter the wheel.
    with tempfile.TemporaryDirectory(prefix="mid360-wheel-") as tmp:
        staging = Path(tmp) / "python"
        staging.mkdir()
        for name in files:
            if name.startswith("integrations/python/"):
                relative = Path(name).relative_to("integrations/python")
                dest = staging / relative
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes((ROOT / name).read_bytes())
        subprocess.run(
            [
                sys.executable,
                "-m",
                "build",
                "--wheel",
                "--no-isolation",
                "--outdir",
                str(output),
                str(staging),
            ],
            check=True,
            env=env,
        )
    command = ["cargo", "package", "--locked"]
    if args.allow_dirty:
        command.append("--allow-dirty")
    subprocess.run(command, cwd=ROOT, check=True)
    metadata = json.loads(
        subprocess.check_output(
            ["cargo", "metadata", "--format-version", "1", "--no-deps"], cwd=ROOT
        )
    )
    shutil.copyfile(
        Path(metadata["target_directory"]) / "package" / f"mid360-sim-{version}.crate",
        output / f"mid360-sim-{version}.crate",
    )
    manifest = {
        "version": version,
        "source_revision": git("rev-parse", "HEAD").decode().strip(),
        "dirty": dirty,
        "archive_epoch": epoch,
        "zstd_version": zstandard.__version__,
        "artifacts": {},
    }
    for path in sorted(output.iterdir()):
        manifest["artifacts"][path.name] = {
            "bytes": path.stat().st_size,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        }
    (output / "release.json").write_text(json.dumps(manifest, indent=2) + "\n")
    sums = [
        f"{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.name}"
        for p in sorted(output.iterdir())
    ]
    (output / "SHA256SUMS").write_text("\n".join(sums) + "\n")
    print(f"Ready in {output}; dirty={dirty}. Nothing uploaded.")


if __name__ == "__main__":
    main()
