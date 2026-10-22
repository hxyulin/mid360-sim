#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Inspect release hashes and required runtime/provenance archive contents."""

import argparse
import hashlib
import json
from pathlib import Path
import tarfile
import zipfile

import zstandard

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("directory", type=Path)
args = ap.parse_args()
root = args.directory
manifest = json.loads((root / "release.json").read_text())
v = manifest["version"]
prefix = f"mid360-sim-{v}"
required = {
    f"{prefix}-source.tar.zst",
    f"{prefix}-gazebo-source.tar.zst",
    f"{prefix}-isaac.zip",
    f"{prefix}.crate",
    f"mid360_sim_adapters-{v}-py3-none-any.whl",
}
assert set(manifest["artifacts"]) == required, "Incomplete/unexpected artifact set"
assert {p.name for p in root.iterdir()} == required | {"release.json", "SHA256SUMS"}
checksum_names = []
for line in (root / "SHA256SUMS").read_text().splitlines():
    digest, name = line.split("  ", 1)
    assert Path(name).name == name
    checksum_names.append(name)
    assert hashlib.sha256((root / name).read_bytes()).hexdigest() == digest, name
assert len(checksum_names) == len(set(checksum_names)), "Duplicate checksum entry"
assert set(checksum_names) == required | {"release.json"}, "Incomplete checksum set"
for name, meta in manifest["artifacts"].items():
    data = (root / name).read_bytes()
    assert (
        len(data) == meta["bytes"]
        and hashlib.sha256(data).hexdigest() == meta["sha256"]
    ), name
for path in root.glob("*.tar.zst"):
    with path.open("rb") as f, zstandard.ZstdDecompressor().stream_reader(
        f
    ) as stream, tarfile.open(fileobj=stream, mode="r|") as archive:
        names = []
        for member in archive:
            parts = Path(member.name).parts
            assert ".." not in parts and not member.name.startswith("/")
            assert not member.issym() and not member.islnk()
            assert not {
                "local-assets",
                "node_modules",
                "__pycache__",
                ".pixi",
                ".ros",
                "target",
            }.intersection(parts), member.name
            assert not member.name.endswith(
                (".bin", ".bag", ".db3", ".lvx2", ".step", ".stp")
            ), member.name
            names.append(member.name)
    assert f"{prefix}/LICENSE-MIT" in names and f"{prefix}/NOTICE.md" in names
    assert f"{prefix}/integrations/gazebo/CMakeLists.txt" in names
    if path.name == f"{prefix}-source.tar.zst":
        for name in (
            "src/transport.rs",
            "src/sensor/articulated_tests.rs",
            "examples/packet_faults.rs",
            "examples/calibration/evaluate.py",
            "examples/calibration/synthetic.py",
            "examples/calibration/test_evaluate.py",
            "docs/release-notes.md",
            "requirements-release.txt",
            "scripts/model_checks.py",
            "scripts/test-model-checks.py",
        ):
            assert f"{prefix}/{name}" in names, name
with zipfile.ZipFile(root / f"{prefix}-isaac.zip") as z:
    assert "hxyulin.mid360/config/extension.toml" in z.namelist()
    assert "hxyulin.mid360/mid360/isaac/nominal.json" in z.namelist()
    assert "hxyulin.mid360/NOTICE.md" in z.namelist()
wheel = next(root.glob("*.whl"))
with zipfile.ZipFile(wheel) as z:
    assert "mid360_sim/nominal.json" in z.namelist()
    for suffix in ["LICENSE-MIT", "LICENSE-APACHE", "NOTICE.md"]:
        assert any(n.endswith("/licenses/" + suffix) for n in z.namelist()), suffix
    assert all(
        "__pycache__" not in n and not n.startswith("tests/") for n in z.namelist()
    )
with tarfile.open(root / f"{prefix}.crate", "r:gz") as z:
    names = z.getnames()
    allowed = {
        ".cargo_vcs_info.json",
        "Cargo.toml",
        "Cargo.toml.orig",
        "Cargo.lock",
        "README.md",
        "LICENSE-MIT",
        "LICENSE-APACHE",
        "NOTICE.md",
    }
    for name in names:
        relative = name.removeprefix(prefix + "/")
        assert "__pycache__" not in Path(relative).parts and not relative.endswith(".pyc"), name
        assert relative in allowed or relative.startswith(("src/", "examples/")), name
    assert f"{prefix}/src/nominal_data.rs" in names
    assert f"{prefix}/NOTICE.md" in names
    for name in (
        "src/transport.rs",
        "src/sensor/articulated_tests.rs",
        "examples/packet_faults.rs",
        "examples/calibration/evaluate.py",
        "examples/calibration/test_evaluate.py",
    ):
        assert f"{prefix}/{name}" in names, name
print(f'All release hashes and package boundaries pass; dirty={manifest["dirty"]}.')
