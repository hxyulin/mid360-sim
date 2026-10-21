#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Check release versions, provenance copies and regenerated optical models."""

import json
from pathlib import Path
import re
import subprocess
import tomllib

ROOT = Path(__file__).resolve().parents[1]
version = tomllib.loads((ROOT / "Cargo.toml").read_text())["package"]["version"]
assert (
    tomllib.loads((ROOT / "integrations/python/pyproject.toml").read_text())["project"][
        "version"
    ]
    == version
)
assert (
    tomllib.loads(
        (
            ROOT / "integrations/isaac-sim/exts/hxyulin.mid360/config/extension.toml"
        ).read_text()
    )["package"]["version"]
    == version
)
assert (
    f"project(mid360_gazebo VERSION {version} LANGUAGES CXX)"
    in (ROOT / "integrations/gazebo/CMakeLists.txt").read_text()
)
for name in ["LICENSE-MIT", "LICENSE-APACHE", "NOTICE.md"]:
    assert (ROOT / name).read_bytes() == (
        ROOT / "integrations/python" / name
    ).read_bytes(), f"Python notice differs: {name}"
model = subprocess.check_output(
    ["cargo", "run", "--quiet", "--locked", "--example", "export_isaac_model"], cwd=ROOT
)
for name in [
    "integrations/python/mid360_sim/nominal.json",
    "integrations/isaac-sim/exts/hxyulin.mid360/mid360/isaac/nominal.json",
]:
    assert json.loads(model) == json.loads(
        (ROOT / name).read_bytes()
    ), f"Stale model: {name}"
header = subprocess.check_output(
    ["cargo", "run", "--quiet", "--locked", "--example", "export_gazebo_model"],
    cwd=ROOT,
)
assert (
    header == (ROOT / "integrations/gazebo/include/mid360/model.hh").read_bytes()
), "Stale Gazebo header"
assert re.fullmatch(r"\d+\.\d+\.\d+(?:-[\w.]+)?", version)
print(f"Repository versions, licenses and generated models agree ({version}).")
