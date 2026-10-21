#!/usr/bin/env bash
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
set -euo pipefail
cd "$(dirname "$0")/.."
fingerprint="$(python - <<'PYTHON'
import hashlib
from pathlib import Path
fingerprint = hashlib.sha256(str(Path.cwd().resolve()).encode())
for path in sorted(Path("interfaces").rglob("*")) + [Path("pixi.lock"), Path("scripts/build-interfaces.sh")]:
    if path.is_file():
        fingerprint.update(str(path).encode())
        fingerprint.update(path.read_bytes())
print(fingerprint.hexdigest())
PYTHON
)"
if [[ -f .ros/interfaces.sha256 && -f .ros/install/local_setup.bash ]] && \
   [[ "$(cat .ros/interfaces.sha256)" == "$fingerprint" ]]; then
  echo "Livox interfaces already built for the locked environment."
  exit 0
fi
# Recreate only the generated install tree: avoids repeated macOS install-name
# edits on already-patched binaries when the input interfaces/toolchain change.
rm -rf .ros/install
colcon --log-base .ros/log build --base-paths interfaces --merge-install \
  --build-base .ros/build --install-base .ros/install \
  --cmake-args "-DCMAKE_PREFIX_PATH=$CONDA_PREFIX" \
  "-DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python"
printf '%s\n' "$fingerprint" > .ros/interfaces.sha256
