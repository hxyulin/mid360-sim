#!/usr/bin/env bash
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
set -eo pipefail
cd "$(dirname "$0")/.."
if [[ ! -f .ros/install/local_setup.bash ]]; then
  echo "Build interfaces first: pixi run interfaces" >&2
  exit 1
fi
source .ros/install/local_setup.bash
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# Cargo-native rclrs needs the activated C libraries at link time and runtime.
export LIBRARY_PATH="$CONDA_PREFIX/lib:$PWD/.ros/install/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$PWD/.ros/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export DYLD_FALLBACK_LIBRARY_PATH="$CONDA_PREFIX/lib:$PWD/.ros/install/lib${DYLD_FALLBACK_LIBRARY_PATH:+:$DYLD_FALLBACK_LIBRARY_PATH}"
export CARGO_TARGET_DIR="$PWD/target"
exec "$@"
