#!/usr/bin/env bash
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
# Run inside the supplied development container, with /workspace/out writable.
set -euo pipefail
cmake -S /workspace/source -B /workspace/out/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build /workspace/out/build -j4
ctest --test-dir /workspace/out/build --output-on-failure
export GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/out/build
for stride in 1 3 1001; do
  sed "s@</link_name>@</link_name><stride>${stride}</stride>@; s@frames.jsonl@frames-${stride}.jsonl@" /workspace/source/tests/enclosure.sdf > "/workspace/out/enclosure-${stride}.sdf"
  /workspace/out/build/native-test "/workspace/out/enclosure-${stride}.sdf" > "/workspace/out/native-${stride}.log" 2>&1
  grep MID360_GAZEBO_RESULT "/workspace/out/native-${stride}.log"
done
