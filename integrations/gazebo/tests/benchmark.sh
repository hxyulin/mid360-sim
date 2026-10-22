#!/usr/bin/env bash
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
# Container paths match tests/run.sh. Optional old plugin directory in $1.
set -euo pipefail
python3 - <<'PY'
import xml.etree.ElementTree as ET
p = ET.parse('/workspace/source/tests/enclosure.sdf')
model = p.find(".//model[@name='sensor']")
for plugin in model.findall('plugin'):
    model.remove(plugin)
p.write('/workspace/out/baseline.sdf')
PY
for i in 1 2 3; do
  GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/out/build /workspace/out/build/native-test /workspace/out/baseline.sdf --baseline > "/workspace/out/perf-baseline-${i}.log" 2>&1
  if [[ $# -gt 0 ]]; then
    GZ_SIM_SYSTEM_PLUGIN_PATH="$1" /workspace/out/build/native-test /workspace/source/tests/enclosure.sdf --benchmark > "/workspace/out/perf-original-${i}.log" 2>&1
  fi
  GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/out/build /workspace/out/build/native-test /workspace/source/tests/enclosure.sdf --benchmark > "/workspace/out/perf-optimized-${i}.log" 2>&1
done
grep MID360_GAZEBO_RESULT /workspace/out/perf-*.log
