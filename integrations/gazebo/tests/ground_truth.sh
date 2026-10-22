#!/usr/bin/env bash
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
# In the development container, after tests/run.sh. Caller supplies the ignored
# /workspace/out/mocap-fixture.csv from check-backend-ground-truth.py prepare.
set -euo pipefail
step_us=${1:-1000}
fixture=${2:-/workspace/out/mocap-fixture.csv}
if [[ "$step_us" != 1000 && "$step_us" != 200 && "$step_us" != 50 ]]; then exit 2; fi
python3 - "$step_us" "$fixture" <<'PY' 
import xml.etree.ElementTree as ET
import sys
p = ET.parse('/workspace/source/tests/enclosure.sdf')
world = p.getroot().find('world')
world.find('physics/max_step_size').text = f"{int(sys.argv[1]) / 1000000:.6f}"
ET.SubElement(world, 'gravity').text = '0 0 0'
for model in world.findall('model'):
    if model.attrib['name'] == 'sensor':
        model.find('static').text = 'false'
        plugin = ET.SubElement(model, 'plugin', filename='replay-test', name='mid360::Replay')
        ET.SubElement(plugin, 'poses_file').text = sys.argv[2]
    else:
        # The ordinary enclosure test has +/-9.9 m inner faces. The shared
        # replay reference has +/-10 m inner faces and overlapping wall edges.
        pose = list(map(float, model.find('pose').text.split()))
        pose[:3] = [value * 1.01 for value in pose[:3]]
        model.find('pose').text = ' '.join(map(str, pose))
        size = model.find('.//box/size')
        size.text = ' '.join('20.2' if float(value) == 20 else value for value in size.text.split())
p.write('/workspace/out/mocap.sdf')
PY
export GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/out/build
/workspace/out/build/native-test /workspace/out/mocap.sdf \
  --replay-output=/workspace/out/mocap-gazebo.bin --iterations="$((2000000 / step_us))" > /workspace/out/mocap.log 2>&1
grep MID360_GAZEBO_RESULT /workspace/out/mocap.log
