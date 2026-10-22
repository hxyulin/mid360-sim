#!/usr/bin/env bash
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
# Caller supplies pinned upstream source and generated 2001-row pose CSV.
set -euo pipefail
if [ "$#" -ne 3 ]; then
  echo 'usage: run.sh UPSTREAM_SOURCE OUTPUT_DIRECTORY POSES_CSV' >&2
  exit 2
fi
source_dir=$(cd "$1" && pwd)
mkdir -p "$2"
output_dir=$(cd "$2" && pwd)
pose_file=$(cd "$(dirname "$3")" && pwd)/$(basename "$3")
harness_dir=$(cd "$(dirname "$0")" && pwd)
python3 "$harness_dir/prepare.py" --output "$output_dir" --min-range 0
image='osrf/ros:melodic-desktop-full@sha256:39b2900892a32886033f168e55fd8ce4e01d804ed1ffdde156639b3cf789ee71'
# Copy source before changing the obsolete protobuf library spelling. The
# upstream .cpp/.h algorithm files remain untouched, with hashes recorded.
docker run --rm -i --network none -v "$source_dir:/input:ro" \
  -v "$harness_dir:/workspace/harness:ro" -v "$output_dir:/workspace/out" \
  -v "$pose_file:/workspace/poses.csv:ro" "$image" bash -s <<'INNER'
set -eo pipefail
source /opt/ros/melodic/setup.bash
cp -a /input /workspace/source
cd /workspace/source
printf '%s\n' \
 '93ee98d13fb67e248574d0d77f633c64b54900dbee4a84385efa329fccad1c02  src/livox_points_plugin.cpp' \
 'ebff472657913be2c4a649c9958921809d8a0ff8d71851a47c0a52b28eb7286f  src/livox_ode_multiray_shape.cpp' | sha256sum --check
sha256sum /workspace/source/src/* /workspace/source/include/livox_laser_simulation/* > /workspace/out/source-sha256.txt
sed -i 's/libprotobuf.so.9/protobuf/g' /workspace/source/CMakeLists.txt
mkdir -p /workspace/out/build /workspace/out/harness-build
cd /workspace/out/build
cmake -DCMAKE_BUILD_TYPE=Release /workspace/source
make -j4
cd /workspace/out/harness-build
cmake -DCMAKE_BUILD_TYPE=Release /workspace/harness
make -j4
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1
roscore > /workspace/out/roscore.log 2>&1 &
ready=false
for attempt in $(seq 1 100); do
 if rosparam set /use_sim_time true 2>/dev/null; then ready=true; break; fi
 sleep .1
done
$ready || { cat /workspace/out/roscore.log; exit 1; }
for trial in 1 2 3; do
 ./official-reference /workspace/out/world.sdf /workspace/out/plugin.sdf /workspace/poses.csv \
   /workspace/out/official-$trial.bin > /workspace/out/official-$trial.log 2>&1
 grep OFFICIAL_RESULT /workspace/out/official-$trial.log
done
INNER
