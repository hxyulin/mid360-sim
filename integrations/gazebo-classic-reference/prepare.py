# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Generate matched-count native Gazebo 9 reference fixtures; no vendor assets."""

import argparse
from pathlib import Path

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--output", type=Path, required=True)
p.add_argument("--min-range", type=float, choices=(0.0, 0.1), default=0.0)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
walls = []
for axis in range(3):
    for sign in (-1, 1):
        pos = [0.0, 0.0, 0.0]
        pos[axis] = sign * 10.1
        size = [20.2] * 3
        size[axis] = 0.2
        walls.append(
            f'<model name="wall{axis}_{sign}"><static>true</static><pose>{" ".join(map(str,pos))} 0 0 0</pose><link name="wall"><collision name="shape"><geometry><box><size>{" ".join(map(str,size))}</size></box></geometry></collision></link></model>'
        )
ray = "<ray><scan><horizontal><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></horizontal></scan><range><min>0.1</min><max>70</max><resolution>0.001</resolution></range></ray>"
sensor = f'<sensor name="lidar" type="ray"><always_on>false</always_on><update_rate>10</update_rate><visualize>false</visualize>{ray}</sensor>'
world = (
    '<sdf version="1.6"><world name="reference"><gravity>0 0 0</gravity><physics type="ode" name="ode"><max_step_size>0.001</max_step_size><real_time_update_rate>0</real_time_update_rate></physics><model name="sensor"><pose>0 0 .8 0 0 0</pose><link name="lidar"><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>'
    + sensor
    + "</link></model>"
    + "".join(walls)
    + "</world></sdf>"
)
# Upstream plugin scan dimensions, matched 20k samples instead of 24k, zero noise.
plugin = """<sdf version="1.6"><model name="config"><link name="lidar"><sensor name="lidar" type="ray"><plugin name="official" filename="liblivox_laser_simulation.so"><samples>20000</samples><downsample>1</downsample><csv_file_name>/workspace/source/scan_mode/mid360.csv</csv_file_name><ros_topic>mid360_points</ros_topic><ray><scan><horizontal><samples>100</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>6.283185307179586</max_angle></horizontal><vertical><samples>360</samples><resolution>1</resolution><min_angle>-0.12601277</min_angle><max_angle>0.96377</max_angle></vertical></scan><range><min>0.1</min><max>70</max><resolution>0.002</resolution></range><noise><type>gaussian</type><mean>0</mean><stddev>0</stddev></noise></ray></plugin></sensor></link></model></sdf>"""
(a.output / "world.sdf").write_text(world)
(a.output / "plugin.sdf").write_text(
    plugin.replace("<min>0.1</min>", f"<min>{a.min_range}</min>")
)
