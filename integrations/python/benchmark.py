# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Headless CPU direction+MuJoCo ray benchmark; excludes physics stepping."""

import json, statistics, time
import numpy as np
import mujoco
from mid360_sim.mujoco import MujocoSensor

model = mujoco.MjModel.from_xml_string(
    """<mujoco><worldbody><site name="lidar" pos="0 0 .8"/>
<geom type="box" pos="10 0 0" size=".1 10 10"/><geom type="box" pos="-10 0 0" size=".1 10 10"/>
<geom type="box" pos="0 10 0" size="10 .1 10"/><geom type="box" pos="0 -10 0" size="10 .1 10"/>
<geom type="box" pos="0 0 10" size="10 10 .1"/><geom type="box" pos="0 0 -10" size="10 10 .1"/>
</worldbody></mujoco>"""
)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
sensor = MujocoSensor(model, data, "lidar")
times = []
for frame in range(35):
    start = time.perf_counter()
    scan = sensor.scan(frame * 20000, 20000)
    assert scan.valid.all()
    if frame >= 5:
        times.append((time.perf_counter() - start) * 1000)
print(
    json.dumps(
        dict(
            mujoco=mujoco.__version__,
            geoms=model.ngeom,
            rays_per_batch=20000,
            median_ms=statistics.median(times),
            p95_ms=float(np.percentile(times, 95)),
            warmups=5,
            measured=30,
            scope="directions + native ray query + owned arrays; static six-wall room, no stepping/render/transport",
        ),
        indent=2,
    )
)
