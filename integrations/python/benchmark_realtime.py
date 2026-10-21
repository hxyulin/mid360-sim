# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""1 kHz live-physics acquisition; compare with a physics-only baseline."""

import argparse
import json
import time
from pathlib import Path

import mujoco
import numpy as np
from mid360_sim.mujoco import MujocoSensor


def run(steps, sensing):
    """Step one moving mounted sensor; no rendering, sleeping or ROS transport."""
    model = mujoco.MjModel.from_xml_string(
        """<mujoco><option timestep=".001" gravity="0 0 0"/>
<worldbody><body pos="0 0 .8"><freejoint/><geom type="sphere" size=".05"
 mass="1" group="1" contype="0" conaffinity="0"/><site name="lidar"/></body>
<geom type="box" pos="10 0 0" size=".1 10 10"/><geom type="box" pos="-10 0 0" size=".1 10 10"/>
<geom type="box" pos="0 10 0" size="10 .1 10"/><geom type="box" pos="0 -10 0" size="10 .1 10"/>
<geom type="box" pos="0 0 10" size="10 10 .1"/><geom type="box" pos="0 0 -10" size="10 10 .1"/>
</worldbody></mujoco>"""
    )
    data = mujoco.MjData(model)
    data.qvel[:] = [0.1, 0, 0, 0, 0, 0.5]
    mujoco.mj_forward(model, data)
    sensor = MujocoSensor(model, data, "lidar", geom_groups=[1, 0, 0, 0, 0, 0])
    # Warm the actual small-batch path, then reset both physics and emission phase.
    for i in range(100):
        mujoco.mj_step(model, data)
        if sensing:
            sensor.scan(i * 200, 200)
    mujoco.mj_resetData(model, data)
    data.qvel[:] = [0.1, 0, 0, 0, 0, 0.5]
    mujoco.mj_forward(model, data)
    timings, pending, messages, points = [], [], 0, 0
    start = time.perf_counter()
    for i in range(steps):
        before = time.perf_counter()
        mujoco.mj_step(model, data)
        # mj_step integrates qpos after computing site transforms: refresh these
        # before querying to avoid a one-step-old mounted pose/geometry capture.
        mujoco.mj_forward(model, data)
        if sensing:
            frame = sensor.scan(i * 200, 200)
            assert frame.valid.all()
            assert int(frame.timestamps_ns[0]) == i * 1_000_000
            fields = frame.livox_fields()
            fields["offset_time"] += (i % 100) * 1_000_000
            pending.append(fields)
            if (i + 1) % 100 == 0:
                cloud = {
                    key: np.concatenate([p[key] for p in pending])
                    for key in ("xyz_m", "offset_time", "line", "tag", "reflectivity")
                }
                assert len(cloud["xyz_m"]) == 20_000
                assert int(cloud["offset_time"][-1]) == 99_995_000
                points += len(cloud["xyz_m"])
                messages += 1
                pending.clear()
        timings.append((time.perf_counter() - before) * 1000)
    elapsed = time.perf_counter() - start
    assert np.isclose(data.site_xpos[0, 0], steps * 0.001 * 0.1)
    return dict(
        wall_s=elapsed,
        simulated_s=steps * 0.001,
        real_time_factor=steps * 0.001 / elapsed,
        step_median_ms=float(np.median(timings)),
        step_p95_ms=float(np.percentile(timings, 95)),
        steps_over_1ms=int(np.sum(np.array(timings) > 1)),
        messages=messages,
        points=points,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=2000)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 100 <= args.steps <= 10000 or args.steps % 100:
        parser.error("steps must be a multiple of 100 in 100..10000")
    report = dict(
        mujoco=mujoco.__version__,
        physics_dt_s=0.001,
        rays_per_step=200,
        publish_period_s=0.1,
        scope="moving mount, physics + capture + arrays; no rendering/transport",
        baseline=run(args.steps, False),
        sensor=run(args.steps, True),
        checks="motion, counts, contiguous emission times, 100 ms aggregation",
    )
    result = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(result)
    print(result)


if __name__ == "__main__":
    main()
