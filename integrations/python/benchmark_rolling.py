# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Interleaved complete rolling acquisitions using supported Python ray APIs.

Includes physics, refreshed geometry, pose capture, optics, queries, owned frames
and 100 ms field assembly. No rendering or transport. Serializes timing runs
with the parent investigation's exclusive advisory lock; never uses raw pointers.
"""

import argparse
import fcntl
import json
import platform
import time
from pathlib import Path

import mujoco
import numpy as np
from mid360_sim.mujoco import MujocoSensor


def run(steps, sensors, stride, clutter, speed, mode):
    walls = []
    for axis in range(3):
        for sign in (-1, 1):
            position, size = [0.0] * 3, [30.1] * 3
            position[axis], size[axis] = sign * 30.1, 0.1
            walls.append(
                f'<geom type="box" pos="{" ".join(map(str, position))}" '
                f'size="{" ".join(map(str, size))}"/>'
            )
    for i in range(128 if clutter else 0):
        walls.append(
            f'<geom type="box" pos="{3 + i % 16} '
            f'{(i // 16 - 3.5) * 2} 1" size=".1 .4 1"/>'
        )
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><option timestep=".001" gravity="0 0 0"/>'
        "<worldbody>"
        + "".join(walls)
        + '<body pos="0 0 .8"><freejoint/><geom type="sphere" size=".05" '
        'mass="1" group="1" contype="0" conaffinity="0"/>'
        '<site name="a"/><site name="b" pos="0 .2 0"/></body>'
        "</worldbody></mujoco>"
    )
    data = mujoco.MjData(model)
    mounted = [
        MujocoSensor(model, data, name, geom_groups=[1, 0, 0, 0, 0, 0])
        for name in ("a", "b")[:sensors]
    ]
    count = len(range(0, 200, stride))
    pending = [[] for _ in mounted]
    points = messages = 0
    timings = []

    def acquire(i, assemble):
        nonlocal points, messages
        positions = data.site_xpos[:sensors].copy()
        q0 = data.qpos[[4, 5, 6, 3]].copy()
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        q1 = data.qpos[[4, 5, 6, 3]].copy()
        for j, sensor in enumerate(mounted):
            frame = sensor.scan_rolling(
                i * 200,
                count,
                (positions[j], q0),
                (data.site_xpos[j], q1),
                stride=stride,
                pose_fraction=(0, (count - 1) * stride / 200),
                query_mode=mode,
            )
            if assemble:
                fields = frame.livox_fields()
                fields["offset_time"] += (i % 100) * 1_000_000
                pending[j].append(fields)
                if (i + 1) % 100 == 0:
                    cloud = {
                        key: np.concatenate([p[key] for p in pending[j]])
                        for key in (
                            "xyz_m",
                            "offset_time",
                            "line",
                            "tag",
                            "reflectivity",
                        )
                    }
                    assert len(cloud["xyz_m"]) == count * 100
                    assert (
                        cloud["offset_time"][-1]
                        == 99_000_000 + (count - 1) * stride * 5000
                    )
                    points += len(cloud["xyz_m"])
                    messages += 1
                    pending[j].clear()

    data.qvel[:] = [speed, 0, 0, 0, 0, 0.5]
    mujoco.mj_forward(model, data)
    for i in range(100):
        acquire(i, False)
    mujoco.mj_resetData(model, data)
    data.qvel[:] = [speed, 0, 0, 0, 0, 0.5]
    mujoco.mj_forward(model, data)
    start = time.perf_counter()
    for i in range(steps):
        before = time.perf_counter()
        acquire(i, True)
        timings.append((time.perf_counter() - before) * 1000)
    elapsed = time.perf_counter() - start
    assert np.isclose(data.time, steps * 0.001)
    assert points == sensors * steps * count
    return {
        "mode": mode,
        "wall_s": elapsed,
        "step_median_ms": float(np.median(timings)),
        "step_p95_ms": float(np.percentile(timings, 95)),
        "steps_over_1ms": int(np.sum(np.asarray(timings) > 1)),
        "points": points,
        "messages": messages,
        "ngeom": model.ngeom,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=1000)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--lock",
        type=Path,
        default=Path(__file__).resolve().parents[2]
        / "local-assets/perf-investigation.lock",
    )
    args = parser.parse_args()
    if not 100 <= args.steps <= 1000 or args.steps % 100 or not 1 <= args.repeats <= 10:
        parser.error("steps must be a multiple of 100 in 100..1000; repeats 1..10")
    report = {
        "mujoco": mujoco.__version__,
        "numpy": np.__version__,
        "machine": platform.platform(),
        "steps": args.steps,
        "dt_s": 0.001,
        "scope": "physics, capture, rolling query, owned output, 100 ms assembly; no transport",
        "exact_implementation": "public Python mj_ray per emission, NOT native fused loop",
        "cases": [],
    }
    # The exclusive lock covers every local timing run, including warmup.
    args.lock.parent.mkdir(parents=True, exist_ok=True)
    with args.lock.open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        for clutter in (False, True):
            for speed in (20.0, 0.1):
                for sensors in (1, 2):
                    for stride in (1, 3):
                        case = {
                            "clutter": clutter,
                            "speed_m_s": speed,
                            "sensors": sensors,
                            "stride": stride,
                            "rays": len(range(0, 200, stride)),
                            "runs": [],
                        }
                        for repeat in range(args.repeats):
                            modes = (
                                ("grouped", "exact")
                                if repeat % 2 == 0
                                else ("exact", "grouped")
                            )
                            for mode in modes:
                                case["runs"].append(
                                    run(
                                        args.steps,
                                        sensors,
                                        stride,
                                        clutter,
                                        speed,
                                        mode,
                                    )
                                )
                        report["cases"].append(case)
                        print(
                            json.dumps({k: v for k, v in case.items() if k != "runs"}),
                            flush=True,
                        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
