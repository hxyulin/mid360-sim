#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Alternate native held/rolling queries against a retained adapter source file.

Requires the optional MuJoCo adapter environment. No physics stepping or ROS;
compare complete owned outputs before timing. Baseline must be trusted Python.
"""

import argparse
import hashlib
import importlib.util
import json
import statistics
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "integrations/python"))
from mid360_sim.mujoco import MujocoSensor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument(
        "--candidate",
        type=Path,
        help="Trusted candidate source; defaults to this checkout",
    )
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location("mid360_sim._baseline", args.baseline)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    model = mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
      <site name="lidar" pos="0 0 .8"/>
      <geom type="box" pos="5 0 0" size=".1 20 20"/>
      <geom type="box" pos="-5 0 0" size=".1 20 20"/>
      </worldbody></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    candidate_class = MujocoSensor
    if args.candidate is not None:
        spec = importlib.util.spec_from_file_location(
            "mid360_sim._candidate", args.candidate
        )
        candidate_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(candidate_module)
        candidate_class = candidate_module.MujocoSensor
    sensors = {
        "baseline": module.MujocoSensor(model, data, "lidar"),
        "candidate": candidate_class(model, data, "lidar"),
    }
    results = []
    calls = 200
    for count, stride in [(200, 1), (67, 3), (2000, 1), (20000, 1)]:
        for rolling in [False, True]:

            def acquire(sensor, index, count=count, stride=stride, rolling=rolling):
                first = index * count * stride
                if not rolling:
                    return sensor.scan(first, count, stride=stride)
                angle = index * 0.001
                p0 = [np.sin(index * 0.01) * 0.1, 0.0, 0.8]
                p1 = [p0[0] + 0.0001, 0.0001, 0.8]
                q0 = [0.0, 0.0, np.sin(angle / 2), np.cos(angle / 2)]
                q1 = [
                    0.0,
                    0.0,
                    np.sin((angle + 0.005) / 2),
                    np.cos((angle + 0.005) / 2),
                ]
                return sensor.scan_rolling(
                    first, count, (p0, q0), (p1, q1), stride=stride
                )

            for index in range(calls):
                old, new = (
                    acquire(sensors[name], index) for name in ["baseline", "candidate"]
                )
                for attr in ["directions", "ranges_m", "geom_ids"]:
                    a, b = getattr(old, attr), getattr(new, attr)
                    assert a.dtype == b.dtype and np.array_equal(
                        a.view(np.uint8), b.view(np.uint8)
                    ), attr
            samples = {name: [] for name in sensors}
            for trial in range(9):
                order = (
                    ["baseline", "candidate"]
                    if trial % 2 == 0
                    else ["candidate", "baseline"]
                )
                for name in order:
                    start = time.perf_counter()
                    for index in range(calls):
                        acquire(sensors[name], index)
                    elapsed = (time.perf_counter() - start) * 1000.0
                    if trial >= 2:
                        samples[name].append(elapsed)
            results.append(
                {
                    "count": count,
                    "stride": stride,
                    "mode": "rolling" if rolling else "held",
                    "calls": calls,
                    "equal_bits": True,
                    "samples_ms": samples,
                    "median_ms": {
                        name: statistics.median(values)
                        for name, values in samples.items()
                    },
                }
            )
    print(
        json.dumps(
            {
                "mujoco": mujoco.__version__,
                "numpy": np.__version__,
                "baseline_sha256": hashlib.sha256(
                    args.baseline.read_bytes()
                ).hexdigest(),
                "candidate_sha256": hashlib.sha256(
                    (
                        args.candidate
                        or ROOT / "integrations/python/mid360_sim/mujoco.py"
                    ).read_bytes()
                ).hexdigest(),
                "protocol": "Two warmups then seven alternating-order trials; static two-box scene; no physics stepping, ROS or GPU.",
                "results": results,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
