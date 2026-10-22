#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Compare native scalar/grouped MuJoCo queries for one or two sensors.

Query-only timings exclude direction/pose generation and physics. Rotation is
per emission; grouped translation uses the adapter's midpoint-origin policy.
Both timed paths use identical grouped origins. Exact-origin reference results
separately quantify that approximation in these synthetic scenes.
"""

import hashlib
import json
import platform
import statistics
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "integrations/python"))
from mid360_sim.pattern import directions


def main():
    results = []
    for clutter in [0, 128]:
        boxes = "".join(
            f'<geom type="box" pos="{(i % 16 - 7.5) * 0.5} '
            f'{(i // 16 - 3.5) * 0.5} 1" size=".04 .04 .5"/>'
            for i in range(clutter)
        )
        model = mujoco.MjModel.from_xml_string(
            '<mujoco><worldbody><geom type="plane" size="10 10 .1"/>'
            '<geom type="box" pos="5 0 1" size=".1 10 10"/>'
            '<geom type="box" pos="-5 0 1" size=".1 10 10"/>'
            '<geom type="box" pos="0 5 1" size="10 .1 10"/>'
            '<geom type="box" pos="0 -5 1" size="10 .1 10"/>'
            '<body mocap="true" pos="2 0 1">'
            '<geom type="box" size=".1 .5 .5"/></body>'
            + boxes
            + "</worldbody></mujoco>"
        )
        data = mujoco.MjData(model)
        for count, stride in [(200, 1), (67, 3)]:
            for displacement in [0.0, 0.002, 0.02]:
                for devices in [1, 2]:
                    t = np.linspace(0.0, 1.0, count)
                    local = directions(7, count, stride)
                    queries = []
                    groups = max(1, int(np.ceil(displacement / 0.001)))
                    for device in range(devices):
                        angle = 0.01 * t + device * 0.3
                        c, s = np.cos(angle), np.sin(angle)
                        world = np.column_stack(
                            (
                                c * local[:, 0] - s * local[:, 1],
                                s * local[:, 0] + c * local[:, 1],
                                local[:, 2],
                            )
                        )
                        origins = np.column_stack(
                            (
                                displacement * t,
                                np.full(count, device * 0.2),
                                np.full(count, 0.8),
                            )
                        )
                        grouped = origins.copy()
                        spans = []
                        for group in range(groups):
                            a, b = (
                                group * count // groups,
                                (group + 1) * count // groups,
                            )
                            grouped[a:b] = (origins[a] + origins[b - 1]) * 0.5
                            spans.append((a, b))
                        queries.append((world, origins, grouped, spans))
                    ranges = np.empty((devices, count))
                    ids = np.empty((devices, count), dtype=np.int32)
                    one_id = np.empty(1, dtype=np.int32)

                    def acquire(
                        batch,
                        exact=False,
                        *,
                        queries=queries,
                        model=model,
                        data=data,
                        ids=ids,
                        ranges=ranges,
                        one_id=one_id,
                        count=count,
                    ):
                        for device, (world, origins, grouped, spans) in enumerate(
                            queries
                        ):
                            if batch:
                                for a, b in spans:
                                    mujoco.mj_multiRay(
                                        model,
                                        data,
                                        grouped[a],
                                        world[a:b].ravel(),
                                        None,
                                        1,
                                        -1,
                                        ids[device, a:b],
                                        ranges[device, a:b],
                                        None,
                                        b - a,
                                        70.0,
                                    )
                            else:
                                for i in range(count):
                                    ranges[device, i] = mujoco.mj_ray(
                                        model,
                                        data,
                                        (origins if exact else grouped)[i],
                                        world[i],
                                        None,
                                        1,
                                        -1,
                                        one_id,
                                    )
                                    ids[device, i] = one_id[0]
                        return ranges.copy(), ids.copy()

                    samples = {"scalar": [], "batch": []}
                    max_error = 0.0
                    changed_hits = 0
                    for trial in range(10):
                        # Capture a new moving blocker snapshot before each pair.
                        # All rays in a call see current geometry, as in the adapter.
                        data.mocap_pos[0, 1] = np.sin(trial * 0.3) * 0.25
                        mujoco.mj_forward(model, data)
                        old, old_ids = acquire(False)
                        new, new_ids = acquire(True)
                        np.testing.assert_allclose(old, new, rtol=0, atol=1e-12)
                        np.testing.assert_array_equal(old_ids, new_ids)
                        exact, exact_ids = acquire(False, exact=True)
                        changed_hits += int(np.count_nonzero(new_ids != exact_ids))
                        both = (new >= 0) & (exact >= 0)
                        if both.any():
                            max_error = max(
                                max_error,
                                float(np.max(np.abs(new[both] - exact[both]))),
                            )
                        for batch in [False, True] if trial % 2 == 0 else [True, False]:
                            start = time.perf_counter()
                            for _ in range(20):
                                acquire(batch)
                            if trial >= 2:
                                samples["batch" if batch else "scalar"].append(
                                    (time.perf_counter() - start) * 1000 / 20
                                )
                    results.append(
                        {
                            "geoms": model.ngeom,
                            "sensors": devices,
                            "count": count,
                            "stride": stride,
                            "displacement_m": displacement,
                            "origin_groups_per_sensor": groups,
                            "max_exact_origin_range_difference_m": max_error,
                            "changed_geom_ids_across_ten_snapshots": changed_hits,
                            "compared_emissions": devices * count * 10,
                            "samples_ms_per_call": samples,
                            "median_ms_per_call": {
                                k: statistics.median(v) for k, v in samples.items()
                            },
                        }
                    )
    print(
        json.dumps(
            {
                "host": platform.platform(),
                "mujoco": mujoco.__version__,
                "numpy": np.__version__,
                "source_sha256": hashlib.sha256(
                    Path(__file__).read_bytes()
                ).hexdigest(),
                "protocol": "Two warmups, eight alternating pairs; twenty calls per timing. Query-only, includes owned output copies in both paths. New coherent blocker snapshot each pair; no physics stepping. Scalar and batch share origins; exact-origin accuracy checked separately. No GPU or ROS.",
                "results": results,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
