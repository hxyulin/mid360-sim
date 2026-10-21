#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Analytic rolling-acquisition comparison, not moving-hardware validation.

Uses nominal rays in a static 20 m cube. Compares acquisition pose policies
against exact per-emission trajectories; all reconstructed points use exact
poses for deskew, so residuals isolate acquisition error rather than odometry.
Requires NumPy; --mujoco additionally checks the real snapshot adapter.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "integrations/python"))
from mid360_sim.pattern import directions  # noqa: E402


def pose(t, accelerating):
    """World position (m), yaw (rad); optional 10 m/s² and 720 deg/s²."""
    t = np.asarray(t)
    x = 3 * t + (5 * t**2 if accelerating else 0)
    yaw = np.pi * t + (2 * np.pi * t**2 if accelerating else 0)
    return np.stack([x, t * 0, t * 0 + 0.8], axis=-1), yaw


def rotate(local, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.column_stack(
        [
            c * local[:, 0] - s * local[:, 1],
            s * local[:, 0] + c * local[:, 1],
            local[:, 2],
        ]
    )


def ranges(local, p, yaw):
    world = rotate(local, yaw)
    with np.errstate(divide="ignore", invalid="ignore"):
        distances = np.where(world > 0, (10 - p) / world, (-10 - p) / world)
    return np.min(distances, axis=1)


def summarize(error):
    return {
        "rms_m": float(np.sqrt(np.mean(error**2))),
        "p95_m": float(np.percentile(np.abs(error), 95)),
        "max_m": float(np.max(np.abs(error))),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--mujoco", action="store_true")
    args = parser.parse_args()
    local = directions(0, 20000)
    times = np.arange(len(local)) * 5e-6
    report = {
        "scope": "analytic acquisition-policy comparison; static cube; no noise; not hardware validation",
        "duration_s": 0.1,
        "points": len(local),
        "cases": {},
    }
    for accelerating in [False, True]:
        true_p, true_yaw = pose(times, accelerating)
        reference = ranges(local, true_p, true_yaw)
        raw = local * reference[:, None]
        # Treating raw instantaneous-frame points as all acquired at frame start.
        raw_world = rotate(raw, 0) + pose(0, accelerating)[0]
        raw_wall = np.min(np.abs(np.abs(raw_world) - 10), axis=1)
        case = {"raw_undeskewed_wall_residual": summarize(raw_wall), "policies": {}}
        for name, duration, interpolate in [
            ("snapshot_100ms", 0.1, False),
            ("step_hold_1ms", 0.001, False),
            ("interpolate_16.667ms", 1 / 60, True),
            ("interpolate_1ms", 0.001, True),
        ]:
            start = np.floor(times / duration) * duration
            p, yaw = pose(start, accelerating)
            if interpolate:
                end_p, end_yaw = pose(start + duration, accelerating)
                f = (times - start) / duration
                p = p + (end_p - p) * f[:, None]
                yaw = yaw + (end_yaw - yaw) * f
            sampled = ranges(local, p, yaw)
            # With identical local directions this equals 3D point displacement
            # after exact per-ray deskew. It includes corner correspondence shifts.
            error = sampled - reference
            case["policies"][name] = summarize(error)
            if interpolate and not accelerating:
                assert np.max(np.abs(error)) < 1e-10
        report["cases"][
            "accelerating" if accelerating else "constant_velocity_yaw"
        ] = case
    if args.mujoco:
        import mujoco
        from mid360_sim.mujoco import MujocoSensor

        walls = []
        for axis in range(3):
            for sign in [-1, 1]:
                pos, size = [0.0, 0.0, 0.0], [11.0, 11.0, 11.0]
                pos[axis], size[axis] = sign * 10.1, 0.1
                walls.append(
                    f'<geom type="box" pos="{" ".join(map(str, pos))}" size="{" ".join(map(str, size))}"/>'
                )
        xml = (
            "<mujoco><worldbody>"
            + "".join(walls)
            + '<body><freejoint/><geom type="sphere" size=".01" group="1"/><site name="lidar"/></body></worldbody></mujoco>'
        )
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        sensor = MujocoSensor(model, data, "lidar", geom_groups=[1, 0, 0, 0, 0, 0])
        actual = []
        for first in range(0, 20000, 200):
            p, yaw = pose(first * 5e-6, False)
            data.qpos[:3] = p
            data.qpos[3:7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
            mujoco.mj_forward(model, data)
            frame = sensor.scan(first, 200)
            assert frame.valid.all()
            actual.extend(frame.ranges_m)
        start = (np.arange(20000) // 200) * 0.001
        p, yaw = pose(start, False)
        error = np.asarray(actual) - ranges(local, p, yaw)
        assert np.max(np.abs(error)) < 1e-10
        report["native_mujoco_matches_1ms_hold"] = summarize(error)
    text = json.dumps(report, indent=2)
    print(text)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n")


if __name__ == "__main__":
    main()
