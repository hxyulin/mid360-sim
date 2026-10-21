#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Math regressions for the independent motion comparison (no dataset needed)."""

import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

spec = importlib.util.spec_from_file_location(
    "comparison", Path(__file__).with_name("compare-motion-ground-truth.py")
)
comparison = importlib.util.module_from_spec(spec)
spec.loader.exec_module(comparison)


spec = importlib.util.spec_from_file_location(
    "interpolation", Path(__file__).with_name("investigate-motion-interpolation.py")
)
interpolation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(interpolation)


class MotionComparisonTests(unittest.TestCase):
    def trajectory(self, path, gap=False):
        times = np.arange(101) * 0.01
        if gap:
            times = times[(times < 0.4) | (times > 0.45)]
        quats = Rotation.from_euler("z", np.pi * times).as_quat()
        # Preserve actual timestamps when constructing a tracking gap.
        rows = [
            f"{100+t:.9f} {3*t:.12f} 0 .8 " + " ".join(map(str, q))
            for t, q in zip(times, quats)
        ]
        path.write_text("\n".join(rows) + "\n")
        return comparison.Trajectory(path)

    def test_constant_motion_interpolation_and_snapshot(self):
        with tempfile.TemporaryDirectory() as tmp:
            gt = self.trajectory(Path(tmp) / "gt.txt")
            times = np.linspace(0.02, 0.98, 2001)
            p, r = gt.pose(times)
            for hz in [60, 128, 1000]:
                pp, rr = gt.sampled(times, 1 / hz, True)
                np.testing.assert_allclose(pp, p, atol=1e-12)
                np.testing.assert_allclose((r.inv() * rr).magnitude(), 0, atol=1e-12)
            pp, rr = gt.sampled(times, 0.001, False)
            self.assertGreater(np.max(np.linalg.norm(pp - p, axis=1)), 0.002)
            self.assertTrue(gt.covered(0.1, 0.2))
            with self.assertRaises(ValueError):
                gt.pose(np.array([-1.0]))

    def test_gap_rejection(self):
        with tempfile.TemporaryDirectory() as tmp:
            gt = self.trajectory(Path(tmp) / "gt.txt", gap=True)
            self.assertFalse(gt.covered(0.39, 0.47))
            self.assertTrue(gt.covered(0.1, 0.2))

    def test_known_plane_and_error_units(self):
        rng = np.random.default_rng(7)
        xy = rng.uniform(-5, 5, size=(1000, 2))
        points = np.column_stack([xy, 0.01 * xy[:, 0] + 0.02 * xy[:, 1] + 0.03])
        n, d = comparison.fit_plane(points)
        residual = np.einsum("ni,i->n", points, n) - d
        self.assertLess(abs(residual).max(), 1e-12)
        m = comparison.metrics(np.array([0.003, 0.004]))
        self.assertAlmostEqual(m["rms"], np.sqrt(0.0000125))


class InterpolationCandidateTests(unittest.TestCase):
    def test_smooth_acceleration_and_stop_guard(self):
        cases = interpolation.controlled()
        smooth = cases["smooth_acceleration"]
        self.assertAlmostEqual(
            smooth["guarded_half_curvature"]["rms"] / smooth["linear"]["rms"],
            0.5,
            places=6,
        )
        stop = cases["stop_at_boundary"]
        self.assertEqual(stop["linear"]["max"], 0)
        self.assertGreater(stop["half_curvature"]["max"], 0.015)
        self.assertEqual(stop["guarded_half_curvature"]["max"], 0)

    def test_endpoints_and_quaternion_sign_invariance(self):
        n = 17
        f = np.linspace(0, 1, n)
        start = Rotation.from_euler("z", np.zeros(n))
        prev = Rotation.from_euler("z", np.full(n, -0.02))
        end = Rotation.from_euler("z", np.full(n, 0.03))
        result = interpolation.rotation_policy(prev, start, end, f, 0.5, True)
        self.assertLess((result[0].inv() * start[0]).magnitude(), 1e-12)
        self.assertLess((result[-1].inv() * end[-1]).magnitude(), 1e-12)
        flipped = interpolation.rotation_policy(
            Rotation.from_quat(-prev.as_quat()),
            start,
            Rotation.from_quat(-end.as_quat()),
            f,
            0.5,
            True,
        )
        np.testing.assert_allclose((result.inv() * flipped).magnitude(), 0, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
