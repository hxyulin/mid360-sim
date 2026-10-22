# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Independent frame direction and scoring checks; ROS is not required."""

import math
import unittest

from lio import interpolate, score
from ros import inverse, multiply, pose_from_rm, relative, rotate


class FramesAndScoring(unittest.TestCase):
    def test_relative_rotating_parent(self):
        q = [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)]
        p, r = relative(([1.0, 2.0, 3.0], q), ([1.0, 4.0, 3.0], [0.0, 0.0, 0.0, 1.0]))
        for a, b in zip(p, [2.0, 0.0, 0.0]):
            self.assertAlmostEqual(a, b)
        for a, b in zip(multiply(q, r), [0.0, 0.0, 0.0, 1.0]):
            self.assertAlmostEqual(a, b)
        self.assertEqual(
            pose_from_rm({"translation_m": [1, 2, 3], "rotation_wxyz": [1, 0, 0, 0]}),
            ([1, 2, 3], [0, 0, 0, 1]),
        )

    def test_interpolation_and_no_extrapolation(self):
        rows = [
            [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [10, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        ]
        p, q = interpolate(rows, 5)
        self.assertEqual(p, [1.0, 0.0, 0.0])
        self.assertAlmostEqual(q[2], math.sqrt(0.5))
        with self.assertRaises(ValueError):
            interpolate(rows, 11)

    def test_first_pose_alignment_does_not_hide_drift(self):
        truth = [[n, 0.1 * n, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0] for n in range(30)]
        q = [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)]
        estimates = []
        for row in truth:
            p = rotate(q, row[1:4])
            estimates.append([row[0], p[0] + 2, p[1] + 3, p[2] + 4, *q])
        self.assertLess(score(truth, estimates)["position_max_m"], 1e-12)
        estimates[-1][1] += 0.5
        self.assertAlmostEqual(score(truth, estimates)["position_max_m"], 0.5)
        with self.assertRaises(ValueError):
            score(truth, estimates[:10])
        self.assertAlmostEqual(multiply(q, inverse(q))[3], 1.0)
        bad = [row[:] for row in truth]
        bad[0][1] = float("nan")
        with self.assertRaises(ValueError):
            score(bad, estimates)
