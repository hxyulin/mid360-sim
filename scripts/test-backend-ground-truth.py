#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Hand-computed ray/box reference checks, independent of all simulator SDKs."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

spec = importlib.util.spec_from_file_location(
    "reference", Path(__file__).with_name("check-backend-ground-truth.py")
)
reference = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reference)


class ReferenceTests(unittest.TestCase):
    def test_translated_rotated_mount_and_axis_parallel_rays(self):
        p = np.tile([2.0, 3.0, 4.0], (3, 1))
        q = np.tile(Rotation.from_euler("z", 90, degrees=True).as_quat(), (3, 1))
        # Local X -> world +Y, local Y -> world -X, local Z -> world +Z.
        np.testing.assert_allclose(
            reference.ranges(p, q, np.eye(3)), [7.0, 12.0, 6.0], atol=1e-12
        )

    def test_diagonal_distance_and_first_face(self):
        p = np.array([[0.0, 0.0, 0.0], [2.0, 3.0, 4.0]])
        q = np.tile([0.0, 0.0, 0.0, 1.0], (2, 1))
        ray = np.tile(np.array([1.0, 1.0, 1.0]) / np.sqrt(3), (2, 1))
        np.testing.assert_allclose(
            reference.ranges(p, q, ray), [10 * np.sqrt(3), 6 * np.sqrt(3)], atol=1e-12
        )


if __name__ == "__main__":
    unittest.main()
