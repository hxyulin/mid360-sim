# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""No-download checks for miss accounting, source integrity and optical meshing."""

import hashlib
import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np

spec = importlib.util.spec_from_file_location(
    "eth3d_example", Path(__file__).with_name("run.py")
)
example = importlib.util.module_from_spec(spec)
spec.loader.exec_module(example)


class ExampleTests(unittest.TestCase):
    def test_preview_keeps_world_up_at_image_top(self):
        rays = example.preview_rays(np.zeros(3), width=4, height=4).numpy()
        self.assertGreater(rays[0, 2, 5], 0)
        self.assertLess(rays[-1, 2, 5], 0)
        self.assertGreater(rays[2, -1, 4], 0)

    def test_misses_are_penalized_in_agreement_not_conditional_errors(self):
        result = example.score(np.array([np.inf, -1, 2.01, 1.0]), np.full(4, 2.0))
        self.assertEqual(result["hit_fraction"], 0.5)
        self.assertEqual(result["within_20mm_fraction"], 0.25)
        self.assertEqual(result["early_over_20mm_fraction"], 0.25)
        self.assertAlmostEqual(result["median_mm"], 505.0)

    def test_all_misses_have_no_fabricated_range_accuracy(self):
        result = example.score(np.array([np.inf, -1]), np.array([1.0, 2.0]))
        self.assertEqual(result["hit_fraction"], 0)
        self.assertIsNone(result["p95_mm"])
        self.assertIsNone(result["rms_mm"])

    def test_source_hash_rejects_modified_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "source"
            path.write_bytes(b"abc")
            digest = hashlib.sha256(b"abc").hexdigest()
            example.verify(path, 3, digest)
            path.write_bytes(b"abd")
            with self.assertRaisesRegex(ValueError, "SHA-256"):
                example.verify(path, 3, digest)

    def test_reconstruction_keeps_known_plane_distance(self):
        y, z = np.meshgrid(np.linspace(-0.15, 0.15, 61), np.linspace(-0.15, 0.15, 61))
        points = np.c_[np.full(y.size, 2.0), y.ravel(), z.ravel()]
        mesh = example.reconstruct(points, np.full(points.shape, 0.5), np.zeros(3))
        self.assertGreater(len(mesh.triangles), 100)
        ranges, truth = example.query(
            example.optical_scene(mesh), np.array([[2.0, 0.01, 0.01]]), np.zeros(3)
        )
        self.assertAlmostEqual(float(ranges[0]), float(truth[0]), places=5)


if __name__ == "__main__":
    unittest.main()
