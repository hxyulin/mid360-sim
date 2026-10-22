# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""No-download checks for miss accounting, source integrity and optical meshing."""

import hashlib
import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np
import open3d as o3d

spec = importlib.util.spec_from_file_location(
    "eth3d_example", Path(__file__).with_name("run.py")
)
example = importlib.util.module_from_spec(spec)
spec.loader.exec_module(example)


class ExampleTests(unittest.TestCase):
    def test_both_views_reserve_validation_before_geometry(self):
        rng = np.random.default_rng(42)
        for count in (101, 123):
            train, sample = example.split_indices(count, 10, rng)
            self.assertTrue(np.all(sample % 5 == 0))
            self.assertFalse(train[sample].any())
            self.assertFalse(train[::5].any())
            self.assertEqual(len(np.unique(sample)), 10)
        with self.assertRaisesRegex(ValueError, "reserved"):
            example.split_indices(10, 4, rng)

    @staticmethod
    def cloud(points, normals):
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        cloud.normals = o3d.utility.Vector3dVector(normals)
        cloud.colors = o3d.utility.Vector3dVector(np.full((len(points), 3), 0.5))
        return cloud

    def test_fusion_keeps_opposing_or_distant_surfaces(self):
        first = self.cloud([[0, 0, 0]], [[1, 0, 0]])
        second = self.cloud(
            [[0.002, 0, 0], [0.001, 0, 0], [0.02, 0, 0]],
            [[1, 0, 0], [-1, 0, 0], [1, 0, 0]],
        )
        fused, stats = example.fuse_clouds(first, second)
        self.assertEqual(stats["merged_additional_representatives"], 1)
        self.assertEqual(len(fused.points), 3)
        np.testing.assert_allclose(np.asarray(fused.points)[0], [0.001, 0, 0])
        np.testing.assert_array_equal(np.asarray(fused.normals)[1], [-1, 0, 0])
        np.testing.assert_array_equal(np.asarray(first.points), [[0, 0, 0]])

    def test_fusion_averages_groups_without_repeated_reference_weight(self):
        first = self.cloud([[0, 0, 0]], [[1, 0, 0]])
        second = self.cloud([[0.002, 0, 0], [0.004, 0, 0]], [[1, 0, 0]] * 2)
        fused, stats = example.fuse_clouds(first, second)
        self.assertEqual(stats["fused_representatives"], 1)
        np.testing.assert_allclose(np.asarray(fused.points), [[0.002, 0, 0]])
        np.testing.assert_allclose(np.linalg.norm(fused.normals, axis=1), [1])

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
        cloud = example.prepare_cloud(points, np.full(points.shape, 0.5), np.zeros(3))
        mesh = example.reconstruct(cloud)
        self.assertGreater(len(mesh.triangles), 100)
        ranges, truth = example.query(
            example.optical_scene(mesh), np.array([[2.0, 0.01, 0.01]]), np.zeros(3)
        )
        self.assertAlmostEqual(float(ranges[0]), float(truth[0]), places=5)


if __name__ == "__main__":
    unittest.main()
