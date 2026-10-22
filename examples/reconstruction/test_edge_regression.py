# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Synthetic edge regression fixtures, independent of private recordings."""

import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from edge_regression import (
    classify,
    compare,
    summarize,
    freeze,
    check,
    MISSING,
    CORRECT,
    EARLY,
    LATE,
)


class ClassificationTests(unittest.TestCase):
    def test_partition_and_inclusive_boundary(self):
        actual = classify(
            np.array([np.inf, -1, 2, 1.98, 2.02, 1.97, 2.03]), np.full(7, 2.0)
        )
        np.testing.assert_array_equal(
            actual, [MISSING, MISSING, CORRECT, CORRECT, CORRECT, EARLY, LATE]
        )

    def test_near_occluder_is_not_ignored(self):
        m = summarize(np.array([0.1, 2, 7, np.inf]), np.array([6, 6, 2, 2]), 3, 5)
        self.assertEqual(m["all"]["below_minimum_occluders"], 1)
        self.assertEqual(m["all"]["false_foreground"], 2)
        self.assertEqual(m["background_to_near_occlusion"], 2)
        self.assertEqual(m["near_to_background_leak"], 1)
        self.assertEqual(
            sum(
                m["all"][k]
                for k in ["missing", "correct", "false_foreground", "false_background"]
            ),
            4,
        )

    def test_improvements_cannot_compensate_for_occlusion(self):
        result = compare(
            np.array([2, 2, 2]), np.array([np.inf, np.inf, 6]), np.array([2, 2, 6])
        )
        self.assertEqual(result["recovered_correct"], 2)
        self.assertEqual(result["lost_correct"], 1)
        self.assertEqual(result["new_false_foreground"], 1)
        self.assertFalse(result["passed"])

    def test_new_bad_hit_fails_even_when_old_ray_was_missing(self):
        result = compare(np.array([2, 2]), np.array([np.inf, np.inf]), np.array([2, 6]))
        self.assertEqual(result["new_false_foreground"], 1)
        self.assertFalse(result["passed"])

    def test_recovery_and_repeated_reference_pass(self):
        result = compare(np.array([2, 6]), np.array([np.inf, 6]), np.array([2, 6]))
        self.assertTrue(result["passed"])
        self.assertTrue(
            compare([2, np.inf], [2, np.inf], [2, 6])["unchanged_classification"]
        )

    def test_invalid_backend_or_truth_fails(self):
        for pred, real in [([np.nan], [2]), ([2], [0]), ([2], [np.inf]), ([], [])]:
            with self.assertRaises(ValueError):
                classify(pred, real)


class FixtureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        raw = np.array(
            [[1.0, y, z] for y in [-0.06, -0.03, 0.03, 0.06] for z in [-0.01, 0.01]]
        )
        directions = raw / np.linalg.norm(raw, axis=1)[:, None]
        depth_x = np.where(raw[:, 1] < 0, 2, 4)
        points = directions * (depth_x / directions[:, 0])[:, None]
        np.savez(
            self.root / "train.npz",
            train=np.tile(points, (4, 1)),
            train_second=np.repeat([0, 10, 20, 30], len(points)),
        )
        np.savez(self.root / "later.npz", points=points)
        (self.root / "cases.json").write_text(
            json.dumps(
                [
                    dict(
                        name="synthetic_edge",
                        azimuth_deg=0,
                        elevation_deg=0,
                        half_width_deg=5,
                        near_max_m=2.5,
                        background_min_m=3.5,
                        expected_training_returns=32,
                        expected_evaluation_returns=8,
                    )
                ]
            )
        )
        self.write_mesh("exact.obj", 0.0)
        self.write_mesh("filled.obj", 0.5)
        self.fixtures = self.root / "fixtures"
        freeze(
            self.root / "train.npz",
            self.root / "later.npz",
            self.root / "cases.json",
            {"exact": self.root / "exact.obj"},
            self.fixtures,
        )

    def tearDown(self):
        self.temp.cleanup()

    def write_mesh(self, name, near_y_max):
        text = []
        for x, lo, hi in [(2, -0.5, near_y_max), (4, -1, 1)]:
            start = sum(line.startswith("v ") for line in text) + 1
            for y, z in [(lo, -0.5), (hi, -0.5), (hi, 0.5), (lo, 0.5)]:
                text.append(f"v {x} {y} {z}")
            text.extend(
                [f"f {start} {start+1} {start+2}", f"f {start} {start+2} {start+3}"]
            )
        (self.root / name).write_text("\n".join(text) + "\n")

    def test_reference_replays_and_filled_edge_fails(self):
        exact = check(self.fixtures, self.root / "exact.obj", "exact")
        self.assertTrue(exact["passed"])
        self.assertEqual(exact["cases"][0]["metrics"]["all"]["correct"], 8)
        filled = check(self.fixtures, self.root / "filled.obj", "exact")
        self.assertFalse(filled["passed"])
        self.assertEqual(len(filled["cases"][0]["failure_rays"]), 4)
        self.assertTrue(
            all(
                r["current_class"] == "false_foreground"
                for r in filled["cases"][0]["failure_rays"]
            )
        )
        self.assertEqual(
            filled["cases"][0]["metrics"]["background_to_near_occlusion"], 4
        )

    def test_corrupted_fixture_is_rejected(self):
        with (self.fixtures / "synthetic_edge.npz").open("ab") as stream:
            stream.write(b"changed")
        with self.assertRaisesRegex(ValueError, "checksum"):
            check(self.fixtures, self.root / "exact.obj", "exact")

    def test_changed_scoring_policy_is_rejected(self):
        path = self.fixtures / "manifest.json"
        manifest = json.loads(path.read_text())
        manifest["tolerance_m"] = 0.1
        path.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError, "scoring policy"):
            check(self.fixtures, self.root / "exact.obj", "exact")

    def test_freeze_refuses_overwrite(self):
        with self.assertRaisesRegex(ValueError, "exists"):
            freeze(
                self.root / "train.npz",
                self.root / "later.npz",
                self.root / "cases.json",
                {"exact": self.root / "exact.obj"},
                self.fixtures,
            )


if __name__ == "__main__":
    unittest.main()
