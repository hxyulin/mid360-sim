# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Independent expected statistics, integrity checks and held-out leakage guards."""

import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from evaluate import COLUMNS, evaluate
from synthetic import create


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.manifest, self.samples = create(self.root / "fixture", 100)

    def edit_manifest(self, change):
        manifest = json.loads(self.manifest.read_text())
        change(manifest)
        self.manifest.write_text(json.dumps(manifest))

    def edit_rows(self, change):
        with self.samples.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        change(rows)
        with self.samples.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, COLUMNS)
            writer.writeheader()
            writer.writerows(rows)
        digest = hashlib.sha256(self.samples.read_bytes()).hexdigest()
        self.edit_manifest(lambda m: m.update(samples_sha256=digest))

    def test_known_detection_bias_and_heldout_residual_without_model_claim(self):
        result = evaluate(self.manifest, self.samples, min_emissions=100)
        self.assertEqual(result["status"], "synthetic_tool_check")
        self.assertFalse(result["calibrated_model"])
        self.assertTrue(result["minimum_sample_counts_met"])
        plane = result["conditions"]["plane"]
        self.assertEqual(plane["train"]["emissions"], 100)
        self.assertEqual(plane["train"]["misses"], 20)
        self.assertEqual(plane["train"]["detection_fraction"], .8)
        self.assertAlmostEqual(plane["validation_after_train_mean_bias"]["bias_m"], .01, places=12)
        self.assertGreater(plane["validation_after_train_mean_bias"]["rms_m"], .009)
        self.assertEqual(len(result["conditions"]["edge"]["edge_profiles"]["validation"]), 4)
        lo, hi = plane["train"]["wilson95_if_independent"]
        self.assertLess(lo, .8)
        self.assertGreater(hi, .8)

    def test_validation_changes_do_not_refit_training_baseline(self):
        before = evaluate(self.manifest, self.samples, 100)
        def shift(rows):
            for row in rows:
                if row["trial"].endswith("validation") and row["measured_range_m"]:
                    row["measured_range_m"] = str(float(row["measured_range_m"]) + .5)
        self.edit_rows(shift)
        after = evaluate(self.manifest, self.samples, 100)
        self.assertEqual(before["conditions"]["plane"]["train"], after["conditions"]["plane"]["train"])
        self.assertAlmostEqual(after["conditions"]["plane"]["validation_after_train_mean_bias"]["bias_m"], .51, places=12)

    def test_all_misses_remain_in_denominator_and_do_not_invent_range_statistics(self):
        def misses(rows):
            for row in rows:
                row.update(measured_range_m="", raw_reflectivity="", raw_tag="")
        self.edit_rows(misses)
        report = evaluate(self.manifest, self.samples, 100)
        plane = report["conditions"]["plane"]
        self.assertEqual(plane["train"]["misses"], 100)
        self.assertEqual(plane["train"]["detection_fraction"], 0)
        self.assertIsNone(plane["train"]["bias_m"])
        self.assertIsNone(plane["validation_after_train_mean_bias"]["rms_m"])
        self.assertFalse(report["calibrated_model"])

    def test_capture_integrity_and_missing_emissions_are_required(self):
        with self.samples.open("a") as source:
            source.write("\n")
        with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
            evaluate(self.manifest, self.samples)
        self.edit_rows(lambda rows: rows.pop(10))
        with self.assertRaisesRegex(ValueError, "indices missing/reordered"):
            evaluate(self.manifest, self.samples)

    def test_session_leakage_and_unknown_material_metadata_are_rejected(self):
        self.edit_manifest(lambda m: m["trials"][1].update(session="synthetic-train"))
        with self.assertRaisesRegex(ValueError, "disjoint sessions"):
            evaluate(self.manifest, self.samples)
        self.edit_manifest(lambda m: m["trials"][1].update(session="synthetic-validation"))
        self.edit_manifest(lambda m: m["conditions"][0].update(reflectance_905nm=None))
        with self.assertRaisesRegex(ValueError, "905 nm reflectance"):
            evaluate(self.manifest, self.samples)

    def test_nonfinite_ranges_and_bad_edge_annotations_are_rejected(self):
        self.edit_rows(lambda rows: rows[1].update(measured_range_m="nan"))
        with self.assertRaisesRegex(ValueError, "measured range"):
            evaluate(self.manifest, self.samples)
        self.edit_rows(lambda rows: rows[1].update(measured_range_m="5.02"))
        self.edit_rows(lambda rows: rows[201].update(expected_region="foreground"))
        with self.assertRaisesRegex(ValueError, "positive edge offsets"):
            evaluate(self.manifest, self.samples)

    def test_cli_keeps_insufficient_evidence_report_and_never_overwrites(self):
        output = self.root / "report.json"
        command = [sys.executable, str(Path(__file__).with_name("evaluate.py")),
                   "--manifest", str(self.manifest), "--samples", str(self.samples), "--output", str(output)]
        result = subprocess.run(command, capture_output=True, text=True)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertFalse(json.loads(output.read_text())["minimum_sample_counts_met"])
        contents = output.read_bytes()
        result = subprocess.run(command, capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("output already exists", result.stderr)
        self.assertEqual(output.read_bytes(), contents)


if __name__ == "__main__":
    unittest.main()
