#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Create explicitly synthetic plane/edge trials to exercise the audit workflow."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

from evaluate import COLUMNS


def create(output, emissions=1000):
    """New directory only. Deliberate 80% detection and 20 mm training bias."""
    output.mkdir(parents=True, exist_ok=False)
    trials = [dict(id=f"{condition}-{split}", condition=condition, split=split,
                   session=f"synthetic-{split}", temperature_c=25,
                   first_index=i * emissions, stride=1, emissions=emissions)
              for i, (condition, split) in enumerate(
                  [(c, s) for c in ("plane", "edge") for s in ("train", "validation")])]
    with (output / "samples.csv").open("w", newline="") as destination:
        writer = csv.writer(destination)
        writer.writerow(COLUMNS)
        for trial in trials:
            for i in range(emissions):
                edge = trial["condition"] == "edge"
                offset = [-.01, -.001, .001, .01][i % 4] if edge else None
                region = ("foreground" if offset > 0 else "background") if edge else "target"
                expected = 8. if region == "background" else 5.
                # Independent fake validation offset shows that scoring doesn't
                # fit away held-out error. All values are fabricated tool fixtures.
                bias = .02 if trial["split"] == "train" else .03
                measured = expected + bias + ((i % 3) - 1) * .001 if i % 5 else ""
                writer.writerow([trial["id"], trial["first_index"] + i,
                                 1_000_000_000 + (trial["first_index"] + i) * 5000,
                                 expected, measured, 100 if measured != "" else "",
                                 0 if measured != "" else "", region, "" if offset is None else offset])
    manifest = {
        "schema": 1, "evidence_kind": "synthetic",
        "samples_sha256": hashlib.sha256((output / "samples.csv").read_bytes()).hexdigest(),
        "device": {"model": "synthetic MID-360-shaped fixture", "serial": "NOT-HARDWARE", "firmware": "none", "wavelength_nm": 905},
        "reference": {"method": "fabricated independent targets, not physical measurements",
                      "range_uncertainty_m": .001, "incidence_uncertainty_deg": .1, "edge_position_uncertainty_m": .0001},
        "conditions": [dict(id=c, target_id="SYNTHETIC-TARGET", reflectance_905nm=.8,
                            reflectance_uncertainty=.01, reflectance_method="fabricated fixture value, not a measurement",
                            incidence_deg=0, nominal_range_m=5, edge=c == "edge") for c in ("plane", "edge")],
        "trials": trials,
    }
    manifest["conditions"][1]["background_target"] = {
        "target_id": "SYNTHETIC-BACKGROUND", "reflectance_905nm": .2,
        "reflectance_uncertainty": .01,
        "reflectance_method": "fabricated fixture value, not a measurement",
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return output / "manifest.json", output / "samples.csv"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    create(args.output)
