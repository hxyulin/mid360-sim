#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Audit known-target measurements and score held-out conditional range bias.

Standard library only; no decoder or datasets bundled. This produces descriptive
measurements and an empirical mean-bias baseline, never a calibrated material,
beam-footprint or correlated-noise model. Preserve misses in the denominator.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

COLUMNS = ["trial", "sample_index", "time_ns", "expected_range_m", "measured_range_m",
           "raw_reflectivity", "raw_tag", "expected_region", "edge_offset_m"]


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def number(value, low, high, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"invalid {label}")
    return value


def integer(value, low, high, label):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"invalid {label}")
    return value


def text(value, label):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"missing {label}")
    return value


def manifest_check(manifest):
    if manifest["schema"] != 1 or manifest["evidence_kind"] not in ("synthetic", "measured"):
        raise ValueError("unsupported schema/evidence_kind")
    if len(manifest["samples_sha256"]) != 64:
        raise ValueError("invalid samples SHA-256")
    device = manifest["device"]
    for name in ("model", "serial", "firmware"):
        text(device[name], f"device {name}")
    if device["wavelength_nm"] != 905:
        raise ValueError("workflow requires explicit 905 nm reflectance")
    reference = manifest["reference"]
    text(reference["method"], "independent reference method")
    number(reference["range_uncertainty_m"], 0, 100, "reference range uncertainty")
    number(reference["incidence_uncertainty_deg"], 0, 90, "reference incidence uncertainty")
    conditions = {}
    for c in manifest["conditions"]:
        key = text(c["id"], "condition ID")
        if key in conditions:
            raise ValueError("duplicate condition ID")
        text(c["target_id"], "target ID")
        text(c["reflectance_method"], "905 nm reflectance measurement method")
        number(c["reflectance_905nm"], 0, 1, "905 nm reflectance")
        number(c["reflectance_uncertainty"], 0, 1, "reflectance uncertainty")
        number(c["incidence_deg"], 0, 90, "incidence angle")
        number(c["nominal_range_m"], .001, 1000, "nominal range")
        if type(c["edge"]) is not bool:
            raise ValueError("edge must be boolean")
        if c["edge"]:
            number(reference["edge_position_uncertainty_m"], 0, 100, "edge reference uncertainty")
            background = c["background_target"]
            text(background["target_id"], "background target ID")
            text(background["reflectance_method"], "background 905 nm measurement method")
            number(background["reflectance_905nm"], 0, 1, "background 905 nm reflectance")
            number(background["reflectance_uncertainty"], 0, 1, "background reflectance uncertainty")
        conditions[key] = c
    trials, sessions = {}, {"train": set(), "validation": set()}
    for t in manifest["trials"]:
        key = text(t["id"], "trial ID")
        if key in trials or t["condition"] not in conditions or t["split"] not in sessions:
            raise ValueError("duplicate trial or unknown condition/split")
        text(t["session"], "session identity")
        number(t["temperature_c"], -100, 200, "device temperature")
        integer(t["first_index"], 0, 2**64 - 1, "first emission index")
        integer(t["stride"], 1, 2**32 - 1, "selection stride")
        integer(t["emissions"], 1, 10_000_000, "expected emissions")
        if t["first_index"] + (t["emissions"] - 1) * t["stride"] >= 2**64:
            raise ValueError("emission index overflow")
        sessions[t["split"]].add(t["session"])
        trials[key] = t
    if sessions["train"] & sessions["validation"]:
        raise ValueError("training and validation must use disjoint sessions")
    if not conditions or not trials:
        raise ValueError("nonempty conditions and trials required")
    for condition in conditions:
        if {t["split"] for t in trials.values() if t["condition"] == condition} != {"train", "validation"}:
            raise ValueError("every condition needs training and held-out validation")
    return conditions, trials


def load_samples(path, conditions, trials):
    samples = {key: [] for key in trials}
    identities = set()
    with path.open(newline="") as source:
        rows = csv.DictReader(source)
        if rows.fieldnames != COLUMNS:
            raise ValueError("CSV columns/order must match documented emission schema")
        for row in rows:
            if row["trial"] not in trials:
                raise ValueError("unknown sample trial")
            trial = trials[row["trial"]]
            records = samples[row["trial"]]
            index, stamp = int(row["sample_index"]), int(row["time_ns"])
            integer(stamp, 0, 2**64 - 1, "original sensor time_ns")
            if index != trial["first_index"] + len(records) * trial["stride"]:
                raise ValueError("emission indices missing/reordered; retain misses")
            if records and stamp <= records[-1]["time_ns"]:
                raise ValueError("sensor times must increase within a trial")
            identity = trial["session"], index
            if identity in identities:
                raise ValueError("reused emission within a session")
            identities.add(identity)
            expected = number(float(row["expected_range_m"]), .000001, 1000, "independent expected range")
            measured = None if row["measured_range_m"] == "" else number(float(row["measured_range_m"]), .000001, 1000, "measured range")
            for field in ("raw_reflectivity", "raw_tag"):
                if row[field] != "":
                    integer(int(row[field]), 0, 255, field)
                elif measured is not None:
                    raise ValueError("valid returns require raw reflectivity/tag bytes")
            edge = conditions[trial["condition"]]["edge"]
            region = row["expected_region"]
            offset = None if row["edge_offset_m"] == "" else number(float(row["edge_offset_m"]), -1000, 1000, "surveyed edge offset")
            if edge:
                if offset is None or region not in ("foreground", "background"):
                    raise ValueError("edge trials need surveyed offsets and foreground/background truth")
                if (offset > 0 and region != "foreground") or (offset < 0 and region != "background"):
                    raise ValueError("positive edge offsets denote foreground")
            elif offset is not None or region != "target":
                raise ValueError("plane trials require target region without edge offset")
            records.append({"sample_index": index, "time_ns": stamp, "expected_range_m": expected,
                            "measured_range_m": measured, "expected_region": region, "edge_offset_m": offset})
    if any(len(samples[key]) != t["emissions"] for key, t in trials.items()):
        raise ValueError("emission count differs from manifest; missing returns must be blank rows")
    return samples


def range_metrics(errors):
    return {"returns": len(errors), "bias_m": statistics.mean(errors) if errors else None,
            "sample_stddev_m": statistics.stdev(errors) if len(errors) > 1 else None,
            "rms_m": math.sqrt(statistics.mean(e * e for e in errors)) if errors else None}


def metrics(rows, tolerance):
    detected = [r for r in rows if r["measured_range_m"] is not None]
    errors = [r["measured_range_m"] - r["expected_range_m"] for r in detected]
    n, k = len(rows), len(detected)
    # Descriptive only: adjacent emissions are not assumed physically independent.
    p, z = k / n, 1.959963984540054
    center = (p + z*z/(2*n)) / (1 + z*z/n)
    radius = z * math.sqrt(p*(1-p)/n + z*z/(4*n*n)) / (1 + z*z/n)
    return {"emissions": n, "misses": n-k, "detection_fraction": p,
            "wilson95_if_independent": [max(0, center-radius), min(1, center+radius)],
            "early_beyond_tolerance": sum(e < -tolerance for e in errors),
            "late_beyond_tolerance": sum(e > tolerance for e in errors),
            **range_metrics(errors)}


def evaluate(manifest_path, samples_path, min_emissions=1000, tolerance_m=.02):
    integer(min_emissions, 1, 10_000_000, "minimum emissions per split/condition")
    number(tolerance_m, 0, 100, "residual classification tolerance")
    manifest_bytes = manifest_path.read_bytes()
    manifest = json.loads(manifest_bytes)
    conditions, trials = manifest_check(manifest)
    if digest(samples_path) != manifest["samples_sha256"]:
        raise ValueError("samples SHA-256 mismatch")
    samples = load_samples(samples_path, conditions, trials)
    if digest(samples_path) != manifest["samples_sha256"]:
        raise ValueError("samples changed during evaluation")
    results = {}
    for key, condition in conditions.items():
        split = {s: [r for name, t in trials.items() if t["condition"] == key and t["split"] == s for r in samples[name]] for s in ("train", "validation")}
        result = {s: metrics(rows, tolerance_m) for s, rows in split.items()}
        bias = result["train"]["bias_m"]
        validation_errors = [r["measured_range_m"] - r["expected_range_m"] - bias for r in split["validation"] if r["measured_range_m"] is not None] if bias is not None else []
        result["validation_after_train_mean_bias"] = range_metrics(validation_errors)
        result["minimum_sample_counts_met"] = all(len(rows) >= min_emissions for rows in split.values())
        if condition["edge"]:
            result["edge_profiles"] = {s: [
                {"edge_offset_m": offset, "expected_region": region,
                 **metrics([r for r in rows if r["edge_offset_m"] == offset and r["expected_region"] == region], tolerance_m)}
                for offset, region in sorted({(r["edge_offset_m"], r["expected_region"]) for r in rows})
            ] for s, rows in split.items()}
        results[key] = result
    sufficient = all(r["minimum_sample_counts_met"] for r in results.values())
    return {"schema": 1, "evidence_kind": manifest["evidence_kind"], "calibrated_model": False,
            "status": "synthetic_tool_check" if manifest["evidence_kind"] == "synthetic" else "measured_descriptive_evaluation",
            "minimum_sample_counts_met": sufficient, "min_emissions": min_emissions,
            "tolerance_m": tolerance_m, "manifest_sha256": hashlib.sha256(manifest_bytes).hexdigest(),
            "reference_range_uncertainty_exceeds_tolerance": manifest["reference"]["range_uncertainty_m"] > tolerance_m,
            "samples_sha256": digest(samples_path), "conditions": results,
            "metadata": manifest,
            "limits": ["No calibrated material, beam footprint or correlated-noise law is fitted.",
                       "Mean-bias baseline uses training only; validation sessions are held out.",
                       "Wilson intervals assume independent Bernoulli trials; correlated emissions violate that assumption.",
                       "Range/edge residuals include reference uncertainty and cannot isolate optical footprint.",
                       "Raw reflectivity/tag bytes are not interpreted as reflectance or return-origin truth."]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--samples", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New local JSON report; never overwrite evidence")
    parser.add_argument("--min-emissions", type=int, default=1000)
    parser.add_argument("--tolerance-m", type=float, default=.02)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists")
    report = evaluate(args.manifest, args.samples, args.min_emissions, args.tolerance_m)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as destination:
        json.dump(report, destination, indent=2, allow_nan=False)
        destination.write("\n")
    print(json.dumps({"status": report["status"], "minimum_sample_counts_met": report["minimum_sample_counts_met"], "calibrated_model": False}))
    if not report["minimum_sample_counts_met"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
