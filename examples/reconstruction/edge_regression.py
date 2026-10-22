# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Freeze source-derived edge fixtures and check meshes on identical real rays.

Optional offline tool. Real fixture arrays stay with the dataset, outside Git;
synthetic tests exercise the evaluator without recordings or simulator runtimes.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re

import numpy as np

FORMAT = 1
TOLERANCE_M = 0.02
MIN_RANGE_M = 0.3
MAX_RANGE_M = 30.0
MISSING, CORRECT, EARLY, LATE = range(4)
CATEGORIES = ("missing", "correct", "false_foreground", "false_background")


def digest(path: Path) -> str:
    """SHA256 of an input, read in bounded chunks."""
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def classify(predicted_m, measured_m):
    """Partition every measured ray; early means a false closer intersection.

    Negative/infinite predictions and intersections beyond30m are misses. NaNs
    are rejected as invalid backend data. Below-minimum first hits still occlude
    farther surfaces and are classified as false foreground, never ignored.
    The20mm boundary is inclusive, allowing1pm numerical comparison tolerance.
    """
    p = np.asarray(predicted_m, dtype=float)
    r = np.asarray(measured_m, dtype=float)
    if p.ndim != 1 or not len(p) or p.shape != r.shape:
        raise ValueError("Expected nonempty matched one-dimensional ranges")
    if (
        np.isnan(p).any()
        or not np.isfinite(r).all()
        or np.any((r < MIN_RANGE_M) | (r > MAX_RANGE_M))
    ):
        raise ValueError("Invalid predictions or measured ranges outside0.3–30m")
    hit = np.isfinite(p) & (p >= 0) & (p <= MAX_RANGE_M)
    categories = np.full(len(p), MISSING, dtype=np.uint8)
    difference = p - r
    epsilon = TOLERANCE_M + 1e-12
    categories[hit & (abs(difference) <= epsilon)] = CORRECT
    categories[hit & (difference < -epsilon)] = EARLY
    categories[hit & (difference > epsilon)] = LATE
    assert np.all(~hit | (categories != MISSING))
    return categories


def summarize(predicted_m, measured_m, near_max_m, background_min_m):
    """Report misses, early/late errors and observed-range strata separately.

    Labels describe actual measured ranges, not manually certified static objects.
    A true background return hit by a near proxy is a specific occlusion failure.
    """
    p, r = np.asarray(predicted_m), np.asarray(measured_m)
    c = classify(p, r)
    if not MIN_RANGE_M < near_max_m < background_min_m < MAX_RANGE_M:
        raise ValueError("Expected separated near/background thresholds")

    def counts(mask):
        values = np.bincount(c[mask], minlength=4)
        error = p[mask & (c != MISSING)] - r[mask & (c != MISSING)]
        return dict(
            rays=int(mask.sum()),
            **{name: int(values[i]) for i, name in enumerate(CATEGORIES)},
            severe_over100mm=int(np.sum(abs(error) > 0.1)),
            below_minimum_occluders=int(
                np.sum(mask & (c != MISSING) & (p < MIN_RANGE_M))
            ),
        )

    near = r <= near_max_m
    background = r >= background_min_m
    return dict(
        all=counts(np.ones(len(r), bool)),
        near_returns=counts(near),
        background_returns=counts(background),
        intermediate_returns=counts(~near & ~background),
        background_to_near_occlusion=int(
            np.sum(background & (c == EARLY) & (p <= near_max_m))
        ),
        near_to_background_leak=int(
            np.sum(near & (c == LATE) & (p >= background_min_m))
        ),
    )


def compare(predicted_m, reference_m, measured_m):
    """Strict per-ray gates: improvements cannot offset newly wrong rays."""
    current = classify(predicted_m, measured_m)
    reference = classify(reference_m, measured_m)
    result = dict(
        new_false_foreground=int(np.sum((current == EARLY) & (reference != EARLY))),
        new_false_background=int(np.sum((current == LATE) & (reference != LATE))),
        newly_missing=int(np.sum((current == MISSING) & (reference != MISSING))),
        lost_correct=int(np.sum((reference == CORRECT) & (current != CORRECT))),
        recovered_correct=int(np.sum((current == CORRECT) & (reference != CORRECT))),
        unchanged_classification=bool(np.array_equal(current, reference)),
    )
    result["passed"] = all(
        result[name] == 0
        for name in (
            "new_false_foreground",
            "new_false_background",
            "newly_missing",
            "lost_correct",
        )
    )
    return result


def raycast(mesh_path: Path, directions):
    """Query first-hit optical triangles from the fixture's stationary origin."""
    import open3d as o3d

    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    if not len(mesh.triangles) or not np.isfinite(np.asarray(mesh.vertices)).all():
        raise ValueError("Expected a finite optical triangle mesh")
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    directions = np.asarray(directions)
    if (
        directions.ndim != 2
        or directions.shape[1] != 3
        or not np.isfinite(directions).all()
        or not np.allclose(np.linalg.norm(directions, axis=1), 1, atol=1e-10)
    ):
        raise ValueError("Expected finite unit directions")
    rays = np.c_[np.zeros_like(directions), directions].astype(np.float32)
    return scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy().astype(float)


def case_mask(points, case):
    """Select one frozen angular box without changing its bounds on later runs."""
    p = np.asarray(points, dtype=float)
    az = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
    el = np.degrees(np.arctan2(p[:, 2], np.linalg.norm(p[:, :2], axis=1)))
    da = (az - case["azimuth_deg"] + 180) % 360 - 180
    half = case["half_width_deg"]
    return (abs(da) <= half) & (abs(el - case["elevation_deg"]) <= half)


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def freeze(training, evaluation, case_file, meshes, output):
    """Create an immutable-by-checksum, local source-data fixture bundle."""
    import open3d as o3d

    if output.exists():
        raise ValueError("Output exists; do not overwrite frozen fixtures")
    cases = json.loads(case_file.read_text())
    if not cases or len({c["name"] for c in cases}) != len(cases):
        raise ValueError("Expected unique named cases")
    for case in cases:
        if (
            not re.fullmatch(r"[A-Za-z0-9_-]+", case["name"])
            or not 0 < case["half_width_deg"] < 180
        ):
            raise ValueError("Invalid case name or angular extent")
        if (
            not MIN_RANGE_M
            < case["near_max_m"]
            < case["background_min_m"]
            < MAX_RANGE_M
        ):
            raise ValueError("Invalid observed-range strata")
    data = np.load(training)
    train = data["train"]
    seconds = data["train_second"]
    later = np.load(evaluation)["points"]
    if (
        train.ndim != 2
        or train.shape[1] != 3
        or seconds.shape != (len(train),)
        or not np.issubdtype(seconds.dtype, np.integer)
    ):
        raise ValueError("Invalid training input")
    if (
        later.ndim != 2
        or later.shape[1] != 3
        or not np.isfinite(later).all()
        or not np.isfinite(train).all()
    ):
        raise ValueError("Invalid point coordinates")
    output.mkdir(parents=True)
    manifest = dict(
        format=FORMAT,
        frame="stationary sensor FLU, metres; origin [0,0,0]",
        interpretation="Selected, repeatedly inspected debugging fixtures, not a blind benchmark. Measured-range labels are not semantic static-scene truth.",
        tolerance_m=TOLERANCE_M,
        min_range_m=MIN_RANGE_M,
        max_range_m=MAX_RANGE_M,
        inputs=dict(
            training_sha256=digest(training),
            evaluation_sha256=digest(evaluation),
            case_spec_sha256=digest(case_file),
        ),
        evaluator_sha256=digest(Path(__file__)),
        open3d_version=o3d.__version__,
        references={},
        cases=[],
    )
    for name, path in meshes.items():
        manifest["references"][name] = dict(mesh_sha256=digest(path))
    for case in cases:
        tm, qm = case_mask(train, case), case_mask(later, case)
        truth = np.linalg.norm(later[qm].astype(float), axis=1)
        if not len(truth) or np.any((truth < MIN_RANGE_M) | (truth > MAX_RANGE_M)):
            raise ValueError(f"No valid evaluation rays in case {case['name']}")
        if int(tm.sum()) != case.get("expected_training_returns", int(tm.sum())) or len(
            truth
        ) != case.get("expected_evaluation_returns", len(truth)):
            raise ValueError("Source selection no longer matches pinned case counts")
        directions = later[qm].astype(float) / truth[:, None]
        # No nearest-angle resampling, median binning or generated scan pattern.
        arrays = dict(
            training_points_m=train[tm],
            training_seconds=seconds[tm],
            directions=directions,
            measured_ranges_m=truth,
        )
        records = {}
        for name, path in meshes.items():
            prediction = raycast(path, directions)
            arrays["reference_" + name] = prediction
            arrays["classes_" + name] = classify(prediction, truth)
            records[name] = summarize(
                prediction, truth, case["near_max_m"], case["background_min_m"]
            )
        path = output / (case["name"] + ".npz")
        np.savez_compressed(path, **arrays)
        manifest["cases"].append(
            dict(
                **case,
                data_file=path.name,
                sha256=digest(path),
                training_returns=int(tm.sum()),
                evaluation_returns=len(truth),
                expected=records,
            )
        )
    write_json(output / "manifest.json", manifest)
    return manifest


def check(fixtures, mesh, against):
    """Replay frozen rays, enforcing integrity and per-case noncompensating gates."""
    manifest = json.loads((fixtures / "manifest.json").read_text())
    if (
        manifest["format"] != FORMAT
        or manifest["tolerance_m"] != TOLERANCE_M
        or manifest["min_range_m"] != MIN_RANGE_M
        or manifest["max_range_m"] != MAX_RANGE_M
    ):
        raise ValueError("Unsupported fixture format or changed scoring policy")
    if against not in manifest["references"]:
        raise ValueError("Unknown frozen reference")
    result = dict(
        mesh_sha256=digest(mesh),
        against=against,
        evaluator_sha256=digest(Path(__file__)),
        cases=[],
        passed=True,
    )
    for case in manifest["cases"]:
        if Path(case["data_file"]).name != case["data_file"]:
            raise ValueError("Fixture filename must be local to its bundle")
        path = fixtures / case["data_file"]
        if digest(path) != case["sha256"]:
            raise ValueError("Fixture checksum mismatch")
        arrays = np.load(path)
        reference = arrays["reference_" + against]
        truth = arrays["measured_ranges_m"]
        if not np.array_equal(classify(reference, truth), arrays["classes_" + against]):
            raise ValueError("Frozen per-ray classifications changed")
        expected = summarize(
            reference, truth, case["near_max_m"], case["background_min_m"]
        )
        if expected != case["expected"][against]:
            raise ValueError("Frozen reference summary does not match scoring policy")
        prediction = raycast(mesh, arrays["directions"])
        metrics = summarize(
            prediction, truth, case["near_max_m"], case["background_min_m"]
        )
        gates = compare(prediction, reference, truth)
        current_class = classify(prediction, truth)
        previous_class = classify(reference, truth)
        newly_wrong = (
            ((current_class == EARLY) & (previous_class != EARLY))
            | ((current_class == LATE) & (previous_class != LATE))
            | ((current_class == MISSING) & (previous_class != MISSING))
            | ((previous_class == CORRECT) & (current_class != CORRECT))
        )
        failures = []
        for index in np.flatnonzero(newly_wrong):
            direction = arrays["directions"][index]
            failures.append(
                dict(
                    ray_index=int(index),
                    direction=direction.tolist(),
                    measured_m=float(truth[index]),
                    predicted_m=(
                        float(prediction[index])
                        if np.isfinite(prediction[index])
                        else None
                    ),
                    previous_class=CATEGORIES[int(previous_class[index])],
                    current_class=CATEGORIES[int(current_class[index])],
                )
            )
        result["cases"].append(
            dict(name=case["name"], metrics=metrics, gates=gates, failure_rays=failures)
        )
        result["passed"] &= gates["passed"]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    build = sub.add_parser(
        "freeze", help="Freeze selected cases from caller-owned private data"
    )
    build.add_argument("--training", type=Path, required=True)
    build.add_argument("--evaluation", type=Path, required=True)
    build.add_argument("--cases", type=Path, required=True)
    build.add_argument(
        "--mesh",
        action="append",
        required=True,
        help="NAME=PATH; repeat for baseline and candidate",
    )
    build.add_argument("--output", type=Path, required=True)
    run = sub.add_parser(
        "check", help="Fail if any case gains a newly wrong or missing ray"
    )
    run.add_argument("--fixtures", type=Path, required=True)
    run.add_argument("--mesh", type=Path, required=True)
    run.add_argument("--against", required=True)
    run.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "freeze":
        meshes = {}
        for entry in args.mesh:
            name, path = entry.split("=", 1)
            if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name) or name in meshes:
                parser.error("Reference names must be unique safe identifiers")
            meshes[name] = Path(path)
        result = freeze(args.training, args.evaluation, args.cases, meshes, args.output)
    else:
        if args.report.exists():
            parser.error("Report exists; preserve previous checks")
        result = check(args.fixtures, args.mesh, args.against)
        write_json(args.report, result)
    print(json.dumps(result, indent=2, allow_nan=False))
    if args.command == "check" and not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
