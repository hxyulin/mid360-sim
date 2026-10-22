<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Frozen edge regressions

`edge_regression.py` checks fixed individual measured directions, without median
binning, nearest-angle resampling or generating a different scan pattern. The
synthetic suite runs without recordings:

```sh
python -m unittest discover -s examples/reconstruction -v
```

Keep real fixture bundles beside their dataset in ignored storage. A case-spec
JSON describes explicitly selected debug boxes and observed-range strata:

```json
[
  {
    "name": "edge",
    "azimuth_deg": 0,
    "elevation_deg": 0,
    "half_width_deg": 0.25,
    "near_max_m": 2.5,
    "background_min_m": 3.5
  }
]
```

Optionally pin `expected_training_returns` and `expected_evaluation_returns` to
catch changed source selections. Thresholds label measured **range groups**, not
certified static foreground/background objects. Do not change the boxes or scoring
policy after inspecting a candidate to manufacture a pass.

```sh
python examples/reconstruction/edge_regression.py freeze \
  --training /path/to/training.npz \
  --evaluation /path/to/later.npz \
  --cases /path/to/cases.json \
  --mesh baseline=/path/to/baseline.ply \
  --mesh candidate=/path/to/candidate.ply \
  --output local-assets/frozen-edges

python examples/reconstruction/edge_regression.py check \
  --fixtures local-assets/frozen-edges \
  --mesh /path/to/candidate.ply \
  --against baseline \
  --report local-assets/candidate-edge-check.json
```

Training uses Nx3 finite `train` points and integer `train_second` elapsed
seconds in one stationary sensor frame; evaluation uses an Nx3 `points`
array of actual valid returns in the same stationary frame, range 0.3–30 m.
Each local fixture stores cropped raw training evidence, exact later directions
and ranges, reference first-hit ranges and per-ray classifications. A manifest
pins source/mesh/evaluator SHA-256, source counts, angular bounds, scoring policy,
Open3D version and checksums of every fixture array file. Replay needs only the
frozen bundle and a mesh; it does not need the original full capture or decoder.
Existing bundles/reports cannot be overwritten by the CLI.

Each measured ray has exactly one category:

| Category | Meaning |
|---|---|
| Missing | No first hit within the 30 m evaluation limit |
| Correct | First-hit range within the inclusive 20 mm tolerance |
| False foreground | First intersection more than 20 mm closer than measured |
| False background | First intersection more than 20 mm farther than measured |

Below-minimum first hits remain occluders and are counted explicitly. NaN backend
outputs fail validation. Reports separate near/background range groups and count
near proxies occluding measured background, far hits leaking through measured
near surfaces, severe errors and each offending ray's direction/ranges.

The diagnostic gate permits recovery of missing/wrong rays but **does not allow
improvements elsewhere to compensate for a newly early, late or missing ray, or
the loss of a correct ray**. A failing candidate writes its JSON report and exits 1.
A mesh replayed against its own frozen reference should pass with identical
classifications. Thus a candidate may reproduce its snapshot perfectly while
still failing the nonregression comparison against another mesh.

These are deliberately strict local debugging gates, not a calibrated physical
acceptance standard. Selected/reused cases are regression fixtures, not a blind
accuracy benchmark. Private arrays, source meshes and captures are never added
to Git or packaged releases; portable synthetic tests cover the same evaluator.
