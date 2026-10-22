<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Validate material and edge measurements

Use the offline workflow in `examples/calibration/` before proposing calibrated
material, range-noise or beam-footprint effects. It audits emission completeness,
source checksums, experimental metadata and held-out sessions. It reports
conditional detection/range statistics and scores a training-only mean-bias
baseline. It does not fit a material law or change the simulator.

There are currently no known-target captures here with independent 905 nm
reflectance, surveyed edges and device/temperature metadata sufficient for those
models. The [stationary capture](../hardware-validation.md) and
[optics fits](../hardware-fit.md) remain evidence for their documented questions.
Their Cartesian returns, visual colour and raw reflectivity do not supply the
missing physical calibration.

## Exercise the workflow without hardware

Python 3.9+ standard library only; use new local directories:

```sh
python3 examples/calibration/synthetic.py \
  --output local-assets/material-check
python3 examples/calibration/evaluate.py \
  --manifest local-assets/material-check/manifest.json \
  --samples local-assets/material-check/samples.csv \
  --output local-assets/material-check/report.json
python3 -m unittest discover -s examples/calibration -v
```

This generates two conditions (plane/edge), each with disjoint fake training and
validation sessions and 1,000 emissions per split. All values are fabricated.
The expected detection fraction is 0.8; training has approximately +20 mm bias,
validation +30 mm. Applying the training bias leaves approximately +10 mm on
validation. `status: synthetic_tool_check` and `calibrated_model: false` remain
explicit even when sample-count checks pass. Synthetic success is a tooling
check, not sensor validation.

## Collect independent measurements

1. Survey target planes and foreground/background edges independently of the
   LiDAR being tested. Record range, incidence and edge-position uncertainty.
   Compute each expected ray intersection from that reference, not its observed
   XYZ. Isolate motion, mounting and timestamp uncertainty before attributing
   residuals to optics.
2. Use targets with measured **905 nm** reflectance and uncertainty, identifying
   the measurement method and target. Visual RGB and the device's raw intensity
   byte are not reflectance measurements. Vary distance/incidence with other
   conditions controlled; record actual device temperature, serial and firmware.
3. Separate complete recording sessions into training and validation before
   fitting. Repeat conditions across sessions and eventually devices/temperatures.
   Adjacent packets from one recording are not independent calibration/validation.
4. Export every eligible emission, including misses, using a fixed index/stride
   selection declared before looking at return validity. Keep original indices,
   device timestamps, raw reflectivity and tags; interpret no quality flag without
   a verified definition. Record the original capture/decoder hashes and survey/
   reflectance calibration records alongside the manifest.

Raw captures, private decoders and generated reports stay outside Git. The
workflow consumes the public CSV schema below; it needs no private decoder.

## Input contract

Start from the generated manifest as a schema example, replacing **all** synthetic
values and setting `evidence_kind: measured` only for actual measurements. Its
`samples_sha256` must match the exact CSV bytes. The report also hashes the
manifest, retains its metadata and refuses to overwrite an existing output.

The manifest requires:

- `schema: 1`, `evidence_kind: synthetic|measured`, `samples_sha256`.
- `device`: `model`, `serial`, `firmware`, `wavelength_nm: 905`.
- `reference`: `method`, `range_uncertainty_m`, `incidence_uncertainty_deg`, and
  `edge_position_uncertainty_m` for edge conditions.
- `conditions`: unique `id`, `target_id`, measured `reflectance_905nm` in `[0,1]`,
  `reflectance_uncertainty`, `reflectance_method`, `incidence_deg` in `[0,90]`,
  `nominal_range_m`, and boolean `edge`. Edge conditions also require
  `background_target` with `target_id`, `reflectance_905nm`,
  `reflectance_uncertainty` and `reflectance_method` for the second material.
- `trials`: unique `id`, `condition`, `split: train|validation`, `session`, actual
  `temperature_c`, `first_index`, positive selection `stride`, and `emissions`.
  Every condition needs both splits, with globally disjoint session identities.

CSV columns, in this order:

```text
trial,sample_index,time_ns,expected_range_m,measured_range_m,raw_reflectivity,raw_tag,expected_region,edge_offset_m
```

For trial row `i`, the index must be `first_index + i*stride`, and original
`time_ns` must increase. Emission identities cannot be reused within a session.
The declared count must match exactly. Use a blank `measured_range_m` for a miss,
not zero or an omitted row; positive measured ranges require raw byte fields in
`[0,255]`. Preserve available raw bytes on misses too. Every row requires a
positive independent expected range, including rays that the device missed.

Plane trials use `expected_region: target` and blank `edge_offset_m`. Edge trials
use surveyed `foreground|background` truth and signed `edge_offset_m`: positive
is inside foreground, negative is background. An exact-zero edge is intrinsically
ambiguous and should be interpreted with survey uncertainty. Offsets must come
from reference geometry, not an inferred edge in the measured point cloud.
Raw intensity/tag values are checked but never converted to reflectance or used
to remove samples.

## Interpret the report

Per-condition train/validation statistics include detection fraction (all eligible
emissions in the denominator), misses, range bias, sample standard deviation,
RMS and early/late residual counts. The default 20 mm classification tolerance
is a configurable analysis choice, **not a device precision calibration**.
`reference_range_uncertainty_exceeds_tolerance` flags an unsuitable reference for
that threshold. Edge profiles group these statistics by surveyed offset/region;
these are observed transition profiles, not a fitted beam diameter or mixed-return
origin classifier.

Only training returns estimate the mean-bias baseline; the corrected validation
residuals never refit that mean. Metrics without enough detected returns are
null. `--min-emissions` defaults to 1,000 per condition/split; insufficient counts
retain a report and exit with status 2. The threshold is a workflow guard, not a
power calculation or proof of independent samples. Malformed metadata, missing
emissions, checksum mismatches and session leakage fail validation.

The descriptive Wilson intervals assume independent Bernoulli detections;
correlated emissions can invalidate that interpretation. Reference, alignment,
clock, angular and range errors can all contribute to edge/range residuals.
Multiple held-out sessions and a documented uncertainty/model analysis are
needed before claiming a calibrated effect. Passing this workflow alone does
not establish one, and every report retains `calibrated_model: false`.
