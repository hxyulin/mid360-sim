<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Stationary hardware capture validation (2026-10-21)

The fitted official-table pattern is **not validated as a calibrated hardware
trajectory** by this recording. Its broad four-lane scan structure is useful,
but the fixed frequencies, direction convention and exact 12-second recurrence
must not be assumed to describe this physical device.

A subsequent [hardware-derived fit](hardware-fit.md) now separates learned optics
from mean clocks and a bounded measured-clock reconstruction. The results below
remain the validation of the original official-table profile.

## Input and integrity

The owner supplied a native Livox SDK callback capture, `stationary_01.bin`,
recorded with the sensor untouched on a table for about 300 s. It was fetched
from the owner's capture machine. SHA-256 matched
before/after transfer:

```text
82ef4b05ad71132999e522aae7b6f927db58a5570876b865a407a9d0faaa5b69
```

Decoded with TriLIO's existing `Decoder(allow_zero_crc=True)`, decoder SHA-256:

```text
ae56ec5f643809ef9a8244c1d8762250872da747e7f3594d321a1b2ad52d3a0a
```

Every record was passed through the decoder, including IMU records. The point
packets have absent (zero) checksums: opting in accepts their absence, **not
checksum verification**. The transfer hash establishes identical bytes, not the
integrity of their original acquisition. The decoder counted 624,993 absent
checksums; other packets passed its checksum branch. No raw capture, private
TriLIO source or sampled return buffers are redistributed by this repository.

| Quantity | Measured |
|---|---:|
| Point packets | 624,993 |
| Emissions, including zero returns | 59,999,328 |
| Valid Cartesian returns | 46,497,769 |
| Zero returns | 13,501,559 |
| IMU samples | 60,000 |
| Point-packet spacing | 480,000 ns, no discontinuities |
| Returns per point packet | 96 |
| Last-minus-first time inside each packet | 475,000 ns |
| Nominal per-emission spacing | 5,000 ns |
| First-to-last point span | 299.996635 s |

The packet interval field is in 100 ns units and describes last minus first:
475,000 / 95 = 5,000 ns. Zero returns retain their positions in the global
emission index; dropping them and renumbering would corrupt this analysis.
The device uses its own clock domain; no callback-arrival timing was fitted.
Reflectivity and raw tags were retained in sampled data, but no unverified tag
classification was applied. Most sampled valid returns have tag zero, so the
simulator's constant `0x10` tag remains a synthetic policy, not a reproduction
of this device's quality flags. Return-derived directions can still contain
measurement errors/outliers; the recurrence statistics use robust percentiles.

## Direction and temporal findings

Directions were normalized from Cartesian returns in sensor coordinates.
Returns closer than 2 m were excluded from phase/recurrence measurements to
reduce millimetre coordinate quantization and close-surface artifacts. Zero
returns supply no direction and cannot validate unobserved coverage.

The initial four seconds were retained densely; thereafter one whole packet
in 40 was retained, preserving all sample positions/lanes within each packet.
Carrier phase was measured in one-second bins, then unwrapped and regressed:

| Quantity | Current official-table fit | Recording estimate |
|---|---:|---:|
| Azimuth carrier, in current sensor FLU convention | negative 182.416667 Hz | positive 181.004102 Hz |
| Elevation fundamental | 10 Hz | 9.807871 Hz |
| Exact same-index recurrence at 12 s | identical directions | median separation 50.02 degrees |

The sense difference is a coordinate/trajectory finding, not a determination of
physical motor rotation. Both azimuth reflections, sweep senses and cyclic/
reversed lane permutations were explored when aligning the existing template.
These are effective direction-signal carriers, not measured encoder frequencies.
The six decimals make the analysis reproducible; they are not calibrated motor
specifications or a guarantee of another device's behaviour.

The 12-second test compares observed ray pairs separated by exactly 2,400,000
emission indices. Both members must have valid >=2 m returns. Their median
separation was 50.02 degrees and 95th percentile 55.95 degrees (256,687 evaluated
pairs after deterministic thinning). Initial phase and a rigid sensor mounting
rotation cannot explain a same-index recurrence failure: neither changes the
angular separation of the pair. Scene filtering limits which pairs are available.
This disproves the exact recurrence for this capture, not every possible period.

Relative to constant 181 Hz and 9.8 Hz reference carriers, phase accumulates
through the recording. After fitting mean rates, the 95th-percentile absolute
phase residuals remain about 13.17 and 16.90 degrees. These include sample
selection, optical modulation and estimator bias; attributing all residuals to
motor drift would require a better fitted optical model or encoder information.

A supplementary template-alignment experiment used the first second to fit
rates, initial phases, lane/reflection convention and constant orientation. On
held-out samples from seconds 1-4 it still had roughly 2.9-degree angular RMS
error. This is a local numerical alignment, not a proven global best fit.
It cannot support the sub-0.16-degree accuracy previously measured **against
Livox's published CSV**. That earlier held-out result remains valid for the CSV,
not for this hardware capture. This initial audit did not replace coefficients;
the later [hardware-derived fit](hardware-fit.md) and representative nominal
profile are separate developments.

## Stationarity and limitations

Mean IMU specific force in the recording's IMU axes was approximately
[-0.011823, -0.007452, 0.992820] g, implying about 0.806 degrees of apparent tilt
if acceleration bias and LiDAR/IMU misalignment are ignored. This is not a precise
table-level measurement. Tilt changes scene/return visibility, but does not
explain the sensor-frame scan direction or recurrence mismatch.

A stationary recording is valuable for trajectory and timing checks. It does
not validate motion distortion, reflective-response physics or independent
angular ground truth: directions here are derived from measured Cartesian XYZ.
This is one device/firmware/session, with missing-checksum point packets.

## Reproduce

Use Python 3.9-3.12 in an isolated development environment, install
`requirements-analysis.txt`, and supply a local TriLIO checkout's `tools/`
directory. The decoder remains external; no ROS installation is necessary.
The output directory must be new, preventing accidental evidence overwrites.

```sh
python scripts/analyze-stationary-capture.py /path/to/stationary_01.bin \
  --decoder-tools /path/to/TriLIO/tools --allow-zero-crc \
  --output local-assets/stationary-validation
python scripts/report-recorded-pattern.py local-assets/stationary-validation
```

The first script validates every record, requires the expected 96-return/5 us
format and continuous point packets, saves the direction-analysis samples and
counts/checksums. The second produces `phase-report.json` and
`hardware-validation.png`. It reproduces the carrier/recurrence findings above;
the supplementary nonlinear template alignment remains exploratory scratch,
not a production calibration routine.

The subsequent hardware-derived fit learns lane geometry on the initial training
interval and validates disjoint later packets using separate measured phase
anchors. Keep the official-table reference profile separately.
Do not force rational clocks or add random per-frame offsets merely to conceal
phase mismatch. Realistic geometry/IMU timing remains necessary for navigation.
