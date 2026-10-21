<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Hardware-derived optics and clock reconstruction

This profile was fitted to the owner's 300-second stationary recording described
in [hardware validation](hardware-validation.md). It separates reusable lane
geometry from carrier phase. It is one device/session, not a manufacturer
calibration or independent angular ground truth.

## Model and validation split

Each of four lanes has azimuth `spin + Fourier correction` and a Fourier elevation
function. The 22 frequency pairs are `(0,k), k=1..7` and `(1,k), k=-7..7`.
Constants plus sine/cosine coefficients for two angles require 360 f64 values,
2,880 bytes. There is no stored per-point direction table.

The initializer fits clock rates/drift and lane geometry to selected, unflagged
(raw tag zero) >=2 m returns in the first two seconds, excluding every source
packet whose zero-based index modulo 5 is 1. It deterministically thins this
geometry seed to about 2,500 observations per lane. This seed is training data.

Subsequent optical refinement uses only phase-calibration packets before 30 s.
Clock calibration uses source packet index modulo 80 equal to zero: nominally one
0.48 ms packet every 38.4 ms. There are 7,004 usable clock knots after rejecting
packets with no eligible returns. Calibration stays in sensor coordinates.

For each calibration packet, the fitter estimates spin/sweep corrections to
181.004 Hz and 9.8079 Hz reference clocks. These reference rates define coordinates
for residual phase, not exact physical motor speeds. Shared optics and packet
phase are alternately fitted for 12 iterations, with a unit-weight second-
difference phase penalty. The regularizer and phase initialization/interpolation
are disconnected at 30 s so future calibration observations do not update the
training side. All phase initialization also uses calibration packets only.

Validation excludes every phase-calibration packet. Before 2 s, it also excludes
all geometry-seed training packets, keeping only packet indices modulo 5 equal
to 1. No evaluation direction participates in coefficient or phase initialization
or fitting. From 30 s onward, geometry is held fixed while clock knots are still
measured from separate calibration packets.

**This is conditional reconstruction of unseen rays between observed clock
anchors. It is not blind prediction of the remaining 270 seconds.** The distinction
matters: a constant clock fitted to the first seconds loses phase over time.

| Withheld interval | Evaluated rays | Median error | Angular RMS | 95th percentile |
|---|---:|---:|---:|---:|
| 0-2 s | 48,995 | ~0.07° | ~0.10° | ~0.19° |
| 2-4 s | 237,093 | ~0.07° | ~0.10° | ~0.18° |
| 4-30 s | 40,128 | ~0.08° | ~0.13° | ~0.26° |
| 30-60 s | 43,956 | ~0.09° | ~0.15° | ~0.29° |
| 60-120 s | 92,090 | ~0.08° | ~0.13° | ~0.25° |
| 120-300 s | 274,695 | ~0.08° | ~0.13° | ~0.24° |

Combined later validation (30-300 s): **410,741 rays, median 0.0831°, RMS 0.1282°,
95th percentile 0.2512°**. These are errors against normalized measured Cartesian
returns, not independent optical encoder truth. Zero returns have no direction;
coverage behind missing returns cannot be validated. Noisy/near returns are
excluded by the declared filter. No perfect accuracy is claimed outside the
observed angular/time domain.

## Runtime profiles

- `nominal`: default representative optics, rounded to quarter-degree integers
  and pruned to 10 harmonics, with a chosen long-cycle fractional clock.
- `official-table`: original published-table reference, selectable explicitly.
- `hardware-mean`: learned lane optics with mean rates/phases fitted to the
  calibration trace. Generates indefinitely and does not force a 12 s repeat.
  It is a representative scan generator; reconstruction error numbers above
  **do not apply** to this clock mode or to future hardware phase.
- `hardware-capture`: the learned optics plus 7,004 sparse recorded phase knots,
  interpolated in time. Bounded to emission indices 0..=59,999,327; it rejects
  overrun instead of wrapping. The complete demo frames must fit within
  299.996640 s. Missing endpoint anchors hold their residual phase constant.

The bounded profile's coefficient/knot buffers total 170,976 bytes: 2,880 bytes of optics
plus 168,096 bytes of time/two-phase knots. It reconstructs a 60-million-emission
session without storing its returns or ray directions. The general mean-clock
model needs only the optics and four mean-clock constants; the shipped crate
also includes the bounded trace for regression/replay.

`HardwareOptics::direction(lane, spin_rad, sweep_rad)` exposes the learned shape
with caller-owned clocks. `HardwareMeanPattern` and `HardwareCapturePattern`
provide convenience generators. `Sensor::sample_pattern_into` selects a
`ScanPattern` while retaining explicit time, generic effects and scene ownership.
Invalid/unsupported windows leave output unchanged. The ray/point ROS frames
and acquisition timestamps are unchanged by profile selection.

```sh
pixi run run --pattern hardware-mean --frames 100
pixi run run --pattern hardware-capture --frames 100
pixi run smoke --pattern hardware-capture --stride 3
```

## Table tilt and physical interpretation

The IMU suggests roughly 0.81 degrees of apparent gravity tilt, subject to bias
and LiDAR/IMU extrinsics. **No leveling rotation was baked into the optical fit.**
Local Cartesian directions already describe the ray in the sensor's own frame;
a fixed mounting rotation preserves pairwise angular errors. Table tilt changes
which scene surfaces are visible, not the intrinsic temporal scan law.

For the room demo, explicit `--mount-roll-deg` and `--mount-pitch-deg` apply
`Rz(yaw) * Ry(pitch) * Rx(roll)` to scene rays. They default to zero. In another
backend, apply the actual sensor mounting pose at query time. Do not silently
set a precise mount from this uncalibrated gravity estimate.

No reflectivity, tags, material response, range noise or IMU noise were calibrated
by this fit. Existing configurable synthetic effects remain explicit. Determining
whether clock changes come from motors, encoder processing or other effects
requires more hardware evidence. The next general-purpose realism improvement
would be a validated stochastic/physical clock model, rather than looping the
recorded trace or claiming its future phase is predictable.

## Reproduce

Use `requirements-analysis.txt` in a development environment. After the full
capture validation/sample extraction described in the previous guide:

```sh
python scripts/fit-recorded-pattern.py local-assets/stationary-validation \
  --output local-assets/hardware-fit
python scripts/plot-hardware-fit.py local-assets/stationary-validation \
  local-assets/hardware-fit/hardware-profile.json \
  --output local-assets/hardware-fit/hardware-fit.png
```

The output directory must be new. The JSON records input/decoder checksums,
training/evaluation policy, coefficients, phase knots and interval statistics.
`hardware_data.rs` is the generated Rust model; install it as `src/hardware_data.rs`
and run `cargo fmt --all`. Raw captures and sampled measurements remain ignored.
The Rust runtime uses no Python/NumPy/ROS dependency for evaluating the profile.

Rust regressions compare normalized directions from deterministic withheld
capture packets, profile bounds, chunking/noise invariance and transactional
end-of-capture rejection. Real DDS smoke checks cover both hardware clocks and
the original profile; a mount test checks wall/ceiling/floor ray directions.

Independent [public-capture transfer checks](external-captures.md) now test the
frozen optics on Livox indoor and Elevator-LIO Office1. Their larger residuals
are not evidence for universal calibration. The full measured profiles remain
unchanged; the default nominal profile deliberately uses simpler optics.


## Default nominal simplification

For the intended navigation/game scale, the accepted policy is a representative
pattern rather than more per-device fitting. `NominalPattern`, `ScanPattern`'s
default, `Sensor::sample_into`, and the ROS demo now use quarter-degree integer
coefficients. Twelve harmonic terms become zero and are omitted: 10 remain
instead of 22. Packed optical coefficients plus frequency-pair indices use
356 bytes (336 + 20), versus 2,880 bytes of original f64 coefficients alone.
The full profiles/recorded trace are still available in the crate; this is not
an equivalent reduction in total binary size. Each nominal ray evaluates 10
harmonic sin/cos pairs instead of 22, not a measured whole-pipeline speedup.

Across a 180 by 180 phase grid in each lane (129,600 directions), simplification
adds 0.314 degrees RMS difference, 0.571 degrees p95 and 0.845 degrees maximum
relative to the unsimplified optics. This is a sampled approximation comparison,
not a certified continuous bound or physical noise distribution. Whole-degree
coefficient rounding would add about 1.01 degrees RMS and 2.65 degrees maximum
on that grid, so quarter-degree coefficients retain more useful shape detail.

The nominal clock now uses 181 Hz spin and exactly 642697/65521 Hz sweep
(approximately 9.809023061 Hz), giving an exact common cycle of 65,521 seconds
(18 h 12 min 1 s). These are explicit simulation choices within the broad
observed carrier neighbourhood, not fitted hardware frequencies. Integer
modular group phases keep precision stable across long runs. Using exactly
181 and 10 Hz would force an identical scan every second. The unsimplified
`hardware-mean` profile retains its measured mean rates for reference. No new drift,
phase jitter or random angular noise is introduced by simplification. Errors
in reproducing a hardware scan direction are not added to simulated returns:
scene queries intersect the generated rays consistently.

For scale, one degree corresponds to about 17 cm transverse displacement at
10 m. That can change the sample distribution near edges, so this remains a
representative navigation sensor rather than a guarantee for precision target
alignment. The accepted approximation does not invent 17 cm of range error.

Reproduce the integer table and optional transfer-check profile:

```sh
python scripts/simplify-hardware-optics.py /path/hardware-profile.json \
  --rust-output local-assets/nominal_data.rs \
  --report local-assets/nominal-report.json \
  --profile-output local-assets/nominal-profile.json
```

The source table lives in `src/nominal_data.rs`; the original measured fits and
bounded replay stay intact. Use `--pattern hardware-mean` or `hardware-capture`
for unsimplified reference comparisons.

On the same independent transfer-check windows, the nominal optics give
1.134 degrees RMS / 1.922 degrees p95 for Livox indoor (61,473 evaluated rays),
and 0.585 degrees RMS / 0.951 degrees p95 for Office1 (25,431 rays, recorded
offsets). Unsimplified optics gave 1.079 and 0.503 degrees RMS respectively.
These checks still fit separate local clock phases/rates: the results validate
conditional shape transfer, not the nominal mean clock's future hardware phase.


The nominal batch path shares clock/harmonic evaluation across the four lanes.
`NominalPattern::fill_directions` fills a caller-owned slice without allocation;
`group_directions` supports amortized acquisition. Scalar, chunked and strided
results remain bit-identical, including across the exact long-cycle boundary.
An [Isaac Sim adapter](install/isaac-sim.md) consumes an exported
copy of these same coefficients/clocks, with Rust reference rays for GPU checks.

The sweep numerator is the nearest integer to
`65521 * (10 - (3 - sqrt(5))/4)`. This golden-ratio-based detuning is a simulation
choice that avoids the almost-20-second recurrence of a rate near 9.85 Hz.
Integer-second sweep phase separations in seconds 1..60 are at least 3.78 degrees;
this diagnostic is not proof that no close recurrence exists at other offsets.
