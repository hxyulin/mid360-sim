// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Capture-derived lane optics with explicitly separate clock models.
//!
//! The mean clock is an unbounded representative generator, not a forecast of
//! hardware phase. Capture replay interpolates sparse measured clock corrections
//! and is bounded to the recording; neither profile loops the official CSV.
use crate::{coefficients::TERMS, hardware_data::*};
/// Last valid zero-based emission index of the fitted stationary recording.
pub const LAST_CAPTURE_SAMPLE: u64 = CAPTURE_LAST_SAMPLE;
/// Number of sparse clock calibration knots, not stored per-point directions.
pub const CLOCK_KNOT_COUNT: usize = CLOCK_KNOTS.len();
/// Capture-derived four-lane optical fit. No world/table tilt is baked into it.
/// Input phases belong to the caller, so other motor/clock models can reuse it.
#[derive(Clone, Copy, Debug, Default)]
pub struct HardwareOptics;
impl HardwareOptics {
    /// Evaluate a lane (0..4) at explicit carrier phases, radians. Returns `None`
    /// for invalid lanes or nonfinite phases. Output is unit sensor FLU.
    ///
    /// ```
    /// let ray = mid360_sim::hardware::HardwareOptics.direction(0, 0., 0.).unwrap();
    /// assert!((ray.iter().map(|v| v*v).sum::<f64>() - 1.).abs() < 1e-12);
    /// ```
    pub fn direction(&self, lane: u8, spin_rad: f64, sweep_rad: f64) -> Option<[f64; 3]> {
        let coefficients = HARDWARE_COEFFICIENTS.get(usize::from(lane))?;
        if !spin_rad.is_finite() || !sweep_rad.is_finite() {
            return None;
        }
        let spin = spin_rad.rem_euclid(std::f64::consts::TAU);
        let sweep = sweep_rad.rem_euclid(std::f64::consts::TAU);
        let mut azimuth = spin + coefficients[0][0];
        let mut elevation = coefficients[0][1];
        for (i, &(j, k)) in TERMS.iter().enumerate() {
            let (sin, cos) = (f64::from(j) * spin + f64::from(k) * sweep).sin_cos();
            azimuth += coefficients[1 + i][0] * cos + coefficients[1 + TERMS.len() + i][0] * sin;
            elevation += coefficients[1 + i][1] * cos + coefficients[1 + TERMS.len() + i][1] * sin;
        }
        let (sa, ca) = azimuth.sin_cos();
        let (se, ce) = elevation.sin_cos();
        Some([ce * ca, ce * sa, se])
    }
}
/// Exact duration of the nominal clock's common cycle, seconds (about 18.2 h).
/// Chosen for simulation coverage, not inferred as a physical motor recurrence.
pub const NOMINAL_CYCLE_S: u64 = 65_521;
/// Number of 20 us groups in the nominal common cycle.
pub const NOMINAL_CYCLE_GROUPS: u64 = NOMINAL_CYCLE_S * 50_000;
/// Nominal spin rate, Hz; an intentionally simple simulation choice.
pub const NOMINAL_SPIN_HZ: f64 = 181.;
/// Exact sweep revolutions in the chosen 65,521-second nominal clock cycle.
pub const NOMINAL_SWEEP_TURNS_PER_CYCLE: u64 = 642_697;
/// Nominal sweep rate, Hz, approximately 9.809023061.
pub const NOMINAL_SWEEP_HZ: f64 = NOMINAL_SWEEP_TURNS_PER_CYCLE as f64 / NOMINAL_CYCLE_S as f64;
/// Compact representative scan with quarter-degree optics and an 18.2 h cycle.
/// Shape approximation error is separate from optional measurement noise.
#[derive(Clone, Copy, Debug, Default)]
pub struct NominalPattern;
impl NominalPattern {
    /// Unit sensor FLU ray at a global emission index. Lanes share a 20 us phase
    /// step; integer modular clocks avoid loss of phase precision in long runs.
    ///
    /// ```
    /// let d = mid360_sim::hardware::NominalPattern.direction(200_000);
    /// assert!((d.iter().map(|v| v*v).sum::<f64>() - 1.).abs() < 1e-12);
    /// ```
    pub fn direction(&self, sample_index: u64) -> [f64; 3] {
        let (spin, sweep) = Self::phases(sample_index / 4);
        Self::at_phases((sample_index % 4) as usize, spin, sweep)
    }

    fn phases(group: u64) -> (f64, f64) {
        let group = group % NOMINAL_CYCLE_GROUPS;
        (
            std::f64::consts::TAU * ((group * NOMINAL_SPIN_HZ as u64) % 50_000) as f64 / 50_000.,
            std::f64::consts::TAU
                * ((group * NOMINAL_SWEEP_TURNS_PER_CYCLE) % NOMINAL_CYCLE_GROUPS) as f64
                / NOMINAL_CYCLE_GROUPS as f64,
        )
    }

    /// Evaluate all four lanes of a group, sharing clocks and harmonic sin/cos.
    /// The input may address any lane; its containing group is evaluated.
    pub fn group_directions(&self, sample_index: u64) -> [[f64; 3]; 4] {
        use crate::nominal_data::{NOMINAL_TERMS, QUARTER_DEGREE_COEFFICIENTS};
        let (spin, sweep) = Self::phases(sample_index / 4);
        let mut angles = QUARTER_DEGREE_COEFFICIENTS.map(|c| c[0].map(f64::from));
        for (i, &(j, k)) in NOMINAL_TERMS.iter().enumerate() {
            let (sin, cos) = (f64::from(j) * spin + f64::from(k) * sweep).sin_cos();
            for (angle, c) in angles.iter_mut().zip(QUARTER_DEGREE_COEFFICIENTS.iter()) {
                for (axis, value) in angle.iter_mut().enumerate() {
                    *value += f64::from(c[1 + i][axis]) * cos
                        + f64::from(c[1 + NOMINAL_TERMS.len() + i][axis]) * sin;
                }
            }
        }
        angles.map(|[a, e]| Self::ray(spin, a, e))
    }

    /// Fill caller-owned directions at `first + i*stride`, sharing four-lane work.
    /// Reject zero stride or index overflow before modifying the output slice.
    /// Empty output is allowed. Results match scalar evaluation bit for bit.
    pub fn fill_directions(
        &self,
        first: u64,
        stride: u32,
        output: &mut [[f64; 3]],
    ) -> Result<(), &'static str> {
        if stride == 0 {
            return Err("zero direction stride");
        }
        if let Some(last) = output.len().checked_sub(1) {
            u64::try_from(last)
                .ok()
                .and_then(|n| n.checked_mul(u64::from(stride)))
                .and_then(|n| first.checked_add(n))
                .ok_or("direction index overflow")?;
        }
        let mut cached = None;
        let mut rays = [[0.; 3]; 4];
        for (i, output) in output.iter_mut().enumerate() {
            let index = first + i as u64 * u64::from(stride);
            if cached != Some(index / 4) {
                rays = self.group_directions(index);
                cached = Some(index / 4);
            }
            *output = rays[(index % 4) as usize];
        }
        Ok(())
    }

    fn at_phases(lane: usize, spin: f64, sweep: f64) -> [f64; 3] {
        use crate::nominal_data::{NOMINAL_TERMS, QUARTER_DEGREE_COEFFICIENTS};
        let c = &QUARTER_DEGREE_COEFFICIENTS[lane];
        let mut azimuth = f64::from(c[0][0]);
        let mut elevation = f64::from(c[0][1]);
        for (i, &(j, k)) in NOMINAL_TERMS.iter().enumerate() {
            let (sin, cos) = (f64::from(j) * spin + f64::from(k) * sweep).sin_cos();
            azimuth +=
                f64::from(c[1 + i][0]) * cos + f64::from(c[1 + NOMINAL_TERMS.len() + i][0]) * sin;
            elevation +=
                f64::from(c[1 + i][1]) * cos + f64::from(c[1 + NOMINAL_TERMS.len() + i][1]) * sin;
        }
        Self::ray(spin, azimuth, elevation)
    }
    fn ray(spin: f64, azimuth: f64, elevation: f64) -> [f64; 3] {
        let unit = std::f64::consts::PI / 720.;
        let (sa, ca) = (spin + azimuth * unit).sin_cos();
        let (se, ce) = (elevation * unit).sin_cos();
        [ce * ca, ce * sa, se]
    }
}
/// Fitted optics driven by mean carrier rates and phases from calibration knots.
/// This generates representative directions indefinitely, but does not reproduce
/// the device's time-varying phase. Accuracy claims for capture replay do not apply.
#[derive(Clone, Copy, Debug, Default)]
pub struct HardwareMeanPattern;
impl HardwareMeanPattern {
    /// Unit sensor FLU ray at a global emission index; four lanes share a 20 us
    /// carrier step. No artificial short-period wrapping is applied.
    pub fn direction(&self, sample_index: u64) -> [f64; 3] {
        let time_s = (sample_index / 4) as f64 / 50_000.;
        HardwareOptics
            .direction(
                (sample_index % 4) as u8,
                std::f64::consts::TAU * MEAN_SPIN_HZ * time_s + MEAN_SPIN_OFFSET_RAD,
                std::f64::consts::TAU * MEAN_SWEEP_HZ * time_s + MEAN_SWEEP_OFFSET_RAD,
            )
            .expect("finite generated mean phases and valid lane")
    }
}
/// Fitted optics with a sparse, capture-derived clock trace for this one session.
/// Bounded reconstruction, not prediction of another recording or future motion.
#[derive(Clone, Copy, Debug, Default)]
pub struct HardwareCapturePattern;
impl HardwareCapturePattern {
    /// Reconstructed unit sensor FLU ray, or `None` beyond the recorded duration.
    /// Missing calibration knots are interpolated in time; outside the first/last
    /// knot the residual correction is held constant within the recording only.
    /// Table/gravity tilt is deliberately left to the scene's sensor mounting pose.
    pub fn direction(&self, sample_index: u64) -> Option<[f64; 3]> {
        if sample_index > LAST_CAPTURE_SAMPLE {
            return None;
        }
        let time_s = (sample_index / 4) as f64 / 50_000.;
        let upper = CLOCK_KNOTS.partition_point(|k| k[0] <= time_s);
        let correction = if upper == 0 {
            [CLOCK_KNOTS[0][1], CLOCK_KNOTS[0][2]]
        } else if upper == CLOCK_KNOTS.len() {
            let k = CLOCK_KNOTS[upper - 1];
            [k[1], k[2]]
        } else {
            let a = CLOCK_KNOTS[upper - 1];
            let b = CLOCK_KNOTS[upper];
            let blend = (time_s - a[0]) / (b[0] - a[0]);
            [a[1] + blend * (b[1] - a[1]), a[2] + blend * (b[2] - a[2])]
        };
        HardwareOptics.direction(
            (sample_index % 4) as u8,
            std::f64::consts::TAU * SPIN_REFERENCE_HZ * time_s + correction[0],
            std::f64::consts::TAU * SWEEP_REFERENCE_HZ * time_s + correction[1],
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn nominal_batch_matches_scalar_across_clock_wrap_and_chunks() {
        let p = NominalPattern;
        for first in [0, 7, NOMINAL_CYCLE_GROUPS * 4 - 31, u64::MAX - 100] {
            for stride in [1, 3, 5] {
                let mut rays = [[0.; 3]; 20];
                p.fill_directions(first, stride, &mut rays).unwrap();
                for (i, ray) in rays.iter().enumerate() {
                    assert_eq!(*ray, p.direction(first + i as u64 * u64::from(stride)));
                }
                let mut split = [[0.; 3]; 20];
                p.fill_directions(first, stride, &mut split[..7]).unwrap();
                p.fill_directions(first + 7 * u64::from(stride), stride, &mut split[7..])
                    .unwrap();
                assert_eq!(rays, split);
            }
        }
        // Avoid a nominal long period that still nearly repeats at ordinary
        // integer-second offsets (e.g. 9.850002 Hz nearly repeats at 20 s).
        for seconds in 1..=60 {
            let phase = (seconds * NOMINAL_SWEEP_TURNS_PER_CYCLE) % NOMINAL_CYCLE_S;
            let separation =
                phase.min(NOMINAL_CYCLE_S - phase) as f64 * 360. / NOMINAL_CYCLE_S as f64;
            assert!(separation > 3.7);
        }
        assert_eq!(
            p.direction(123),
            p.direction(123 + NOMINAL_CYCLE_GROUPS * 4)
        );
        let mut out = [[9.; 3]; 2];
        assert!(p.fill_directions(u64::MAX, 1, &mut out).is_err());
        assert_eq!(out, [[9.; 3]; 2]);
    }
    #[test]
    fn nominal_approximation_and_scan_diversity() {
        let mut sum = 0.;
        let mut count = 0;
        for lane in 0..4 {
            for a in 0..64 {
                for b in 0..64 {
                    let spin = std::f64::consts::TAU * f64::from(a) / 64.;
                    let sweep = std::f64::consts::TAU * f64::from(b) / 64.;
                    let exact = HardwareOptics.direction(lane, spin, sweep).unwrap();
                    let nominal = NominalPattern::at_phases(usize::from(lane), spin, sweep);
                    let dot = exact
                        .iter()
                        .zip(nominal)
                        .map(|(a, b)| a * b)
                        .sum::<f64>()
                        .clamp(-1., 1.);
                    let error_deg = dot.acos().to_degrees();
                    assert!(error_deg < 0.9);
                    sum += error_deg * error_deg;
                    count += 1;
                    assert!((nominal.iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 1e-12);
                }
            }
        }
        assert!((sum / f64::from(count)).sqrt() < 0.32);
        for offset in [200_000, 1_000_000, 2_400_000] {
            assert_ne!(
                NominalPattern.direction(0),
                NominalPattern.direction(offset)
            );
        }
        assert_eq!(
            crate::pattern::ScanPattern::default(),
            crate::pattern::ScanPattern::Nominal
        );
    }
    #[test]
    fn agrees_with_unseen_capture_directions() {
        // Normalized sensor directions, not raw positions/ranges, from withheld
        // packets of the owner-provided capture (provenance in docs/hardware-fit.md).
        // This bound is approximation error against measured XYZ, not ground truth.
        let fixtures = [
            (
                420000,
                [-0.6759644147638402, 0.7324894280232905, 0.08081675449491663],
            ),
            (
                420001,
                [-0.6912316236081563, 0.7142175115972543, 0.10996448813984502],
            ),
            (
                420002,
                [-0.7046951802533512, 0.6963588808024046, 0.13597430659988427],
            ),
            (
                420003,
                [-0.7177303121694163, 0.676807092615897, 0.1636928782140774],
            ),
            (
                1002240,
                [-0.6673027067694155, 0.4377926676004893, 0.6025318893913075],
            ),
            (
                1002241,
                [-0.6724217813049685, 0.39745097756910136, 0.6244050515939303],
            ),
            (
                1002242,
                [-0.6757451525504069, 0.35777027180419413, 0.6444912112804799],
            ),
            (
                1002243,
                [-0.6773096911575356, 0.3119268397349809, 0.6662981531694586],
            ),
            (
                6024960,
                [0.7474149079747412, -0.35295318043375834, 0.5628454563721768],
            ),
            (
                6024961,
                [0.7726222585345697, -0.34113711311633294, 0.5354253595708695],
            ),
            (
                6024962,
                [0.7938432054489918, -0.33090384128571937, 0.5102113414898037],
            ),
            (
                6024963,
                [0.8144038273822898, -0.3211927227290071, 0.48330284585444433],
            ),
            (
                12023040,
                [-0.7009573224086287, 0.7118578184608452, 0.04378673837924223],
            ),
            (
                12023041,
                [-0.7166691161723573, 0.6934594528523994, 0.07415770475401169],
            ),
            (
                12023042,
                [-0.729931370739782, 0.6759423640282597, 0.10149933261766199],
            ),
            (
                12023043,
                [-0.7429449942878318, 0.6565829399440465, 0.130121398843873],
            ),
            (
                30001920,
                [-0.4040252496929958, -0.5794769553467134, 0.7077923818695815],
            ),
            (
                30001921,
                [
                    -0.40059768301513576,
                    -0.5548966097461013,
                    0.7291167594118159,
                ],
            ),
            (
                30001922,
                [
                    -0.39563002390047775,
                    -0.5308762347140331,
                    0.7494313228070724,
                ],
            ),
            (
                30001923,
                [-0.3883019738227093, -0.50282267633448, 0.7722635128563428],
            ),
            (
                59800320,
                [0.0562514948468133, -0.964574813274864, 0.2577425050767921],
            ),
            (
                59800321,
                [0.0290500751682531, -0.9694291213405752, 0.24364579173373568],
            ),
            (
                59800322,
                [
                    0.0033440644486147607,
                    -0.9731227545468953,
                    0.23026272346175922,
                ],
            ),
        ];
        for (index, actual) in fixtures {
            let predicted = HardwareCapturePattern.direction(index).unwrap();
            let dot = actual
                .iter()
                .zip(predicted)
                .map(|(a, b)| a * b)
                .sum::<f64>()
                .clamp(-1., 1.);
            assert!(
                dot.acos().to_degrees() < 0.8,
                "capture direction at {index}"
            );
        }
    }
    #[test]
    fn bounded_capture_and_independent_mean_clock() {
        let capture = HardwareCapturePattern;
        for index in [0, 1, 2, 3, 20_000, 2_400_000, LAST_CAPTURE_SAMPLE] {
            let d = capture.direction(index).unwrap();
            assert!((d.iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 1e-12);
        }
        assert!(capture.direction(LAST_CAPTURE_SAMPLE + 1).is_none());
        assert!(capture.direction(u64::MAX).is_none());
        assert_ne!(capture.direction(0), capture.direction(2_400_000));
        let d = HardwareMeanPattern.direction(LAST_CAPTURE_SAMPLE + 1);
        assert!((d.iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 1e-12);
        assert!(HardwareOptics.direction(4, 0., 0.).is_none());
        assert!(HardwareOptics.direction(0, f64::NAN, 0.).is_none());
    }
}
