// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Compact empirical fit of the official four-second angle table.
use crate::coefficients::*;
/// Nominal point cadence (200,000 first-return emissions/s), not calibrated lane timing.
pub const POINT_INTERVAL_NS: u64 = 5_000;
/// Inferred rational model period; not an established hardware recurrence.
pub const MODEL_PERIOD_SAMPLES: u64 = 2_400_000;
/// Stateless four-lane, two-clock Fourier pattern. Timing and coefficients are
/// empirical hypotheses; see docs/realism.md.
#[derive(Clone, Copy, Debug, Default)]
pub struct Mid360Pattern;
impl Mid360Pattern {
    /// Sensor FLU unit direction at a global emission index. Never reset per packet.
    pub fn direction(&self, sample_index: u64) -> [f64; 3] {
        let lane = (sample_index % 4) as usize;
        let group = (sample_index % MODEL_PERIOD_SAMPLES) / 4;
        let time_s = group as f64 / 50_000.;
        let spin = std::f64::consts::TAU * (2189. / 12.) * time_s;
        let sweep = std::f64::consts::TAU * 10. * time_s;
        let mut azimuth = COEFFICIENTS_RAD[0][lane] - spin;
        let mut elevation = COEFFICIENTS_RAD[0][lane + 4];
        for (i, &(j, k)) in TERMS.iter().enumerate() {
            let (sin, cos) = (f64::from(j) * spin + f64::from(k) * sweep).sin_cos();
            let c = &COEFFICIENTS_RAD[1 + i];
            let s = &COEFFICIENTS_RAD[1 + TERMS.len() + i];
            azimuth += c[lane] * cos + s[lane] * sin;
            elevation += c[lane + 4] * cos + s[lane + 4] * sin;
        }
        let (sa, ca) = azimuth.sin_cos();
        let (se, ce) = elevation.sin_cos();
        [ce * ca, ce * sa, se]
    }

    /// Checked nominal emission time relative to the acquisition epoch, nanoseconds.
    pub fn point_offset_ns(index: u64) -> Option<u64> {
        index.checked_mul(POINT_INTERVAL_NS)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn matches_published_reference_directions() {
        // Measurement excerpts: pinned Livox CSV, MIT notice in NOTICE.md.
        for (index, azimuth, zenith) in [
            (0, 268.99_f64, 37.838_f64),
            (1, 269., 40.825),
            (2, 269., 43.581),
            (3, 269., 46.152),
            (400000, 326.13, 40.04),
            (400003, 331.64, 44.277),
            (799996, 27.654, 44.261),
            (799999, 33.069, 40.061),
        ] {
            let a = azimuth.to_radians();
            let z = zenith.to_radians();
            let actual = [z.sin() * a.cos(), z.sin() * a.sin(), z.cos()];
            let predicted = Mid360Pattern.direction(index);
            let dot = actual
                .iter()
                .zip(predicted)
                .map(|(x, y)| x * y)
                .sum::<f64>()
                .clamp(-1., 1.);
            assert!(dot.acos().to_degrees() < 0.16);
        }
    }
    #[test]
    fn unit_directions_and_period() {
        let p = Mid360Pattern;
        for i in [0, 1, 2, 3, 20000, 799999, 800000, u64::MAX - 2_400_000] {
            let d = p.direction(i);
            assert!((d.iter().map(|x| x * x).sum::<f64>() - 1.).abs() < 1e-12);
            assert_eq!(d, p.direction(i + MODEL_PERIOD_SAMPLES));
        }
        assert_eq!(Mid360Pattern::point_offset_ns(u64::MAX), None);
    }
}
