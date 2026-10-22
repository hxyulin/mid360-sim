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
    pub(crate) fn group_directions(&self, sample_index: u64) -> [[f64; 3]; 4] {
        let group = (sample_index % MODEL_PERIOD_SAMPLES) / 4;
        let time_s = group as f64 / 50_000.;
        let spin = std::f64::consts::TAU * (2189. / 12.) * time_s;
        let sweep = std::f64::consts::TAU * 10. * time_s;
        let mut angles = std::array::from_fn::<_, 4, _>(|lane| {
            [
                COEFFICIENTS_RAD[0][lane] - spin,
                COEFFICIENTS_RAD[0][lane + 4],
            ]
        });
        for (i, &(j, k)) in TERMS.iter().enumerate() {
            let (sin, cos) = (f64::from(j) * spin + f64::from(k) * sweep).sin_cos();
            let c = &COEFFICIENTS_RAD[1 + i];
            let s = &COEFFICIENTS_RAD[1 + TERMS.len() + i];
            for (lane, angle) in angles.iter_mut().enumerate() {
                angle[0] += c[lane] * cos + s[lane] * sin;
                angle[1] += c[lane + 4] * cos + s[lane + 4] * sin;
            }
        }
        angles.map(|[a, e]| {
            let (sa, ca) = a.sin_cos();
            let (se, ce) = e.sin_cos();
            [ce * ca, ce * sa, se]
        })
    }

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
/// Explicit scan profile; the default is a compact representative hardware pattern.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ScanPattern {
    /// Quarter-degree hardware optics with mean clocks, for general simulation.
    #[default]
    Nominal,
    /// Original fit of the published four-second table; inferred 12 s period.
    OfficialTable,
    /// Hardware-derived optics with mean clocks; representative, not exact replay.
    HardwareMean,
    /// Hardware-derived optics with bounded 300 s measured clock reconstruction.
    HardwareCapture,
}
impl ScanPattern {
    // All profiles have contiguous support starting at zero.
    pub(crate) fn supports(self, last: u64) -> bool {
        self != Self::HardwareCapture || last <= crate::hardware::LAST_CAPTURE_SAMPLE
    }

    /// Unit sensor FLU direction, or `None` beyond a bounded profile's support.
    pub fn direction(self, sample_index: u64) -> Option<[f64; 3]> {
        match self {
            Self::Nominal => Some(crate::hardware::NominalPattern.direction(sample_index)),
            Self::OfficialTable => Some(Mid360Pattern.direction(sample_index)),
            Self::HardwareMean => {
                Some(crate::hardware::HardwareMeanPattern.direction(sample_index))
            }
            Self::HardwareCapture => {
                crate::hardware::HardwareCapturePattern.direction(sample_index)
            }
        }
    }
}
// Acquisition-local cache. Only dense strides evaluate unretained lanes.
pub(crate) struct DirectionCursor {
    pattern: ScanPattern,
    dense: bool,
    group: Option<u64>,
    rays: [[f64; 3]; 4],
    capture: Option<crate::hardware::CaptureCursor>,
}
impl DirectionCursor {
    pub(crate) fn new(pattern: ScanPattern, first: u64, stride: u32) -> Self {
        Self {
            pattern,
            dense: stride <= 2,
            group: None,
            rays: [[0.; 3]; 4],
            capture: (pattern == ScanPattern::HardwareCapture)
                .then(|| crate::hardware::CaptureCursor::new(first)),
        }
    }

    pub(crate) fn direction(&mut self, index: u64) -> [f64; 3] {
        if !self.dense {
            return match &mut self.capture {
                Some(cursor) => cursor.direction(index),
                None => self
                    .pattern
                    .direction(index)
                    .expect("validated profile window"),
            };
        }
        if self.group != Some(index / 4) {
            self.rays = match self.pattern {
                ScanPattern::Nominal => crate::hardware::NominalPattern.group_directions(index),
                ScanPattern::OfficialTable => Mid360Pattern.group_directions(index),
                ScanPattern::HardwareMean => {
                    crate::hardware::HardwareMeanPattern.group_directions(index)
                }
                ScanPattern::HardwareCapture => self
                    .capture
                    .as_mut()
                    .expect("capture cursor")
                    .group_directions(index),
            };
            self.group = Some(index / 4);
        }
        self.rays[(index % 4) as usize]
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    fn check_cursor(pattern: ScanPattern, first: u64, count: usize, stride: u32) {
        let mut whole = DirectionCursor::new(pattern, first, stride);
        let expected: Vec<_> = (0..count)
            .map(|i| {
                let index = first + i as u64 * u64::from(stride);
                let ray = whole.direction(index).map(f64::to_bits);
                assert_eq!(
                    ray,
                    pattern.direction(index).unwrap().map(f64::to_bits),
                    "{pattern:?}, index {index}, stride {stride}"
                );
                ray
            })
            .collect();
        let mut offset = 0;
        for chunk in [1, 7, 2, 19, 3, 31, count] {
            let end = (offset + chunk).min(count);
            let mut split =
                DirectionCursor::new(pattern, first + offset as u64 * u64::from(stride), stride);
            for (i, ray) in expected.iter().enumerate().take(end).skip(offset) {
                assert_eq!(
                    split
                        .direction(first + i as u64 * u64::from(stride))
                        .map(f64::to_bits),
                    *ray
                );
            }
            offset = end;
        }
        assert_eq!(offset, count);
    }

    #[test]
    fn all_profiles_match_scalar_bits_across_lanes_chunks_and_strides() {
        for pattern in [
            ScanPattern::Nominal,
            ScanPattern::OfficialTable,
            ScanPattern::HardwareMean,
            ScanPattern::HardwareCapture,
        ] {
            for base in [
                0,
                399_996,
                MODEL_PERIOD_SAMPLES - 12,
                crate::hardware::LAST_CAPTURE_SAMPLE - 1023,
            ] {
                for lane in 0..4 {
                    for stride in [1, 2, 3, 4, 7] {
                        check_cursor(pattern, base + lane, 101, stride);
                    }
                }
            }
            if pattern != ScanPattern::HardwareCapture {
                for lane in 0..4 {
                    for stride in [1, 2, 3, 4, 7] {
                        check_cursor(pattern, u64::MAX - 1023 + lane, 101, stride);
                    }
                }
            }
        }
        for stride in [1, 2, 3, 4, 7] {
            check_cursor(
                ScanPattern::HardwareCapture,
                crate::hardware::LAST_CAPTURE_SAMPLE - 10 * u64::from(stride),
                11,
                stride,
            );
        }
    }

    #[test]
    fn acquisition_cache_preserves_seeded_scalar_returns_and_atomic_rejection() {
        use crate::sensor::{Effects, RollingFrame, Scene, Sensor, SurfaceHit};
        struct MovingSurface;
        impl Scene for MovingSurface {
            fn nearest_hit(&self, time_ns: u64, direction: [f64; 3], _: f64) -> Option<SurfaceHit> {
                Some(SurfaceHit {
                    distance_m: 5. + direction[0] + (time_ns % 100_003) as f64 * 1e-6,
                    incidence_cosine: 0.9,
                    reflectance: 0.8,
                })
            }
        }
        let sensor = Sensor::new(
            0.1,
            40.,
            10,
            Effects {
                range_sigma_m: 0.02,
                angular_sigma_rad: 0.001,
                dropout: 0.1,
                diffuse_dropout: true,
                ..Default::default()
            },
        )
        .unwrap();
        for pattern in [
            ScanPattern::Nominal,
            ScanPattern::OfficialTable,
            ScanPattern::HardwareMean,
            ScanPattern::HardwareCapture,
        ] {
            for stride in [1, 2, 3, 4, 7] {
                let first = crate::hardware::LAST_CAPTURE_SAMPLE - 1021;
                let mut all = RollingFrame::default();
                sensor
                    .sample_pattern_into(&MovingSurface, pattern, first, 101, stride, &mut all)
                    .unwrap();
                let mut single = RollingFrame::default();
                for (i, point) in all.points.iter().enumerate() {
                    // Stride seven selects the scalar path, even for a single ray.
                    sensor
                        .sample_pattern_into(
                            &MovingSurface,
                            pattern,
                            first + i as u64 * u64::from(stride),
                            1,
                            7,
                            &mut single,
                        )
                        .unwrap();
                    assert_eq!(*point, single.points[0]);
                }
                let original = all.clone();
                for (first, count, stride) in [
                    (u64::MAX, 2, 1),
                    (u64::MAX / POINT_INTERVAL_NS, 2, 1),
                    (0, 0, 1),
                    (0, 1, 0),
                ] {
                    assert!(
                        sensor
                            .sample_pattern_into(
                                &MovingSurface,
                                pattern,
                                first,
                                count,
                                stride,
                                &mut all
                            )
                            .is_err()
                    );
                    assert_eq!(all, original);
                }
                if pattern == ScanPattern::HardwareCapture {
                    for (first, count) in [
                        (crate::hardware::LAST_CAPTURE_SAMPLE, 2),
                        (crate::hardware::LAST_CAPTURE_SAMPLE + 1, 1),
                    ] {
                        assert!(
                            sensor
                                .sample_pattern_into(
                                    &MovingSurface,
                                    pattern,
                                    first,
                                    count,
                                    stride,
                                    &mut all
                                )
                                .is_err()
                        );
                        assert_eq!(all, original);
                    }
                }
            }
        }
    }

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
