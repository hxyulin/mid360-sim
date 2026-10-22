// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Backend-independent acquisitions. Scene queries receive each emission's time;
//! the caller transforms rays and interpolates moving geometry at that time.
use crate::{
    effects::{AngularNoise, normal, uniform},
    pattern::{POINT_INTERVAL_NS, ScanPattern},
};
/// Maximum emissions per acquisition (bounded caller allocation).
pub const MAX_LIDAR_RAYS: usize = 262_144;
/// Nearest physical surface; below-minimum hits still occlude farther objects.
#[derive(Clone, Copy, Debug)]
pub struct SurfaceHit {
    /// Distance from the sensor origin in metres, finite and nonnegative.
    pub distance_m: f64,
    /// Absolute normal/ray cosine in `[0,1]`.
    pub incidence_cosine: f64,
    /// Diffuse reflectance in `[0,1]`, supplied by a sensor material, not RGB colour.
    pub reflectance: f64,
}
/// Geometry backend, independent of Rapier, Bevy, ROS and any robot simulator.
pub trait Scene {
    /// Return the nearest surface, including surfaces below minimum range.
    /// `direction` is a unit vector in sensor FLU at `time_ns`; transform it and
    /// the sensor origin into your scene using that emission's actual pose.
    fn nearest_hit(
        &self,
        time_ns: u64,
        direction: [f64; 3],
        max_range_m: f64,
    ) -> Option<SurfaceHit>;
}
/// One nominal emission, including misses; coordinates remain in emission FLU.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TimedReturn {
    /// Global emission index, independent of packet boundaries.
    pub sample_index: u64,
    /// Simulation timestamp, nanoseconds.
    pub time_ns: u64,
    /// Measured range in metres, absent on a miss.
    pub range_m: Option<f64>,
    /// Measured local sensor coordinates in metres, absent on a miss.
    pub point_sensor_m: Option<[f64; 3]>,
}
/// Ordered rolling acquisition, preserving the timestamps of misses.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RollingFrame {
    /// All emissions in acquisition order.
    pub points: Vec<TimedReturn>,
}
/// Generic effects, deliberately distinct from calibrated MID-360 behaviour.
#[derive(Clone, Copy, Debug)]
pub struct Effects {
    /// Counter-based device seed; batching and decimation never alter retained noise.
    pub seed: u64,
    /// Gaussian range standard deviation, metres.
    pub range_sigma_m: f64,
    /// Constant range bias, metres.
    pub range_bias_m: f64,
    /// Independent tangent-plane angular standard deviation per axis, radians.
    /// Must be at most pi radians. Perturbs emitted rays before collision; not inferred optical fit error.
    pub angular_sigma_rad: f64,
    /// Independent probability of dropping a valid return, `[0,1]`.
    pub dropout: f64,
    /// Optional qualitative diffuse response: survival probability is reflectance
    /// times incidence cosine. This is NOT a calibrated detection probability.
    pub diffuse_dropout: bool,
}
impl Default for Effects {
    fn default() -> Self {
        Self {
            seed: 7,
            range_sigma_m: 0.,
            range_bias_m: 0.,
            angular_sigma_rad: 0.,
            dropout: 0.,
            diffuse_dropout: false,
        }
    }
}
/// Validated immutable acquisition configuration.
pub struct Sensor {
    min_range_m: f64,
    max_range_m: f64,
    epoch_ns: u64,
    effects: Effects,
    angular_noise: AngularNoise,
}
impl Sensor {
    /// Construct an explicit-clock sensor. All effects default to ideal/noiseless.
    ///
    /// ```
    /// let sensor = mid360_sim::sensor::Sensor::new(0.1, 40., 0, Default::default())?;
    /// # let _ = sensor;
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn new(
        min_range_m: f64,
        max_range_m: f64,
        epoch_ns: u64,
        effects: Effects,
    ) -> Result<Self, &'static str> {
        if !min_range_m.is_finite()
            || !max_range_m.is_finite()
            || min_range_m < 0.
            || max_range_m <= min_range_m
            || !effects.range_sigma_m.is_finite()
            || effects.range_sigma_m < 0.
            || !effects.range_bias_m.is_finite()
            || !effects.angular_sigma_rad.is_finite()
            || !(0. ..=std::f64::consts::PI).contains(&effects.angular_sigma_rad)
            || !effects.dropout.is_finite()
            || !(0. ..=1.).contains(&effects.dropout)
        {
            return Err("invalid sensor range or effects");
        }
        Ok(Self {
            min_range_m,
            max_range_m,
            epoch_ns,
            effects,
            angular_noise: AngularNoise::new(effects.angular_sigma_rad, effects.seed)?,
        })
    }
    /// Sample the default nominal profile at `first + i*stride`. Cadence is 5 us.
    /// Overflow and allocation limits are checked before changing `out`.
    /// Backend failures/invalid surfaces are misses; a rejected nearest surface
    /// never exposes a farther one. Odd strides retain all four fitted lanes.
    pub fn sample_into(
        &self,
        scene: &impl Scene,
        first: u64,
        count: u32,
        stride: u32,
        out: &mut RollingFrame,
    ) -> Result<(), &'static str> {
        self.sample_pattern_into(scene, ScanPattern::default(), first, count, stride, out)
    }
    /// Sample an explicit nominal, reference, mean-clock or bounded capture profile.
    /// An unsupported window is rejected before output changes. Other acquisition
    /// timing, noise and scene-query semantics match [`Self::sample_into`].
    #[allow(clippy::too_many_arguments)]
    pub fn sample_pattern_into(
        &self,
        scene: &impl Scene,
        pattern: ScanPattern,
        first: u64,
        count: u32,
        stride: u32,
        out: &mut RollingFrame,
    ) -> Result<(), &'static str> {
        if count == 0 || count as usize > MAX_LIDAR_RAYS || stride == 0 {
            return Err("invalid acquisition size or stride");
        }
        let last = first
            .checked_add(u64::from(count - 1) * u64::from(stride))
            .ok_or("index overflow")?;
        last.checked_mul(POINT_INTERVAL_NS)
            .and_then(|t| t.checked_add(self.epoch_ns))
            .ok_or("timestamp overflow")?;
        if pattern.direction(first).is_none() || pattern.direction(last).is_none() {
            return Err("acquisition exceeds scan profile duration");
        }
        // Choose the range-noise path once per acquisition so enabled noise
        // does not pay for an extra conditional at every valid return.
        if self.effects.range_sigma_m > 0. {
            self.sample_validated::<true>(scene, pattern, first, count, stride, out);
        } else {
            self.sample_validated::<false>(scene, pattern, first, count, stride, out);
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn sample_validated<const RANGE_NOISE: bool>(
        &self,
        scene: &impl Scene,
        pattern: ScanPattern,
        first: u64,
        count: u32,
        stride: u32,
        out: &mut RollingFrame,
    ) {
        out.points.clear();
        let mut nominal_group = None;
        let mut nominal_rays = [[0.; 3]; 4];
        for offset in 0..count {
            let index = first + u64::from(offset) * u64::from(stride);
            let time_ns = self.epoch_ns + index * POINT_INTERVAL_NS;
            let e = self.effects;
            let d = match pattern {
                ScanPattern::Nominal if stride <= 2 => {
                    if nominal_group != Some(index / 4) {
                        nominal_rays = crate::hardware::NominalPattern.group_directions(index);
                        nominal_group = Some(index / 4);
                    }
                    nominal_rays[(index % 4) as usize]
                }
                ScanPattern::Nominal => crate::hardware::NominalPattern.direction(index),
                _ => pattern.direction(index).expect("validated profile window"),
            };
            let d = self.angular_noise.perturb_unit(index, d);
            let mut point = TimedReturn {
                sample_index: index,
                time_ns,
                range_m: None,
                point_sensor_m: None,
            };
            if let Some(hit) = scene.nearest_hit(time_ns, d, self.max_range_m) {
                let valid = hit.distance_m.is_finite()
                    && hit.distance_m >= self.min_range_m
                    && hit.distance_m <= self.max_range_m
                    && hit.incidence_cosine.is_finite()
                    && (0. ..=1.).contains(&hit.incidence_cosine)
                    && hit.reflectance.is_finite()
                    && (0. ..=1.).contains(&hit.reflectance);
                let response = if e.diffuse_dropout {
                    hit.reflectance * hit.incidence_cosine
                } else {
                    1.
                };
                if valid
                    && (e.dropout == 0. || uniform(e.seed, index, 1) >= e.dropout)
                    && uniform(e.seed, index, 2) < response
                {
                    let mut range = hit.distance_m + e.range_bias_m;
                    if RANGE_NOISE {
                        range += e.range_sigma_m * normal(e.seed, index, 3);
                    }
                    if range.is_finite()
                        && range > 0.
                        && range >= self.min_range_m
                        && range <= self.max_range_m
                    {
                        point.range_m = Some(range);
                        point.point_sensor_m = Some(d.map(|v| v * range));
                    }
                }
            }
            out.points.push(point);
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    struct Surface(f64);
    impl Scene for Surface {
        fn nearest_hit(&self, _: u64, _: [f64; 3], _: f64) -> Option<SurfaceHit> {
            Some(SurfaceHit {
                distance_m: self.0,
                incidence_cosine: 1.,
                reflectance: 1.,
            })
        }
    }
    #[test]
    fn capture_profile_preserves_chunking_and_rejects_overrun_atomically() {
        let s = Sensor::new(
            0.1,
            40.,
            10,
            Effects {
                range_sigma_m: 0.02,
                ..Default::default()
            },
        )
        .unwrap();
        let mut all = RollingFrame::default();
        let mut a = RollingFrame::default();
        let mut b = RollingFrame::default();
        let p = ScanPattern::HardwareCapture;
        s.sample_pattern_into(&Surface(10.), p, 7, 1000, 3, &mut all)
            .unwrap();
        s.sample_pattern_into(&Surface(10.), p, 7, 400, 3, &mut a)
            .unwrap();
        s.sample_pattern_into(&Surface(10.), p, 1207, 600, 3, &mut b)
            .unwrap();
        a.points.append(&mut b.points);
        assert_eq!(all, a);
        let original = all.clone();
        assert!(
            s.sample_pattern_into(
                &Surface(10.),
                p,
                crate::hardware::LAST_CAPTURE_SAMPLE,
                2,
                1,
                &mut all
            )
            .is_err()
        );
        assert_eq!(original, all);
    }
    #[test]
    fn chunk_and_stride_invariant_noise() {
        for effects in [
            Effects::default(),
            Effects {
                range_sigma_m: 0.02,
                ..Default::default()
            },
            Effects {
                dropout: 0.1,
                ..Default::default()
            },
            Effects {
                range_sigma_m: 0.02,
                angular_sigma_rad: 0.001,
                dropout: 0.1,
                diffuse_dropout: true,
                ..Default::default()
            },
        ] {
            let s = Sensor::new(0.1, 40., 10, effects).unwrap();
            let mut all = RollingFrame::default();
            s.sample_into(&Surface(10.), 7, 1000, 1, &mut all).unwrap();
            let mut a = RollingFrame::default();
            let mut b = RollingFrame::default();
            s.sample_into(&Surface(10.), 7, 499, 1, &mut a).unwrap();
            s.sample_into(&Surface(10.), 506, 501, 1, &mut b).unwrap();
            a.points.append(&mut b.points);
            assert_eq!(all, a);
            for stride in [1, 2, 3, 4, 7, 31] {
                s.sample_into(&Surface(10.), 7, 1000u32.div_ceil(stride), stride, &mut b)
                    .unwrap();
                assert_eq!(
                    b.points,
                    all.points
                        .iter()
                        .step_by(stride as usize)
                        .copied()
                        .collect::<Vec<_>>()
                );
            }
        }
    }
    #[test]
    fn occlusion_and_atomic_validation() {
        let s = Sensor::new(0.1, 40., 0, Effects::default()).unwrap();
        let mut f = RollingFrame::default();
        s.sample_into(&Surface(0.05), 0, 10, 1, &mut f).unwrap();
        assert!(f.points.iter().all(|p| p.range_m.is_none()));
        let original = f.clone();
        assert!(
            s.sample_into(&Surface(10.), u64::MAX, 10, 1, &mut f)
                .is_err()
        );
        assert_eq!(f, original);
    }
    #[test]
    fn angular_noise_is_applied_before_geometry() {
        struct Directional;
        impl Scene for Directional {
            fn nearest_hit(&self, _: u64, d: [f64; 3], _: f64) -> Option<SurfaceHit> {
                Some(SurfaceHit {
                    distance_m: 10. + d[0],
                    incidence_cosine: 1.,
                    reflectance: 1.,
                })
            }
        }
        let s = Sensor::new(
            0.1,
            40.,
            0,
            Effects {
                angular_sigma_rad: 0.01,
                ..Default::default()
            },
        )
        .unwrap();
        let mut f = RollingFrame::default();
        s.sample_into(&Directional, 0, 100, 1, &mut f).unwrap();
        for p in f.points {
            let r = p.range_m.unwrap();
            assert!((r - 10. - p.point_sensor_m.unwrap()[0] / r).abs() < 1e-12);
        }
    }

    #[test]
    fn angular_noise_changes_edge_occlusion_without_seeing_through_near_hits() {
        struct Edge {
            forward: [f64; 3],
            left: [f64; 3],
            front_m: f64,
        }
        impl Scene for Edge {
            fn nearest_hit(&self, _: u64, d: [f64; 3], max: f64) -> Option<SurfaceHit> {
                let dot = |a: [f64; 3], b: [f64; 3]| {
                    a.into_iter().zip(b).map(|(a, b)| a * b).sum::<f64>()
                };
                let cosine = dot(d, self.forward);
                // An opaque half-plane ahead of a full background plane. The
                // nominal ray lies on its edge; noisy rays may clear the edge.
                let depth = if dot(d, self.left) >= -1e-12 {
                    self.front_m
                } else {
                    8.
                };
                let distance = depth / cosine;
                (cosine > 0. && distance <= max).then_some(SurfaceHit {
                    distance_m: distance,
                    incidence_cosine: cosine.clamp(0., 1.),
                    reflectance: 1.,
                })
            }
        }
        let forward = crate::hardware::NominalPattern.direction(0);
        let horizontal = forward[0].hypot(forward[1]);
        for front_m in [0.05, 2.] {
            let edge = Edge {
                forward,
                left: [-forward[1] / horizontal, forward[0] / horizontal, 0.],
                front_m,
            };
            let mut scan = RollingFrame::default();
            Sensor::new(0.1, 40., 123, Effects::default())
                .unwrap()
                .sample_into(&edge, 0, 1, 1, &mut scan)
                .unwrap();
            assert_eq!(scan.points[0].range_m.is_some(), front_m >= 0.1);
            let (mut foreground, mut background) = (0, 0);
            for seed in 0..32 {
                let effects = Effects {
                    seed,
                    angular_sigma_rad: 0.01,
                    ..Default::default()
                };
                Sensor::new(0.1, 40., 123, effects)
                    .unwrap()
                    .sample_into(&edge, 0, 1, 1, &mut scan)
                    .unwrap();
                let p = scan.points[0];
                assert_eq!((p.sample_index, p.time_ns), (0, 123));
                if let Some(range) = p.range_m {
                    let d = AngularNoise::new(0.01, seed)
                        .unwrap()
                        .perturb(0, forward)
                        .unwrap();
                    assert_eq!(p.point_sensor_m, Some(d.map(|v| v * range)));
                    if range > 7. {
                        background += 1;
                    } else {
                        assert!(front_m >= 0.1 && range < 3.);
                        foreground += 1;
                    }
                } else {
                    assert!(front_m < 0.1);
                    foreground += 1;
                }
            }
            assert!(foreground > 0 && background > 0);
        }
    }

    #[test]
    fn shared_angular_effects_preserve_per_emission_dynamic_mount_queries() {
        struct RotatingWall;
        impl Scene for RotatingWall {
            fn nearest_hit(&self, time_ns: u64, d: [f64; 3], max: f64) -> Option<SurfaceHit> {
                let t = time_ns as f64 * 1e-9;
                let (s, c) = (100. * t).sin_cos();
                let origin_x = (3. * t).sin() + 0.2 * c;
                let distance = (-4. - origin_x) / (c * d[0] - s * d[1]);
                (distance >= 0. && distance <= max).then_some(SurfaceHit {
                    distance_m: distance,
                    incidence_cosine: 1.,
                    reflectance: 1.,
                })
            }
        }
        let sensor = Sensor::new(
            0.1,
            40.,
            0,
            Effects {
                angular_sigma_rad: 0.001,
                ..Default::default()
            },
        )
        .unwrap();
        let mut scan = RollingFrame::default();
        sensor
            .sample_into(&RotatingWall, 0, 2000, 1, &mut scan)
            .unwrap();
        let mut hits = 0;
        for p in scan.points {
            if let Some(local) = p.point_sensor_m {
                let t = p.time_ns as f64 * 1e-9;
                let (s, c) = (100. * t).sin_cos();
                // Rotate the raw point at its own time and apply the offset
                // mount. Every return must lie on the fixed world wall x=-4.
                let world_x = (3. * t).sin() + c * (0.2 + local[0]) - s * local[1];
                assert!((world_x + 4.).abs() < 1e-10);
                hits += 1;
            }
        }
        assert!(hits > 100);
    }
    #[test]
    fn diffuse_response_is_opt_in_and_black_surface_occludes() {
        struct Black;
        impl Scene for Black {
            fn nearest_hit(&self, _: u64, _: [f64; 3], _: f64) -> Option<SurfaceHit> {
                Some(SurfaceHit {
                    distance_m: 2.,
                    incidence_cosine: 1.,
                    reflectance: 0.,
                })
            }
        }
        for enabled in [false, true] {
            let s = Sensor::new(
                0.1,
                40.,
                0,
                Effects {
                    diffuse_dropout: enabled,
                    ..Default::default()
                },
            )
            .unwrap();
            let mut f = RollingFrame::default();
            s.sample_into(&Black, 0, 100, 1, &mut f).unwrap();
            assert!(f.points.iter().all(|p| p.range_m.is_some() != enabled));
        }
    }
}

#[cfg(test)]
#[path = "sensor/articulated_tests.rs"]
mod articulated_tests;
