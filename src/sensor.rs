// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Backend-independent acquisitions. Scene queries receive each emission's time;
//! the caller transforms rays and interpolates moving geometry at that time.
use crate::pattern::{Mid360Pattern, POINT_INTERVAL_NS};
/// Maximum emissions per acquisition (bounded caller allocation).
pub const MAX_LIDAR_RAYS: usize = 262_144;
/// Nearest physical surface; below-minimum hits still occlude farther objects.
#[derive(Clone, Copy, Debug)]
pub struct SurfaceHit {
    /// Distance from the sensor origin in metres, finite and nonnegative.
    pub distance_m: f64,
    /// Absolute normal/ray cosine in [0,1].
    pub incidence_cosine: f64,
    /// Diffuse reflectance in [0,1], supplied by a sensor material, not RGB colour.
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
    /// Independent probability of dropping a valid return, [0,1].
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
fn uniform(seed: u64, index: u64, stream: u64) -> f64 {
    let mut x =
        seed ^ index.wrapping_mul(0x9e3779b97f4a7c15) ^ stream.wrapping_mul(0xd1b54a32d192ed03);
    x = (x ^ (x >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94d049bb133111eb);
    (((x ^ (x >> 31)) >> 11) as f64 + 0.5) / 9007199254740992.
}
fn normal(seed: u64, index: u64, stream: u64) -> f64 {
    (-2. * uniform(seed, index, stream).ln()).sqrt()
        * (std::f64::consts::TAU * uniform(seed, index, stream + 1)).cos()
}
/// Validated immutable acquisition configuration.
pub struct Sensor {
    min_range_m: f64,
    max_range_m: f64,
    epoch_ns: u64,
    effects: Effects,
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
        })
    }
    /// Sample global indices `first + i*stride`. Full-rate cadence is 5 us.
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
        if count == 0 || count as usize > MAX_LIDAR_RAYS || stride == 0 {
            return Err("invalid acquisition size or stride");
        }
        let last = first
            .checked_add(u64::from(count - 1) * u64::from(stride))
            .ok_or("index overflow")?;
        last.checked_mul(POINT_INTERVAL_NS)
            .and_then(|t| t.checked_add(self.epoch_ns))
            .ok_or("timestamp overflow")?;
        out.points.clear();
        for offset in 0..count {
            let index = first + u64::from(offset) * u64::from(stride);
            let time_ns = self.epoch_ns + index * POINT_INTERVAL_NS;
            let e = self.effects;
            let mut d = Mid360Pattern.direction(index);
            if e.angular_sigma_rad > 0. {
                let a = d[1].atan2(d[0]);
                let z = d[2];
                let c = (1. - z * z).max(0.).sqrt();
                let tangent = [-a.sin(), a.cos(), 0.];
                let vertical = [-z * a.cos(), -z * a.sin(), c];
                let x = e.angular_sigma_rad * normal(e.seed, index, 10);
                let y = e.angular_sigma_rad * normal(e.seed, index, 12);
                for k in 0..3 {
                    d[k] += x * tangent[k] + y * vertical[k];
                }
                let norm = d.iter().map(|x| x * x).sum::<f64>().sqrt();
                for v in &mut d {
                    *v /= norm;
                }
            }
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
                    && uniform(e.seed, index, 1) >= e.dropout
                    && uniform(e.seed, index, 2) < response
                {
                    let range = hit.distance_m
                        + e.range_bias_m
                        + e.range_sigma_m * normal(e.seed, index, 3);
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
        Ok(())
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
    fn chunk_and_stride_invariant_noise() {
        let s = Sensor::new(
            0.1,
            40.,
            10,
            Effects {
                range_sigma_m: 0.02,
                angular_sigma_rad: 0.001,
                dropout: 0.1,
                ..Default::default()
            },
        )
        .unwrap();
        let mut all = RollingFrame::default();
        s.sample_into(&Surface(10.), 0, 1000, 1, &mut all).unwrap();
        let mut a = RollingFrame::default();
        let mut b = RollingFrame::default();
        s.sample_into(&Surface(10.), 0, 500, 1, &mut a).unwrap();
        s.sample_into(&Surface(10.), 500, 500, 1, &mut b).unwrap();
        a.points.append(&mut b.points);
        assert_eq!(all, a);
        s.sample_into(&Surface(10.), 0, 334, 3, &mut b).unwrap();
        assert_eq!(
            b.points,
            all.points.iter().step_by(3).copied().collect::<Vec<_>>()
        );
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
