// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Backend-independent acquisitions. Scene queries receive each emission's time;
//! the caller transforms rays and interpolates moving geometry at that time.
//!
//! A minimal backend can describe a spherical surface centred on the sensor.
//! Real integrations evaluate the actual sensor pose and geometry at `time_ns`,
//! including dynamic mount rotations, before returning the nearest hit.
//!
//! ```
//! use mid360_sim::{LivoxConfig, LivoxPacket};
//! use mid360_sim::sensor::{RollingFrame, Scene, Sensor, SurfaceHit};
//!
//! struct SphericalRoom;
//! impl Scene for SphericalRoom {
//!     fn nearest_hit(&self, _time_ns: u64, _direction: [f64; 3], max_range_m: f64)
//!         -> Option<SurfaceHit>
//!     {
//!         (2. <= max_range_m).then_some(SurfaceHit {
//!             distance_m: 2., incidence_cosine: 1., reflectance: 0.8,
//!         })
//!     }
//! }
//!
//! let sensor = Sensor::new(0.1, 40., 1_000_000_000, Default::default())?;
//! let mut frame = RollingFrame::default();
//! sensor.sample_into(&SphericalRoom, 7, 3, 3, &mut frame)?;
//! assert_eq!(frame.points.iter().map(|p| p.time_ns).collect::<Vec<_>>(),
//!            [1_000_035_000, 1_000_050_000, 1_000_065_000]);
//! let packet = LivoxPacket::from_scan(&frame, &LivoxConfig::default())?;
//! assert_eq!(packet.points[2].offset_time_ns, 30_000);
//!
//! // Reuse the buffer; continue global indices rather than restarting at zero.
//! sensor.sample_into(&SphericalRoom, 16, 2, 3, &mut frame)?;
//! assert_eq!(frame.points.len(), 2);
//! assert_eq!(frame.points[0].sample_index, 16);
//! # Ok::<(), &'static str>(())
//! ```
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
    /// `time_ns` is absolute simulation nanoseconds; `max_range_m` is metres.
    /// Return `None` for a miss or backend failure. Do not skip a near, dark or
    /// otherwise undetectable first surface to return an object behind it.
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
/// Immutable optical directions for one caller-selected emission window.
///
/// Share a window between sensors only when their scan profile, global indices
/// and stride match. Directions are local FLU unit vectors before angular noise;
/// no pose, geometry, measurement noise or absolute timestamp is cached. Each
/// sensor still applies its own effects and queries its scene at every emission.
/// This amortizes direction generation across aligned sensors, at the cost of
/// 24 bytes of direction payload per retained ray (up to 6 MiB), plus container
/// overhead. A single sensor normally benefits
/// from the allocation-free direction cursor in [`Sensor::sample_into`] instead.
///
/// ```
/// use mid360_sim::{pattern::ScanPattern, sensor::PreparedRays};
/// let rays = PreparedRays::new(ScanPattern::Nominal, 7, 200, 3)?;
/// assert_eq!((rays.first_index(), rays.stride()), (7, 3));
/// assert_eq!(rays.directions().len(), 200);
/// assert_eq!(rays.directions()[1], ScanPattern::Nominal.direction(10).unwrap());
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Debug)]
pub struct PreparedRays {
    pattern: ScanPattern,
    first: u64,
    stride: u32,
    last_offset_ns: u64,
    directions: Vec<[f64; 3]>,
}
impl PreparedRays {
    /// Prepare `count` unit directions at global indices `first + i * stride`.
    ///
    /// Indices/stride are dimensionless; the nominal cadence is 5,000 ns. Rejects
    /// empty/oversized windows, zero stride, index/time overflow and unsupported
    /// capture windows before allocating. Absolute epoch validation happens when
    /// a sensor acquires the window. Preparation never reads or advances a clock.
    pub fn new(
        pattern: ScanPattern,
        first: u64,
        count: u32,
        stride: u32,
    ) -> Result<Self, &'static str> {
        let last_offset_ns = checked_window(pattern, first, count, stride, 0)?;
        let mut cursor = crate::pattern::DirectionCursor::new(pattern, first, stride);
        let directions = (0..count)
            .map(|i| cursor.direction(first + u64::from(i) * u64::from(stride)))
            .collect();
        Ok(Self {
            pattern,
            first,
            stride,
            last_offset_ns,
            directions,
        })
    }
    /// The optical profile used to generate this window; no device phase is inferred.
    pub fn pattern(&self) -> ScanPattern {
        self.pattern
    }
    /// First global emission index, dimensionless and independent of packet boundaries.
    pub fn first_index(&self) -> u64 {
        self.first
    }
    /// Global index spacing between retained emissions, dimensionless and nonzero.
    pub fn stride(&self) -> u32 {
        self.stride
    }
    /// Borrow immutable local FLU unit directions in acquisition order.
    /// Misses are not known until acquisition; all scheduled emissions are present.
    pub fn directions(&self) -> &[[f64; 3]] {
        &self.directions
    }
}

// Return the final simulation timestamp (ns), validating before caller output changes.
#[inline]
fn checked_window(
    pattern: ScanPattern,
    first: u64,
    count: u32,
    stride: u32,
    epoch_ns: u64,
) -> Result<u64, &'static str> {
    if count == 0 || count as usize > MAX_LIDAR_RAYS || stride == 0 {
        return Err("invalid acquisition size or stride");
    }
    let last = first
        .checked_add(u64::from(count - 1) * u64::from(stride))
        .ok_or("index overflow")?;
    let last_ns = last
        .checked_mul(POINT_INTERVAL_NS)
        .and_then(|t| t.checked_add(epoch_ns))
        .ok_or("timestamp overflow")?;
    if !pattern.supports(last) {
        return Err("acquisition exceeds scan profile duration");
    }
    Ok(last_ns)
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
    /// Construct an explicit-clock sensor with range limits in metres.
    /// `epoch_ns` is the simulation timestamp of global emission index zero.
    /// [`Effects::default`] disables noise, bias and dropout.
    ///
    /// # Errors
    /// Returns an error for nonfinite or unordered range limits, negative noise
    /// standard deviations, angular sigma above pi radians, or dropout outside
    /// `[0, 1]`. The minimum range may be zero; measured returns must be positive.
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
    ///
    /// `count` counts retained emissions, including misses, rather than hits.
    /// Each timestamp is `epoch_ns + (first + i * stride) * 5_000` nanoseconds;
    /// decimation retains time gaps. Success replaces `out.points` while reusing
    /// its allocation. Points stay in their individual acquisition frames.
    ///
    /// # Errors
    /// Returns an error without modifying `out` if `count` is zero or exceeds
    /// [`MAX_LIDAR_RAYS`], `stride` is zero, or an index/timestamp overflows.
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
        checked_window(pattern, first, count, stride, self.epoch_ns)?;
        // Choose the range-noise path once per acquisition so enabled noise
        // does not pay for an extra conditional at every valid return.
        let mut directions = crate::pattern::DirectionCursor::new(pattern, first, stride);
        if self.effects.range_sigma_m > 0. {
            self.sample_validated::<true, _>(scene, first, count, stride, out, |_, index| {
                directions.direction(index)
            });
        } else {
            self.sample_validated::<false, _>(scene, first, count, stride, out, |_, index| {
                directions.direction(index)
            });
        }
        Ok(())
    }

    /// Acquire immutable prepared rays with this sensor's epoch and effects.
    ///
    /// Per-emission scene queries, first-surface occlusion, misses and output
    /// coordinates match [`Self::sample_pattern_into`]. Success replaces `out`
    /// while reusing its allocation. Different sensor epochs are supported:
    /// timestamps remain `epoch_ns + global_index * 5_000`, in nanoseconds.
    /// Returns an error without changing `out` if this epoch would overflow.
    /// Sharing the directions never shares noisy measurements or deskewed points.
    ///
    /// ```
    /// use mid360_sim::{pattern::ScanPattern, sensor::{PreparedRays, RollingFrame, Scene, Sensor, SurfaceHit}};
    /// struct EmptyScene;
    /// impl Scene for EmptyScene {
    ///     fn nearest_hit(&self, _: u64, _: [f64; 3], _: f64) -> Option<SurfaceHit> { None }
    /// }
    /// let rays = PreparedRays::new(ScanPattern::Nominal, 7, 2, 3)?;
    /// let mut frame = RollingFrame::default();
    /// for epoch_ns in [1_000_000_000, 2_000_000_000] {
    ///     let sensor = Sensor::new(0.1, 40., epoch_ns, Default::default())?;
    ///     sensor.sample_prepared_into(&EmptyScene, &rays, &mut frame)?;
    ///     assert_eq!(frame.points[0].time_ns, epoch_ns + 35_000);
    ///     assert_eq!(frame.points[1].time_ns, epoch_ns + 50_000);
    ///     assert!(frame.points.iter().all(|p| p.range_m.is_none()));
    /// }
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn sample_prepared_into(
        &self,
        scene: &impl Scene,
        prepared: &PreparedRays,
        out: &mut RollingFrame,
    ) -> Result<(), &'static str> {
        prepared
            .last_offset_ns
            .checked_add(self.epoch_ns)
            .ok_or("timestamp overflow")?;
        // Construction bounds this length to MAX_LIDAR_RAYS and rejects zero.
        let count = prepared.directions.len() as u32;
        if self.effects.range_sigma_m > 0. {
            self.sample_validated::<true, _>(
                scene,
                prepared.first,
                count,
                prepared.stride,
                out,
                |i, _| prepared.directions[i as usize],
            );
        } else {
            self.sample_validated::<false, _>(
                scene,
                prepared.first,
                count,
                prepared.stride,
                out,
                |i, _| prepared.directions[i as usize],
            );
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn sample_validated<const RANGE_NOISE: bool, D: FnMut(u32, u64) -> [f64; 3]>(
        &self,
        scene: &impl Scene,
        first: u64,
        count: u32,
        stride: u32,
        out: &mut RollingFrame,
        mut direction: D,
    ) {
        out.points.clear();
        for offset in 0..count {
            let index = first + u64::from(offset) * u64::from(stride);
            let time_ns = self.epoch_ns + index * POINT_INTERVAL_NS;
            let e = self.effects;
            let d = direction(offset, index);
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
                    && (e.dropout == 0.
                        || (e.dropout < 1. && uniform(e.seed, index, 1) >= e.dropout))
                    // The historical uniform mapping can round to 1. Keep its
                    // seeded streams intact, but make probability endpoints exact.
                    && (!e.diffuse_dropout
                        || response == 1.
                        || uniform(e.seed, index, 2) < response)
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
    fn assert_frame_bits(a: &RollingFrame, b: &RollingFrame) {
        assert_eq!(a.points.len(), b.points.len());
        for (a, b) in a.points.iter().zip(&b.points) {
            assert_eq!((a.sample_index, a.time_ns), (b.sample_index, b.time_ns));
            assert_eq!(a.range_m.map(f64::to_bits), b.range_m.map(f64::to_bits));
            assert_eq!(
                a.point_sensor_m.map(|p| p.map(f64::to_bits)),
                b.point_sensor_m.map(|p| p.map(f64::to_bits))
            );
        }
    }

    #[test]
    fn prepared_windows_preserve_live_queries_and_device_measurement_bits() {
        #[derive(Default)]
        struct MovingWall(std::cell::RefCell<Vec<(u64, [u64; 3])>>);
        impl Scene for MovingWall {
            fn nearest_hit(
                &self,
                time_ns: u64,
                direction: [f64; 3],
                max: f64,
            ) -> Option<SurfaceHit> {
                self.0
                    .borrow_mut()
                    .push((time_ns, direction.map(f64::to_bits)));
                let t = time_ns as f64 * 1e-9;
                let (s, c) = (3. * t).sin_cos();
                let cosine = c * direction[0] - s * direction[1];
                let distance = (5. - (2. * t).sin() * 0.2) / cosine;
                (distance >= 0. && distance <= max).then_some(SurfaceHit {
                    distance_m: distance,
                    incidence_cosine: cosine.abs().min(1.),
                    reflectance: 0.7,
                })
            }
        }
        for pattern in [
            ScanPattern::Nominal,
            ScanPattern::OfficialTable,
            ScanPattern::HardwareMean,
            ScanPattern::HardwareCapture,
        ] {
            for first in [7, crate::hardware::LAST_CAPTURE_SAMPLE - 1023] {
                for stride in [1, 2, 3, 7] {
                    let rays = PreparedRays::new(pattern, first, 32, stride).unwrap();
                    assert_eq!(
                        (rays.pattern(), rays.first_index(), rays.stride()),
                        (pattern, first, stride)
                    );
                    assert_eq!(rays.directions().len(), 32);
                    for (i, ray) in rays.directions().iter().enumerate() {
                        assert_eq!(
                            ray.map(f64::to_bits),
                            pattern
                                .direction(first + i as u64 * u64::from(stride))
                                .unwrap()
                                .map(f64::to_bits)
                        );
                    }
                    for epoch in [0, 1_000_000_123] {
                        for effects in [
                            Effects::default(),
                            Effects {
                                seed: 17,
                                range_sigma_m: 0.02,
                                angular_sigma_rad: 0.001,
                                dropout: 0.03,
                                diffuse_dropout: true,
                                ..Default::default()
                            },
                        ] {
                            let sensor = Sensor::new(0.1, 40., epoch, effects).unwrap();
                            let (old_scene, new_scene) =
                                (MovingWall::default(), MovingWall::default());
                            let (mut old, mut new) =
                                (RollingFrame::default(), RollingFrame::default());
                            sensor
                                .sample_pattern_into(
                                    &old_scene, pattern, first, 32, stride, &mut old,
                                )
                                .unwrap();
                            sensor
                                .sample_prepared_into(&new_scene, &rays, &mut new)
                                .unwrap();
                            assert_frame_bits(&old, &new);
                            assert_eq!(*old_scene.0.borrow(), *new_scene.0.borrow());
                            assert_eq!(new_scene.0.borrow().len(), 32);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn prepared_windows_preserve_chunking_decimation_occlusion_and_atomic_errors() {
        let sensor = Sensor::new(
            0.1,
            40.,
            10,
            Effects {
                range_sigma_m: 0.02,
                angular_sigma_rad: 0.001,
                ..Default::default()
            },
        )
        .unwrap();
        let mut all = RollingFrame::default();
        let rays = PreparedRays::new(ScanPattern::Nominal, 7, 101, 1).unwrap();
        sensor
            .sample_prepared_into(&Surface(10.), &rays, &mut all)
            .unwrap();
        let (mut a, mut b) = (RollingFrame::default(), RollingFrame::default());
        sensor
            .sample_prepared_into(
                &Surface(10.),
                &PreparedRays::new(ScanPattern::Nominal, 7, 37, 1).unwrap(),
                &mut a,
            )
            .unwrap();
        sensor
            .sample_prepared_into(
                &Surface(10.),
                &PreparedRays::new(ScanPattern::Nominal, 44, 64, 1).unwrap(),
                &mut b,
            )
            .unwrap();
        a.points.append(&mut b.points);
        assert_frame_bits(&all, &a);
        sensor
            .sample_prepared_into(
                &Surface(10.),
                &PreparedRays::new(ScanPattern::Nominal, 7, 34, 3).unwrap(),
                &mut b,
            )
            .unwrap();
        assert_frame_bits(
            &RollingFrame {
                points: all.points.iter().step_by(3).copied().collect(),
            },
            &b,
        );
        sensor
            .sample_prepared_into(&Surface(0.05), &rays, &mut b)
            .unwrap();
        assert!(b.points.iter().all(|p| p.range_m.is_none()));
        let original = all.clone();
        let overflowing = Sensor::new(0.1, 40., u64::MAX, Effects::default()).unwrap();
        assert!(
            overflowing
                .sample_prepared_into(&Surface(10.), &rays, &mut all)
                .is_err()
        );
        assert_frame_bits(&original, &all);
        for (pattern, first, count, stride) in [
            (ScanPattern::Nominal, 0, 0, 1),
            (ScanPattern::Nominal, 0, MAX_LIDAR_RAYS as u32 + 1, 1),
            (ScanPattern::Nominal, 0, 1, 0),
            (ScanPattern::Nominal, u64::MAX, 2, 1),
            (ScanPattern::Nominal, u64::MAX / POINT_INTERVAL_NS + 1, 1, 1),
            (
                ScanPattern::HardwareCapture,
                crate::hardware::LAST_CAPTURE_SAMPLE,
                2,
                1,
            ),
        ] {
            assert!(PreparedRays::new(pattern, first, count, stride).is_err());
        }
        assert!(
            PreparedRays::new(
                ScanPattern::HardwareCapture,
                crate::hardware::LAST_CAPTURE_SAMPLE,
                1,
                1
            )
            .is_ok()
        );
        fn shareable<T: Send + Sync>() {}
        shareable::<PreparedRays>();
    }

    #[test]
    fn probability_endpoints_are_exact_even_when_uniform_rounds_to_one() {
        const SEED: u64 = 7_849_933_349_262_882_758;
        // Preserve the historical mapping, including its rounded endpoint.
        assert_eq!(uniform(SEED, 0, 2).to_bits(), 1_f64.to_bits());
        let independent_seed = SEED ^ 2_u64.wrapping_mul(0xd1b54a32d192ed03) ^ 0xd1b54a32d192ed03;
        assert_eq!(uniform(independent_seed, 0, 1).to_bits(), 1_f64.to_bits());
        struct FirstSurface {
            distance_m: f64,
            response: f64,
            queries: std::cell::Cell<usize>,
        }
        impl Scene for FirstSurface {
            fn nearest_hit(&self, _: u64, _: [f64; 3], _: f64) -> Option<SurfaceHit> {
                let queries = self.queries.get();
                self.queries.set(queries + 1);
                // Retrying a rejected first surface would expose a background.
                Some(SurfaceHit {
                    distance_m: if queries == 0 { self.distance_m } else { 20. },
                    incidence_cosine: 1.,
                    reflectance: if queries == 0 { self.response } else { 1. },
                })
            }
        }
        for seed in [0, 7, SEED, independent_seed, u64::MAX] {
            for diffuse_dropout in [false, true] {
                for dropout in [0., 1.] {
                    for response in [0., 1.] {
                        for distance_m in [0.05, 2.] {
                            let scene = FirstSurface {
                                distance_m,
                                response,
                                queries: Default::default(),
                            };
                            let sensor = Sensor::new(
                                0.1,
                                40.,
                                123,
                                Effects {
                                    seed,
                                    diffuse_dropout,
                                    dropout,
                                    ..Default::default()
                                },
                            )
                            .unwrap();
                            let mut frame = RollingFrame::default();
                            sensor.sample_into(&scene, 0, 1, 1, &mut frame).unwrap();
                            let expected = (dropout == 0.
                                && distance_m >= 0.1
                                && (!diffuse_dropout || response == 1.))
                                .then_some(2.);
                            assert_eq!(frame.points[0].range_m, expected);
                            assert_eq!(frame.points[0].time_ns, 123);
                            assert_eq!(scene.queries.get(), 1);
                        }
                    }
                }
            }
        }
    }

    // Scalar pre-fix acceptance and measurement arithmetic. This deliberately
    // retains the unconditional diffuse draw and historical stream numbers.
    fn historical_return(e: Effects, index: u64, hit: SurfaceHit) -> TimedReturn {
        let d = AngularNoise::new(e.angular_sigma_rad, e.seed)
            .unwrap()
            .perturb_unit(index, crate::hardware::NominalPattern.direction(index));
        let mut point = TimedReturn {
            sample_index: index,
            time_ns: 10 + index * POINT_INTERVAL_NS,
            range_m: None,
            point_sensor_m: None,
        };
        let response = if e.diffuse_dropout {
            hit.reflectance * hit.incidence_cosine
        } else {
            1.
        };
        if (e.dropout == 0. || uniform(e.seed, index, 1) >= e.dropout)
            && uniform(e.seed, index, 2) < response
        {
            let mut range = hit.distance_m + e.range_bias_m;
            if e.range_sigma_m > 0. {
                range += e.range_sigma_m * normal(e.seed, index, 3);
            }
            if range.is_finite() && range > 0. && (0.1..=40.).contains(&range) {
                point.range_m = Some(range);
                point.point_sensor_m = Some(d.map(|v| v * range));
            }
        }
        point
    }

    #[test]
    fn nonendpoint_acceptance_preserves_historical_measurement_bits() {
        struct Material(SurfaceHit);
        impl Scene for Material {
            fn nearest_hit(&self, _: u64, _: [f64; 3], _: f64) -> Option<SurfaceHit> {
                Some(self.0)
            }
        }
        let scene = Material(SurfaceHit {
            distance_m: 10.,
            incidence_cosine: 0.7,
            reflectance: 0.6,
        });
        for seed in [0, 7, 17, u64::MAX] {
            for diffuse_dropout in [false, true] {
                for dropout in [0., 0.1, 0.9] {
                    for noisy in [false, true] {
                        let effects = Effects {
                            seed,
                            diffuse_dropout,
                            dropout,
                            range_sigma_m: if noisy { 0.02 } else { 0. },
                            angular_sigma_rad: if noisy { 0.001 } else { 0. },
                            range_bias_m: -0.003,
                        };
                        let sensor = Sensor::new(0.1, 40., 10, effects).unwrap();
                        let mut frame = RollingFrame::default();
                        sensor.sample_into(&scene, 7, 256, 1, &mut frame).unwrap();
                        for actual in frame.points {
                            let expected = historical_return(effects, actual.sample_index, scene.0);
                            assert_eq!(actual.sample_index, expected.sample_index);
                            assert_eq!(actual.time_ns, expected.time_ns);
                            assert_eq!(
                                actual.range_m.map(f64::to_bits),
                                expected.range_m.map(f64::to_bits)
                            );
                            assert_eq!(
                                actual.point_sensor_m.map(|p| p.map(f64::to_bits)),
                                expected.point_sensor_m.map(|p| p.map(f64::to_bits))
                            );
                        }
                    }
                }
            }
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
                seed: 7_849_933_349_262_882_758,
                range_sigma_m: 0.02,
                angular_sigma_rad: 0.001,
                ..Default::default()
            },
            Effects {
                seed: 7_849_933_349_262_882_758,
                diffuse_dropout: true,
                range_sigma_m: 0.02,
                angular_sigma_rad: 0.001,
                ..Default::default()
            },
            Effects {
                dropout: 1.,
                ..Default::default()
            },
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
            s.sample_into(&Surface(10.), 0, 1000, 1, &mut all).unwrap();
            let mut a = RollingFrame::default();
            let mut b = RollingFrame::default();
            s.sample_into(&Surface(10.), 0, 499, 1, &mut a).unwrap();
            s.sample_into(&Surface(10.), 499, 501, 1, &mut b).unwrap();
            a.points.append(&mut b.points);
            assert_eq!(all, a);
            for stride in [1, 2, 3, 4, 7, 31] {
                s.sample_into(&Surface(10.), 0, 1000u32.div_ceil(stride), stride, &mut b)
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
