// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Reusable measurement effects without geometry, clocks or mutable RNG state.
//!
//! Angular noise perturbs an emitted ray before the caller's nearest-hit query.
//! It is an optional generic stress model, not calibrated encoder uncertainty,
//! a beam footprint, or error inferred from the fitted scan pattern.

/// Independent Gaussian offsets in the emitted ray's two tangent directions.
///
/// A device seed and global emission index identify each perturbation. Calling
/// order, batching, misses and decimation do not consume or change RNG state.
/// The resulting ray is normalized; zero noise preserves input bits exactly.
///
/// ```
/// use mid360_sim::{effects::AngularNoise, hardware::NominalPattern};
/// let noise = AngularNoise::new(0.001, 7)?; // radians per tangent axis
/// let index = 42;
/// let nominal = NominalPattern.direction(index);
/// let emitted = noise.perturb(index, nominal)?;
/// // Transform emitted with the actual sensor pose, then query your scene.
/// assert!((emitted.iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 1e-12);
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AngularNoise {
    standard_deviation_rad: f64,
    seed: u64,
}

impl Default for AngularNoise {
    fn default() -> Self {
        Self {
            standard_deviation_rad: 0.,
            seed: 7,
        }
    }
}

impl AngularNoise {
    /// Validate standard deviation per tangent axis, radians in `[0, pi]`,
    /// and select a reproducible device seed. Zero disables the perturbation.
    pub fn new(standard_deviation_rad: f64, seed: u64) -> Result<Self, &'static str> {
        if !standard_deviation_rad.is_finite()
            || !(0. ..=std::f64::consts::PI).contains(&standard_deviation_rad)
        {
            return Err("angular standard deviation must be finite and in [0, pi] radians");
        }
        Ok(Self {
            standard_deviation_rad,
            seed,
        })
    }

    /// Perturb a local sensor-FLU unit direction for a global emission index.
    /// Reject nonfinite inputs or squared norms differing from one by more
    /// than `1e-9`. Output is a unit direction, not a measured point.
    ///
    /// Apply this BEFORE intersection, and use the same emitted direction for
    /// reported local XYZ. The caller still owns poses, times and occlusion.
    /// This matches the angular model used by [`crate::sensor::Sensor`].
    pub fn perturb(
        &self,
        sample_index: u64,
        direction: [f64; 3],
    ) -> Result<[f64; 3], &'static str> {
        let norm_squared = direction.iter().map(|v| v * v).sum::<f64>();
        if !norm_squared.is_finite() || (norm_squared - 1.).abs() > 1e-9 {
            return Err("angular noise requires a finite unit direction");
        }
        Ok(self.perturb_unit(sample_index, direction))
    }

    #[inline]
    pub(crate) fn perturb_unit(&self, sample_index: u64, mut d: [f64; 3]) -> [f64; 3] {
        if self.standard_deviation_rad == 0. {
            return d;
        }
        let a = d[1].atan2(d[0]);
        let z = d[2];
        let c = (1. - z * z).max(0.).sqrt();
        let tangent = [-a.sin(), a.cos(), 0.];
        let vertical = [-z * a.cos(), -z * a.sin(), c];
        // Retain the original Sensor streams and arithmetic so extracting this
        // module does not change existing seeded angular/range measurements.
        let x = self.standard_deviation_rad * normal(self.seed, sample_index, 10);
        let y = self.standard_deviation_rad * normal(self.seed, sample_index, 12);
        for k in 0..3 {
            d[k] += x * tangent[k] + y * vertical[k];
        }
        let norm = d.iter().map(|v| v * v).sum::<f64>().sqrt();
        for v in &mut d {
            *v /= norm;
        }
        d
    }
}

pub(crate) fn uniform(seed: u64, index: u64, stream: u64) -> f64 {
    let mut x =
        seed ^ index.wrapping_mul(0x9e3779b97f4a7c15) ^ stream.wrapping_mul(0xd1b54a32d192ed03);
    x = (x ^ (x >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94d049bb133111eb);
    (((x ^ (x >> 31)) >> 11) as f64 + 0.5) / 9007199254740992.
}

pub(crate) fn normal(seed: u64, index: u64, stream: u64) -> f64 {
    (-2. * uniform(seed, index, stream).ln()).sqrt()
        * (std::f64::consts::TAU * uniform(seed, index, stream + 1)).cos()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hardware::NominalPattern;

    #[test]
    fn validates_configuration_and_unit_directions() {
        for sigma in [
            f64::NAN,
            f64::INFINITY,
            -0.001,
            std::f64::consts::PI + 0.001,
        ] {
            assert!(AngularNoise::new(sigma, 7).is_err());
        }
        let noise = AngularNoise::new(std::f64::consts::PI, 7).unwrap();
        for d in [[0.; 3], [2., 0., 0.], [f64::NAN, 0., 0.], [f64::MAX; 3]] {
            assert!(noise.perturb(0, d).is_err());
        }
        for d in [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.], [0., 0., -1.]] {
            let out = noise.perturb(u64::MAX, d).unwrap();
            assert!((out.iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 1e-12);
        }
    }

    #[test]
    fn zero_noise_preserves_bits_including_signed_zero() {
        for seed in [0, 7, u64::MAX] {
            let noise = AngularNoise::new(0., seed).unwrap();
            for d in [[-0., 1., -0.], [0., -0., -1.], NominalPattern.direction(7)] {
                assert_eq!(
                    noise.perturb(42, d).unwrap().map(f64::to_bits),
                    d.map(f64::to_bits)
                );
                assert_eq!(AngularNoise::default().perturb(42, d).unwrap(), d);
            }
        }
    }

    #[test]
    fn indexing_preserves_reordering_chunks_and_decimation() {
        let noise = AngularNoise::new(0.001, 17).unwrap();
        let sample = |index| {
            noise
                .perturb(index, NominalPattern.direction(index))
                .unwrap()
        };
        let all: Vec<_> = (7..1007).map(sample).collect();
        let joined: Vec<_> = (7..506).chain(506..1007).map(sample).collect();
        assert_eq!(all, joined);
        for stride in [1, 2, 3, 4, 7, 31] {
            let retained: Vec<_> = (7..1007).step_by(stride).map(sample).collect();
            assert_eq!(
                retained,
                all.iter().step_by(stride).copied().collect::<Vec<_>>()
            );
        }
        let mut reversed: Vec<_> = (7..1007).rev().map(sample).collect();
        reversed.reverse();
        assert_eq!(all, reversed);
        assert_ne!(
            all[0],
            AngularNoise::new(0.001, 18)
                .unwrap()
                .perturb(7, NominalPattern.direction(7))
                .unwrap()
        );
    }

    #[test]
    fn small_perturbations_have_the_requested_independent_axis_spread() {
        let sigma = 0.001;
        let noise = AngularNoise::new(sigma, 7).unwrap();
        let count = 20_000;
        let (mut sums, mut squares, mut cross) = ([0.; 2], [0.; 2], 0.);
        for index in 0..count {
            let d = noise.perturb(index, [1., 0., 0.]).unwrap();
            assert!((d.iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 1e-12);
            for axis in 0..2 {
                sums[axis] += d[axis + 1];
                squares[axis] += d[axis + 1] * d[axis + 1];
            }
            cross += d[1] * d[2];
        }
        for axis in 0..2 {
            assert!((sums[axis] / count as f64).abs() < sigma * 0.05);
            assert!(((squares[axis] / count as f64).sqrt() - sigma).abs() < sigma * 0.05);
        }
        assert!((cross / count as f64).abs() < sigma * sigma * 0.05);
    }
}
