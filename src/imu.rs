// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Caller-provided inertial measurements. This module owns no synthesis or clock.

/// One externally acquired IMU measurement in the physical IMU's FLU frame.
/// Its clock must match the LiDAR clock. No orientation truth is included.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuSample {
    /// Acquisition timestamp in simulation nanoseconds, not publication wall time.
    /// For averaged samples, callers should use the averaging interval's end.
    pub time_ns: u64,
    /// Angular velocity about sensor x/y/z, radians per second.
    pub angular_velocity_rad_s: [f64; 3],
    /// Specific force along sensor x/y/z, metres per second squared.
    /// An upright stationary sensor reads positive gravity on z, not zero.
    pub specific_force_m_s2: [f64; 3],
}
impl ImuSample {
    /// Reject nonfinite measurements without changing their units or timestamp.
    ///
    /// ```
    /// use mid360_sim::imu::ImuSample;
    /// let sample = ImuSample {
    ///     time_ns: 5_000_000,
    ///     angular_velocity_rad_s: [0.; 3],
    ///     specific_force_m_s2: [0., 0., 9.80665],
    /// };
    /// sample.validate()?;
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn validate(&self) -> Result<(), &'static str> {
        if self
            .angular_velocity_rad_s
            .iter()
            .chain(&self.specific_force_m_s2)
            .all(|v| v.is_finite())
        {
            Ok(())
        } else {
            Err("nonfinite inertial measurement")
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn validates_both_vectors_without_scaling_or_retiming() {
        let sample = ImuSample {
            time_ns: u64::MAX,
            angular_velocity_rad_s: [1., 2., 3.],
            specific_force_m_s2: [0., 0., 9.81],
        };
        assert_eq!(sample.validate(), Ok(()));
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for axis in 0..3 {
                let mut bad = sample;
                bad.angular_velocity_rad_s[axis] = value;
                assert!(bad.validate().is_err());
                let mut bad = sample;
                bad.specific_force_m_s2[axis] = value;
                assert!(bad.validate().is_err());
            }
        }
    }
}
