// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Standalone deterministic fitted MID-360 patterns and scene-query acquisitions.
//! The optional `ros2` feature enables rclrs and requires a sourced ROS installation.
//! Measured points remain in each emission's sensor frame; no truth pose is sent.
mod coefficients;
pub mod pattern;
pub mod sensor;
use sensor::{MAX_LIDAR_RAYS, RollingFrame};

#[cfg(feature = "ros2")]
pub mod publisher;

/// Constant synthetic intensity policy. No material reflectivity is simulated.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LivoxConfig {
    /// ROS frame of the physical LiDAR, normally `livox_frame`.
    pub frame_id: String,
    /// Driver-compatible device number; use distinct IDs for multiple sensors.
    pub lidar_id: u8,
    /// Placeholder reflectivity in [0,255], not a physical intensity measurement.
    pub reflectivity: u8,
}
impl Default for LivoxConfig {
    fn default() -> Self {
        Self {
            frame_id: "livox_frame".into(),
            lidar_id: 0,
            reflectivity: 100,
        }
    }
}
/// Checked ROS builtin_interfaces/Time representation of simulation nanoseconds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RosTime {
    /// Whole seconds, restricted to ROS's nonnegative signed 32-bit range.
    pub sec: i32,
    /// Remaining nanoseconds, strictly below one billion.
    pub nanosec: u32,
}
impl RosTime {
    /// Convert without clamping or wrapping ROS's signed seconds field.
    ///
    /// ```
    /// use mid360_sim::RosTime;
    /// let time = RosTime::from_ns(1_500_000_123)?;
    /// assert_eq!((time.sec, time.nanosec), (1, 500_000_123));
    /// # Ok::<(), &'static str>(())
    /// ```
    pub fn from_ns(ns: u64) -> Result<Self, &'static str> {
        Ok(Self {
            sec: i32::try_from(ns / 1_000_000_000).map_err(|_| "ROS timestamp seconds overflow")?,
            nanosec: (ns % 1_000_000_000) as u32,
        })
    }
}
/// Driver 2 point payload, using the upstream field units and names.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LivoxPoint {
    /// Nanoseconds relative to the packet's `timebase`, checked for uint32 overflow.
    pub offset_time_ns: u32,
    /// Local sensor forward coordinate, metres.
    pub x_m: f32,
    /// Local sensor left coordinate, metres.
    pub y_m: f32,
    /// Local sensor upward coordinate, metres.
    pub z_m: f32,
    /// Explicitly synthetic constant intensity, configured by the caller.
    pub reflectivity: u8,
    /// Normal first-return tag policy, 0x10; no confidence/noise classification.
    pub tag: u8,
    /// Inferred pattern lane, `sample_index % 4`, not reconstructed hardware timing.
    pub line: u8,
}
/// Validated CustomMsg payload. Reserved bytes are zero in the ROS encoder.
#[derive(Clone, Debug, PartialEq)]
pub struct LivoxPacket {
    /// Header frame/stationary intensity configuration.
    pub config: LivoxConfig,
    /// Header stamp, equal to `timebase` expressed as ROS time.
    pub stamp: RosTime,
    /// First emission's absolute simulation time, nanoseconds, even when it missed.
    pub timebase_ns: u64,
    /// Valid measured returns only, in original time order. Misses are omitted;
    /// offsets retain their actual timing rather than being renumbered.
    pub points: Vec<LivoxPoint>,
}
impl LivoxPacket {
    /// Convert a nonempty acquisition to CustomMsg fields. All-miss acquisitions
    /// remain valid empty point packets with a timestamp; zero-emission frames
    /// are errors because they have no acquisition time. Reject malformed points,
    /// nonmonotonic indices/times, unbounded buffers and offsets above uint32.
    pub fn from_scan(scan: &RollingFrame, config: &LivoxConfig) -> Result<Self, &'static str> {
        if config.frame_id.is_empty() || config.frame_id.contains('\0') {
            return Err("invalid LiDAR frame ID");
        }
        let first = scan
            .points
            .first()
            .ok_or("scan contains no emission timestamps")?;
        if scan.points.len() > MAX_LIDAR_RAYS {
            return Err("LiDAR packet exceeds allocation budget");
        }
        let mut packet = Self {
            config: config.clone(),
            stamp: RosTime::from_ns(first.time_ns)?,
            timebase_ns: first.time_ns,
            points: Vec::with_capacity(scan.points.len()),
        };
        let mut previous = None;
        for point in &scan.points {
            if previous.is_some_and(|(i, t)| point.sample_index <= i || point.time_ns <= t) {
                return Err("nonmonotonic LiDAR acquisition");
            }
            previous = Some((point.sample_index, point.time_ns));
            let offset_time = u32::try_from(
                point
                    .time_ns
                    .checked_sub(packet.timebase_ns)
                    .ok_or("point precedes timebase")?,
            )
            .map_err(|_| "Livox offset_time overflow; split the acquisition")?;
            match (point.range_m, point.point_sensor_m) {
                (None, None) => continue,
                (Some(range), Some(xyz))
                    if range.is_finite()
                        && range > 0.
                        && xyz.iter().all(|v| v.is_finite() && (*v as f32).is_finite()) =>
                {
                    packet.points.push(LivoxPoint {
                        offset_time_ns: offset_time,
                        x_m: xyz[0] as f32,
                        y_m: xyz[1] as f32,
                        z_m: xyz[2] as f32,
                        reflectivity: config.reflectivity,
                        tag: 0x10,
                        line: (point.sample_index % 4) as u8,
                    });
                }
                _ => return Err("inconsistent or non-finite measured LiDAR point"),
            }
        }
        Ok(packet)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensor::TimedReturn;
    fn hit(index: u64, time: u64) -> TimedReturn {
        TimedReturn {
            sample_index: index,
            time_ns: time,
            range_m: Some(2.),
            point_sensor_m: Some([1., -1., std::f64::consts::SQRT_2]),
        }
    }
    #[test]
    fn misses_preserve_timebase_offsets_and_lanes() {
        let mut miss = hit(100, 1_000_000_000);
        miss.range_m = None;
        miss.point_sensor_m = None;
        let frame = RollingFrame {
            points: vec![miss, hit(103, 1_000_015_000), hit(106, 1_000_030_000)],
        };
        let p = LivoxPacket::from_scan(&frame, &LivoxConfig::default()).unwrap();
        assert_eq!(p.timebase_ns, 1_000_000_000);
        assert_eq!(p.stamp, RosTime { sec: 1, nanosec: 0 });
        assert_eq!(p.points.len(), 2);
        assert_eq!(p.points[0].offset_time_ns, 15000);
        assert_eq!(p.points[0].line, 3);
        assert_eq!(p.points[1].line, 2);
        assert_eq!(p.points[0].tag, 0x10);
        assert_eq!(p.points[0].y_m, -1.);
        let empty = LivoxPacket::from_scan(
            &RollingFrame { points: vec![miss] },
            &LivoxConfig::default(),
        )
        .unwrap();
        assert!(empty.points.is_empty());
        assert!(LivoxPacket::from_scan(&RollingFrame::default(), &LivoxConfig::default()).is_err());
    }
    #[test]
    fn malformed_data_and_timestamp_overflows_fail() {
        for frame in [
            RollingFrame {
                points: vec![hit(1, 5), hit(2, 5)],
            },
            RollingFrame {
                points: vec![hit(2, 5), hit(1, 10)],
            },
            RollingFrame {
                points: vec![hit(1, 0), hit(2, u64::from(u32::MAX) + 1)],
            },
            RollingFrame {
                points: vec![TimedReturn {
                    point_sensor_m: Some([f64::NAN, 0., 0.]),
                    ..hit(1, 0)
                }],
            },
            RollingFrame {
                points: vec![TimedReturn {
                    point_sensor_m: None,
                    ..hit(1, 0)
                }],
            },
        ] {
            assert!(LivoxPacket::from_scan(&frame, &LivoxConfig::default()).is_err());
        }
        assert!(RosTime::from_ns((i32::MAX as u64 + 1) * 1_000_000_000).is_err());
        assert_eq!(
            RosTime::from_ns(1_000_000_001).unwrap(),
            RosTime { sec: 1, nanosec: 1 }
        );
    }
}
