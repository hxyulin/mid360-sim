// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Dependency-free PointCloud2 wire layout. Raw points retain instantaneous frames.
use crate::LivoxPacket;
/// Packed little-endian record size in bytes, including one zero padding byte.
pub const POINT_STEP: u32 = 20;
/// `(name, byte offset, ROS PointField datatype)`; all fields have count one.
/// XYZ are FLOAT32 metres, offset_time is UINT32 nanoseconds relative to header,
/// reflectivity/tag/line are UINT8 and retain the Livox policies.
pub const FIELDS: [(&str, u32, u8); 7] = [
    ("x", 0, 7),
    ("y", 4, 7),
    ("z", 8, 7),
    ("offset_time", 12, 6),
    ("reflectivity", 16, 2),
    ("tag", 17, 2),
    ("line", 18, 2),
];
/// Encode the valid returns of a checked Livox packet as little-endian records.
/// Omits misses without closing time gaps. Rejects mutated malformed payloads.
pub fn encode(packet: &LivoxPacket) -> Result<Vec<u8>, &'static str> {
    if packet.points.len() > crate::sensor::MAX_LIDAR_RAYS {
        return Err("point cloud exceeds allocation budget");
    }
    let mut data = Vec::with_capacity(packet.points.len() * POINT_STEP as usize);
    let mut previous = None;
    for p in &packet.points {
        if ![p.x_m, p.y_m, p.z_m].iter().all(|v| v.is_finite())
            || previous.is_some_and(|t| p.offset_time_ns <= t)
        {
            return Err("nonfinite or nonmonotonic cloud point");
        }
        previous = Some(p.offset_time_ns);
        for v in [p.x_m, p.y_m, p.z_m] {
            data.extend(v.to_le_bytes());
        }
        data.extend(p.offset_time_ns.to_le_bytes());
        data.extend([p.reflectivity, p.tag, p.line, 0]);
    }
    Ok(data)
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        LivoxConfig,
        sensor::{RollingFrame, TimedReturn},
    };
    #[test]
    fn wire_layout_keeps_misses_timestamps_and_lanes() {
        let scan = RollingFrame {
            points: vec![
                TimedReturn {
                    sample_index: 0,
                    time_ns: 1_000_000_000,
                    range_m: None,
                    point_sensor_m: None,
                },
                TimedReturn {
                    sample_index: 3,
                    time_ns: 1_000_015_000,
                    range_m: Some(5.),
                    point_sensor_m: Some([3., -4., 0.]),
                },
            ],
        };
        let p = LivoxPacket::from_scan(&scan, &LivoxConfig::default()).unwrap();
        let b = encode(&p).unwrap();
        assert_eq!(b.len(), 20);
        assert_eq!(f32::from_le_bytes(b[4..8].try_into().unwrap()), -4.);
        assert_eq!(u32::from_le_bytes(b[12..16].try_into().unwrap()), 15_000);
        assert_eq!(&b[16..], &[100, 0x10, 3, 0]);
        let empty = LivoxPacket::from_scan(
            &RollingFrame {
                points: vec![scan.points[0]],
            },
            &LivoxConfig::default(),
        )
        .unwrap();
        assert!(encode(&empty).unwrap().is_empty());
        let mut bad = p;
        bad.points[0].x_m = f32::NAN;
        assert!(encode(&bad).is_err());
    }
}
