// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! ROS metadata construction microbenchmark, excluding field population and DDS.
use rclrs::{DynamicMessage, DynamicMessageMetadata, MessageTypeName};
use std::{hint::black_box, time::Instant};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    for name in [
        "livox_ros_driver2/msg/CustomMsg",
        "rosgraph_msgs/msg/Clock",
        "sensor_msgs/msg/Imu",
        "sensor_msgs/msg/PointCloud2",
        "tf2_msgs/msg/TFMessage",
    ] {
        let metadata = DynamicMessageMetadata::new(MessageTypeName::try_from(name)?)?;
        let mut fresh = Vec::new();
        let mut cached = Vec::new();
        for trial in 0..12 {
            // Alternate case order to limit thermal/order bias.
            for reuse in if trial % 2 == 0 {
                [false, true]
            } else {
                [true, false]
            } {
                let start = Instant::now();
                for _ in 0..1000 {
                    let message = if reuse {
                        metadata.create()?
                    } else {
                        DynamicMessage::new(MessageTypeName::try_from(black_box(name))?)?
                    };
                    black_box(message);
                }
                if trial >= 2 {
                    let samples = if reuse { &mut cached } else { &mut fresh };
                    samples.push(start.elapsed().as_secs_f64() * 1000.);
                }
            }
        }
        fresh.sort_by(f64::total_cmp);
        cached.sort_by(f64::total_cmp);
        println!(
            "{{\"type\":\"{name}\",\"messages\":1000,\"trials\":10,\"fresh_median_ms\":{:.6},\"cached_median_ms\":{:.6}}}",
            (fresh[4] + fresh[5]) * 0.5,
            (cached[4] + cached[5]) * 0.5,
        );
    }
    Ok(())
}
