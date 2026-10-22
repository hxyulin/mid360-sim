// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Native publisher regression, run by `pixi run test` with installed interfaces.
#![cfg(feature = "ros2")]

use anyhow::Result;
use mid360_sim::{
    LivoxConfig, LivoxPacket, RosTime,
    publisher::{LivoxPublisher, PointCloudPublisher},
    sensor::{RollingFrame, TimedReturn},
};
use rclrs::CreateBasicExecutor;
use std::{
    process::Command,
    time::{Duration, Instant},
};

struct Subscriber(std::process::Child);
impl Drop for Subscriber {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

#[test]
fn packet_size_transitions_preserve_native_fields() -> Result<()> {
    let context = rclrs::Context::default_from_env()?;
    let executor = context.create_basic_executor();
    let name = format!("publisher_regression_{}", std::process::id());
    let node = executor.create_node(name.as_str())?;
    let topic = format!("/publisher_regression_{}", std::process::id());
    let livox = LivoxPublisher::new(&node, &format!("{topic}/livox"))?;
    let cloud = PointCloudPublisher::new(&node, &format!("{topic}/cloud"))?;
    let mut subscriber = Subscriber(
        Command::new("python")
            .arg(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/tests/ros2_publisher_check.py"
            ))
            .arg(&topic)
            .spawn()?,
    );
    let start = Instant::now();
    while livox.subscription_count()? == 0 || cloud.subscription_count()? == 0 {
        anyhow::ensure!(
            start.elapsed() < Duration::from_secs(15),
            "discovery timeout"
        );
        anyhow::ensure!(
            subscriber.0.try_wait()?.is_none(),
            "subscriber exited early"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
    // Graph discovery can precede completion of best-effort endpoint matching.
    std::thread::sleep(Duration::from_millis(300));
    for (i, n) in [0_usize, 1, 2, 3, 1, 0, 3, 2, 0].into_iter().enumerate() {
        let base = 2_000_000_000 + i as u64 * 100_000_000;
        // The first emission misses, so every retained offset includes the gap.
        let frame = RollingFrame {
            points: (0..=n)
                .map(|j| TimedReturn {
                    sample_index: i as u64 * 100 + j as u64,
                    time_ns: base + j as u64 * 5_000,
                    range_m: (j > 0).then_some(2.),
                    point_sensor_m: (j > 0).then_some([2., j as f64, -1.]),
                })
                .collect(),
        };
        let config = LivoxConfig {
            frame_id: format!("regression_{i}"),
            lidar_id: 7,
            reflectivity: 83,
        };
        let packet = LivoxPacket::from_scan(&frame, &config).map_err(anyhow::Error::msg)?;
        assert_eq!(packet.points.len(), n);
        cloud.publish(&packet)?;
        // A panic fails this test directly; a rejected packet must leave the
        // same cached publisher usable for the following empty/dense packets.
        let result = livox.publish(&packet);
        if n == 1 {
            let error = result
                .expect_err("single-point Livox must return an error")
                .to_string();
            assert!(error.contains("single-point Livox"), "{error}");
            assert!(error.contains("PointCloud2"), "{error}");
        } else {
            result?;
        }
        livox.publish_clock(base)?;
        let mut invalid = packet.clone();
        invalid.stamp = RosTime::from_ns(base + 1).map_err(anyhow::Error::msg)?;
        assert!(cloud.publish(&invalid).is_err());
        assert!(livox.publish_clock(u64::MAX).is_err());
        std::thread::sleep(Duration::from_millis(60));
    }
    let start = Instant::now();
    loop {
        if let Some(status) = subscriber.0.try_wait()? {
            anyhow::ensure!(status.success(), "native field checker failed: {status}");
            break;
        }
        anyhow::ensure!(
            start.elapsed() < Duration::from_secs(20),
            "subscriber timeout"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    Ok(())
}
