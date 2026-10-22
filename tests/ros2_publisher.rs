// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Native publisher regression, run by `pixi run test` with installed interfaces.
#![cfg(feature = "ros2")]

use anyhow::{Context, Result};
use mid360_sim::{
    LivoxConfig, LivoxPacket, RosTime,
    publisher::{LivoxPublisher, PointCloudPublisher},
    sensor::{RollingFrame, TimedReturn},
};
use rclrs::CreateBasicExecutor;
use std::{
    io::{BufRead, BufReader},
    process::{Command, Stdio},
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
    check_publisher::<LivoxPublisher>()?;
    #[cfg(feature = "ros2-typed")]
    check_publisher::<mid360_sim::publisher::TypedLivoxPublisher>()?;
    Ok(())
}

trait TestPublisher: Sized {
    const SINGLETON: bool;
    fn new(node: &rclrs::Node, topic: &str) -> Result<Self>;
    fn subscription_count(&self) -> Result<usize>;
    fn publish(&self, packet: &LivoxPacket) -> Result<()>;
    fn publish_clock(&self, time: u64) -> Result<()>;
}
macro_rules! test_publisher {
    ($ty:ty, $singleton:expr) => {
        impl TestPublisher for $ty {
            const SINGLETON: bool = $singleton;
            fn new(node: &rclrs::Node, topic: &str) -> Result<Self> {
                <$ty>::new(node, topic)
            }
            fn subscription_count(&self) -> Result<usize> {
                <$ty>::subscription_count(self)
            }
            fn publish(&self, packet: &LivoxPacket) -> Result<()> {
                <$ty>::publish(self, packet)
            }
            fn publish_clock(&self, time: u64) -> Result<()> {
                <$ty>::publish_clock(self, time)
            }
        }
    };
}
test_publisher!(LivoxPublisher, false);
#[cfg(feature = "ros2-typed")]
test_publisher!(mid360_sim::publisher::TypedLivoxPublisher, true);

fn check_publisher<P: TestPublisher>() -> Result<()> {
    let context = rclrs::Context::new(
        std::iter::empty::<String>(),
        rclrs::InitOptions::new().with_domain_id(Some(125)),
    )?;
    let executor = context.create_basic_executor();
    let mode = if P::SINGLETON { "typed" } else { "dynamic" };
    let name = format!("publisher_regression_{}_{mode}", std::process::id());
    let node = executor.create_node(name.as_str())?;
    let topic = format!("/publisher_regression_{}_{mode}", std::process::id());
    let livox = P::new(&node, &format!("{topic}/livox"))?;
    let cloud = PointCloudPublisher::new(&node, &format!("{topic}/cloud"))?;
    let mut subscriber = Subscriber(
        Command::new("python")
            .arg(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/tests/ros2_publisher_check.py"
            ))
            .arg(&topic)
            .arg(if P::SINGLETON { "typed" } else { "dynamic" })
            .env("ROS_DOMAIN_ID", "125")
            .stdout(Stdio::piped())
            .spawn()?,
    );
    let stdout = subscriber.0.stdout.take().expect("piped receiver output");
    let (sent, received) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        for line in BufReader::new(stdout).lines() {
            match line {
                Ok(line) if line.starts_with("ACK ") || line.starts_with("CLOUD ") => {
                    if sent.send(line).is_err() {
                        break;
                    }
                }
                Ok(line) => println!("{line}"),
                Err(_) => break,
            }
        }
    });
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
    for (i, n) in [0_usize, 1, 2, 20_000, 3, 1, 0, 3, 2, 0]
        .into_iter()
        .enumerate()
    {
        let base = 2_000_000_000 + i as u64 * 100_000_000;
        // The first emission misses, so every retained offset includes the gap.
        let frame = RollingFrame {
            points: (0..=n)
                .map(|j| TimedReturn {
                    sample_index: i as u64 * 30_000 + j as u64,
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
        // Wait for the cloud before sending a second large representation over
        // best-effort DDS. The fast typed path otherwise turns this field test
        // into a back-to-back 800 kB transport burst at the 20,000-point case.
        let cloud_ack = received
            .recv_timeout(Duration::from_secs(5))
            .with_context(|| format!("{mode} packet {i} cloud receive timeout"))?;
        anyhow::ensure!(
            cloud_ack == format!("CLOUD {i}"),
            "unexpected cloud ack: {cloud_ack}"
        );
        // A panic fails this test directly; a rejected packet must leave the
        // same cached publisher usable for the following empty/dense packets.
        let result = livox.publish(&packet);
        if n == 1 && !P::SINGLETON {
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
        if P::SINGLETON {
            assert!(livox.publish(&invalid).is_err());
        }
        assert!(livox.publish_clock(u64::MAX).is_err());
        // This checks fields/ownership, not overload delivery. Keep one packet
        // outstanding so a slow rclpy callback cannot overrun best-effort history.
        // No retry hides a missing message: every packet still needs its own ack.
        let ack = received
            .recv_timeout(Duration::from_secs(5))
            .with_context(|| format!("{mode} packet {i} ({n} points) receive timeout"))?;
        anyhow::ensure!(ack == format!("ACK {i}"), "unexpected receiver ack: {ack}");
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
