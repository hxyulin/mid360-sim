// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Native transport benchmark; host monotonic telemetry stays outside the core.
// Share acquisition math; the fixture also contains scoring helpers unused here.
#[allow(dead_code)]
#[path = "synchronized/fixture.rs"]
mod fixture;
use fixture::{EPOCH_NS, IMU_PERIOD_NS, Motion, PACKET_NS, Room};
use mid360_sim::{
    LivoxConfig, LivoxPacket,
    publisher::{ImuPublisher, LivoxPublisher, PointCloudPublisher},
    sensor::{Effects, RollingFrame, Sensor},
};
use rclrs::CreateBasicExecutor;
use std::{
    io::{self, BufRead, Write},
    time::{Duration, Instant},
};
fn main() -> anyhow::Result<()> {
    let origin = Instant::now();
    let mut frames = 60u64;
    let mut stride = 1u32;
    let mut sensors = 1u32;
    let mut format = String::from("pointcloud");
    let mut motion = Motion::Articulated;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let value = args
            .next()
            .ok_or_else(|| anyhow::anyhow!("missing value for {arg}"))?;
        match arg.as_str() {
            "--frames" => frames = value.parse()?,
            "--stride" => stride = value.parse()?,
            "--sensors" => sensors = value.parse()?,
            "--format" => format = value,
            "--motion" => motion = Motion::parse(&value).map_err(anyhow::Error::msg)?,
            _ => anyhow::bail!("unknown option {arg}"),
        }
    }
    anyhow::ensure!(
        (6..=600).contains(&frames) && (1..=1001).contains(&stride) && (1..=4).contains(&sensors),
        "invalid bounded workload"
    );
    anyhow::ensure!(
        ["livox", "pointcloud"].contains(&format.as_str()),
        "invalid format"
    );
    let context = rclrs::Context::new(std::iter::empty::<String>(), Default::default())?;
    let executor = context.create_basic_executor();
    let node = executor.create_node("mid360_transport_benchmark")?;
    let mut publishers = Vec::new();
    for i in 0..sensors {
        let prefix = format!("/transport/sensor_{i}");
        let lidar = LivoxPublisher::new(&node, &format!("{prefix}/livox"))?;
        let cloud = PointCloudPublisher::new(&node, &format!("{prefix}/points"))?;
        let imu = ImuPublisher::new(&node, &format!("{prefix}/imu"), &format!("sensor_{i}_imu"))?;
        publishers.push((lidar, cloud, imu));
    }
    let wait = Instant::now();
    loop {
        let ready = publishers.iter().all(|(lidar, cloud, imu)| {
            let n = if format == "livox" {
                lidar.subscription_count()
            } else {
                cloud.subscription_count()
            };
            n.unwrap_or(0) > 0 && imu.subscription_count().unwrap_or(0) > 0
        });
        if ready {
            break;
        }
        anyhow::ensure!(
            wait.elapsed() < Duration::from_secs(10),
            "subscriber discovery timed out"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
    println!("READY");
    io::stdout().flush()?;
    let stdin = io::stdin();
    let mut lines = stdin.lock().lines();
    loop {
        match lines.next().transpose()?.as_deref() {
            Some("PING") => {
                println!("PONG {}", origin.elapsed().as_nanos());
                io::stdout().flush()?;
            }
            Some("GO") => break,
            _ => anyhow::bail!("expected bounded clock handshake or GO"),
        }
    }
    let sensor = Sensor::new(0.1, 40., EPOCH_NS, Effects::default()).map_err(anyhow::Error::msg)?;
    let room = Room(motion);
    let start = Instant::now();
    let start_ns = origin.elapsed().as_nanos();
    let mut scan = RollingFrame::default();
    for frame in 0..frames {
        let target = Duration::from_nanos((frame + 1) * PACKET_NS);
        if let Some(wait) = target.checked_sub(start.elapsed()) {
            std::thread::sleep(wait);
        }
        for (i, (lidar, cloud, imu)) in publishers.iter().enumerate() {
            let first = (frame * 20_000).div_ceil(u64::from(stride)) * u64::from(stride);
            let stop = (frame + 1) * 20_000;
            let count = (stop - first).div_ceil(u64::from(stride)) as u32;
            let acquire_start = origin.elapsed().as_nanos();
            sensor
                .sample_into(&room, first, count, stride, &mut scan)
                .map_err(anyhow::Error::msg)?;
            let samples: Vec<_> = (1..=20)
                .map(|j| fixture::imu(EPOCH_NS + frame * PACKET_NS + j * IMU_PERIOD_NS, motion))
                .collect();
            let acquired = origin.elapsed().as_nanos();
            let packet = LivoxPacket::from_scan(
                &scan,
                &LivoxConfig {
                    frame_id: format!("sensor_{i}_lidar"),
                    ..Default::default()
                },
            )
            .map_err(anyhow::Error::msg)?;
            let converted = origin.elapsed().as_nanos();
            // IMU arrives in the same 20-sample bursts as the synchronized bridge.
            let imu_begin = origin.elapsed().as_nanos();
            for sample in &samples {
                imu.publish(sample)?;
            }
            let begin = origin.elapsed().as_nanos();
            if format == "livox" {
                lidar.publish(&packet)?;
            } else {
                cloud.publish(&packet)?;
            }
            let done = origin.elapsed().as_nanos();
            println!(
                "EVENT {i} {frame} {} {} {acquire_start} {acquired} {converted} {imu_begin} {begin} {done} {}",
                packet.timebase_ns,
                packet.points.len(),
                start_ns + u128::from((frame + 1) * PACKET_NS)
            );
        }
        io::stdout().flush()?;
    }
    std::thread::sleep(Duration::from_millis(100));
    Ok(())
}
