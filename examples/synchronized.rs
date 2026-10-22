// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Caller-owned clocks, kinematic fixture and IMU, with optional rclrs output.
//! cargo run --release --example synchronized -- --evaluate
#[path = "synchronized/fixture.rs"]
mod fixture;
use fixture::{EPOCH_NS, IMU_PERIOD_NS, Motion, PACKET_NS, Room};
use mid360_sim::{
    LivoxConfig, LivoxPacket,
    imu::ImuSample,
    sensor::{Effects, RollingFrame, Sensor},
};
use std::{error::Error, time::Instant};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

struct Options {
    motion: Motion,
    frames: u64,
    stride: u32,
    chunk_ns: u64,
    evaluate: bool,
    ros: bool,
}
fn options() -> Result<Options> {
    let mut opt = Options {
        motion: Motion::Articulated,
        frames: 20,
        stride: 1,
        chunk_ns: 7_812_500,
        evaluate: false,
        ros: false,
    };
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--motion" => opt.motion = Motion::parse(&args.next().ok_or("missing motion")?)?,
            "--frames" => opt.frames = args.next().ok_or("missing frame count")?.parse()?,
            "--stride" => opt.stride = args.next().ok_or("missing stride")?.parse()?,
            "--chunk-ns" => opt.chunk_ns = args.next().ok_or("missing chunk duration")?.parse()?,
            "--evaluate" => opt.evaluate = true,
            "--ros" => opt.ros = true,
            "--help" => {
                println!(
                    "synchronized [--evaluate] [--motion stationary|yaw|articulated] [--frames 20] [--stride 1] [--chunk-ns 7812500] [--ros]"
                );
                std::process::exit(0);
            }
            _ => return Err(format!("unknown argument: {arg}").into()),
        }
    }
    if !(1..=600).contains(&opt.frames)
        || !(1..=1001).contains(&opt.stride)
        || opt.chunk_ns == 0
        || opt.chunk_ns > PACKET_NS
    {
        return Err("invalid bounded frame count, stride or chunk duration".into());
    }
    if opt.ros && opt.evaluate {
        return Err("--evaluate is offline; use --ros separately".into());
    }
    Ok(opt)
}
#[derive(Default)]
struct Metrics {
    packets: u64,
    emissions: u64,
    returns: u64,
    imu: u64,
    squared_wall_error: f64,
    squared_frozen_error: f64,
    hash: u64,
    retained_hash: u64,
    elapsed_s: f64,
}
fn hash(hash: &mut u64, value: u64) {
    *hash = (*hash ^ value).wrapping_mul(0x100000001b3);
}
fn point_hash(hash_value: &mut u64, p: &mid360_sim::sensor::TimedReturn) {
    hash(hash_value, p.sample_index);
    hash(hash_value, p.time_ns);
    hash(hash_value, p.range_m.map_or(u64::MAX, f64::to_bits));
    for v in p.point_sensor_m.unwrap_or([f64::INFINITY; 3]) {
        hash(hash_value, v.to_bits());
    }
}
fn acquire(
    motion: Motion,
    frames: u64,
    stride: u32,
    chunk_ns: u64,
    effects: Effects,
    mut output: impl FnMut(&RollingFrame, &[ImuSample], u64) -> Result<()>,
) -> Result<Metrics> {
    let sensor = Sensor::new(0.1, 40., EPOCH_NS, effects)?;
    let room = Room(motion);
    let start = Instant::now();
    let finish = frames * PACKET_NS;
    let mut metrics = Metrics {
        hash: 0xcbf29ce484222325,
        retained_hash: 0xcbf29ce484222325,
        ..Default::default()
    };
    let mut pending = RollingFrame::default();
    let mut chunk = RollingFrame::default();
    let mut boundary = PACKET_NS;
    let mut step = 0;
    while step < finish {
        let after = (step + chunk_ns).min(finish);
        let mut cursor = step;
        while cursor < after {
            let end = after.min(boundary);
            let first = cursor.div_ceil(5000).div_ceil(u64::from(stride)) * u64::from(stride);
            let last = end.div_ceil(5000);
            if first < last {
                sensor.sample_into(
                    &room,
                    first,
                    ((last - first).div_ceil(u64::from(stride))) as u32,
                    stride,
                    &mut chunk,
                )?;
                pending.points.extend_from_slice(&chunk.points);
            }
            if end == boundary {
                let packet_start = EPOCH_NS + boundary - PACKET_NS;
                let inertial: Vec<_> = (1..=PACKET_NS / IMU_PERIOD_NS)
                    .map(|i| fixture::imu(packet_start + i * IMU_PERIOD_NS, motion))
                    .collect();
                for sample in &inertial {
                    sample.validate()?;
                }
                for p in &pending.points {
                    point_hash(&mut metrics.hash, p);
                    if p.sample_index % 3 == 0 {
                        point_hash(&mut metrics.retained_hash, p);
                    }
                    if let Some(xyz) = p.point_sensor_m {
                        metrics.returns += 1;
                        metrics.squared_wall_error +=
                            fixture::wall_error(fixture::world_point(p.time_ns, xyz, motion))
                                .powi(2);
                        metrics.squared_frozen_error +=
                            fixture::wall_error(fixture::world_point(packet_start, xyz, motion))
                                .powi(2);
                    }
                }
                metrics.emissions += pending.points.len() as u64;
                metrics.imu += inertial.len() as u64;
                metrics.packets += 1;
                output(&pending, &inertial, EPOCH_NS + boundary)?;
                pending.points.clear();
                boundary += PACKET_NS;
            }
            cursor = end;
        }
        step = after;
    }
    metrics.elapsed_s = start.elapsed().as_secs_f64();
    Ok(metrics)
}
fn print_metrics(mode: Motion, stride: u32, metrics: &Metrics) {
    let n = metrics.returns.max(1) as f64;
    println!(
        "{{\"motion\":\"{}\",\"stride\":{},\"packets\":{},\"emissions\":{},\"returns\":{},\"imu_samples\":{},\"wall_rms_m\":{:.12},\"frozen_wall_rms_m\":{:.9},\"elapsed_s\":{:.6}}}",
        mode.name(),
        stride,
        metrics.packets,
        metrics.emissions,
        metrics.returns,
        metrics.imu,
        (metrics.squared_wall_error / n).sqrt(),
        (metrics.squared_frozen_error / n).sqrt(),
        metrics.elapsed_s
    );
}
fn main() -> Result<()> {
    let opt = options()?;
    if opt.evaluate {
        for mode in [Motion::Stationary, Motion::Yaw, Motion::Articulated] {
            let full = acquire(
                mode,
                opt.frames,
                1,
                PACKET_NS,
                Effects::default(),
                |_, _, _| Ok(()),
            )?;
            let fragmented = acquire(
                mode,
                opt.frames,
                1,
                7_812_500,
                Effects::default(),
                |_, _, _| Ok(()),
            )?;
            let sparse = acquire(
                mode,
                opt.frames,
                3,
                7_812_500,
                Effects::default(),
                |_, _, _| Ok(()),
            )?;
            assert_eq!(full.hash, fragmented.hash, "chunking changed measurements");
            assert_eq!(
                full.retained_hash, sparse.hash,
                "decimation changed retained measurements"
            );
            assert!((full.squared_wall_error / full.returns as f64).sqrt() < 1e-10);
            for stride in [1, 3] {
                let effects = Effects {
                    range_sigma_m: 0.02,
                    angular_sigma_rad: 0.001,
                    dropout: 0.01,
                    ..Default::default()
                };
                let a = acquire(mode, opt.frames, stride, PACKET_NS, effects, |_, _, _| {
                    Ok(())
                })?;
                let b = acquire(mode, opt.frames, stride, 7_812_500, effects, |_, _, _| {
                    Ok(())
                })?;
                assert_eq!(a.hash, b.hash, "chunking changed noisy measurements");
            }
            print_metrics(mode, 1, &fragmented);
            print_metrics(mode, 3, &sparse);
        }
        println!(
            "PASS: shared integer clock, ideal/noisy chunking invariance, retained decimation and analytic wall consistency"
        );
        return Ok(());
    }
    let mut output = RosOutput::new(opt.ros)?;
    let metrics = acquire(
        opt.motion,
        opt.frames,
        opt.stride,
        opt.chunk_ns,
        Effects::default(),
        |scan, imu, end| output.publish(scan, imu, end),
    )?;
    print_metrics(opt.motion, opt.stride, &metrics);
    output.finish();
    Ok(())
}
#[cfg(feature = "ros2")]
struct RosOutput {
    publishers: Option<(
        mid360_sim::publisher::LivoxPublisher,
        mid360_sim::publisher::ImuPublisher,
    )>,
    _node: Option<rclrs::Node>,
    _executor: Option<rclrs::Executor>,
    _context: Option<rclrs::Context>,
    start: Instant,
}
#[cfg(feature = "ros2")]
impl RosOutput {
    fn new(enabled: bool) -> Result<Self> {
        use rclrs::CreateBasicExecutor;
        let mut out = Self {
            publishers: None,
            _node: None,
            _executor: None,
            _context: None,
            start: Instant::now(),
        };
        if enabled {
            let context = rclrs::Context::new(std::iter::empty::<String>(), Default::default())?;
            let executor = context.create_basic_executor();
            let node = executor.create_node("mid360_synchronized_example")?;
            let lidar = mid360_sim::publisher::LivoxPublisher::new(&node, "/synchronized/lidar")?;
            let imu = mid360_sim::publisher::ImuPublisher::new(
                &node,
                "/synchronized/imu",
                "example_imu",
            )?;
            let wait = Instant::now();
            while (lidar.subscription_count()? == 0 || imu.subscription_count()? == 0)
                && wait.elapsed() < std::time::Duration::from_secs(5)
            {
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
            if lidar.subscription_count()? == 0 || imu.subscription_count()? == 0 {
                return Err("both LiDAR and IMU subscribers are required within 5 seconds".into());
            }
            lidar.publish_clock(EPOCH_NS)?;
            out.publishers = Some((lidar, imu));
            out._node = Some(node);
            out._executor = Some(executor);
            out._context = Some(context);
            out.start = Instant::now();
        }
        Ok(out)
    }
    fn publish(&mut self, scan: &RollingFrame, imu: &[ImuSample], end: u64) -> Result<()> {
        if let Some((lidar, publisher)) = &self.publishers {
            let target = std::time::Duration::from_nanos(end - EPOCH_NS);
            if let Some(wait) = target.checked_sub(self.start.elapsed()) {
                std::thread::sleep(wait);
            }
            for sample in imu {
                publisher.publish(sample)?;
            }
            let packet = LivoxPacket::from_scan(
                scan,
                &LivoxConfig {
                    frame_id: "example_lidar".into(),
                    ..Default::default()
                },
            )?;
            lidar.publish(&packet)?;
            lidar.publish_clock(end)?;
        }
        Ok(())
    }
    fn finish(&self) {
        if self.publishers.is_some() {
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
    }
}
#[cfg(not(feature = "ros2"))]
struct RosOutput;
#[cfg(not(feature = "ros2"))]
impl RosOutput {
    fn new(enabled: bool) -> Result<Self> {
        if enabled {
            Err("--ros requires --features ros2 and a sourced runtime".into())
        } else {
            Ok(Self)
        }
    }
    fn publish(&mut self, scan: &RollingFrame, _imu: &[ImuSample], _end: u64) -> Result<()> {
        let _ = LivoxPacket::from_scan(scan, &LivoxConfig::default())?;
        Ok(())
    }
    fn finish(&self) {}
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn packet_splits_keep_imu_endpoints_and_source_indices() {
        let metrics = acquire(
            Motion::Articulated,
            2,
            3,
            7_812_500,
            Effects::default(),
            |scan, imu, end| {
                assert_eq!(imu.len(), 20);
                assert_eq!(imu.last().unwrap().time_ns, end);
                assert!(scan.points.iter().all(|p| p.time_ns >= end - PACKET_NS
                    && p.time_ns < end
                    && p.sample_index % 3 == 0));
                assert!(
                    imu.windows(2)
                        .all(|p| p[1].time_ns - p[0].time_ns == IMU_PERIOD_NS)
                );
                Ok(())
            },
        )
        .unwrap();
        assert_eq!(metrics.emissions, 13_334);
        assert_eq!(metrics.imu, 40);
    }
}
