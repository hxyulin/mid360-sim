// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Native construction experiment; no production publisher selection is changed.
use anyhow::{Result, bail};
use mid360_sim::{LivoxPoint, pointcloud};
use rclrs::{
    CreateBasicExecutor, DynamicMessage, DynamicMessageMetadata, IntoPrimitiveOptions,
    MessageTypeName, SequenceValueMut, SimpleValueMut, ValueMut,
};
use ros_env::livox_ros_driver2::msg::rmw::{CustomMsg, CustomPoint};
use std::{
    hint::black_box,
    process::Command,
    time::{Duration, Instant},
};

macro_rules! scalar {
    ($view:expr, $name:expr, $kind:ident, $value:expr) => {
        match $view.get_mut($name) {
            Some(ValueMut::Simple(SimpleValueMut::$kind(v))) => *v = $value,
            _ => bail!("schema mismatch: {}", $name),
        }
    };
}

fn dynamic(
    metadata: &DynamicMessageMetadata,
    points: &[LivoxPoint],
) -> Result<(DynamicMessage, [f64; 3])> {
    let start = Instant::now();
    let mut msg = metadata.create()?;
    scalar!(msg, "timebase", Uint64, 2_000_000_000);
    scalar!(msg, "point_num", Uint32, points.len() as u32);
    scalar!(msg, "lidar_id", Uint8, 7);
    if let Some(ValueMut::Simple(SimpleValueMut::Message(mut h))) = msg.get_mut("header") {
        scalar!(h, "frame_id", String, "benchmark_lidar".into());
        if let Some(ValueMut::Simple(SimpleValueMut::Message(mut t))) = h.get_mut("stamp") {
            scalar!(t, "sec", Int32, 2);
            scalar!(t, "nanosec", Uint32, 0);
        } else {
            bail!("stamp schema")
        }
    } else {
        bail!("header schema")
    }
    let create = start.elapsed().as_secs_f64() * 1000.;
    if let Some(ValueMut::Sequence(SequenceValueMut::MessageSequence(mut seq))) =
        msg.get_mut("points")
    {
        let start = Instant::now();
        seq.reset(points.len());
        let reset = start.elapsed().as_secs_f64() * 1000.;
        let start = Instant::now();
        for (view, p) in seq.as_mut_slice().iter_mut().zip(points) {
            scalar!(view, "offset_time", Uint32, p.offset_time_ns);
            scalar!(view, "x", Float, p.x_m);
            scalar!(view, "y", Float, p.y_m);
            scalar!(view, "z", Float, p.z_m);
            scalar!(view, "reflectivity", Uint8, p.reflectivity);
            scalar!(view, "tag", Uint8, p.tag);
            scalar!(view, "line", Uint8, p.line);
        }
        let fill = start.elapsed().as_secs_f64() * 1000.;
        Ok((msg, [create, reset, fill]))
    } else {
        bail!("points schema")
    }
}

fn typed(points: &[LivoxPoint]) -> CustomMsg {
    // Use generated RMW-native structs directly: no intermediate idiomatic Vec
    // converted on publish, no byte casts, no local unsafe code.
    let mut msg = CustomMsg::default();
    msg.header.stamp.sec = 2;
    msg.header.frame_id = "benchmark_lidar".into();
    msg.timebase = 2_000_000_000;
    msg.point_num = points.len() as u32;
    msg.lidar_id = 7;
    msg.points = points
        .iter()
        .map(|p| CustomPoint {
            offset_time: p.offset_time_ns,
            x: p.x_m,
            y: p.y_m,
            z: p.z_m,
            reflectivity: p.reflectivity,
            tag: p.tag,
            line: p.line,
        })
        .collect();
    msg
}

fn median(v: &[f64]) -> f64 {
    let mut values = v.to_vec();
    values.sort_by(f64::total_cmp);
    (values[4] + values[5]) * 0.5
}

fn points(n: usize) -> Vec<LivoxPoint> {
    (0..n)
        .map(|i| LivoxPoint {
            offset_time_ns: (i as u32 + 1) * 5000,
            x_m: 2.,
            y_m: i as f32 * 0.001,
            z_m: -1.,
            reflectivity: 83,
            tag: 0x10,
            line: (i % 4) as u8,
        })
        .collect()
}

struct Receiver(std::process::Child);
impl Drop for Receiver {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

fn validate(node: &rclrs::Node) -> Result<()> {
    let topic = format!("/typed_construction_check_{}", std::process::id());
    let publisher = node.create_publisher::<CustomMsg>(topic.as_str().keep_last(10).reliable())?;
    let mut receiver = Receiver(
        Command::new("python")
            .arg(concat!(env!("CARGO_MANIFEST_DIR"), "/check.py"))
            .arg(topic)
            .spawn()?,
    );
    let start = Instant::now();
    while publisher.get_subscription_count()? == 0 {
        anyhow::ensure!(
            start.elapsed() < Duration::from_secs(15),
            "discovery timeout"
        );
        anyhow::ensure!(receiver.0.try_wait()?.is_none(), "receiver exited early");
        std::thread::sleep(Duration::from_millis(10));
    }
    std::thread::sleep(Duration::from_millis(300));
    for (i, n) in [0, 1, 2, 20000, 3, 0, 1].into_iter().enumerate() {
        let mut msg = typed(&points(n));
        msg.header.stamp.sec = 3 + i as i32;
        msg.timebase = (3 + i as u64) * 1_000_000_000;
        publisher.publish(msg)?;
        std::thread::sleep(Duration::from_millis(50));
    }
    let start = Instant::now();
    loop {
        if let Some(status) = receiver.0.try_wait()? {
            anyhow::ensure!(status.success(), "subscriber field check failed");
            break;
        }
        anyhow::ensure!(
            start.elapsed() < Duration::from_secs(20),
            "receiver timeout"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    Ok(())
}

fn main() -> Result<()> {
    let context = rclrs::Context::default_from_env()?;
    let executor = context.create_basic_executor();
    let node = executor.create_node("construction_bench")?;
    if std::env::args().any(|a| a == "--production") {
        return production(&node);
    }
    if std::env::args().any(|a| a == "--validate") {
        return validate(&node);
    }
    let ty = MessageTypeName::try_from("livox_ros_driver2/msg/CustomMsg")?;
    let metadata = DynamicMessageMetadata::new(ty.clone())?;
    let dynamic_pubs = (0..2)
        .map(|i| {
            node.create_dynamic_publisher(
                ty.clone(),
                format!("/construction_dynamic_{i}")
                    .keep_last(5)
                    .best_effort(),
            )
        })
        .collect::<std::result::Result<Vec<_>, _>>()?;
    let typed_pubs = (0..2)
        .map(|i| {
            node.create_publisher::<CustomMsg>(
                format!("/construction_typed_{i}")
                    .keep_last(5)
                    .best_effort(),
            )
        })
        .collect::<std::result::Result<Vec<_>, _>>()?;
    for n in [0, 2, 67, 200, 6667, 20000] {
        for sensors in [1, 2] {
            for i in 0..sensors {
                anyhow::ensure!(
                    dynamic_pubs[i].get_subscription_count()? == 0
                        && typed_pubs[i].get_subscription_count()? == 0,
                    "timing requires unmatched endpoints; run field validation separately"
                );
            }
            let points: Vec<_> = (0..n)
                .map(|i| LivoxPoint {
                    offset_time_ns: (i as u32 + 1) * 5000,
                    x_m: 2.,
                    y_m: i as f32 * 0.001,
                    z_m: -1.,
                    reflectivity: 83,
                    tag: 0x10,
                    line: (i % 4) as u8,
                })
                .collect();
            // Compare all fields outside timing, including zero reserved bytes.
            let (old, _) = dynamic(&metadata, &points)?;
            let old = old
                .convert_into_rmw_message::<CustomMsg>()
                .map_err(|_| anyhow::anyhow!("native type conversion failed"))?;
            let new = typed(&points);
            anyhow::ensure!(old == new, "typed/dynamic fields differ");
            let mut samples: [Vec<f64>; 6] = std::array::from_fn(|_| Vec::new());
            for trial in 0..12 {
                for use_typed in if trial % 2 == 0 {
                    [false, true]
                } else {
                    [true, false]
                } {
                    let mut sums = [0.; 6];
                    for device in 0..sensors {
                        if use_typed {
                            let start = Instant::now();
                            let msg = typed(black_box(&points));
                            sums[4] += start.elapsed().as_secs_f64() * 1000.;
                            let start = Instant::now();
                            typed_pubs[device].publish(msg)?;
                            sums[5] += start.elapsed().as_secs_f64() * 1000.;
                        } else {
                            let (msg, times) = dynamic(&metadata, black_box(&points))?;
                            for i in 0..3 {
                                sums[i] += times[i];
                            }
                            let start = Instant::now();
                            dynamic_pubs[device].publish(msg)?;
                            sums[3] += start.elapsed().as_secs_f64() * 1000.;
                        }
                    }
                    if trial >= 2 {
                        for i in if use_typed { 4..6 } else { 0..4 } {
                            samples[i].push(sums[i]);
                        }
                    }
                }
            }
            let medians = samples.each_ref().map(|s| median(s));
            for i in 0..sensors {
                anyhow::ensure!(
                    dynamic_pubs[i].get_subscription_count()? == 0
                        && typed_pubs[i].get_subscription_count()? == 0,
                    "subscriber appeared during timing"
                );
            }
            // Independent byte encoder control; no ROS sequence copy in this control.
            let packet = mid360_sim::LivoxPacket {
                timebase_ns: 2_000_000_000,
                stamp: mid360_sim::RosTime { sec: 2, nanosec: 0 },
                config: mid360_sim::LivoxConfig::default(),
                points,
            };
            let start = Instant::now();
            for _ in 0..100 {
                black_box(pointcloud::encode(black_box(&packet)).map_err(anyhow::Error::msg)?);
            }
            println!(
                "{{\"points\":{n},\"sensors\":{sensors},\"stages\":[\"dynamic_create_header\",\"dynamic_sequence_reset\",\"dynamic_fill\",\"dynamic_publish_drop\",\"typed_build\",\"typed_publish_drop\"],\"median_ms\":{medians:?},\"samples_ms\":{samples:?},\"encode_ms\":{:.6}}}",
                start.elapsed().as_secs_f64() * 10.
            );
        }
    }
    Ok(())
}

fn production(node: &rclrs::Node) -> Result<()> {
    use mid360_sim::publisher::{LivoxPublisher, TypedLivoxPublisher};
    let dynamic = (0..2)
        .map(|i| LivoxPublisher::new(node, &format!("/production_dynamic_{i}")))
        .collect::<Result<Vec<_>>>()?;
    let typed = (0..2)
        .map(|i| TypedLivoxPublisher::new(node, &format!("/production_typed_{i}")))
        .collect::<Result<Vec<_>>>()?;
    for n in [6667, 20000] {
        let packet = mid360_sim::LivoxPacket {
            config: mid360_sim::LivoxConfig::default(),
            stamp: mid360_sim::RosTime { sec: 2, nanosec: 0 },
            timebase_ns: 2_000_000_000,
            points: points(n),
        };
        for sensors in [1, 2] {
            let mut samples = [Vec::new(), Vec::new()];
            for trial in 0..12 {
                for use_typed in if trial % 2 == 0 {
                    [false, true]
                } else {
                    [true, false]
                } {
                    for i in 0..sensors {
                        anyhow::ensure!(
                            dynamic[i].subscription_count()? == 0
                                && typed[i].subscription_count()? == 0,
                            "timing requires unmatched endpoints"
                        );
                    }
                    let start = Instant::now();
                    for _ in 0..10 {
                        for i in 0..sensors {
                            if use_typed {
                                typed[i].publish(black_box(&packet))?;
                            } else {
                                dynamic[i].publish(black_box(&packet))?;
                            }
                        }
                    }
                    let elapsed = start.elapsed().as_secs_f64() * 100.;
                    for i in 0..sensors {
                        anyhow::ensure!(
                            dynamic[i].subscription_count()? == 0
                                && typed[i].subscription_count()? == 0,
                            "subscriber appeared during timing"
                        );
                    }
                    if trial >= 2 {
                        samples[usize::from(use_typed)].push(elapsed);
                    }
                }
            }
            println!(
                "{{\"points\":{n},\"sensors\":{sensors},\"dynamic_ms\":{:.6},\"typed_ms\":{:.6},\"samples_ms\":{samples:?}}}",
                median(&samples[0]),
                median(&samples[1])
            );
        }
    }
    Ok(())
}
