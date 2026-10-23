// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Fixed-rate matched endpoint probe; profiling clocks never enter sensor stamps.
use anyhow::{Result, ensure};
use mid360_sim::{
    LivoxConfig, LivoxPacket, LivoxPoint, RosTime,
    publisher::{LivoxPublisher, TypedLivoxPublisher},
};
use std::{
    io::{self, Write},
    time::{Duration, Instant},
};

fn packet(n: usize, sensor: usize, sequence: usize) -> LivoxPacket {
    let stride = if n == 6667 { 3 } else { 1 };
    let timebase_ns = 2_000_000_000 + sequence as u64 * 100_000_000;
    LivoxPacket {
        config: LivoxConfig {
            frame_id: format!("matched_{sensor}"),
            lidar_id: sensor as u8,
            ..Default::default()
        },
        stamp: RosTime::from_ns(timebase_ns).unwrap(),
        timebase_ns,
        points: (0..n)
            .map(|i| LivoxPoint {
                offset_time_ns: (i * stride * 5000) as u32,
                x_m: sequence as f32 + 0.25,
                y_m: i as f32 * 0.125,
                z_m: -(sensor as f32) - 1.,
                reflectivity: (sequence % 256) as u8,
                tag: 0x10,
                line: ((i * stride) % 4) as u8,
            })
            .collect(),
    }
}

pub(super) fn run(node: &rclrs::Node) -> Result<()> {
    let args: Vec<_> = std::env::args().skip(2).collect();
    ensure!(
        args.len() == 8,
        "--matched dynamic|typed points sensors packets topic_prefix best_effort|reliable depth spacing_ms"
    );
    ensure!(
        matches!(args[0].as_str(), "dynamic" | "typed"),
        "invalid endpoint"
    );
    let use_typed = args[0] == "typed";
    let n: usize = args[1].parse()?;
    let sensors: usize = args[2].parse()?;
    let count: usize = args[3].parse()?;
    ensure!(
        matches!(args[5].as_str(), "best_effort" | "reliable"),
        "invalid QoS"
    );
    let depth: u32 = args[6].parse()?;
    let spacing_ms: u64 = args[7].parse()?;
    ensure!(depth > 0 && spacing_ms < 100, "invalid delivery controls");
    let qos = rclrs::QoSProfile::default().keep_last(depth);
    let qos = if args[5] == "reliable" {
        qos.reliable()
    } else {
        qos.best_effort()
    };
    ensure!(
        [6667, 20000].contains(&n) && [1, 2].contains(&sensors) && count > 0,
        "invalid workload"
    );
    let mut dynamic = Vec::new();
    let mut typed = Vec::new();
    for sensor in 0..sensors {
        let topic = format!("{}_{sensor}", args[4]);
        if use_typed {
            typed.push(TypedLivoxPublisher::new_with_qos(node, &topic, qos)?);
        } else {
            dynamic.push(LivoxPublisher::new_with_qos(node, &topic, qos)?);
        }
    }
    let deadline = Instant::now() + Duration::from_secs(15);
    for sensor in 0..sensors {
        while if use_typed {
            typed[sensor].subscription_count()?
        } else {
            dynamic[sensor].subscription_count()?
        } != 1
        {
            ensure!(
                Instant::now() < deadline,
                "expected exactly one matched subscriber"
            );
            std::thread::sleep(Duration::from_millis(10));
        }
    }
    let publish = |sensor: usize, packet: &LivoxPacket| -> Result<()> {
        if use_typed {
            typed[sensor].publish(packet)
        } else {
            dynamic[sensor].publish(packet)
        }
    };
    // Allow discovery to settle, then space startup checks only. One final
    // acknowledgement gates the fixed-rate run; measured losses are never retried.
    std::thread::sleep(Duration::from_millis(300));
    for sensor in 0..sensors {
        publish(sensor, &packet(n, sensor, 0))?;
        std::thread::sleep(Duration::from_millis(100));
    }
    println!("{{\"event\":\"ready\"}}");
    io::stdout().flush()?;
    let mut line = String::new();
    io::stdin().read_line(&mut line)?;
    ensure!(line.trim() == "go", "missing startup acknowledgement");
    let clock = rclrs::Clock::steady();
    let start = Instant::now();
    let mut samples = Vec::new();
    for sequence in 1..=count {
        let due = start + Duration::from_millis((sequence as u64 - 1) * 100);
        std::thread::sleep(due.saturating_duration_since(Instant::now()));
        for sensor in 0..sensors {
            let sensor_due = due + Duration::from_millis(sensor as u64 * spacing_ms);
            std::thread::sleep(sensor_due.saturating_duration_since(Instant::now()));
            // Caller packet construction is deliberately outside publish timing.
            let packet = packet(n, sensor, sequence);
            let before = clock.now().nsec;
            publish(sensor, &packet)?;
            let after = clock.now().nsec;
            samples.push(format!(
                "[{sequence},{sensor},{before},{after},{}]",
                sensor_due.elapsed().as_nanos()
            ));
        }
    }
    println!(
        "{{\"event\":\"done\",\"elapsed_ns\":{},\"samples\":[{}]}}",
        start.elapsed().as_nanos(),
        samples.join(",")
    );
    io::stdout().flush()?;
    line.clear();
    io::stdin().read_line(&mut line)?;
    Ok(())
}
