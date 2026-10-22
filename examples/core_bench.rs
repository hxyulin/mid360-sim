// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Release microbenchmarks for acquisition, encoding and packet scheduling.
//! No ROS, sleeping or GPU; timings are host wall time, never sensor stamps.
#[allow(dead_code)]
#[path = "synchronized/fixture.rs"]
mod fixture;

use mid360_sim::{
    LivoxConfig, LivoxPacket,
    pattern::ScanPattern,
    pointcloud,
    sensor::{Effects, RollingFrame, Scene, Sensor, SurfaceHit},
    transport::{FaultConfig, FaultInjector, PacketId},
};
use std::{hint::black_box, time::Instant};

struct Sphere;
impl Scene for Sphere {
    fn nearest_hit(&self, _: u64, _: [f64; 3], _: f64) -> Option<SurfaceHit> {
        Some(SurfaceHit {
            distance_m: 2.,
            incidence_cosine: 1.,
            reflectance: 0.8,
        })
    }
}
fn measure(name: &str, units: usize, mut work: impl FnMut()) {
    let mut samples = Vec::new();
    for rep in 0..18 {
        let start = Instant::now();
        work();
        if rep >= 3 {
            samples.push(start.elapsed().as_secs_f64() * 1e3);
        }
    }
    samples.sort_by(f64::total_cmp);
    println!(
        "{{\"case\":\"{name}\",\"units\":{units},\"median_ms\":{:.6},\"min_ms\":{:.6},\"max_ms\":{:.6},\"trials\":15}}",
        samples[7], samples[0], samples[14]
    );
}
fn acquisition(scene: &impl Scene, name: &str, effects: Effects, stride: u32) {
    let sensor = Sensor::new(0.1, 40., fixture::EPOCH_NS, effects).unwrap();
    let mut frame = RollingFrame::default();
    let count = 20_000u32.div_ceil(stride);
    measure(name, count as usize * 10, || {
        for packet in 0..10 {
            sensor
                .sample_into(black_box(scene), packet * 20_000, count, stride, &mut frame)
                .unwrap();
            black_box(&frame);
        }
    });
}
fn main() {
    let range = Effects {
        range_sigma_m: 0.02,
        ..Default::default()
    };
    let angular = Effects {
        angular_sigma_rad: 0.001,
        ..Default::default()
    };
    let all = Effects {
        range_sigma_m: 0.02,
        angular_sigma_rad: 0.001,
        dropout: 0.01,
        diffuse_dropout: true,
        ..Default::default()
    };
    for (name, effects) in [
        ("sphere/ideal", Effects::default()),
        ("sphere/range", range),
        ("sphere/angular", angular),
        ("sphere/all-effects", all),
    ] {
        acquisition(&Sphere, name, effects, 1);
    }
    let room = fixture::Room(fixture::Motion::Articulated);
    acquisition(&room, "articulated/ideal", Effects::default(), 1);
    acquisition(&room, "articulated/all-effects", all, 1);
    acquisition(&room, "articulated/all-effects/stride3", all, 3);

    for (name, profile) in [
        ("sphere/hardware-mean", ScanPattern::HardwareMean),
        ("sphere/hardware-capture", ScanPattern::HardwareCapture),
    ] {
        let sensor = Sensor::new(0.1, 40., fixture::EPOCH_NS, Effects::default()).unwrap();
        let mut frame = RollingFrame::default();
        measure(name, 200_000, || {
            for packet in 0..10 {
                sensor
                    .sample_pattern_into(&Sphere, profile, packet * 20_000, 20_000, 1, &mut frame)
                    .unwrap();
                black_box(&frame);
            }
        });
    }
    measure("imu/articulated-fixture", 200_000, || {
        for index in 0..200_000u64 {
            black_box(fixture::imu(
                black_box(fixture::EPOCH_NS + (index + 1) * fixture::IMU_PERIOD_NS),
                fixture::Motion::Articulated,
            ));
        }
    });

    let sensor = Sensor::new(0.1, 40., fixture::EPOCH_NS, Effects::default()).unwrap();
    let mut frame = RollingFrame::default();
    sensor
        .sample_into(&Sphere, 0, 20_000, 1, &mut frame)
        .unwrap();
    let config = LivoxConfig::default();
    measure("livox/allocate", 200_000, || {
        for _ in 0..10 {
            black_box(LivoxPacket::from_scan(black_box(&frame), &config).unwrap());
        }
    });
    let packet = LivoxPacket::from_scan(&frame, &config).unwrap();
    measure("pointcloud/allocate", 200_000, || {
        for _ in 0..10 {
            black_box(pointcloud::encode(black_box(&packet)).unwrap());
        }
    });
    let faults = FaultConfig {
        loss_probability: 0.01,
        delay_ns: 20_000_000,
        jitter_ns: 300_000_000,
        ..Default::default()
    };
    measure("faults/decision", 100_000, || {
        for sequence in 0..100_000 {
            black_box(
                faults
                    .release_ns(
                        black_box(PacketId {
                            stream: 0,
                            sequence,
                        }),
                        0,
                    )
                    .unwrap(),
            );
        }
    });
    for capacity in [8, 256, 4096] {
        // No acquisition or payload allocation: isolate queue bookkeeping.
        measure(&format!("faults/queue/{capacity}"), 100_000, || {
            let mut queue = FaultInjector::new(faults, capacity).unwrap();
            for sequence in 0..100_000u64 {
                if sequence % capacity as u64 == 0 {
                    while let Some(delivery) = queue.pop_due(u64::MAX) {
                        black_box(delivery);
                    }
                }
                black_box(
                    queue
                        .submit(
                            PacketId {
                                stream: 0,
                                sequence,
                            },
                            0,
                            sequence,
                        )
                        .unwrap(),
                );
            }
            while let Some(delivery) = queue.pop_due(u64::MAX) {
                black_box(delivery);
            }
        });
    }
}
