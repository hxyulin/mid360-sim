// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Explicit-time fault injection after real fixture acquisition; no ROS or sleeping.
// The shared fixture also contains scoring helpers unused by this adapter demo.
#[allow(dead_code)]
#[path = "synchronized/fixture.rs"]
mod fixture;
use mid360_sim::{
    imu::ImuSample,
    sensor::{Effects, RollingFrame, Sensor},
    transport::{FaultConfig, FaultInjector, PacketId},
};
use std::{collections::BTreeMap, error::Error};

struct Bundle {
    lidar: RollingFrame,
    imu: Vec<ImuSample>,
}
fn fingerprint(bundle: &Bundle) -> u64 {
    let mut hash = 0xcbf29ce484222325u64;
    let mut add = |value: u64| hash = (hash ^ value).wrapping_mul(0x100000001b3);
    for p in &bundle.lidar.points {
        add(p.sample_index);
        add(p.time_ns);
        add(p.range_m.map_or(u64::MAX, f64::to_bits));
        for v in p.point_sensor_m.unwrap_or([f64::INFINITY; 3]) {
            add(v.to_bits());
        }
    }
    for s in &bundle.imu {
        add(s.time_ns);
        for v in s
            .angular_velocity_rad_s
            .into_iter()
            .chain(s.specific_force_m_s2)
        {
            add(v.to_bits());
        }
    }
    hash
}
fn main() -> Result<(), Box<dyn Error>> {
    let mut config = FaultConfig {
        loss_probability: 0.1,
        delay_ns: 20_000_000,
        jitter_ns: 300_000_000,
        ..Default::default()
    };
    let (mut frames, mut stride, mut capacity) = (30u64, 3u32, 8usize);
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--help" {
            println!(
                "packet_faults [--frames 30] [--stride 3] [--seed 7] [--loss 0.1] [--delay-ns 20000000] [--jitter-ns 300000000] [--capacity 8]"
            );
            return Ok(());
        }
        let value = args.next().ok_or("missing option value")?;
        match arg.as_str() {
            "--frames" => frames = value.parse()?,
            "--stride" => stride = value.parse()?,
            "--seed" => config.seed = value.parse()?,
            "--loss" => config.loss_probability = value.parse()?,
            "--delay-ns" => config.delay_ns = value.parse()?,
            "--jitter-ns" => config.jitter_ns = value.parse()?,
            "--capacity" => capacity = value.parse()?,
            _ => return Err(format!("unknown option {arg}").into()),
        }
    }
    if !(1..=600).contains(&frames)
        || !(1..=1001).contains(&stride)
        || !(1..=4096).contains(&capacity)
    {
        return Err("frame/stride/capacity outside bounded example limits".into());
    }
    let mut queue = FaultInjector::new(config, capacity)?;
    let sensor = Sensor::new(
        0.1,
        40.,
        fixture::EPOCH_NS,
        Effects {
            seed: 17,
            range_sigma_m: 0.02,
            angular_sigma_rad: 0.001,
            dropout: 0.01,
            ..Default::default()
        },
    )?;
    let room = fixture::Room(fixture::Motion::Articulated);
    let (mut fingerprints, mut arrivals) = (BTreeMap::new(), Vec::new());
    let mut drain = |queue: &mut FaultInjector<Bundle>, now_ns, expected: &BTreeMap<u64, u64>| {
        while let Some(packet) = queue.pop_due(now_ns) {
            assert_eq!(fingerprint(&packet.payload), expected[&packet.id.sequence]);
            assert!(packet.release_ns >= packet.available_ns);
            arrivals.push(packet.id.sequence);
        }
    };
    for sequence in 0..frames {
        let start = sequence * fixture::PACKET_NS;
        let end = start + fixture::PACKET_NS;
        let available_ns = fixture::EPOCH_NS + end;
        drain(&mut queue, available_ns, &fingerprints);
        let first = (start / 5000).div_ceil(u64::from(stride)) * u64::from(stride);
        let count = (end / 5000 - first).div_ceil(u64::from(stride)) as u32;
        let mut lidar = RollingFrame::default();
        sensor.sample_into(&room, first, count, stride, &mut lidar)?;
        let imu = (1..=20)
            .map(|i| {
                fixture::imu(
                    fixture::EPOCH_NS + start + i * fixture::IMU_PERIOD_NS,
                    fixture::Motion::Articulated,
                )
            })
            .collect();
        let payload = Bundle { lidar, imu };
        fingerprints.insert(sequence, fingerprint(&payload));
        queue.submit(
            PacketId {
                stream: 0,
                sequence,
            },
            available_ns,
            payload,
        )?;
        drain(&mut queue, available_ns, &fingerprints);
    }
    // Offline final drain, not an acquisition timestamp or real-time pacing claim.
    drain(&mut queue, u64::MAX, &fingerprints);
    let statistics = queue.statistics();
    let backward_steps = arrivals.windows(2).filter(|p| p[0] > p[1]).count();
    println!(
        "{{\"submitted\":{frames},\"delivered\":{},\"injected_losses\":{},\"capacity_drops\":{},\"backward_sequence_steps\":{backward_steps},\"payload_hashes_unchanged\":true,\"arrival_sequences\":{arrivals:?}}}",
        statistics.delivered, statistics.injected_losses, statistics.capacity_drops
    );
    Ok(())
}
