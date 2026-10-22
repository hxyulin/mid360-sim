// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Acquisition crossover benchmark: all profiles, small packets and sparse strides.
//! Geometry is one ideal surface; no ROS, physics stepping or hardware claims.
use mid360_sim::{
    hardware::LAST_CAPTURE_SAMPLE,
    pattern::ScanPattern,
    sensor::{RollingFrame, Scene, Sensor, SurfaceHit},
};
use std::{hint::black_box, time::Instant};
struct Surface;
impl Scene for Surface {
    fn nearest_hit(&self, _: u64, _: [f64; 3], _: f64) -> Option<SurfaceHit> {
        Some(SurfaceHit {
            distance_m: 2.,
            incidence_cosine: 1.,
            reflectance: 1.,
        })
    }
}
fn main() {
    // Optional exact case filter permits rapid alternating process comparisons.
    let filter: Vec<String> = std::env::args().skip(1).collect();
    assert!(
        filter.is_empty() || filter.len() == 3,
        "expected [profile count stride]"
    );
    let sensor = Sensor::new(0.1, 40., 0, Default::default()).unwrap();
    let mut frame = RollingFrame::default();
    for profile in [
        ScanPattern::Nominal,
        ScanPattern::OfficialTable,
        ScanPattern::HardwareMean,
        ScanPattern::HardwareCapture,
    ] {
        for count in [1u32, 200, 20_000] {
            for stride in [1u32, 2, 3, 7] {
                if !filter.is_empty()
                    && (filter[0] != format!("{profile:?}")
                        || filter[1] != count.to_string()
                        || filter[2] != stride.to_string())
                {
                    continue;
                }
                let calls = 20_000 / count;
                let first = if profile == ScanPattern::HardwareCapture {
                    LAST_CAPTURE_SAMPLE - 1_000_000
                } else {
                    7
                };
                let mut samples = Vec::new();
                for trial in 0..12 {
                    let start = Instant::now();
                    for call in 0..calls {
                        let first = first + u64::from(call * count) * u64::from(stride);
                        sensor
                            .sample_pattern_into(
                                black_box(&Surface),
                                profile,
                                first,
                                count,
                                stride,
                                &mut frame,
                            )
                            .unwrap();
                        black_box(&frame);
                    }
                    if trial >= 2 {
                        samples.push(start.elapsed().as_secs_f64() * 1e3);
                    }
                }
                samples.sort_by(f64::total_cmp);
                println!(
                    "{{\"profile\":\"{profile:?}\",\"count\":{count},\"stride\":{stride},\"first\":{first},\"retained\":20000,\"calls\":{calls},\"median_ms\":{:.6},\"min_ms\":{:.6},\"max_ms\":{:.6}}}",
                    (samples[4] + samples[5]) * 0.5,
                    samples[0],
                    samples[9],
                );
            }
        }
    }
}
