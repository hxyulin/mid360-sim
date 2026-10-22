// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Shared direction preparation for aligned sensors; includes preparation and allocation cost.
//! Uses dynamic analytic planes, optional generic noise, no native physics/ROS/GPU.
use mid360_sim::{
    pattern::ScanPattern,
    sensor::{Effects, PreparedRays, RollingFrame, Scene, Sensor, SurfaceHit},
};
use std::{hint::black_box, time::Instant};
struct Plane(f64);
impl Scene for Plane {
    fn nearest_hit(&self, time: u64, d: [f64; 3], max: f64) -> Option<SurfaceHit> {
        let t = time as f64 * 1e-9;
        let (s, c) = (2. * t + self.0).sin_cos();
        let cosine = c * d[0] - s * d[1];
        let distance = (5. - (3. * t).sin() * 0.2) / cosine;
        (distance > 0. && distance <= max).then_some(SurfaceHit {
            distance_m: distance,
            incidence_cosine: cosine.abs().min(1.),
            reflectance: 0.8,
        })
    }
}
fn main() {
    for profile in [ScanPattern::Nominal, ScanPattern::HardwareCapture] {
        for noisy in [false, true] {
            for n in [1, 2, 8] {
                let sensors: Vec<_> = (0..n)
                    .map(|i| {
                        Sensor::new(
                            0.1,
                            40.,
                            100,
                            Effects {
                                seed: 7 + i as u64,
                                angular_sigma_rad: if noisy { 0.001 } else { 0. },
                                range_sigma_m: if noisy { 0.02 } else { 0. },
                                ..Default::default()
                            },
                        )
                        .unwrap()
                    })
                    .collect();
                let scenes: Vec<_> = (0..n).map(|i| Plane(i as f64 * 0.2)).collect();
                let mut frames = vec![RollingFrame::default(); n];
                let mut samples = [Vec::new(), Vec::new()];
                let p = PreparedRays::new(profile, 7, 20000, 1).unwrap();
                for i in 0..n {
                    let mut old = RollingFrame::default();
                    let mut new = RollingFrame::default();
                    sensors[i]
                        .sample_pattern_into(&scenes[i], profile, 7, 20000, 1, &mut old)
                        .unwrap();
                    sensors[i]
                        .sample_prepared_into(&scenes[i], &p, &mut new)
                        .unwrap();
                    assert_eq!(old.points.len(), new.points.len());
                    for (a, b) in old.points.iter().zip(&new.points) {
                        assert_eq!((a.sample_index, a.time_ns), (b.sample_index, b.time_ns));
                        assert_eq!(a.range_m.map(f64::to_bits), b.range_m.map(f64::to_bits));
                        assert_eq!(
                            a.point_sensor_m.map(|p| p.map(f64::to_bits)),
                            b.point_sensor_m.map(|p| p.map(f64::to_bits))
                        );
                    }
                }
                for trial in 0..12 {
                    for shared in if trial % 2 == 0 {
                        [false, true]
                    } else {
                        [true, false]
                    } {
                        let start = Instant::now();
                        for packet in 0..4 {
                            let first = 7 + packet * 20000;
                            if shared {
                                let prepared = PreparedRays::new(profile, first, 20000, 1).unwrap();
                                for i in 0..n {
                                    sensors[i]
                                        .sample_prepared_into(
                                            black_box(&scenes[i]),
                                            &prepared,
                                            &mut frames[i],
                                        )
                                        .unwrap();
                                }
                            } else {
                                for i in 0..n {
                                    sensors[i]
                                        .sample_pattern_into(
                                            black_box(&scenes[i]),
                                            profile,
                                            first,
                                            20000,
                                            1,
                                            &mut frames[i],
                                        )
                                        .unwrap();
                                }
                            }
                            black_box(&frames);
                        }
                        if trial >= 2 {
                            samples[usize::from(shared)]
                                .push(start.elapsed().as_secs_f64() * 1000.);
                        }
                    }
                }
                let ordered_samples = samples.clone();
                for s in &mut samples {
                    s.sort_by(f64::total_cmp);
                }
                println!(
                    "{{\"profile\":\"{profile:?}\",\"noisy\":{noisy},\"sensors\":{n},\"rays_per_sensor\":80000,\"fresh_ms\":{:.6},\"prepared_ms\":{:.6},\"fresh_samples_ms\":{:?},\"prepared_samples_ms\":{:?}}}",
                    (samples[0][4] + samples[0][5]) * 0.5,
                    (samples[1][4] + samples[1][5]) * 0.5,
                    ordered_samples[0],
                    ordered_samples[1]
                );
            }
        }
    }
}
