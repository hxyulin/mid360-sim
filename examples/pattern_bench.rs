// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Direction-only release benchmark; excludes geometry, transport and effects.
use mid360_sim::hardware::NominalPattern;
use std::{hint::black_box, time::Instant};
fn main() {
    let p = NominalPattern;
    let mut out = vec![[0.; 3]; 20_000];
    for stride in [1, 2, 3, 4, 7, 31] {
        for batch in [false, true] {
            let mut samples = Vec::new();
            for rep in 0..35 {
                let start = Instant::now();
                for frame in 0..10 {
                    let first = (rep * 10 + frame) * 20_000 * u64::from(stride);
                    if batch {
                        p.fill_directions(black_box(first), stride, black_box(&mut out))
                            .unwrap();
                    } else {
                        for (i, r) in out.iter_mut().enumerate() {
                            *r = p.direction(black_box(first + i as u64 * u64::from(stride)));
                        }
                    }
                    black_box(&out);
                }
                if rep >= 5 {
                    samples.push(start.elapsed().as_secs_f64() * 1000.);
                }
            }
            samples.sort_by(f64::total_cmp);
            println!(
                "stride {stride}, {}: {:.3} ms / 200k retained rays, median of 30 (5 warmups)",
                if batch { "adaptive" } else { "scalar" },
                samples[15]
            );
        }
    }
}
