// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Direction-only release benchmark; excludes geometry, transport and effects.
use mid360_sim::hardware::NominalPattern;
use std::{hint::black_box, time::Instant};
fn main() {
    let p = NominalPattern;
    let mut out = vec![[0.; 3]; 20_000];
    for batch in [false, true] {
        let mut samples = Vec::new();
        for rep in 0..35 {
            let start = Instant::now();
            for frame in 0..10 {
                let first = (rep * 10 + frame) * 20_000;
                if batch {
                    p.fill_directions(black_box(first), 1, black_box(&mut out))
                        .unwrap();
                } else {
                    for (i, r) in out.iter_mut().enumerate() {
                        *r = p.direction(black_box(first + i as u64));
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
            "{}: {:.3} ms / 200k rays, median of 30 (5 warmups)",
            if batch { "grouped" } else { "scalar" },
            samples[15]
        );
    }
}
