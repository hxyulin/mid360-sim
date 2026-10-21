// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Export the exact nominal table, clocks and cross-language test rays as JSON.
mod data {
    include!("../src/nominal_data.rs");
}
use mid360_sim::hardware::{
    NOMINAL_CYCLE_GROUPS, NOMINAL_CYCLE_S, NOMINAL_SPIN_HZ, NOMINAL_SWEEP_TURNS_PER_CYCLE,
    NominalPattern,
};
fn main() {
    let terms: Vec<_> = data::NOMINAL_TERMS.iter().map(|&(a, b)| [a, b]).collect();
    let fixtures: Vec<_> = [
        0,
        1,
        2,
        3,
        200_000,
        2_400_003,
        NOMINAL_CYCLE_GROUPS * 4 - 1,
        NOMINAL_CYCLE_GROUPS * 4,
        1_000_000_000_000,
    ]
    .into_iter()
    .map(|i| {
        format!(
            "{{\"index\":{i},\"direction\":{:?}}}",
            NominalPattern.direction(i)
        )
    })
    .collect();
    println!(
        "{{\"schema\":1,\"cycle_s\":{NOMINAL_CYCLE_S},\"groups_per_cycle\":{NOMINAL_CYCLE_GROUPS},\"spin_hz\":{NOMINAL_SPIN_HZ},\"sweep_numerator\":{NOMINAL_SWEEP_TURNS_PER_CYCLE},\"sweep_denominator\":{NOMINAL_CYCLE_S},\"point_interval_ns\":5000,\"coefficient_unit_degrees\":0.25,\"terms\":{terms:?},\"coefficients\":{:?},\"reference_rays\":[{}]}}",
        data::QUARTER_DEGREE_COEFFICIENTS,
        fixtures.join(",")
    );
}
