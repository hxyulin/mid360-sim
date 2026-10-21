// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Standalone moving-sensor room demo; no rm-simulator dependency.
use anyhow::{Result, ensure};
use clap::{Parser, ValueEnum};
use mid360_sim::{
    LivoxConfig, LivoxPacket,
    pattern::ScanPattern,
    publisher::LivoxPublisher,
    sensor::{Effects, RollingFrame, Scene, Sensor, SurfaceHit},
};
use rclrs::CreateBasicExecutor;
use std::time::{Duration, Instant};
#[derive(Clone, Copy, ValueEnum, PartialEq, Eq)]
enum Profile {
    Nominal,
    OfficialTable,
    HardwareMean,
    HardwareCapture,
}
impl From<Profile> for ScanPattern {
    fn from(profile: Profile) -> Self {
        match profile {
            Profile::Nominal => Self::Nominal,
            Profile::OfficialTable => Self::OfficialTable,
            Profile::HardwareMean => Self::HardwareMean,
            Profile::HardwareCapture => Self::HardwareCapture,
        }
    }
}
#[derive(Parser)]
struct Args {
    #[arg(long, value_enum, default_value_t = Profile::Nominal)]
    pattern: Profile,
    #[arg(long, default_value_t = 0.)]
    mount_roll_deg: f64,
    #[arg(long, default_value_t = 0.)]
    mount_pitch_deg: f64,
    #[arg(long, default_value = "/livox/lidar")]
    topic: String,
    #[arg(long, default_value = "livox_frame")]
    frame_id: String,
    #[arg(long, default_value_t = 0)]
    lidar_id: u8,
    #[arg(long,default_value_t=20,value_parser=clap::value_parser!(u32).range(1..=36000))]
    frames: u32,
    #[arg(long,default_value_t=10,value_parser=clap::value_parser!(u32).range(1..=100))]
    rate_hz: u32,
    #[arg(long,default_value_t=1,value_parser=clap::value_parser!(u32).range(1..=1001))]
    stride: u32,
    #[arg(long, default_value_t = 0.02)]
    range_noise_m: f64,
    #[arg(long, default_value_t = 0.)]
    angular_noise_deg: f64,
    #[arg(long, default_value_t = 0.01)]
    dropout: f64,
    #[arg(long, default_value_t = 7)]
    seed: u64,
    #[arg(long, default_value_t = 2000)]
    wait_for_subscriber_ms: u64,
    #[arg(long)]
    require_subscriber: bool,
    #[arg(long)]
    fast: bool,
}
struct Room {
    roll_rad: f64,
    pitch_rad: f64,
}
impl Scene for Room {
    fn nearest_hit(&self, time_ns: u64, d: [f64; 3], max: f64) -> Option<SurfaceHit> {
        let t = (time_ns - 1_000_000_000) as f64 / 1e9;
        let (sr, cr) = self.roll_rad.sin_cos();
        let rolled = [d[0], cr * d[1] - sr * d[2], sr * d[1] + cr * d[2]];
        let (sp, cp) = self.pitch_rad.sin_cos();
        let pitched = [
            cp * rolled[0] + sp * rolled[2],
            rolled[1],
            -sp * rolled[0] + cp * rolled[2],
        ];
        let (s, c) = (0.2 * t).sin_cos();
        let world = [
            c * pitched[0] - s * pitched[1],
            s * pitched[0] + c * pitched[1],
            pitched[2],
        ];
        let origin = [t.sin(), 0., 0.8];
        let mut best = max;
        let mut cosine = 0.;
        let mut found = false;
        for axis in 0..3 {
            for bound in if axis == 2 { [0., 4.] } else { [-10., 10.] } {
                let r = (bound - origin[axis]) / world[axis];
                if r.is_finite() && r >= 0. && r < best {
                    best = r;
                    cosine = world[axis].abs();
                    found = true;
                }
            }
        }
        found.then_some(SurfaceHit {
            distance_m: best,
            incidence_cosine: cosine,
            reflectance: 0.8,
        })
    }
}
fn main() -> Result<()> {
    let a = Args::parse_from(rclrs::extract_non_ros_args(std::env::args())?);
    ensure!(
        1_000_000_000u64.is_multiple_of(u64::from(a.rate_hz)),
        "rate must divide one billion nanoseconds"
    );
    let period_ns = 1_000_000_000 / u64::from(a.rate_hz);
    ensure!(
        a.pattern != Profile::HardwareCapture
            || u64::from(a.frames) * period_ns
                <= (mid360_sim::hardware::LAST_CAPTURE_SAMPLE + 1) * 5_000,
        "hardware-capture is bounded to 299.996640 s; use fewer frames or hardware-mean"
    );
    ensure!(
        a.mount_roll_deg.is_finite() && a.mount_pitch_deg.is_finite(),
        "mount angles must be finite"
    );
    let room = Room {
        roll_rad: a.mount_roll_deg.to_radians(),
        pitch_rad: a.mount_pitch_deg.to_radians(),
    };
    let context = rclrs::Context::default_from_env()?;
    let executor = context.create_basic_executor();
    let node = executor.create_node("mid360_sim")?;
    let publisher = LivoxPublisher::new(&node, &a.topic)?;
    let wait = Instant::now();
    while publisher.subscription_count()? == 0
        && wait.elapsed() < Duration::from_millis(a.wait_for_subscriber_ms)
    {
        std::thread::sleep(Duration::from_millis(10));
    }
    ensure!(
        !a.require_subscriber || publisher.subscription_count()? > 0,
        "DDS discovery timed out"
    );
    let sensor = Sensor::new(
        0.1,
        70.,
        1_000_000_000,
        Effects {
            seed: a.seed,
            range_sigma_m: a.range_noise_m,
            angular_sigma_rad: a.angular_noise_deg.to_radians(),
            dropout: a.dropout,
            ..Default::default()
        },
    )
    .map_err(anyhow::Error::msg)?;
    let config = LivoxConfig {
        frame_id: a.frame_id,
        lidar_id: a.lidar_id,
        ..Default::default()
    };
    let period = 1_000_000_000 / u64::from(a.rate_hz);
    let mut scan = RollingFrame::default();
    let start = Instant::now();
    for frame in 0..u64::from(a.frames) {
        let lo = (frame * period).div_ceil(5000);
        let hi = ((frame + 1) * period).div_ceil(5000);
        let first = lo.div_ceil(u64::from(a.stride)) * u64::from(a.stride);
        let count = (hi - first).div_ceil(u64::from(a.stride));
        sensor
            .sample_pattern_into(
                &room,
                a.pattern.into(),
                first,
                u32::try_from(count)?,
                a.stride,
                &mut scan,
            )
            .map_err(anyhow::Error::msg)?;
        let packet = LivoxPacket::from_scan(&scan, &config).map_err(anyhow::Error::msg)?;
        publisher.publish(&packet)?;
        publisher.publish_clock(1_000_000_000 + (frame + 1) * period)?;
        if !a.fast
            && let Some(wait) =
                Duration::from_nanos((frame + 1) * period).checked_sub(start.elapsed())
        {
            std::thread::sleep(wait);
        }
    }
    std::thread::sleep(Duration::from_millis(100));
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn mounting_tilt_rotates_scene_queries() {
        let upright = Room {
            roll_rad: 0.,
            pitch_rad: 0.,
        };
        let roll = Room {
            roll_rad: std::f64::consts::FRAC_PI_2,
            pitch_rad: 0.,
        };
        let pitch = Room {
            roll_rad: 0.,
            pitch_rad: std::f64::consts::FRAC_PI_2,
        };
        assert!(
            (upright
                .nearest_hit(1_000_000_000, [0., 1., 0.], 70.)
                .unwrap()
                .distance_m
                - 10.)
                .abs()
                < 1e-12
        );
        assert!(
            (roll
                .nearest_hit(1_000_000_000, [0., 1., 0.], 70.)
                .unwrap()
                .distance_m
                - 3.2)
                .abs()
                < 1e-12
        );
        assert!(
            (pitch
                .nearest_hit(1_000_000_000, [1., 0., 0.], 70.)
                .unwrap()
                .distance_m
                - 0.8)
                .abs()
                < 1e-12
        );
    }
}
