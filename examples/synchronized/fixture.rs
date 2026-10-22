// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Analytic kinematic fixture only: not a dynamics world or a device IMU model.
use mid360_sim::{
    imu::ImuSample,
    sensor::{Scene, SurfaceHit},
};
use std::ops::{Add, Mul, Neg};
pub const EPOCH_NS: u64 = 1_000_000_000;
pub const IMU_PERIOD_NS: u64 = 5_000_000;
pub const PACKET_NS: u64 = 100_000_000;
pub const GRAVITY: f64 = 9.80665;

/// Value and exact first/second derivatives with respect to seconds.
#[derive(Clone, Copy, Default)]
struct Jet {
    v: f64,
    d: f64,
    dd: f64,
}
impl From<f64> for Jet {
    fn from(v: f64) -> Self {
        Self {
            v,
            ..Default::default()
        }
    }
}
impl From<i32> for Jet {
    fn from(v: i32) -> Self {
        Self::from(f64::from(v))
    }
}
impl Jet {
    fn sin(self) -> Self {
        let (s, c) = self.v.sin_cos();
        Self {
            v: s,
            d: c * self.d,
            dd: c * self.dd - s * self.d * self.d,
        }
    }
    fn cos(self) -> Self {
        let (s, c) = self.v.sin_cos();
        Self {
            v: c,
            d: -s * self.d,
            dd: -s * self.dd - c * self.d * self.d,
        }
    }
}
impl Add for Jet {
    type Output = Self;
    fn add(self, b: Self) -> Self {
        Self {
            v: self.v + b.v,
            d: self.d + b.d,
            dd: self.dd + b.dd,
        }
    }
}
impl Neg for Jet {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            v: -self.v,
            d: -self.d,
            dd: -self.dd,
        }
    }
}
impl Mul for Jet {
    type Output = Self;
    fn mul(self, b: Self) -> Self {
        Self {
            v: self.v * b.v,
            d: self.d * b.v + self.v * b.d,
            dd: self.dd * b.v + 2. * self.d * b.d + self.v * b.dd,
        }
    }
}
type Matrix = [[Jet; 3]; 3];
fn product(a: Matrix, b: Matrix) -> Matrix {
    std::array::from_fn(|i| {
        std::array::from_fn(|j| (0..3).fold(Jet::default(), |sum, k| sum + a[i][k] * b[k][j]))
    })
}
fn apply(a: Matrix, p: [Jet; 3]) -> [Jet; 3] {
    std::array::from_fn(|i| (0..3).fold(Jet::default(), |sum, j| sum + a[i][j] * p[j]))
}
fn rz(a: Jet) -> Matrix {
    let (s, c) = (a.sin(), a.cos());
    let (z, o) = (0.into(), 1.into());
    [[c, -s, z], [s, c, z], [z, z, o]]
}
fn ry(a: Jet) -> Matrix {
    let (s, c) = (a.sin(), a.cos());
    let (z, o) = (0.into(), 1.into());
    [[c, z, s], [z, o, z], [-s, z, c]]
}
#[derive(Clone, Copy, Debug)]
pub enum Motion {
    Stationary,
    Yaw,
    Articulated,
}
impl Motion {
    pub fn name(self) -> &'static str {
        match self {
            Self::Stationary => "stationary",
            Self::Yaw => "yaw",
            Self::Articulated => "articulated",
        }
    }
    pub fn parse(s: &str) -> Result<Self, &'static str> {
        match s {
            "stationary" => Ok(Self::Stationary),
            "yaw" => Ok(Self::Yaw),
            "articulated" => Ok(Self::Articulated),
            _ => Err("motion must be stationary, yaw or articulated"),
        }
    }
}
#[derive(Clone, Copy)]
struct Pose {
    p: [Jet; 3],
    r: Matrix,
}
fn lidar_pose(t: f64, motion: Motion) -> Pose {
    let t = Jet {
        v: t,
        d: 1.,
        dd: 0.,
    };
    let (base, head, p) = match motion {
        Motion::Stationary => (rz(0.into()), ry(0.into()), [0.into(), 0.into(), 1.2.into()]),
        Motion::Yaw => (
            rz(t * 0.8.into()),
            ry(0.into()),
            [0.into(), 0.into(), 1.2.into()],
        ),
        Motion::Articulated => (
            product(
                rz(t * 0.6.into() + (t * 1.3.into()).sin() * 0.2.into()),
                ry((t * 0.7.into()).sin() * 0.15.into()),
            ),
            product(
                rz((t * 0.8.into()).sin() * (-0.2).into()),
                ry((t * 0.5.into()).cos() * 0.12.into()),
            ),
            [
                (t * 0.6.into()).sin() * 0.5.into(),
                (t * 0.4.into()).cos() * 0.4.into(),
                Jet::from(1.2) + (t * 0.3.into()).sin() * 0.1.into(),
            ],
        ),
    };
    let offset = apply(base, [0.25.into(), 0.into(), 0.2.into()]);
    Pose {
        p: std::array::from_fn(|i| p[i] + offset[i]),
        r: product(base, head),
    }
}
fn instantaneous_imu(t: f64, motion: Motion) -> ([f64; 3], [f64; 3]) {
    let lidar = lidar_pose(t, motion);
    // Explicit example extrinsic: 4/-2/1.5 cm in LiDAR FLU, pitch +0.1 rad.
    let offset = apply(lidar.r, [0.04.into(), (-0.02).into(), 0.015.into()]);
    let p: [Jet; 3] = std::array::from_fn(|i| lidar.p[i] + offset[i]);
    let r = product(lidar.r, ry(0.1.into()));
    let w: [[f64; 3]; 3] = std::array::from_fn(|i| {
        std::array::from_fn(|j| (0..3).map(|k| r[k][i].v * r[k][j].d).sum())
    });
    let omega = [
        (w[2][1] - w[1][2]) * 0.5,
        (w[0][2] - w[2][0]) * 0.5,
        (w[1][0] - w[0][1]) * 0.5,
    ];
    let force = std::array::from_fn(|i| {
        (0..3)
            .map(|k| r[k][i].v * (p[k].dd + if k == 2 { GRAVITY } else { 0. }))
            .sum()
    });
    (omega, force)
}
pub fn imu(time_ns: u64, motion: Motion) -> ImuSample {
    let mut omega = [0.; 3];
    let mut force = [0.; 3];
    // Caller-synthesized preceding-period means, five midpoint samples per 5 ms.
    for i in 0..5 {
        let t = (time_ns - EPOCH_NS - IMU_PERIOD_NS) as f64 * 1e-9 + (i as f64 + 0.5) * 0.001;
        let (w, f) = instantaneous_imu(t, motion);
        for axis in 0..3 {
            omega[axis] += w[axis] / 5.;
            force[axis] += f[axis] / 5.;
        }
    }
    ImuSample {
        time_ns,
        angular_velocity_rad_s: omega,
        specific_force_m_s2: force,
    }
}
pub fn world_point(time_ns: u64, point: [f64; 3], motion: Motion) -> [f64; 3] {
    let pose = lidar_pose((time_ns - EPOCH_NS) as f64 * 1e-9, motion);
    let q = apply(pose.r, point.map(Jet::from));
    std::array::from_fn(|i| pose.p[i].v + q[i].v)
}
pub fn wall_error(p: [f64; 3]) -> f64 {
    [p[0].abs() - 10., p[1].abs() - 10., p[2], p[2] - 4.]
        .iter()
        .map(|v| v.abs())
        .fold(f64::INFINITY, f64::min)
}
pub struct Room(pub Motion);
impl Scene for Room {
    fn nearest_hit(
        &self,
        time_ns: u64,
        direction: [f64; 3],
        max_range_m: f64,
    ) -> Option<SurfaceHit> {
        let pose = lidar_pose((time_ns - EPOCH_NS) as f64 * 1e-9, self.0);
        let d = apply(pose.r, direction.map(Jet::from));
        let mut nearest = max_range_m;
        let mut incidence = 0.;
        for (axis, component) in d.iter().enumerate() {
            if component.v.abs() < 1e-12 {
                continue;
            }
            for plane in if axis == 2 { [0., 4.] } else { [-10., 10.] } {
                let range = (plane - pose.p[axis].v) / d[axis].v;
                if range >= 0. && range < nearest {
                    nearest = range;
                    incidence = d[axis].v.abs();
                }
            }
        }
        (incidence > 0.).then_some(SurfaceHit {
            distance_m: nearest,
            incidence_cosine: incidence,
            reflectance: 0.8,
        })
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn stationary_and_constant_spin_have_independent_analytic_references() {
        for n in 1..=200 {
            let quiet = imu(EPOCH_NS + n * IMU_PERIOD_NS, Motion::Stationary);
            assert!(quiet.angular_velocity_rad_s.iter().all(|v| v.abs() < 1e-14));
            let norm = quiet
                .specific_force_m_s2
                .iter()
                .map(|v| v * v)
                .sum::<f64>()
                .sqrt();
            assert!((norm - GRAVITY).abs() < 1e-12);
            let spinning = imu(EPOCH_NS + n * IMU_PERIOD_NS, Motion::Yaw);
            let (c, s) = (0.1f64.cos(), 0.1f64.sin());
            let expected_w = [-0.8 * s, 0., 0.8 * c];
            let expected_f = [
                -0.8 * 0.8 * 0.29 * c - GRAVITY * s,
                0.8 * 0.8 * 0.02,
                -0.8 * 0.8 * 0.29 * s + GRAVITY * c,
            ];
            for axis in 0..3 {
                assert!((spinning.angular_velocity_rad_s[axis] - expected_w[axis]).abs() < 1e-12);
                assert!((spinning.specific_force_m_s2[axis] - expected_f[axis]).abs() < 1e-11);
            }
        }
    }
}
