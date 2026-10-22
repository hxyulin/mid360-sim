// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Caller-owned physical transforms. No world pose, interpolation or clock is inferred.

/// Rigid child-to-parent transform: `p_parent = R * p_child + translation_m`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    /// Child origin in parent FLU coordinates, metres.
    pub translation_m: [f64; 3],
    /// Unit quaternion in ROS xyzw order, dimensionless.
    pub rotation_xyzw: [f64; 4],
}
impl Transform {
    /// Identity transform with zero translation in metres.
    pub const IDENTITY: Self = Self {
        translation_m: [0.; 3],
        rotation_xyzw: [0., 0., 0., 1.],
    };
    /// Reject nonfinite translations and nonunit quaternions (tolerance 1e-9).
    pub fn validate(&self) -> Result<(), &'static str> {
        if !self
            .translation_m
            .iter()
            .chain(self.rotation_xyzw.iter())
            .all(|v| v.is_finite())
            || (self.rotation_xyzw.iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-9
        {
            return Err("transform needs finite metres and a unit xyzw quaternion");
        }
        Ok(())
    }
    /// Rotate a vector, retaining its input units. Validate the transform first.
    pub fn rotate(&self, v: [f64; 3]) -> [f64; 3] {
        let [x, y, z, w] = self.rotation_xyzw;
        let q = [x, y, z];
        let cross = |a: [f64; 3], b: [f64; 3]| {
            [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ]
        };
        let t = cross(q, v).map(|x| 2. * x);
        let c = cross(q, t);
        std::array::from_fn(|i| v[i] + w * t[i] + c[i])
    }
    /// Transform a child point in metres to the parent frame in metres.
    pub fn point(&self, v: [f64; 3]) -> [f64; 3] {
        let p = self.rotate(v);
        std::array::from_fn(|i| p[i] + self.translation_m[i])
    }
    /// Return the parent-to-child transform, with translation in metres.
    pub fn inverse(&self) -> Self {
        let [x, y, z, w] = self.rotation_xyzw;
        let mut out = Self {
            translation_m: [0.; 3],
            rotation_xyzw: [-x, -y, -z, w],
        };
        out.translation_m = out.rotate(self.translation_m.map(|v| -v));
        out
    }
}
/// A caller-stamped physical frame edge, suitable for static or dynamic TF.
#[derive(Clone, Debug, PartialEq)]
pub struct FrameTransform {
    /// Physical parent frame; no leading slash, whitespace or NUL.
    pub parent: String,
    /// Distinct physical child frame; same naming rules as parent.
    pub child: String,
    /// Simulation acquisition time, nanoseconds. Static edges conventionally use zero.
    pub time_ns: u64,
    /// Child-to-parent transform in metres and unit xyzw quaternion.
    pub transform: Transform,
}
impl FrameTransform {
    /// Validate frame names, rigid transform and ROS timestamp range.
    pub fn validate(&self) -> Result<(), &'static str> {
        let valid = |s: &str| {
            !s.is_empty()
                && !s.starts_with('/')
                && !s.chars().any(|c| c.is_whitespace() || c == '\0')
        };
        if !valid(&self.parent) || !valid(&self.child) || self.parent == self.child {
            return Err("invalid physical TF frame edge");
        }
        crate::RosTime::from_ns(self.time_ns)?;
        self.transform.validate()
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rotated_lever_arm_and_inverse() {
        let t = Transform {
            translation_m: [1., 2., 3.],
            rotation_xyzw: [
                0.,
                0.,
                std::f64::consts::FRAC_1_SQRT_2,
                std::f64::consts::FRAC_1_SQRT_2,
            ],
        };
        t.validate().unwrap();
        let p = t.point([2., 0., 0.]);
        assert!((p[0] - 1.).abs() < 1e-12 && (p[1] - 4.).abs() < 1e-12);
        for (a, b) in t.inverse().point(p).iter().zip([2., 0., 0.]) {
            assert!((a - b).abs() < 1e-12);
        }
        let mut bad = t;
        bad.rotation_xyzw = [0.; 4];
        assert!(bad.validate().is_err());
    }
}
