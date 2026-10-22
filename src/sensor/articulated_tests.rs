// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Moving sensor/links: inverse-box adapter against independent forward triangles.
use crate::{
    frames::Transform,
    hardware::NominalPattern,
    sensor::{Effects, RollingFrame, Scene, Sensor, SurfaceHit},
};
type V = [f64; 3];
type M = [[f64; 3]; 3];
const EPOCH: u64 = 4_000_000_000;
fn add(a: V, b: V) -> V {
    std::array::from_fn(|i| a[i] + b[i])
}
fn sub(a: V, b: V) -> V {
    std::array::from_fn(|i| a[i] - b[i])
}
fn scale(a: V, b: f64) -> V {
    a.map(|v| v * b)
}
fn dot(a: V, b: V) -> f64 {
    a.into_iter().zip(b).map(|(a, b)| a * b).sum()
}
fn cross(a: V, b: V) -> V {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn rotation(axis: usize, angle: f64, p: V) -> Transform {
    let mut q = [0., 0., 0., (angle * 0.5).cos()];
    q[axis] = (angle * 0.5).sin();
    Transform {
        translation_m: p,
        rotation_xyzw: q,
    }
}
fn compose(a: Transform, b: Transform) -> Transform {
    let [ax, ay, az, aw] = a.rotation_xyzw;
    let [bx, by, bz, bw] = b.rotation_xyzw;
    Transform {
        translation_m: a.point(b.translation_m),
        rotation_xyzw: [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
    }
}
fn angles(t: f64) -> [f64; 4] {
    [
        0.8 * t,
        0.7 * (1.3 * t).sin(),
        -0.8 - 0.5 * (2. * t).sin(),
        1.2 * (3. * t).sin(),
    ]
}
fn poses(t: f64) -> (Transform, [(Transform, V); 3]) {
    let [body, head, shoulder, elbow] = angles(t);
    let base = rotation(2, body, [0.3 * t.sin(), 0.2 * (0.7 * t).cos(), 0.]);
    let sensor = compose(
        base,
        compose(
            rotation(2, head, [0.4, 0., 1.15]),
            rotation(1, 0.2 * (1.7 * t).sin(), [0.12, 0., 0.]),
        ),
    );
    let arm = compose(base, rotation(1, shoulder, [0.9, 0., 0.65]));
    let forearm = compose(arm, rotation(1, elbow, [1.2, 0., 0.]));
    (
        sensor,
        [
            (
                compose(base, rotation(0, 0., [0., 0., 0.9])),
                [0.12, 0.12, 0.8],
            ),
            (
                compose(arm, rotation(0, 0., [0.6, 0., 0.])),
                [0.6, 0.12, 0.12],
            ),
            (
                compose(forearm, rotation(0, 0., [0.5, 0., 0.])),
                [0.5, 0.1, 0.1],
            ),
        ],
    )
}
fn box_range(origin: V, direction: V, pose: Transform, half: V) -> Option<f64> {
    let inverse = pose.inverse();
    let (o, d) = (inverse.point(origin), inverse.rotate(direction));
    let (mut lo, mut hi) = (f64::NEG_INFINITY, f64::INFINITY);
    for k in 0..3 {
        if d[k].abs() < 1e-12 {
            if o[k].abs() > half[k] {
                return None;
            }
        } else {
            let (a, b) = ((-half[k] - o[k]) / d[k], (half[k] - o[k]) / d[k]);
            lo = lo.max(a.min(b));
            hi = hi.min(a.max(b));
        }
    }
    if lo > hi || hi < 0. {
        None
    } else {
        Some(if lo >= 0. { lo } else { hi })
    }
}
struct Articulated {
    frozen_geometry: bool,
    near: Option<u64>,
}
impl Scene for Articulated {
    fn nearest_hit(&self, time_ns: u64, direction: V, max: f64) -> Option<SurfaceHit> {
        let t = (time_ns - EPOCH) as f64 * 1e-9;
        let (sensor, _) = poses(t);
        let (_, mut links) = poses(if self.frozen_geometry { 1. } else { t });
        let (origin, ray) = (sensor.translation_m, sensor.rotate(direction));
        let mut nearest = f64::INFINITY;
        for (axis, &component) in ray.iter().enumerate() {
            if component.abs() > 1e-12 {
                for wall in if axis == 2 { [0., 5.] } else { [-8., 8.] } {
                    let distance = (wall - origin[axis]) / component;
                    if distance >= 0. {
                        nearest = nearest.min(distance);
                    }
                }
            }
        }
        if let Some(index) = self.near {
            links[0] = (
                Transform {
                    translation_m: sensor.point(scale(NominalPattern.direction(index), 0.05)),
                    ..sensor
                },
                [0.003; 3],
            );
        }
        for (pose, half) in links {
            if let Some(range) = box_range(origin, ray, pose, half) {
                nearest = nearest.min(range);
            }
        }
        (nearest <= max).then_some(SurfaceHit {
            distance_m: nearest,
            incidence_cosine: 1.,
            reflectance: 0.8,
        })
    }
}
// Independent reference: explicit matrices, joint pivot chains and forward faces.
// No Transform rotation/inverse or slab intersections are used below.
fn matrix(axis: usize, angle: f64) -> M {
    let (s, c) = angle.sin_cos();
    match axis {
        1 => [[c, 0., s], [0., 1., 0.], [-s, 0., c]],
        2 => [[c, -s, 0.], [s, c, 0.], [0., 0., 1.]],
        _ => panic!("unsupported reference axis"),
    }
}
fn mv(m: M, v: V) -> V {
    m.map(|row| dot(row, v))
}
fn mm(a: M, b: M) -> M {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| a[i][k] * b[k][j]).sum()))
}
fn reference(t: f64, local: V) -> (f64, usize) {
    let [body, head, shoulder, elbow] = angles(t);
    let base_p = [0.3 * t.sin(), 0.2 * (0.7 * t).cos(), 0.];
    let base = matrix(2, body);
    let head_r = mm(base, matrix(2, head));
    let sensor_p = add(
        add(base_p, mv(base, [0.4, 0., 1.15])),
        mv(head_r, [0.12, 0., 0.]),
    );
    let sensor_r = mm(head_r, matrix(1, 0.2 * (1.7 * t).sin()));
    let ray = mv(sensor_r, local);
    let arm_p = add(base_p, mv(base, [0.9, 0., 0.65]));
    let arm_r = mm(base, matrix(1, shoulder));
    let fore_p = add(arm_p, mv(arm_r, [1.2, 0., 0.]));
    let fore_r = mm(arm_r, matrix(1, elbow));
    let objects = [
        ([0., 0., 2.5], matrix(2, 0.), [8., 8., 2.5]),
        (
            add(base_p, mv(base, [0., 0., 0.9])),
            base,
            [0.12, 0.12, 0.8],
        ),
        (
            add(arm_p, mv(arm_r, [0.6, 0., 0.])),
            arm_r,
            [0.6, 0.12, 0.12],
        ),
        (
            add(fore_p, mv(fore_r, [0.5, 0., 0.])),
            fore_r,
            [0.5, 0.1, 0.1],
        ),
    ];
    let mut nearest = (f64::INFINITY, 0);
    for (id, (p, r, h)) in objects.into_iter().enumerate() {
        for axis in 0..3 {
            let (a, b) = ((axis + 1) % 3, (axis + 2) % 3);
            for side in [-1., 1.] {
                let corners: [V; 4] =
                    [(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)].map(|(sa, sb)| {
                        let mut v = [0.; 3];
                        v[axis] = side * h[axis];
                        v[a] = sa * h[a];
                        v[b] = sb * h[b];
                        add(p, mv(r, v))
                    });
                for [i, j, k] in [[0, 1, 2], [0, 2, 3]] {
                    if let Some(distance) =
                        triangle(sensor_p, ray, corners[i], corners[j], corners[k])
                        && distance < nearest.0
                    {
                        nearest = (distance, id);
                    }
                }
            }
        }
    }
    nearest
}
fn triangle(o: V, d: V, a: V, b: V, c: V) -> Option<f64> {
    let (e1, e2) = (sub(b, a), sub(c, a));
    let p = cross(d, e2);
    let det = dot(e1, p);
    if det.abs() < 1e-12 {
        return None;
    }
    let t = sub(o, a);
    let u = dot(t, p) / det;
    let q = cross(t, e1);
    let v = dot(d, q) / det;
    let distance = dot(e2, q) / det;
    (u >= -1e-12 && v >= -1e-12 && u + v <= 1. + 1e-12 && distance >= 0.).then_some(distance)
}
#[test]
fn moving_links_and_sensor_match_independent_triangles_and_expose_frozen_geometry() {
    let sensor = Sensor::new(0.1, 40., EPOCH, Effects::default()).unwrap();
    let scene = Articulated {
        frozen_geometry: false,
        near: None,
    };
    let mut frame = RollingFrame::default();
    sensor
        .sample_into(&scene, 200_000, 8000, 17, &mut frame)
        .unwrap();
    let mut objects = [0usize; 4];
    let mut changed = 0;
    let frozen = Articulated {
        frozen_geometry: true,
        near: None,
    };
    for p in frame.points {
        let d = NominalPattern.direction(p.sample_index);
        let (range, object) = reference((p.time_ns - EPOCH) as f64 * 1e-9, d);
        objects[object] += 1;
        assert!(
            (p.range_m.unwrap() - range).abs() < 1e-9,
            "index {}",
            p.sample_index
        );
        let old = frozen.nearest_hit(p.time_ns, d, 40.).unwrap().distance_m;
        if (old - range).abs() > 0.01 {
            changed += 1;
        }
    }
    assert!(
        objects.into_iter().all(|n| n > 10),
        "missing exercised links: {objects:?}"
    );
    assert!(
        changed > 100,
        "frozen geometry negative control did not differ"
    );
}
#[test]
fn articulated_acquisition_preserves_seeded_values_across_chunks_and_decimation() {
    let sensor = Sensor::new(
        0.1,
        40.,
        EPOCH,
        Effects {
            range_sigma_m: 0.02,
            angular_sigma_rad: 0.001,
            dropout: 0.03,
            ..Default::default()
        },
    )
    .unwrap();
    let scene = Articulated {
        frozen_geometry: false,
        near: None,
    };
    let (mut all, mut a, mut b) = (
        RollingFrame::default(),
        RollingFrame::default(),
        RollingFrame::default(),
    );
    sensor
        .sample_into(&scene, 200_000, 6000, 1, &mut all)
        .unwrap();
    sensor
        .sample_into(&scene, 200_000, 2137, 1, &mut a)
        .unwrap();
    sensor
        .sample_into(&scene, 202_137, 3863, 1, &mut b)
        .unwrap();
    a.points.extend(b.points);
    assert_eq!(a, all);
    for stride in [3u32, 7, 31] {
        sensor
            .sample_into(&scene, 200_000, 6000u32.div_ceil(stride), stride, &mut a)
            .unwrap();
        assert_eq!(
            a.points,
            all.points
                .iter()
                .step_by(stride as usize)
                .copied()
                .collect::<Vec<_>>()
        );
    }
}
#[test]
fn mounted_near_link_occludes_background_at_multiple_joint_poses() {
    let sensor = Sensor::new(0.1, 40., EPOCH, Effects::default()).unwrap();
    for index in [0, 200_000, 800_000] {
        let scene = Articulated {
            frozen_geometry: false,
            near: Some(index),
        };
        let mut frame = RollingFrame::default();
        sensor.sample_into(&scene, index, 1, 1, &mut frame).unwrap();
        let time = EPOCH + index * 5000;
        let first = scene
            .nearest_hit(time, NominalPattern.direction(index), 40.)
            .unwrap()
            .distance_m;
        assert!((0.04..0.06).contains(&first));
        assert_eq!(frame.points[0].range_m, None);
        assert_eq!(frame.points[0].time_ns, time);
        assert!(
            Articulated {
                frozen_geometry: false,
                near: None
            }
            .nearest_hit(time, NominalPattern.direction(index), 40.)
            .unwrap()
            .distance_m
                > 0.1
        );
    }
}
