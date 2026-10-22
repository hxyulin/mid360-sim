# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Real MuJoCo ray tests without a renderer or a display."""

import unittest
from unittest.mock import patch

import mujoco
import numpy as np
from mid360_sim.mujoco import MujocoSensor
from mid360_sim.pattern import MODEL, directions

XML = """<mujoco><worldbody>
<geom name="wall" type="box" pos="5 0 0" size=".1 20 20" group="0"/>
<body name="robot"><joint type="slide" axis="1 0 0"/><geom type="sphere" size=".05" contype="0" conaffinity="0" group="1"/><site name="lidar" pos="0 0 .8"/></body>
</worldbody></mujoco>"""


def reference_rolling_rays(local, pose_start, pose_end, fraction):
    """Original allocating SLERP/rotation expressions, kept as a bitwise oracle."""
    _p0, q0 = (np.asarray(v, dtype=float) for v in pose_start)
    _p1, q1 = (np.asarray(v, dtype=float) for v in pose_end)
    q0, q1 = q0 / np.linalg.norm(q0), q1 / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0:
        q1, dot = -q1, -dot
    t = np.linspace(fraction[0], fraction[1], len(local))
    if dot > 0.99999999:
        q = q0 + t[:, None] * (q1 - q0)
        q /= np.linalg.norm(q, axis=1)[:, None]
    else:
        angle = np.arccos(np.clip(dot, -1.0, 1.0))
        q = (
            np.sin((1 - t) * angle)[:, None] * q0 + np.sin(t * angle)[:, None] * q1
        ) / np.sin(angle)
    x, y, z, w = q.T
    dx, dy, dz = local.T
    tx = 2 * (y * dz - z * dy)
    ty = 2 * (z * dx - x * dz)
    tz = 2 * (x * dy - y * dx)
    world = np.empty_like(local)
    world[:, 0] = dx + w * tx + y * tz - z * ty
    world[:, 1] = dy + w * ty + z * tx - x * tz
    world[:, 2] = dz + w * tz + x * ty - y * tx
    return t, world


class MujocoTests(unittest.TestCase):
    def setup_sensor(self, xml=XML, **kwargs):
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        return model, data, MujocoSensor(model, data, "lidar", **kwargs)

    def test_consecutive_group_fast_path_matches_strided_queries(self):
        # Odd starts/counts and wrap boundaries exercise lane/group indexing.
        for first in (0, 1, 3, 199999, MODEL["groups_per_cycle"] * 4 - 3):
            full = directions(first, 201)
            np.testing.assert_array_equal(full[::3], directions(first, 67, stride=3))

    def test_rust_fixtures_and_long_cycle(self):
        for f in MODEL["reference_rays"]:
            np.testing.assert_allclose(
                directions(f["index"], 1)[0], f["direction"], rtol=0, atol=2e-14
            )
        np.testing.assert_array_equal(
            directions(7, 1000), directions(7 + MODEL["groups_per_cycle"] * 4, 1000)
        )

    def test_native_wall_distances_and_dynamic_pose(self):
        model, data, sensor = self.setup_sensor(max_range_m=70)
        frame = sensor.scan(0, 20000, epoch_ns=100)
        direction = frame.directions
        distance = np.where(direction[:, 0] > 0, 4.9 / direction[:, 0], np.inf)
        xyz = direction * distance[:, None]
        expected = (
            (distance <= 70)
            & (np.abs(xyz[:, 1]) <= 20)
            & (np.abs(xyz[:, 2] + 0.8) <= 20)
        )
        np.testing.assert_array_equal(frame.valid, expected)
        np.testing.assert_allclose(
            frame.ranges_m[expected], distance[expected], rtol=1e-12
        )
        data.qpos[0] = 1
        mujoco.mj_forward(model, data)
        moved = sensor.scan(0, 20000)
        np.testing.assert_allclose(
            moved.ranges_m[expected], 3.9 / direction[expected, 0], rtol=1e-12
        )
        self.assertEqual(frame.timestamps_ns[1], 5100)

    def test_chunk_stride_owned_frames_preserve_dynamics(self):
        _model, data, sensor = self.setup_sensor()
        time_before = data.time
        qpos = data.qpos.copy()
        full = sensor.scan(7, 1000, stride=3)
        first = sensor.scan(7, 400, stride=3)
        second = sensor.scan(1207, 600, stride=3)
        np.testing.assert_array_equal(
            full.directions, np.r_[first.directions, second.directions]
        )
        np.testing.assert_array_equal(
            full.ranges_m, np.r_[first.ranges_m, second.ranges_m]
        )
        self.assertEqual(data.time, time_before)
        np.testing.assert_array_equal(data.qpos, qpos)
        p = full.livox_fields()
        self.assertEqual(p["timebase"], 7 * 5000)
        np.testing.assert_array_equal(p["line"], full.indices[full.valid] % 4)
        np.testing.assert_array_equal(
            p["offset_time"], np.flatnonzero(full.valid) * 3 * 5000
        )

    def test_near_geometry_occludes_farther_wall_and_group_filter(self):
        xml = XML.replace(
            '<site name="lidar"',
            '<geom type="sphere" pos="0 0 .8" size=".05" group="2" contype="0" conaffinity="0"/><site name="lidar"',
        )
        model, data, sensor = self.setup_sensor(xml)
        self.assertFalse(sensor.scan(0, 1000).valid.any())
        filtered = MujocoSensor(model, data, "lidar", geom_groups=[1, 1, 0, 1, 1, 1])
        self.assertTrue(filtered.scan(0, 1000).valid.any())

    def test_rolling_matches_individual_native_rays_and_owns_results(self):
        model, data, sensor = self.setup_sensor()
        p0 = np.array([0.0, 0.0, 0.8])
        p1 = np.array([0.4, 0.2, 1.0])
        q0 = np.array([0.0, 0.0, 0.0, 1.0])
        q1 = np.array([0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)])
        frame = sensor.scan_rolling(
            1000,
            201,
            (p0, q0),
            (p1, q1),
            stride=3,
            pose_fraction=(0.2, 0.8),
            origin_tolerance_m=1e-6,
        )
        expected = []
        geom = np.empty(1, dtype=np.int32)
        for t, local in zip(np.linspace(0.2, 0.8, 201), frame.directions):
            angle = t * np.pi / 2
            rotation = np.array(
                [
                    [np.cos(angle), -np.sin(angle), 0],
                    [np.sin(angle), np.cos(angle), 0],
                    [0, 0, 1.0],
                ]
            )
            distance = mujoco.mj_ray(
                model, data, p0 + (p1 - p0) * t, rotation @ local, None, 1, -1, geom
            )
            expected.append(distance if 0.1 <= distance <= 70 else -1)
        np.testing.assert_allclose(frame.ranges_m, expected, atol=1e-11)
        self.assertTrue(frame.valid.any())
        saved = frame.ranges_m.copy()
        sensor.scan(0, 1000)
        np.testing.assert_array_equal(frame.ranges_m, saved)
        np.testing.assert_array_equal(frame.timestamps_ns, frame.indices * 5000)
        self.assertEqual(data.time, 0.0)

    def test_rolling_small_and_near_half_turn_about_arbitrary_axis(self):
        walls = []
        for dimension in range(3):
            for sign in (-1, 1):
                position, half = [0.0] * 3, [10.1] * 3
                position[dimension], half[dimension] = sign * 10.1, 0.1
                walls.append(
                    f'<geom type="box" pos="{" ".join(map(str, position))}" '
                    f'size="{" ".join(map(str, half))}"/>'
                )
        model, data, sensor = self.setup_sensor(
            '<mujoco><worldbody><site name="lidar" pos="0 0 .8"/>'
            + "".join(walls)
            + "</worldbody></mujoco>"
        )
        origin = np.array([0.0, 0.0, 0.8])
        axis = np.array([1.0, 2.0, -3.0]) / np.sqrt(14.0)
        skew = np.array(
            [[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]]
        )
        geom = np.empty(1, dtype=np.int32)
        for angle in (1e-5, np.pi - 1e-6):
            # Negated endpoint must follow the same shortest rotation.
            q1 = -np.r_[axis * np.sin(angle / 2), np.cos(angle / 2)]
            frame = sensor.scan_rolling(
                1000, 67, (origin, [0, 0, 0, 1]), (origin, q1), stride=3
            )
            expected = []
            for t, local in zip(np.linspace(0, 1, 67), frame.directions):
                rotation = (
                    np.eye(3)
                    + np.sin(t * angle) * skew
                    + (1 - np.cos(t * angle)) * (skew @ skew)
                )
                distance = mujoco.mj_ray(
                    model, data, origin, rotation @ local, None, 1, -1, geom
                )
                expected.append(distance if 0.1 <= distance <= 70 else -1)
            self.assertTrue(frame.valid.any())
            np.testing.assert_allclose(frame.ranges_m, expected, rtol=0, atol=1e-10)

    def test_mixed_queries_match_native_reference_and_own_outputs(self):
        model, data, sensor = self.setup_sensor()
        native_multi_ray = mujoco.mj_multiRay
        saved = []
        axis = np.array([1.0, 2.0, -3.0]) / np.sqrt(14.0)

        def compare_query(*args):
            nonlocal offset
            n = args[10]
            a, b = offset, offset + n
            np.testing.assert_array_equal(args[3].reshape(-1, 3), expected_world[a:b])
            expected_origin = p0 + (p1 - p0) * ((t[a] + t[b - 1]) * 0.5)
            np.testing.assert_array_equal(args[2], expected_origin)
            native_multi_ray(
                model,
                data,
                expected_origin,
                expected_world[a:b].ravel(),
                None,
                1,
                -1,
                expected_ids[a:b],
                expected_ranges[a:b],
                None,
                n,
                70.0,
            )
            offset = b
            return native_multi_ray(*args)

        # Interleave held/rolling calls, growth/shrink, both SLERP branches,
        # shortest-arc sign reversal, half turns and changing live geometry.
        for step, (count, angle, stride) in enumerate(
            (
                (201, 0.0, 1),
                (17, 1e-5, 3),
                (301, np.pi, 7),
                (1, np.pi - 1e-6, 1),
                (67, 0.7, 3),
            )
        ):
            data.qpos[0] = step * 0.1
            mujoco.mj_forward(model, data)
            held = sensor.scan(1001, count, stride=stride)
            p0 = np.array([0.03 * step, -0.02 * step, 0.8])
            p1 = p0 + [0.006, 0.004, 0.002]
            q0 = [0.0, 0.0, 0.0, 1.0]
            q1 = -np.r_[axis * np.sin(angle / 2), np.cos(angle / 2)]
            fraction = (0.2, 0.8)
            t, expected_world = reference_rolling_rays(
                directions(1001, count, stride), (p0, q0), (p1, q1), fraction
            )
            expected_ranges = np.empty(count)
            expected_ids = np.empty(count, dtype=np.int32)
            offset = 0

            with patch(
                "mid360_sim.mujoco.mujoco.mj_multiRay", side_effect=compare_query
            ):
                frame = sensor.scan_rolling(
                    1001,
                    count,
                    (p0, q0),
                    (p1, q1),
                    stride=stride,
                    epoch_ns=123,
                    pose_fraction=fraction,
                    origin_tolerance_m=0.001,
                )
            self.assertEqual(offset, count)
            invalid = (
                (expected_ranges < 0.1)
                | (expected_ranges > 70)
                | ~np.isfinite(expected_ranges)
            )
            expected_ranges[invalid], expected_ids[invalid] = -1, -1
            np.testing.assert_array_equal(frame.ranges_m, expected_ranges)
            np.testing.assert_array_equal(frame.geom_ids, expected_ids)
            for result in (held, frame):
                arrays = (result.directions, result.ranges_m, result.geom_ids)
                self.assertTrue(all(array.flags.owndata for array in arrays))
                saved.append((result, tuple(array.copy() for array in arrays)))
            for old, copies in saved:
                for array, copy in zip(
                    (old.directions, old.ranges_m, old.geom_ids), copies
                ):
                    np.testing.assert_array_equal(array, copy)
        self.assertEqual(sensor.capacity, 301)
        sensor.scan_rolling(0, 200, (p0, q0), (p1, q1))
        with self.assertRaises(ValueError):
            sensor.scan(0, 200001)
        self.assertEqual(sensor.capacity, 301)

    def test_rolling_chunk_stride_and_near_occlusion(self):
        model, data, sensor = self.setup_sensor()
        p0, p1 = np.array([0.0, 0.0, 0.8]), np.array([0.02, 0.01, 0.8])
        q0, q1 = [0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 1.0, 0.0]
        # One native origin per emission avoids the documented grouping error.
        kwargs = {"stride": 3, "origin_tolerance_m": 1e-9}
        full = sensor.scan_rolling(7, 201, (p0, q0), (p1, q1), **kwargs)
        first = sensor.scan_rolling(
            7, 101, (p0, q0), (p1, q1), pose_fraction=(0.0, 0.5), **kwargs
        )
        second = sensor.scan_rolling(
            310, 100, (p0, q0), (p1, q1), pose_fraction=(0.505, 1.0), **kwargs
        )
        np.testing.assert_array_equal(
            full.directions, np.r_[first.directions, second.directions]
        )
        np.testing.assert_array_equal(
            full.geom_ids, np.r_[first.geom_ids, second.geom_ids]
        )
        np.testing.assert_allclose(
            full.ranges_m, np.r_[first.ranges_m, second.ranges_m], rtol=0, atol=1e-12
        )
        xml = XML.replace(
            '<site name="lidar"',
            '<geom type="sphere" pos="0 0 .8" size=".05" group="2" contype="0" conaffinity="0"/><site name="lidar"',
        )
        model, data, sensor = self.setup_sensor(xml)
        blocked = sensor.scan_rolling(0, 201, (p0, q0), (p0, q1))
        self.assertFalse(blocked.valid.any())
        np.testing.assert_array_equal(blocked.geom_ids, -1)
        filtered = MujocoSensor(model, data, "lidar", geom_groups=[1, 1, 0, 1, 1, 1])
        self.assertTrue(filtered.scan_rolling(0, 201, (p0, q0), (p0, q1)).valid.any())

    def test_rolling_queries_current_nearest_moving_geometry(self):
        xml = XML.replace(
            '<body name="robot">',
            '<body name="obstacle" pos="2 0 0">'
            '<joint type="slide" axis="1 0 0"/>'
            '<geom name="near" type="box" size=".1 20 20"/></body>'
            '<body name="robot">',
        )
        model, data, sensor = self.setup_sensor(xml)
        origin, q0, q1 = (
            [0.0, 0.0, 0.8],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.1, np.sqrt(0.99)],
        )
        first = int(np.argmax(directions(0, 20000)[:, 0]))
        _, world = reference_rolling_rays(
            directions(first, 201), (origin, q0), (origin, q1), (0.0, 1.0)
        )
        forward = world[:, 0] > 0.5
        self.assertTrue(forward.any())
        near_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "near")
        for displacement in (0.0, 0.5, -0.2):
            data.qpos[0] = displacement
            mujoco.mj_forward(model, data)
            frame = sensor.scan_rolling(first, 201, (origin, q0), (origin, q1))
            np.testing.assert_array_equal(frame.geom_ids[forward], near_id)
            np.testing.assert_allclose(
                frame.ranges_m[forward],
                (1.9 + displacement) / world[forward, 0],
                rtol=1e-12,
            )

    def test_rolling_origin_tolerance_and_invalid_poses(self):
        _model, _data, sensor = self.setup_sensor()
        p0 = np.array([0.0, 0.0, 0.8])
        p1 = np.array([0.01, 0.0, 0.8])
        q = [0.0, 0.0, 0.0, 1.0]
        frame = sensor.scan_rolling(
            1000, 1001, (p0, q), (p1, q), origin_tolerance_m=0.002
        )
        t = np.linspace(0, 1, 1001)
        keep = frame.valid
        # The wall plane converts range error into an X-origin displacement.
        error = frame.ranges_m[keep] * frame.directions[keep, 0] - (
            4.9 - 0.01 * t[keep]
        )
        self.assertLessEqual(float(np.max(abs(error))), 0.002 + 1e-12)
        for kwargs in ({"origin_tolerance_m": 0.0}, {"pose_fraction": (0.9, 0.1)}):
            with self.assertRaises(ValueError):
                sensor.scan_rolling(0, 200, (p0, q), (p1, q), **kwargs)
        with self.assertRaises(ValueError):
            sensor.scan_rolling(0, 200, (p0, [0, 0, 0, 0]), (p1, q))

    def test_validation(self):
        model, data, sensor = self.setup_sensor()
        for args in [(0, 0), (-1, 100), (2**64, 100)]:
            with self.assertRaises(ValueError):
                sensor.scan(*args)
        with self.assertRaises(ValueError):
            MujocoSensor(model, data, "missing")
        with self.assertRaises(ValueError):
            MujocoSensor(model, data, "lidar", geom_groups=[1] * 7)


if __name__ == "__main__":
    unittest.main()
