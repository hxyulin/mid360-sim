# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Real MuJoCo ray tests without a renderer or a display."""

import unittest
import numpy as np
import mujoco
from mid360_sim.pattern import MODEL, directions
from mid360_sim.mujoco import MujocoSensor

XML = """<mujoco><worldbody>
<geom name="wall" type="box" pos="5 0 0" size=".1 20 20" group="0"/>
<body name="robot"><joint type="slide" axis="1 0 0"/><geom type="sphere" size=".05" contype="0" conaffinity="0" group="1"/><site name="lidar" pos="0 0 .8"/></body>
</worldbody></mujoco>"""


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
        model, data, sensor = self.setup_sensor()
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

    def test_rolling_origin_tolerance_and_invalid_poses(self):
        model, data, sensor = self.setup_sensor()
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
