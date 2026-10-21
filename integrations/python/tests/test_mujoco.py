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
