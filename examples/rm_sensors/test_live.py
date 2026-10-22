# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Independent wire-contract checks, including corrupt and discontinuous packets."""

import copy
import unittest
from live import Validator


CONFIG = {"lidar_period_ns": 10_000_000, "imu_period_ns": 5_000_000, "stride": 1000}


def packet(sequence=0):
    epoch = 1_000_000_000
    start = epoch + sequence * 10_000_000
    return {
        "sequence": sequence,
        "start_ns": start,
        "end_ns": start + 10_000_000,
        "discontinuities": 1,
        "lidar": {
            "points": [
                {
                    "sample_index": sequence * 2000,
                    "time_ns": start,
                    "range_m": 2.0,
                    "point_sensor_m": [2.0, 0.0, 0.0],
                },
                {
                    "sample_index": sequence * 2000 + 1000,
                    "time_ns": start + 5_000_000,
                    "range_m": None,
                    "point_sensor_m": None,
                },
            ]
        },
        "imu": [
            {
                "time_ns": start + i * 5_000_000,
                "angular_velocity_rad_s": [0.0, 0.0, 0.5],
                "specific_force_m_s2": [0.0, 0.0, 9.81],
            }
            for i in (1, 2)
        ],
    }


class ContractTests(unittest.TestCase):
    def test_retains_misses_and_checks_both_clocks(self):
        validator = Validator(CONFIG)
        validator.accept(packet(), 1_010_000_000)
        validator.accept(packet(1), 1_021_000_000)
        result = validator.report()
        self.assertEqual(
            (
                result["frames"],
                result["emissions"],
                result["returns"],
                result["imu_samples"],
            ),
            (2, 4, 2, 4),
        )
        self.assertEqual(result["max_simulation_delivery_age_ms"], 1.0)
        self.assertEqual(result["packet_gaps"], 0)

    def test_rejects_bad_offsets_cadence_missing_and_nonfinite_measurements(self):
        original = packet()
        for mutate in (
            lambda p: p["lidar"]["points"][0].update(time_ns=1_000_000_001),
            lambda p: p["lidar"]["points"].pop(),
            lambda p: p["lidar"]["points"][0].update(range_m=3.0),
            lambda p: p["imu"].pop(),
            lambda p: p["imu"][0].update(time_ns=1_000_000_000),
            lambda p: p["imu"][0].update(
                angular_velocity_rad_s=[float("nan"), 0.0, 0.0]
            ),
        ):
            bad = copy.deepcopy(original)
            mutate(bad)
            with self.assertRaises(ValueError):
                Validator(CONFIG).accept(bad, 1_010_000_000)

    def test_exposes_gaps_without_compressing_time(self):
        validator = Validator(CONFIG)
        validator.accept(packet(), 1_010_000_000)
        validator.accept(packet(2), 1_030_000_000)
        self.assertEqual(validator.report()["packet_gaps"], 1)
        with self.assertRaises(ValueError):
            validator.accept(packet(2), 1_030_000_000)

    def test_rejects_a_future_measurement(self):
        with self.assertRaises(ValueError):
            Validator(CONFIG).accept(packet(), 1_000_000_000)


if __name__ == "__main__":
    unittest.main()
