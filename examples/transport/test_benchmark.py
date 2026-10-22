# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Measurement clock bounds and lossless codec boundary checks, without ROS."""

import copy
import math
import struct
import unittest

import codec
from benchmark import clock_interval, distribution, validate_metadata


def bundle():
    # Absolute times above float64's exact-integer range must remain integers.
    stamp = 10**18 + 123
    frame = {
        "sequence": 0,
        "start_ns": stamp,
        "end_ns": stamp + 100_000_000,
        "discontinuities": 2,
        "lidar": {
            "points": [
                {
                    "sample_index": 0,
                    "time_ns": stamp,
                    "range_m": None,
                    "point_sensor_m": None,
                },
                {
                    "sample_index": 3,
                    "time_ns": stamp + 15000,
                    "range_m": 5.0,
                    "point_sensor_m": [3.0, -4.0, 0.0],
                },
            ]
        },
        "imu": [
            {
                "sample_index": 2,
                "time_ns": stamp + 5_000_000,
                "angular_velocity_rad_s": [0.0, 0.5, -0.1],
                "specific_force_m_s2": [0.0, 0.0, 9.81],
            }
        ],
    }
    return {"time_ns": stamp + 100_000_000, "dropped_intervals": 7, "frames": [frame]}


class MeasurementTests(unittest.TestCase):
    def test_clock_interval_intersection_and_inconsistency(self):
        self.assertEqual(
            clock_interval([(1000, 1100, 100), (1020, 1060, 100)]), (940.0, 20.0)
        )
        with self.assertRaises(ValueError):
            clock_interval([(1000, 1100, 100), (2000, 2100, 100)])

    def test_nearest_rank_tail_retains_outlier(self):
        result = distribution([1] * 99 + [1000])
        self.assertEqual(result["p99"], 1)
        self.assertEqual(result["max"], 1000)
        self.assertEqual(distribution([]), {"n": 0})

    def test_binary_keeps_integer_times_indices_misses_and_si(self):
        original = bundle()
        self.assertEqual(codec.decode(codec.encode(original)), original)
        original["frames"][0]["imu"][0]["specific_force_m_s2"][0] = -0.0
        restored = codec.decode(codec.encode(original))
        self.assertEqual(
            math.copysign(
                1.0, restored["frames"][0]["imu"][0]["specific_force_m_s2"][0]
            ),
            -1.0,
        )
        empty = {"time_ns": 123, "dropped_intervals": 0, "frames": []}
        self.assertEqual(codec.decode(codec.encode(empty)), empty)

    def test_codec_rejects_truncation_flags_counts_and_nonfinite(self):
        raw = codec.encode(bundle())
        for n in [
            0,
            4,
            codec.HEADER.size,
            codec.HEADER.size + codec.FRAME.size,
            len(raw) - 1,
        ]:
            with self.assertRaises(ValueError):
                codec.decode(raw[:n])
        for raw_bad in [b"badmagic" + raw[8:], raw + b"x"]:
            with self.assertRaises(ValueError):
                codec.decode(raw_bad)
        modified = bytearray(raw)
        modified[codec.HEADER.size + codec.FRAME.size + 16] = 2
        with self.assertRaises(ValueError):
            codec.decode(modified)
        modified = bytearray(raw)
        struct.pack_into("<I", modified, codec.HEADER.size + 32, 200_001)
        with self.assertRaises(ValueError):
            codec.decode(modified)
        original = bundle()
        original["frames"][0]["imu"][0]["specific_force_m_s2"][0] = float("nan")
        with self.assertRaises(ValueError):
            codec.encode(original)

    def test_metadata_keeps_decimation_epoch_and_miss_times(self):
        frame = bundle()["frames"][0]
        epoch = frame["start_ns"]
        # 1001 stride gives 20 emissions, including the leading miss.
        frame["lidar"]["points"] = [
            {"sample_index": i * 1001, "time_ns": epoch + i * 1001 * 5000}
            for i in range(20)
        ]
        frame["imu"] = [{"time_ns": epoch + (i + 1) * 5_000_000} for i in range(20)]
        validate_metadata(frame, 1001, epoch)
        broken = copy.deepcopy(frame)
        broken["lidar"]["points"][-1]["time_ns"] += 1
        with self.assertRaises(ValueError):
            validate_metadata(broken, 1001, epoch)
