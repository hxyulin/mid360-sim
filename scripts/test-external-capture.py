# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Regression checks for analysis-container parsing and timing hypotheses."""

import contextlib
import importlib.util
import io
import json
from pathlib import Path
import struct
import tempfile
import unittest

import numpy as np

spec = importlib.util.spec_from_file_location(
    "external_capture", Path(__file__).with_name("analyze-external-capture.py")
)
external = importlib.util.module_from_spec(spec)
spec.loader.exec_module(external)


def capture_bytes(extrinsics=(0, 0, 0, 0, 0, 0), following=None):
    header = struct.pack(
        "<16s4sIIB", b"livox_tech" + b"\0" * 6, b"\x02\0\0\0", 0xAC0EA767, 50, 1
    )
    device = struct.pack("<16s16sIBBB6f", b"synthetic", b"", 42, 0, 9, 1, *extrinsics)
    packets = []
    for packet in range(2):
        p = np.zeros(96, dtype=external.POINTS)
        # Misses remain in the original emission positions and lane schedule.
        p["xyz"][1] = [3000, 1000, 500]
        packets.append(
            external.PACKAGE.pack(
                0,
                42,
                0,
                0,
                1000000 + packet * 480000,
                packet,
                1,
                p.nbytes,
                0,
                b"\0" * 4,
            )
            + p.tobytes()
        )
    offset = len(header) + len(device)
    end = offset + 24 + sum(map(len, packets))
    return (
        header
        + device
        + struct.pack("<QQQ", offset, end if following is None else following, 0)
        + b"".join(packets)
    )


class ExternalCaptureTests(unittest.TestCase):
    def test_lvx_retains_miss_timing_and_lane(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "synthetic.lvx2"
            path.write_bytes(capture_bytes())
            output = Path(directory) / "analysis"
            with contextlib.redirect_stdout(io.StringIO()):
                external.extract(path, output)
            d = np.load(output / "samples.npz")
            np.testing.assert_array_equal(d["time_ns"], [5000, 485000])
            np.testing.assert_array_equal(d["lane"], [1, 1])
            report = json.loads((output / "decode-report.json").read_text())
            self.assertEqual(report["emissions"], 192)
            self.assertEqual(report["zero"], 190)
            self.assertEqual(report["message_boundary_spacing_ns"], {"5000": 1})
            with self.assertRaises(FileExistsError):
                external.extract(path, output)

    def test_lvx_rejects_truncation_and_frame_overrun(self):
        for data in [capture_bytes()[:-1], capture_bytes(following=999999)]:
            with self.subTest(
                length=len(data)
            ), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "synthetic.lvx2"
                path.write_bytes(data)
                with self.assertRaises(ValueError):
                    list(external.lvx2_packets(path, {}))

    def test_lvx_rejects_nonidentity_extrinsics(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "synthetic.lvx2"
            path.write_bytes(capture_bytes(extrinsics=(0, 0, 0, 1, 0, 0)))
            with self.assertRaisesRegex(ValueError, "extrinsics"):
                list(external.lvx2_packets(path, {}))

    def test_driver_timing_reconstruction_preserves_group_starts(self):
        original = np.r_[np.arange(96) * 4947, 480000 + np.arange(96) * 4988]
        corrected = external.reconstruct_ros_offsets(original)
        np.testing.assert_array_equal(corrected[:96], np.arange(96) * 5000)
        self.assertEqual(corrected[96], 480000)
        self.assertEqual(corrected[-1], 480000 + 478900)
        self.assertEqual(original[95], 469965)

    def test_driver_timing_reconstruction_rejects_unknown_layout(self):
        for offsets in [
            np.arange(95) * 4947,
            np.arange(96) * 4947 + np.arange(96) % 2,
            np.arange(96) * 24,
        ]:
            with self.subTest(length=len(offsets)), self.assertRaises(ValueError):
                external.reconstruct_ros_offsets(offsets)


if __name__ == "__main__":
    unittest.main()
