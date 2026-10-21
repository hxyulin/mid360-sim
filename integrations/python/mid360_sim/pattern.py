# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Vectorized nominal optics and transport-free outputs, exported from Rust.

This NumPy implementation is checked against the generated Rust reference rays.
No random noise or manufacturer calibration is implied by the optical fit.
"""

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

MODEL = json.loads(Path(__file__).with_name("nominal.json").read_text())
TERMS = np.array(MODEL["terms"], dtype=np.int64)
COEFFICIENTS = np.array(MODEL["coefficients"], dtype=np.float64)


def validate_window(first_index, count, stride, epoch_ns=0):
    """Validate a bounded emission window before allocation/querying."""
    if (
        not all(isinstance(v, int) for v in [first_index, count, stride, epoch_ns])
        or first_index < 0
        or epoch_ns < 0
        or not 1 <= count <= 200000
        or not 1 <= stride <= 1001
    ):
        raise ValueError("invalid acquisition window")
    if epoch_ns + (first_index + (count - 1) * stride) * 5000 > np.iinfo(np.uint64).max:
        raise ValueError("timestamp overflow")


def directions(first_index, count, stride=1):
    """Unit sensor-FLU rays at explicit emission indices, without a per-ray table."""
    validate_window(first_index, count, stride)
    indices = first_index + np.arange(count, dtype=np.uint64) * stride
    if stride == 1:
        # Consecutive emissions already form consecutive four-lane groups.
        # Avoid sorting/allocating np.unique's general inverse mapping at 1 kHz.
        lane_offset = first_index % 4
        group_count = (lane_offset + count + 3) // 4
        groups = (first_index // 4 + np.arange(group_count, dtype=np.uint64)) % MODEL[
            "groups_per_cycle"
        ]
        inverse = (np.arange(count) + lane_offset) // 4
    else:
        groups, inverse = np.unique(
            (indices // 4) % MODEL["groups_per_cycle"], return_inverse=True
        )
    spin = (groups * MODEL["spin_hz"] % 50000).astype(float) * (2 * np.pi / 50000)
    sweep = (groups * MODEL["sweep_numerator"] % MODEL["groups_per_cycle"]).astype(
        float
    ) * (2 * np.pi / MODEL["groups_per_cycle"])
    phase = spin[:, None] * TERMS[:, 0] + sweep[:, None] * TERMS[:, 1]
    n = len(TERMS)
    angles = (
        COEFFICIENTS[:, 0]
        + np.einsum("gt,ltc->glc", np.cos(phase), COEFFICIENTS[:, 1 : 1 + n])
        + np.einsum("gt,ltc->glc", np.sin(phase), COEFFICIENTS[:, 1 + n :])
    )
    selected = angles[inverse, (indices % 4).astype(int)] * (np.pi / 720)
    az = spin[inverse] + selected[:, 0]
    el = selected[:, 1]
    return np.column_stack(
        [np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)]
    )


@dataclass
class ScanFrame:
    """Local rays and ranges (metres, -1 on miss), with acquisition identities."""

    first_index: int
    stride: int
    epoch_ns: int
    directions: np.ndarray
    ranges_m: np.ndarray
    geom_ids: np.ndarray

    @property
    def indices(self):
        return (
            self.first_index
            + np.arange(len(self.ranges_m), dtype=np.uint64) * self.stride
        )

    @property
    def timestamps_ns(self):
        return self.epoch_ns + self.indices * 5000

    @property
    def valid(self):
        return self.ranges_m >= 0

    def livox_fields(self):
        """CustomPoint-compatible arrays; misses preserve original timing offsets."""
        count = len(self.ranges_m)
        if count == 0 or (count - 1) * self.stride * 5000 > np.iinfo(np.uint32).max:
            raise ValueError("empty scan or offset_time overflow")
        keep = self.valid
        return dict(
            timebase=self.epoch_ns + self.first_index * 5000,
            offset_time=(np.arange(count, dtype=np.uint64) * self.stride * 5000)[
                keep
            ].astype(np.uint32),
            xyz_m=(self.directions * self.ranges_m[:, None])[keep],
            line=(self.indices[keep] % 4).astype(np.uint8),
            tag=np.full(int(keep.sum()), 0x10, dtype=np.uint8),
            reflectivity=np.full(int(keep.sum()), 100, dtype=np.uint8),
        )
