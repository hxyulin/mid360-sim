# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Benchmark-only lossless RM bundle codec; not a production transport/API.

Little endian, f64 SI values and u64 stamps/indices; explicit hit flags keep
misses. No sensor stamps are rescaled. Counts and allocation are bounded.
"""

import math
import struct

HEADER = struct.Struct("<8sQQI")
FRAME = struct.Struct("<QQQQII")
POINT = struct.Struct("<QQBdddd")
IMU = struct.Struct("<QQdddddd")
MAX_BYTES = 32 * 1024 * 1024


def encode(batch):
    if len(batch["frames"]) > 2:
        raise ValueError("at most two frames")
    data = bytearray(
        HEADER.pack(
            b"M360BN01",
            batch["time_ns"],
            batch["dropped_intervals"],
            len(batch["frames"]),
        )
    )
    for f in batch["frames"]:
        points, imu = f["lidar"]["points"], f["imu"]
        if len(points) > 200_000 or len(imu) > 1000:
            raise ValueError("sensor count exceeds budget")
        data.extend(
            FRAME.pack(
                f["sequence"],
                f["start_ns"],
                f["end_ns"],
                f["discontinuities"],
                len(points),
                len(imu),
            )
        )
        for p in points:
            hit = p["range_m"] is not None
            if hit != (p["point_sensor_m"] is not None):
                raise ValueError("inconsistent miss")
            values = (
                (p["range_m"], *p["point_sensor_m"]) if hit else (0.0, 0.0, 0.0, 0.0)
            )
            if not all(math.isfinite(v) for v in values):
                raise ValueError("nonfinite point")
            data.extend(POINT.pack(p["sample_index"], p["time_ns"], hit, *values))
        for s in imu:
            values = [*s["angular_velocity_rad_s"], *s["specific_force_m_s2"]]
            if not all(math.isfinite(v) for v in values):
                raise ValueError("nonfinite IMU")
            data.extend(IMU.pack(s["sample_index"], s["time_ns"], *values))
    if len(data) > MAX_BYTES:
        raise ValueError("response exceeds budget")
    return bytes(data)


def decode(data):
    if len(data) > MAX_BYTES:
        raise ValueError("response exceeds budget")
    offset = 0

    def read(layout):
        nonlocal offset
        if offset + layout.size > len(data):
            raise ValueError("truncated bundle")
        value = layout.unpack_from(data, offset)
        offset += layout.size
        return value

    magic, clock, drops, count = read(HEADER)
    if magic != b"M360BN01" or count > 2:
        raise ValueError("unsupported header")
    frames = []
    for _ in range(count):
        sequence, start, end, discontinuities, n, ni = read(FRAME)
        if (
            n > 200_000
            or ni > 1000
            or n * POINT.size + ni * IMU.size > len(data) - offset
        ):
            raise ValueError("invalid bounded counts")
        points, imu = [], []
        for _ in range(n):
            index, stamp, hit, r, x, y, z = read(POINT)
            if hit not in (0, 1) or not all(math.isfinite(v) for v in [r, x, y, z]):
                raise ValueError("invalid point")
            if not hit and any(v != 0 for v in [r, x, y, z]):
                raise ValueError("noncanonical miss")
            points.append(
                dict(
                    sample_index=index,
                    time_ns=stamp,
                    range_m=r if hit else None,
                    point_sensor_m=[x, y, z] if hit else None,
                )
            )
        for _ in range(ni):
            index, stamp, *values = read(IMU)
            if not all(math.isfinite(v) for v in values):
                raise ValueError("invalid IMU")
            imu.append(
                dict(
                    sample_index=index,
                    time_ns=stamp,
                    angular_velocity_rad_s=values[:3],
                    specific_force_m_s2=values[3:],
                )
            )
        frames.append(
            dict(
                sequence=sequence,
                start_ns=start,
                end_ns=end,
                discontinuities=discontinuities,
                lidar=dict(points=points),
                imu=imu,
            )
        )
    if offset != len(data):
        raise ValueError("trailing bytes")
    return dict(time_ns=clock, dropped_intervals=drops, frames=frames)
