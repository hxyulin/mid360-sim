#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Consume measured synchronized sensors from an existing RM host, without ROS.

No world or robot controls are created here. Requires rm-simulator c3784e1 or a
compatible host. Only measured packets are inspected, not simulator pose truth.
"""

import argparse
import json
import math
from pathlib import Path
import statistics
import time
import urllib.error
import urllib.request


def post(host, path, payload):
    request = urllib.request.Request(
        host.rstrip("/") + path,
        data=json.dumps(payload, allow_nan=False).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=3) as response:
        body = response.read(32 * 1024 * 1024 + 1)
    if len(body) > 32 * 1024 * 1024:
        raise ValueError("sensor response exceeds 32 MiB")
    return json.loads(body)


class Validator:
    """Validate the public measurement contract independently of the producer."""

    def __init__(self, config):
        self.period = config.get("lidar_period_ns", 100_000_000)
        self.imu_period = config.get("imu_period_ns", 5_000_000)
        self.stride = config.get("stride", 1)
        self.minimum = config.get("min_range_m", 0.1)
        self.maximum = config.get("max_range_m", 40.0)
        self.epoch = None
        self.previous_sequence = None
        self.last_imu_ns = None
        self.frames = self.emissions = self.returns = self.inertial = 0
        self.sequence_gaps = 0
        self.angular_peak = 0.0
        self.force_norms = []
        self.age_ns = []
        self.discontinuities = 0

    def accept(self, frame, clock_ns):
        start, end, sequence = frame["start_ns"], frame["end_ns"], frame["sequence"]
        if self.epoch is None:
            self.epoch = start - sequence * self.period
        if end - start != self.period or start != self.epoch + sequence * self.period:
            raise ValueError("packet interval changed its clock or epoch")
        if self.previous_sequence is not None:
            if sequence <= self.previous_sequence:
                raise ValueError("nonmonotonic packet sequence")
            self.sequence_gaps += sequence - self.previous_sequence - 1
        if end > clock_ns:
            raise ValueError("measurement is newer than the host clock")
        self.age_ns.append(clock_ns - end)
        first = (
            ((start - self.epoch + 4999) // 5000 + self.stride - 1)
            // self.stride
            * self.stride
        )
        stop = (end - self.epoch + 4999) // 5000
        expected_count = max(0, (stop - first + self.stride - 1) // self.stride)
        points = frame["lidar"]["points"]
        if len(points) != expected_count:
            raise ValueError(
                "missing or duplicated retained emission (misses must remain explicit)"
            )
        for n, point in enumerate(points):
            index = first + n * self.stride
            if (
                point["sample_index"] != index
                or point["time_ns"] != self.epoch + index * 5000
            ):
                raise ValueError("emission index/time changed")
            xyz, distance = point["point_sensor_m"], point["range_m"]
            if (xyz is None) != (distance is None):
                raise ValueError("inconsistent miss fields")
            if xyz is not None:
                if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
                    raise ValueError("invalid local point")
                norm = math.sqrt(sum(v * v for v in xyz))
                if not self.minimum <= distance <= self.maximum or not math.isclose(
                    norm, distance, rel_tol=1e-10, abs_tol=1e-9
                ):
                    raise ValueError("local point/range or metric bounds disagree")
                self.returns += 1
        samples = frame["imu"]
        if len(samples) != self.period // self.imu_period:
            raise ValueError("incomplete synchronized IMU interval")
        for n, sample in enumerate(samples):
            if sample["time_ns"] != start + (n + 1) * self.imu_period:
                raise ValueError("IMU timestamp/cadence mismatch")
            if self.last_imu_ns is not None and sample["time_ns"] <= self.last_imu_ns:
                raise ValueError("nonmonotonic IMU clock")
            omega, force = (
                sample["angular_velocity_rad_s"],
                sample["specific_force_m_s2"],
            )
            if (
                len(omega) != 3
                or len(force) != 3
                or not all(math.isfinite(v) for v in omega + force)
            ):
                raise ValueError("nonfinite inertial measurement")
            self.angular_peak = max(
                self.angular_peak, math.sqrt(sum(v * v for v in omega))
            )
            self.force_norms.append(math.sqrt(sum(v * v for v in force)))
            self.last_imu_ns = sample["time_ns"]
        self.discontinuities = max(self.discontinuities, frame["discontinuities"])
        self.previous_sequence = sequence
        self.frames += 1
        self.emissions += len(points)
        self.inertial += len(samples)

    def report(self):
        return {
            "frames": self.frames,
            "emissions": self.emissions,
            "returns": self.returns,
            "imu_samples": self.inertial,
            "packet_gaps": self.sequence_gaps,
            "input_discontinuities": self.discontinuities,
            "peak_angular_rate_rad_s": self.angular_peak,
            "median_specific_force_m_s2": statistics.median(self.force_norms)
            if self.force_norms
            else None,
            "max_simulation_delivery_age_ms": max(self.age_ns, default=0) / 1e6,
        }


def collect(
    host, attachment, config, frames=20, timeout=15, allow_drops=False, validator=None
):
    validator = validator if validator is not None else Validator(config)
    initial_frames = validator.frames
    target_frames = initial_frames + frames
    deadline = time.monotonic() + timeout
    drops = 0
    requests = 0
    durations = []
    while validator.frames < target_frames and time.monotonic() < deadline:
        start = time.monotonic()
        batch = post(host, "/api/sensors/poll", attachment)
        durations.append((time.monotonic() - start) * 1000)
        requests += 1
        drops = max(drops, batch["dropped_intervals"])
        for frame in batch["frames"]:
            validator.accept(frame, batch["time_ns"])
        if validator.frames < target_frames:
            time.sleep(0.01)
    if validator.frames < target_frames:
        raise TimeoutError(
            f"only {validator.frames - initial_frames}/{frames} complete synchronized packets"
        )
    result = validator.report()
    result.update(
        collected_frames=validator.frames - initial_frames,
        dropped_intervals=drops,
        poll_requests=requests,
        max_poll_wall_ms=max(durations),
    )
    if not allow_drops and (drops or validator.sequence_gaps):
        raise ValueError(f"sensor stream lost intervals: {result}")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="http://127.0.0.1:7780")
    parser.add_argument("--robot", type=int, default=0)
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--allow-drops", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.frames <= 600 or not 1 <= args.stride <= 1001:
        parser.error("frames must be 1–600 and stride 1–1001")
    config = (
        json.loads(args.config.read_text())
        if args.config
        else {"robot": args.robot, "stride": args.stride}
    )
    attachment = post(args.host, "/api/sensors/attach", config)["id"]
    try:
        result = collect(
            args.host,
            attachment,
            config,
            args.frames,
            timeout=max(15, args.frames * 0.5),
            allow_drops=args.allow_drops,
        )
        print(json.dumps(result, indent=2, allow_nan=False))
    finally:
        try:
            post(args.host, "/api/sensors/detach", attachment)
        except (OSError, urllib.error.HTTPError):
            pass  # Host expiry is still bounded if the peer vanished.


if __name__ == "__main__":
    main()
