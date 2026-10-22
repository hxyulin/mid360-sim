#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Validate the native rclrs example's synchronized measurements and shutdown."""

import argparse
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rmw", default="rmw_cyclonedds_cpp")
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not args.worker:
        with tempfile.TemporaryFile(mode="w+") as output:
            child = subprocess.Popen(
                [
                    sys.executable,
                    str(Path(__file__).resolve()),
                    *sys.argv[1:],
                    "--worker",
                ],
                stdout=output,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                child.wait(timeout=40)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
                raise SystemExit("FAIL: synchronized ROS lifecycle exceeded 40 seconds")
            finally:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                output.seek(0)
                print(output.read(), end="")
            raise SystemExit(child.returncode)
    root = Path(__file__).resolve().parents[2]
    os.environ["RMW_IMPLEMENTATION"] = args.rmw
    os.environ["ROS_DOMAIN_ID"] = "80"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "SYSTEM_DEFAULT"
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    os.environ.setdefault(
        "CYCLONEDDS_URI", "file://" + str(root / "config/cyclonedds-loopback.xml")
    )
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from livox_ros_driver2.msg import CustomMsg
    from sensor_msgs.msg import Imu
    from rosgraph_msgs.msg import Clock

    rclpy.init(args=[])
    node = rclpy.create_node("mid360_synchronized_check")
    scans, samples, clocks = [], [], []
    qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(CustomMsg, "/synchronized/lidar", scans.append, qos)
    node.create_subscription(Imu, "/synchronized/imu", samples.append, qos)
    node.create_subscription(Clock, "/clock", clocks.append, qos)
    with tempfile.TemporaryFile(mode="w+") as log:
        child = subprocess.Popen(
            [
                str(root / "target/release/examples/synchronized"),
                "--ros",
                "--frames",
                "6",
                "--stride",
                str(args.stride),
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        try:
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
                if (
                    child.poll() is not None
                    and len(scans) == 6
                    and len(samples) == 120
                    and clocks
                ):
                    break
                if child.poll() not in (None, 0):
                    raise RuntimeError("native publisher failed")
            assert child.poll() == 0, "native publisher failed or hung"
            assert len(scans) == 6 and len(samples) == 120, (len(scans), len(samples))
            assert [
                m.header.stamp.sec * 10**9 + m.header.stamp.nanosec for m in samples
            ] == [1_000_000_000 + (i + 1) * 5_000_000 for i in range(120)]
            for n, scan in enumerate(scans):
                base = (
                    1_000_000_000
                    + ((n * 20_000 + args.stride - 1) // args.stride)
                    * args.stride
                    * 5000
                )
                assert scan.timebase == base and scan.header.frame_id == "example_lidar"
                assert scan.header.stamp.sec * 10**9 + scan.header.stamp.nanosec == base
                assert scan.point_num == len(scan.points) > 0
                assert all(
                    base + p.offset_time < 1_000_000_000 + (n + 1) * 100_000_000
                    for p in scan.points
                )
            for msg in samples:
                assert msg.header.frame_id == "example_imu"
                assert msg.orientation_covariance[0] == -1
                assert all(
                    math.isfinite(v)
                    for v in (
                        msg.angular_velocity.x,
                        msg.angular_velocity.y,
                        msg.angular_velocity.z,
                        msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z,
                    )
                )
            norm = [
                math.sqrt(
                    m.linear_acceleration.x**2
                    + m.linear_acceleration.y**2
                    + m.linear_acceleration.z**2
                )
                for m in samples
            ]
            assert 3 < statistics.median(norm) < 30, "incorrect acceleration units"
            assert (
                clocks[-1].clock.sec * 10**9 + clocks[-1].clock.nanosec == 1_600_000_000
            )
        finally:
            if child.poll() is None:
                child.kill()
                child.wait(timeout=5)
            log.seek(0)
            print(log.read())
            node.destroy_node()
            rclpy.shutdown()
    print(
        "PASS: six native ROS LiDAR packets, 120 SI IMU samples, shared clock and clean shutdown"
    )


if __name__ == "__main__":
    main()
