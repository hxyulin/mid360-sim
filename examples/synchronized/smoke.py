#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Validate the native rclrs example's synchronized measurements and shutdown."""

import argparse
import math
import os
import signal
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path


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
    import struct

    import rclpy
    from livox_ros_driver2.msg import CustomMsg
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, PointCloud2
    from tf2_msgs.msg import TFMessage

    rclpy.init(args=[])
    node = rclpy.create_node("mid360_synchronized_check")
    scans, samples, clocks = [], [], []
    clouds, moving, fixed = [], [], []
    late_fixed = []
    late_sub = None
    qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(CustomMsg, "/synchronized/lidar", scans.append, qos)
    node.create_subscription(Imu, "/synchronized/imu", samples.append, qos)
    node.create_subscription(Clock, "/clock", clocks.append, qos)
    node.create_subscription(PointCloud2, "/synchronized/points", clouds.append, qos)
    node.create_subscription(TFMessage, "/tf", moving.append, 100)
    node.create_subscription(
        TFMessage,
        "/tf_static",
        fixed.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    with tempfile.TemporaryFile(mode="w+") as log:
        child = subprocess.Popen(
            [
                str(root / "target/release/examples/synchronized"),
                "--ros",
                "--tf",
                "--pointcloud",
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
                if moving and late_sub is None:
                    late_sub = node.create_subscription(
                        TFMessage,
                        "/tf_static",
                        late_fixed.append,
                        QoSProfile(
                            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
                        ),
                    )
                if (
                    child.poll() is not None
                    and len(scans) == 6
                    and len(samples) == 120
                    and len(clouds) == 6
                    and moving
                    and fixed
                    and late_fixed
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
            assert len(clouds) == 6 and fixed and moving and late_fixed
            assert late_fixed[-1] == fixed[-1], (
                "late subscriber lost latched static mounts"
            )
            static = [t for msg in fixed for t in msg.transforms]
            assert all(
                t.header.frame_id == "example_lidar"
                and t.child_frame_id == "example_imu"
                for t in static
            )
            assert math.isclose(static[0].transform.translation.x, 0.04)
            assert math.isclose(static[0].transform.rotation.y, math.sin(0.05))
            edges = [t for msg in moving for t in msg.transforms]
            assert len(edges) == 121
            assert all(
                t.header.frame_id == "example_body"
                and t.child_frame_id == "example_lidar"
                for t in edges
            )
            assert edges[0].transform.rotation != edges[-1].transform.rotation
            assert [
                t.header.stamp.sec * 10**9 + t.header.stamp.nanosec for t in edges
            ] == [10**9 + i * 5_000_000 for i in range(121)]
            for scan, cloud in zip(scans, clouds):
                assert (
                    cloud.header == scan.header
                    and cloud.height == 1
                    and cloud.width == len(scan.points)
                )
                assert (
                    cloud.point_step == 20
                    and cloud.row_step == 20 * cloud.width
                    and cloud.is_dense
                    and not cloud.is_bigendian
                )
                assert [
                    (f.name, f.offset, f.datatype, f.count) for f in cloud.fields
                ] == [
                    ("x", 0, 7, 1),
                    ("y", 4, 7, 1),
                    ("z", 8, 7, 1),
                    ("offset_time", 12, 6, 1),
                    ("reflectivity", 16, 2, 1),
                    ("tag", 17, 2, 1),
                    ("line", 18, 2, 1),
                ]
                decoded = list(struct.iter_unpack("<fffIBBBx", bytes(cloud.data)))
                assert decoded == [
                    (p.x, p.y, p.z, p.offset_time, p.reflectivity, p.tag, p.line)
                    for p in scan.points
                ]
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
        "PASS: six native ROS LiDAR packets, 120 SI IMU samples, shared clock, identical PointCloud2, dynamic/static TF and clean shutdown"
    )


if __name__ == "__main__":
    main()
