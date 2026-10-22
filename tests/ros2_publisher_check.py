# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Received-field checks for the feature-gated Rust native publisher test."""

import struct
import sys
import time

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2

SIZES = [0, 1, 2, 20000, 3, 1, 0, 3, 2, 0]
FIELDS = [
    ("x", 0, 7, 1),
    ("y", 4, 7, 1),
    ("z", 8, 7, 1),
    ("offset_time", 12, 6, 1),
    ("reflectivity", 16, 2, 1),
    ("tag", 17, 2, 1),
    ("line", 18, 2, 1),
]


def stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def main():
    typed = sys.argv[2] == "typed"
    rclpy.init()
    node = rclpy.create_node("publisher_regression_check")
    livox, clouds, clocks = [], [], []

    def receive_cloud(msg):
        clouds.append(msg)
        print(f"CLOUD {len(clouds) - 1}", flush=True)

    subscriptions = [
        node.create_subscription(
            ty,
            topic,
            receive_cloud if dest is clouds else dest.append,
            qos_profile_sensor_data,
        )
        for ty, topic, dest in [
            (CustomMsg, sys.argv[1] + "/livox", livox),
            (PointCloud2, sys.argv[1] + "/cloud", clouds),
            (Clock, "/clock", clocks),
        ]
    ]
    try:
        deadline = time.monotonic() + 20
        acknowledged = 0
        report = time.monotonic() + 2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() >= report:
                print(
                    f"WAIT {sys.argv[2]} packet {acknowledged}: clouds={[stamp_ns(m.header.stamp) for m in clouds]}, livox={[m.timebase for m in livox]}, clocks={[stamp_ns(m.clock) for m in clocks]}",
                    flush=True,
                )
                report = time.monotonic() + 2
            # Let the sender pace field checks without silently retrying any loss.
            if acknowledged < len(SIZES):
                base = 2_000_000_000 + acknowledged * 100_000_000
                if (
                    len(clouds) > acknowledged
                    and any(stamp_ns(c.clock) == base for c in clocks)
                    and (
                        (SIZES[acknowledged] == 1 and not typed)
                        or any(m.timebase == base for m in livox)
                    )
                ):
                    print(f"ACK {acknowledged}", flush=True)
                    acknowledged += 1
            if len(clouds) >= len(SIZES) and len(clocks) >= len(SIZES):
                # Allow any unexpected singleton delivery to arrive, too.
                settle = time.monotonic() + 0.3
                while time.monotonic() < settle:
                    rclpy.spin_once(node, timeout_sec=0.05)
                break
        assert len(clouds) == len(SIZES), len(clouds)
        assert len(livox) == sum(typed or n != 1 for n in SIZES), len(livox)
        by_time = {x.timebase: x for x in livox}
        assert len(by_time) == len(livox)
        for i, (cloud, n) in enumerate(zip(clouds, SIZES)):
            base = 2_000_000_000 + i * 100_000_000
            assert stamp_ns(cloud.header.stamp) == base
            assert cloud.header.frame_id == f"regression_{i}"
            assert cloud.width == n and cloud.height == 1
            assert cloud.is_dense and not cloud.is_bigendian
            assert cloud.point_step == 20
            assert cloud.row_step == len(cloud.data) == n * 20
            assert [
                (f.name, f.offset, f.datatype, f.count) for f in cloud.fields
            ] == FIELDS
            assert any(stamp_ns(c.clock) == base for c in clocks)
            if n == 1 and not typed:
                assert base not in by_time
            else:
                msg = by_time[base]
                assert msg.header == cloud.header
                assert msg.point_num == len(msg.points) == n
                assert msg.lidar_id == 7 and list(msg.rsvd) == [0, 0, 0]
            data = bytes(cloud.data)
            for j in range(n):
                expected = (
                    2.0,
                    j + 1.0,
                    -1.0,
                    (j + 1) * 5000,
                    83,
                    0x10,
                    (i * 100 + j + 1) % 4,
                )
                assert struct.unpack_from("<fffIBBB", data, j * 20) == expected
                if n != 1 or typed:
                    p = msg.points[j]
                    assert (
                        p.x,
                        p.y,
                        p.z,
                        p.offset_time,
                        p.reflectivity,
                        p.tag,
                        p.line,
                    ) == expected
        print(
            f"PASS: {sys.argv[2]} native 0/1/2/20000/3 transitions, exact fields, clock and PointCloud2"
        )
    finally:
        del subscriptions
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
