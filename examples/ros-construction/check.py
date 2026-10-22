# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Exact received-field check for the isolated typed construction prototype."""

import struct
import sys
import time

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.qos import QoSProfile, ReliabilityPolicy


def main():
    rclpy.init()
    node = rclpy.create_node("typed_construction_check")
    received = []
    subscription = node.create_subscription(
        CustomMsg,
        sys.argv[1],
        received.append,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
    )
    sizes = [0, 1, 2, 20000, 3, 0, 1]
    try:
        deadline = time.monotonic() + 20
        while len(received) < len(sizes) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert len(received) == len(sizes), len(received)
        for i, (msg, n) in enumerate(zip(received, sizes)):
            assert msg.header.stamp.sec == 3 + i and msg.header.stamp.nanosec == 0
            assert msg.timebase == (3 + i) * 10**9
            assert msg.header.frame_id == "benchmark_lidar"
            assert msg.point_num == len(msg.points) == n
            assert msg.lidar_id == 7 and list(msg.rsvd) == [0, 0, 0]
            sigma = struct.unpack("f", struct.pack("f", 0.001))[0]
            for j, p in enumerate(msg.points):
                expected_y = struct.unpack("f", struct.pack("f", j * sigma))[0]
                assert (p.x, p.y, p.z) == (2.0, expected_y, -1.0)
                assert p.offset_time == (j + 1) * 5000
                assert (p.reflectivity, p.tag, p.line) == (83, 0x10, j % 4)
        print("PASS: typed native fields and 0/1/2/20000/3/0/1 transitions")
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
