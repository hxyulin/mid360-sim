# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Exercise actual DDS delivery with generated Livox Python messages and rclrs.

Run through the Pixi smoke task after building. A separate ROS domain isolates
this test. No driver, device, ROS daemon or running simulator is required.
"""
import argparse
import math
import os
from pathlib import Path
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--binary",
        type=Path,
        default=Path("target/release/mid360-sim"),
    )
    parser.add_argument("--stride", type=int, default=1)
    args = parser.parse_args()
    if args.stride < 1:
        parser.error("stride must be positive")
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ["ROS_DOMAIN_ID"] = "73"
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from livox_ros_driver2.msg import CustomMsg
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    node = rclpy.create_node("rm_simulator_livox_smoke")
    frames = []
    clocks = []

    def received(msg):
        assert msg.header.frame_id == "test_livox"
        assert msg.lidar_id == 9
        assert list(msg.rsvd) == [0, 0, 0]
        assert (
            msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            == msg.timebase
        )
        assert msg.point_num == len(msg.points) > 0
        assert all(
            a.offset_time < b.offset_time for a, b in zip(msg.points, msg.points[1:])
        )
        for p in msg.points:
            assert 0 <= p.offset_time < 100_000_000
            assert p.offset_time % 5_000 == 0
            assert 0 <= p.line < 4
            assert (
                p.line == ((msg.timebase - 1_000_000_000 + p.offset_time) // 5_000) % 4
            )
            assert p.tag == 0x10 and p.reflectivity == 100
            assert all(math.isfinite(v) for v in (p.x, p.y, p.z))
            assert math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z) >= 0.0999
        frames.append(msg)

    point_sub = node.create_subscription(
        CustomMsg, "/rm_simulator_test/livox", received, qos_profile_sensor_data
    )
    clock_sub = node.create_subscription(
        Clock,
        "/clock",
        lambda m: clocks.append(m.clock.sec * 1_000_000_000 + m.clock.nanosec),
        qos_profile_sensor_data,
    )
    proc = subprocess.Popen(
        [
            str(args.binary.resolve()),
            "--frames",
            "6",
            "--require-subscriber",
            "--wait-for-subscriber-ms",
            "5000",
            "--topic",
            "/rm_simulator_test/livox",
            "--stride",
            str(args.stride),
            "--frame-id",
            "test_livox",
            "--lidar-id",
            "9",
            "--range-noise-m",
            "0",
            "--dropout",
            "0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if (
                proc.poll() is not None
                and len(frames) == 6
                and clocks
                and clocks[-1] == 1_600_000_000
            ):
                break
            if proc.poll() not in (None, 0):
                raise RuntimeError(proc.stdout.read())
        output, _ = proc.communicate(timeout=5)
        assert proc.returncode == 0, output
        assert len(frames) == 6, f"received {len(frames)} / 6 frames; {output}"
        assert [m.timebase for m in frames] == [
            1_000_000_000
            + ((n * 20_000 + args.stride - 1) // args.stride) * args.stride * 5_000
            for n in range(6)
        ]
        assert clocks and clocks[-1] == 1_600_000_000
        assert all(a <= b for a, b in zip(clocks, clocks[1:]))
        print(output.strip())
        print(
            f"PASS: {len(frames)} real DDS CustomMsg packets, {sum(m.point_num for m in frames)} returns, {len(clocks)} clock messages; schema, offsets, lanes, tags, frame and device IDs verified"
        )
    finally:
        if proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=5)
        node.destroy_subscription(point_sub)
        node.destroy_subscription(clock_sub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
