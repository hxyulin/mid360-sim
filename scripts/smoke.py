# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Exercise actual ROS middleware delivery with generated Livox Python messages and rclrs.

Run through the Pixi smoke task after building. A separate ROS domain isolates
this test. No driver, device, ROS daemon or running simulator is required.
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--binary",
        type=Path,
        default=Path("target/release/mid360-sim"),
    )
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--frames", type=int, default=6)
    parser.add_argument("--angular-noise-deg", type=float, default=0.0)
    parser.add_argument(
        "--pattern",
        choices=["nominal", "official-table", "hardware-mean", "hardware-capture"],
        default="nominal",
    )
    parser.add_argument(
        "--rmw",
        default=os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        help="installed ROS middleware shared by publisher and subscriber",
    )
    parser.add_argument("--domain-id", type=int, default=73)
    parser.add_argument(
        "--discovery-range",
        choices=["LOCALHOST", "SUBNET", "SYSTEM_DEFAULT"],
        default="LOCALHOST",
    )
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not 1 <= args.frames <= 100:
        parser.error("frames must be in [1, 100] (bounded smoke duration)")
    if args.stride < 1:
        parser.error("stride must be positive")
    if (
        not math.isfinite(args.angular_noise_deg)
        or not 0 <= args.angular_noise_deg <= 180
    ):
        parser.error("angular noise must be finite and in [0, 180] degrees")
    if not 0 <= args.domain_id <= 232:
        parser.error("domain ID must be in [0, 232]")
    if not args.worker:
        # Bound the entire lifecycle, including rclpy destruction and native RMW
        # teardown. A worker finally block cannot interrupt a stuck C destructor.
        with tempfile.TemporaryFile(mode="w+t") as output:
            worker = subprocess.Popen(
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
                worker.wait(timeout=40)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(worker.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                worker.wait()
                output.seek(0)
                print(output.read(), end="", flush=True)
                raise SystemExit(
                    "FAIL: transport lifecycle exceeded 40 s; worker group killed"
                )
            finally:
                # Also reap descendants if the worker failed before its cleanup.
                try:
                    os.killpg(worker.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            output.seek(0)
            print(output.read(), end="", flush=True)
            raise SystemExit(worker.returncode)
    print(
        f"Transport: {args.rmw}; domain {args.domain_id}; discovery {args.discovery_range}",
        flush=True,
    )
    os.environ["RMW_IMPLEMENTATION"] = args.rmw
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = args.discovery_range
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from livox_ros_driver2.msg import CustomMsg
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    node = rclpy.create_node("rm_simulator_livox_smoke")
    frames = []
    clocks = []

    def validate_frame(msg):
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

    def received(msg):
        # Keep field validation off the receive path: a dense packet contains
        # 20,000 Python point objects. Inline checks can starve both subscriptions.
        assert len(frames) < args.frames, "received an unexpected extra frame"
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
            str(args.frames),
            "--pattern",
            args.pattern,
            "--require-subscriber",
            "--wait-for-subscriber-ms",
            "5000",
            "--topic",
            "/rm_simulator_test/livox",
            "--stride",
            str(args.stride),
            "--angular-noise-deg",
            str(args.angular_noise_deg),
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
                and len(frames) == args.frames
                and clocks
                and clocks[-1] == 1_000_000_000 + args.frames * 100_000_000
            ):
                break
            if proc.poll() not in (None, 0):
                raise RuntimeError(proc.stdout.read())
        output, _ = proc.communicate(timeout=5)
        assert proc.returncode == 0, output
        assert len(frames) == args.frames, (
            f"received {len(frames)} / {args.frames} frames; {output}"
        )
        for msg in frames:
            validate_frame(msg)
        assert [m.timebase for m in frames] == [
            1_000_000_000
            + ((n * 20_000 + args.stride - 1) // args.stride) * args.stride * 5_000
            for n in range(args.frames)
        ]
        assert clocks and clocks[-1] == 1_000_000_000 + args.frames * 100_000_000
        assert all(a <= b for a, b in zip(clocks, clocks[1:]))
        print(output.strip())
        print(
            f"Validated: {len(frames)} real ROS CustomMsg packets, {sum(m.point_num for m in frames)} returns, {len(clocks)} clock messages; schema, offsets, lanes, tags, frame and device IDs verified"
        )
    finally:
        print(
            f"Received {len(frames)} / {args.frames} frames; "
            f"{len(clocks)} clocks; destroying endpoints",
            flush=True,
        )
        if proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=5)
        node.destroy_subscription(point_sub)
        node.destroy_subscription(clock_sub)
        node.destroy_node()
        rclpy.shutdown()
    print("PASS: message checks and clean publisher/subscriber shutdown", flush=True)


if __name__ == "__main__":
    main()
