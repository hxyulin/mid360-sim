# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Bounded matched native ROS comparison; run in the activated ROS environment."""

import argparse
import fcntl
import json
import os
import platform
import queue
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path

import psutil
import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message


def fields(msg, sensor, indices):
    return {
        "stamp": msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
        "timebase": msg.timebase,
        "frame": msg.header.frame_id,
        "count": msg.point_num,
        "length": len(msg.points),
        "id": msg.lidar_id,
        "reserved": list(msg.rsvd),
        "points": [
            (i, p.offset_time, p.x, p.y, p.z, p.reflectivity, p.tag, p.line)
            for i in indices
            for p in [msg.points[i]]
        ],
    }


def check(value, n, sensor, sequence):
    stamp = 2 * 10**9 + sequence * 100_000_000
    assert value["stamp"] == value["timebase"] == stamp, value
    assert value["frame"] == f"matched_{sensor}" and value["id"] == sensor
    assert value["count"] == value["length"] == n
    assert value["reserved"] == [0, 0, 0]
    stride = 3 if n == 6667 else 1
    for i, offset, x, y, z, reflectivity, tag, line in value["points"]:
        assert (offset, x, y, z, reflectivity, tag, line) == (
            i * stride * 5000,
            sequence + 0.25,
            i * 0.125,
            -sensor - 1,
            sequence % 256,
            0x10,
            (i * stride) % 4,
        ), (sequence, sensor, i)


def percentile(values, fraction):
    return sorted(values)[min(len(values) - 1, int((len(values) - 1) * fraction))]


def run(
    binary,
    node,
    clock,
    kind,
    n,
    sensors,
    packets,
    pair,
    consumer,
    reliability,
    depth,
    spacing_ms,
):
    prefix = f"/matched_{os.getpid()}_{pair}_{kind}_{n}_{sensors}"
    startup, received = {}, []
    measuring = False

    def callback(sensor, msg):
        arrival = clock.now().nanoseconds
        if consumer == "raw":
            if measuring:
                received.append((sensor, arrival, msg))
                return
            msg = deserialize_message(msg, CustomMsg)
        sequence = (msg.timebase - 2 * 10**9) // 100_000_000
        if sequence == 0:
            startup[sensor] = msg
        else:
            # Full native deserialization has occurred before callback entry.
            # Capture a small changing-field sample; assertions run after timing.
            received.append(
                (sequence, sensor, arrival, fields(msg, sensor, [0, n // 2, n - 1]))
            )

    qos = QoSProfile(
        depth=depth,
        reliability=(
            ReliabilityPolicy.RELIABLE
            if reliability == "reliable"
            else ReliabilityPolicy.BEST_EFFORT
        ),
        durability=DurabilityPolicy.VOLATILE,
    )
    subscriptions = [
        node.create_subscription(
            CustomMsg,
            f"{prefix}_{sensor}",
            lambda msg, sensor=sensor: callback(sensor, msg),
            qos,
            raw=consumer == "raw",
        )
        for sensor in range(sensors)
    ]
    child = subprocess.Popen(
        [
            str(binary),
            "--matched",
            kind,
            str(n),
            str(sensors),
            str(packets),
            prefix,
            reliability,
            str(depth),
            str(spacing_ms),
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
    )
    events = queue.Queue()

    def read_output():
        for line in child.stdout:
            events.put(json.loads(line))

    reader = threading.Thread(target=read_output, daemon=True)
    reader.start()
    proc = psutil.Process(child.pid)
    try:
        deadline = time.monotonic() + 20
        ready = False
        while not ready or len(startup) != sensors:
            rclpy.spin_once(node, timeout_sec=0.01)
            while not events.empty():
                ready = events.get_nowait()["event"] == "ready"
            assert child.poll() is None, "publisher exited during startup"
            assert time.monotonic() < deadline, (
                "startup packet lost / discovery timeout (no retry)"
            )
        for sensor, msg in startup.items():
            check(fields(msg, sensor, range(n)), n, sensor, 0)
        startup.clear()
        publisher_cpu_start = sum(proc.cpu_times()[:2])
        receiver_cpu_start = time.process_time()
        measuring = True
        child.stdin.write("go\n")
        child.stdin.flush()
        done = None
        deadline = time.monotonic() + packets * 0.1 + 10
        while done is None:
            rclpy.spin_once(node, timeout_sec=0.005)
            if not events.empty():
                done = events.get_nowait()
            assert time.monotonic() < deadline, "publisher timeout"
            assert child.poll() is None, "publisher exited during measurement"
        publisher_cpu = sum(proc.cpu_times()[:2]) - publisher_cpu_start
        receiver_active_cpu = time.process_time() - receiver_cpu_start
        # A fixed drain allows delayed callbacks, without affecting send cadence.
        drain_end = time.monotonic() + 0.3
        while time.monotonic() < drain_end:
            rclpy.spin_once(node, timeout_sec=0.005)
        receiver_cpu = time.process_time() - receiver_cpu_start
        child.stdin.write("exit\n")
        child.stdin.flush()
        assert child.wait(timeout=10) == 0
        if consumer == "raw":
            decoded = []
            for sensor, arrival, payload in received:
                msg = deserialize_message(payload, CustomMsg)
                seq = (msg.timebase - 2 * 10**9) // 100_000_000
                decoded.append(
                    (seq, sensor, arrival, fields(msg, sensor, [0, n // 2, n - 1]))
                )
            received = decoded
        sends = {
            (seq, sensor): (before, after, lateness)
            for seq, sensor, before, after, lateness in done["samples"]
        }
        seen, latencies = set(), []
        duplicates = 0
        for seq, sensor, arrival, value in received:
            assert (seq, sensor) in sends, "unexpected sequence"
            check(value, n, sensor, seq)
            duplicates += (seq, sensor) in seen
            seen.add((seq, sensor))
            latency = (arrival - sends[seq, sensor][0]) / 1e6
            assert latency >= 0, "steady clocks disagree"
            latencies.append(latency)
        publish_ms = [(after - before) / 1e6 for before, after, _ in sends.values()]
        set_ms = [
            sum(
                (sends[seq, sensor][1] - sends[seq, sensor][0]) / 1e6
                for sensor in range(sensors)
            )
            for seq in range(1, packets + 1)
        ]
        return {
            "kind": kind,
            "points": n,
            "sensors": sensors,
            "pair": pair,
            "consumer": consumer,
            "reliability": reliability,
            "depth": depth,
            "spacing_ms": spacing_ms,
            "sent": packets * sensors,
            "received": len(received),
            "unique": len(seen),
            "drops": packets * sensors - len(seen),
            "duplicates": duplicates,
            "elapsed_ns": done["elapsed_ns"],
            "publisher_interval_cpu_s": publisher_cpu,
            "receiver_active_cpu_s": receiver_active_cpu,
            "receiver_with_drain_cpu_s": receiver_cpu,
            "transport_cpu_s": publisher_cpu + receiver_active_cpu,
            "publish_call_median_ms": statistics.median(publish_ms),
            "publish_set_median_ms": statistics.median(set_ms),
            "latency_median_ms": statistics.median(latencies) if latencies else None,
            "latency_p95_ms": percentile(latencies, 0.95) if latencies else None,
            "sends": done["samples"],
            "arrivals": [[s, i, a] for s, i, a, _ in received],
        }
    finally:
        if child.poll() is None:
            child.kill()
            child.wait()
        for subscription in subscriptions:
            node.destroy_subscription(subscription)


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--lock", type=Path, required=True)
    parser.add_argument("--pairs", type=int, default=3)
    parser.add_argument("--packets", type=int, default=30)
    parser.add_argument(
        "--points", type=int, nargs="+", choices=[6667, 20000], default=[6667, 20000]
    )
    parser.add_argument(
        "--sensors", type=int, nargs="+", choices=[1, 2], default=[1, 2]
    )
    parser.add_argument("--consumer", choices=["python", "raw"], default="python")
    parser.add_argument(
        "--reliability", choices=["best_effort", "reliable"], default="best_effort"
    )
    parser.add_argument("--depth", type=int, default=5)
    parser.add_argument("--spacing-ms", type=int, default=0)
    args = parser.parse_args()
    assert 1 <= args.pairs <= 10 and 2 <= args.packets <= 300
    assert 1 <= args.depth <= 1000 and 0 <= args.spacing_ms < 100
    args.lock.parent.mkdir(parents=True, exist_ok=True)
    with args.lock.open("a") as lock:
        print("Waiting for exclusive timing lock", file=sys.stderr)
        fcntl.flock(lock, fcntl.LOCK_EX)
        rclpy.init()
        node = rclpy.create_node(f"matched_receiver_{os.getpid()}")
        clock = Clock(clock_type=ClockType.STEADY_TIME)
        try:
            print(
                json.dumps(
                    {
                        "platform": platform.platform(),
                        "machine": platform.machine(),
                        "rmw": os.getenv("RMW_IMPLEMENTATION"),
                        "domain": os.getenv("ROS_DOMAIN_ID"),
                        "pairs": args.pairs,
                        "packets": args.packets,
                        "qos": f"{args.reliability}/volatile/keep_last={args.depth}",
                        "consumer": args.consumer,
                        "spacing_ms": args.spacing_ms,
                        "rate_hz": 10,
                    }
                ),
                flush=True,
            )
            for n in args.points:
                for sensors in args.sensors:
                    for pair in range(args.pairs):
                        for kind in (
                            ["dynamic", "typed"]
                            if pair % 2 == 0
                            else ["typed", "dynamic"]
                        ):
                            result = run(
                                args.binary,
                                node,
                                clock,
                                kind,
                                n,
                                sensors,
                                args.packets,
                                pair,
                                args.consumer,
                                args.reliability,
                                args.depth,
                                args.spacing_ms,
                            )
                            print(json.dumps(result), flush=True)
                            print(
                                f"{n}/{sensors}/{pair}/{kind}: {result['unique']}/{result['sent']}",
                                file=sys.stderr,
                                flush=True,
                            )
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
