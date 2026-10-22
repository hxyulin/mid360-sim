#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Bounded transport measurements; sensor stamps never become host timestamps.

HTTP mode starts one fresh external RM CAD host. ROS mode runs native rclrs
publishers in a separate process. All artifacts are local, never published.
"""

import argparse
import hashlib
import json
import math
import os
import platform
import queue
import resource
import signal
import struct
import subprocess
import sys
import threading
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import codec

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples/rm_sensors"))
from lio import start_rm
from live import post


def distribution(values):
    """Nearest-rank quantiles; no interpolation, filtering or tail extrapolation."""
    if not values:
        return {"n": 0}
    ordered = sorted(values)
    if not all(math.isfinite(v) for v in ordered):
        raise ValueError("nonfinite measurement")
    return {
        "n": len(ordered),
        "min": ordered[0],
        "p50": ordered[math.ceil(0.50 * len(ordered)) - 1],
        "p95": ordered[math.ceil(0.95 * len(ordered)) - 1],
        "p99": ordered[math.ceil(0.99 * len(ordered)) - 1],
        "max": ordered[-1],
    }


def clock_interval(samples):
    """Intersect Python-before/after minus Rust stamp to bound clock offset ns."""
    lower = max(before - stamp for before, after, stamp in samples)
    upper = min(after - stamp for before, after, stamp in samples)
    if lower > upper:
        raise ValueError("clock calibration intervals do not intersect")
    return (lower + upper) / 2, (upper - lower) / 2


def cpu_seconds(pid):
    """ps process CPU time, seconds; coarse (~10 ms), not per-thread CPU."""
    text = subprocess.check_output(
        ["ps", "-p", str(pid), "-o", "time="], text=True
    ).strip()
    days, text = text.split("-", 1) if "-" in text else (0, text)
    value = 0.0
    for field in text.split(":"):
        value = 60 * value + float(field)
    return int(days) * 86400 + value


def timed_poll(host, attachment):
    payload = json.dumps(attachment).encode()
    request = urllib.request.Request(
        host + "/api/sensors/poll",
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    start = time.monotonic_ns()
    with urllib.request.urlopen(request, timeout=3) as response:
        headers = time.monotonic_ns()
        data = response.read(32 * 1024 * 1024 + 1)
    read = time.monotonic_ns()
    if len(data) > 32 * 1024 * 1024:
        raise ValueError("response exceeds budget")
    batch = json.loads(data)
    decoded = time.monotonic_ns()
    return (
        batch,
        len(data),
        (headers - start) / 1e6,
        (read - headers) / 1e6,
        (decoded - read) / 1e6,
    )


def validate_metadata(frame, stride, epoch):
    """Check every IMU time, counts and three retained rays, outside JSON decode.

    Full per-point validation is provided by live.py, not added to transport CPU.
    """
    start, end = frame["start_ns"], frame["end_ns"]
    if end - start != 100_000_000 or start != epoch + frame["sequence"] * 100_000_000:
        raise ValueError("packet epoch changed")
    first = ((start - epoch) // 5000 + stride - 1) // stride * stride
    stop = (end - epoch) // 5000
    points = frame["lidar"]["points"]
    if len(points) != (stop - first + stride - 1) // stride or len(frame["imu"]) != 20:
        raise ValueError("incomplete retained emissions/IMU")
    for n in {0, len(points) // 2, len(points) - 1}:
        p = points[n]
        if (
            p["sample_index"] != first + n * stride
            or p["time_ns"] != epoch + (first + n * stride) * 5000
        ):
            raise ValueError("changed emission index/time")
    if [s["time_ns"] for s in frame["imu"]] != [
        start + (i + 1) * 5_000_000 for i in range(20)
    ]:
        raise ValueError("IMU cadence mismatch")


def codecs(batch, repeats):
    """Local Python object codec comparison, not Rust server encoding or a link."""
    methods = {
        "json": (
            lambda b: json.dumps(b, separators=(",", ":"), allow_nan=False).encode(),
            json.loads,
        ),
        "binary_prototype": (codec.encode, codec.decode),
    }
    report = {}
    for name, (encode, decode) in methods.items():
        data = encode(batch)
        if decode(data) != batch:
            raise ValueError("codec roundtrip changed measurements")
        enc, dec = [], []
        for _ in range(repeats):
            t = time.monotonic_ns()
            payload = encode(batch)
            after = time.monotonic_ns()
            restored = decode(payload)
            end = time.monotonic_ns()
            enc.append((after - t) / 1e6)
            dec.append((end - after) / 1e6)
            del restored
        report[name] = {
            "bytes": len(data),
            "encode_ms": distribution(enc),
            "decode_ms": distribution(dec),
        }
    report["scope"] = (
        "one captured single-frame bundle; local Python codecs, f64 SI, u64 times/indices, explicit misses; no network or Rust serialization claim"
    )
    return report


def http_run(args, out):
    server, host, robot = start_rm(args, out)
    attached, records = [], []
    pool = (
        ThreadPoolExecutor(max_workers=args.http_concurrency)
        if args.http_concurrency > 1
        else None
    )
    trace = []
    try:
        for _ in range(args.sensors):
            cfg = {"robot": robot, "stride": args.stride, "carrier": args.carrier}
            attached.append(post(host, "/api/sensors/attach", cfg)["id"])
            records.append(
                {
                    "received": 0,
                    "sequence": None,
                    "epoch": None,
                    "gaps": 0,
                    "drops": 0,
                    "age": [],
                    "discontinuities": 0,
                }
            )
        if args.motion != "stationary":
            post(
                host,
                "/api/command",
                {
                    "Chassis": {
                        "chassis": robot,
                        "command": {
                            "forward_m_s": 0.0,
                            "left_m_s": 0.0,
                            "yaw_rate_rad_s": 0.5,
                        },
                    }
                },
            )
        metrics = {
            name: []
            for name in [
                "headers_wait_ms",
                "body_read_ms",
                "json_decode_ms",
                "json_decode_nonempty_ms",
                "json_decode_empty_ms",
                "metadata_check_ms",
                "nonempty_http_ms",
                "empty_http_ms",
                "poll_cycle_ms",
            ]
        }
        requests = total_bytes = 0
        measurement_start = None
        representative = None
        start, cpu_start = time.monotonic(), time.process_time()
        server_cpu_start = cpu_seconds(server.pid)
        deadline = start + max(30.0, args.frames * 0.5)
        last_aim = -1.0
        target = args.frames + args.warmup
        while (
            min(r["received"] for r in records) < target and time.monotonic() < deadline
        ):
            cycle = time.monotonic_ns()
            elapsed = time.monotonic() - start
            if args.motion == "articulated" and elapsed - last_aim > 0.5:
                post(
                    host,
                    "/api/command",
                    {
                        "Chassis": {
                            "chassis": robot,
                            "command": {
                                "forward_m_s": 0.0,
                                "left_m_s": 0.0,
                                "yaw_rate_rad_s": 0.5,
                                "aim_yaw_rad": math.sin(elapsed),
                            },
                        }
                    },
                )
                last_aim = elapsed
            jobs = [(a, r) for a, r in zip(attached, records) if r["received"] < target]
            pending = (
                [pool.submit(timed_poll, host, a) for a, _ in jobs] if pool else None
            )
            for index, (attachment, r) in enumerate(jobs):
                active = all(item["received"] >= args.warmup for item in records)
                batch, n, headers, read, decode = (
                    pending[index].result() if pending else timed_poll(host, attachment)
                )
                checked = time.monotonic_ns()
                r["drops"] = max(r["drops"], batch["dropped_intervals"])
                for f in batch["frames"]:
                    if r["epoch"] is None:
                        r["epoch"] = f["start_ns"] - f["sequence"] * 100_000_000
                    validate_metadata(f, args.stride, r["epoch"])
                    if f["end_ns"] > batch["time_ns"]:
                        raise ValueError("measurement newer than host clock")
                    if r["sequence"] is not None:
                        if f["sequence"] <= r["sequence"]:
                            raise ValueError("nonmonotonic sequence")
                        r["gaps"] += f["sequence"] - r["sequence"] - 1
                    r["sequence"] = f["sequence"]
                    r["discontinuities"] = max(
                        r["discontinuities"], f["discontinuities"]
                    )
                    r["received"] += 1
                    if r["received"] > args.warmup:
                        r["age"].append((batch["time_ns"] - f["end_ns"]) / 1e6)
                        representative = {
                            "time_ns": batch["time_ns"],
                            "dropped_intervals": batch["dropped_intervals"],
                            "frames": [f],
                        }
                validation = (time.monotonic_ns() - checked) / 1e6
                if active:
                    if measurement_start is None:
                        measurement_start = (
                            time.monotonic()
                            - (headers + read + decode + validation) / 1000
                        )
                    requests += 1
                    total_bytes += n
                    for key, value in [
                        ("headers_wait_ms", headers),
                        ("body_read_ms", read),
                        ("json_decode_ms", decode),
                        ("metadata_check_ms", validation),
                    ]:
                        metrics[key].append(value)
                    metrics[
                        "json_decode_nonempty_ms"
                        if batch["frames"]
                        else "json_decode_empty_ms"
                    ].append(decode)
                    metrics[
                        "nonempty_http_ms" if batch["frames"] else "empty_http_ms"
                    ].append(headers + read)
                trace.append(
                    {
                        "attachment": attachment,
                        "observed_at_ns": time.monotonic_ns(),
                        "host_time_ns": batch["time_ns"],
                        "payload_bytes": n,
                        "headers_wait_ms": headers,
                        "body_read_ms": read,
                        "json_decode_ms": decode,
                        "metadata_ms": validation,
                        "drops": batch["dropped_intervals"],
                        "frames": [
                            [
                                f["sequence"],
                                f["start_ns"],
                                f["end_ns"],
                                f["discontinuities"],
                            ]
                            for f in batch["frames"]
                        ],
                    }
                )
            metrics["poll_cycle_ms"].append((time.monotonic_ns() - cycle) / 1e6)
            time.sleep(args.poll_ms / 1000)
        wall, client_cpu = time.monotonic() - start, time.process_time() - cpu_start
        server_cpu = cpu_seconds(server.pid) - server_cpu_start
        measured_s = (
            time.monotonic() - measurement_start
            if measurement_start is not None
            else 0.0
        )
        for attachment in attached:
            post(host, "/api/sensors/detach", attachment)
        attached.clear()
        status = (
            "pass"
            if all(
                r["received"] >= target and not r["drops"] and not r["gaps"]
                for r in records
            )
            else "loss_or_timeout"
        )
        result = {
            "status": status,
            "wall_s_including_warmup": wall,
            "client_cpu_s": client_cpu,
            "server_cpu_s_approx": server_cpu,
            "server_cpu_cores_approx": server_cpu / wall,
            "client_cpu_cores": client_cpu / wall,
            "requests_after_warmup": requests,
            "measurement_wall_s": measured_s,
            "response_payload_MiB_s": total_bytes / 2**20 / measured_s
            if measured_s
            else None,
            "response_payload_bytes_after_warmup": total_bytes,
            "stage_ms": {k: distribution(v) for k, v in metrics.items()},
            "streams": [
                {
                    "received_including_warmup": r["received"],
                    "packet_gaps": r["gaps"],
                    "dropped_intervals": r["drops"],
                    "input_discontinuities": r["discontinuities"],
                    "packet_end_simulation_age_ms": distribution(r["age"]),
                }
                for r in records
            ],
            "rm_commit": subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=args.rm_checkout, text=True
            ).strip(),
            "server_binary_sha256": hashlib.sha256(
                (args.rm_checkout / "target/nix/debug/rm-simulator-server").read_bytes()
            ).hexdigest(),
            "unavailable": "acquisition and output queue wall timing are unavailable. HTTP headers wait combines owner request wait, native JSON encoding, connection/handler and client scheduling; it does not time acquisition. Simulation age is not wall latency",
        }
    finally:
        if pool:
            pool.shutdown(wait=True, cancel_futures=True)
        for attachment in attached:
            try:
                post(host, "/api/sensors/detach", attachment)
            except OSError:
                pass
        if server.poll() is None:
            server.kill()
        server.wait(timeout=5)
    (out / "http.trace.json").write_text(json.dumps(trace) + "\n")
    # Keep Python codec work out of the live server/HTTP CPU and loss interval.
    if representative is not None:
        result["codec_microbenchmark"] = codecs(representative, args.codec_repeats)
        (out / "codec-sample.bin").write_bytes(codec.encode(representative))
    return result


def ros_run(args, out):
    import rclpy
    from livox_ros_driver2.msg import CustomMsg
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu, PointCloud2

    rclpy.init(args=[])
    node = rclpy.create_node("transport_receiver")
    received, inertial = {}, {}
    last = {}
    qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)

    def cloud(sensor, msg):
        now = time.monotonic_ns()
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if (sensor, stamp) in received:
            raise ValueError("duplicate cloud")
        n = len(msg.points) if args.format == "livox" else msg.width
        if msg.header.frame_id != f"sensor_{sensor}_lidar" or n == 0:
            raise ValueError("bad cloud frame/count")
        if args.format == "pointcloud" and (
            len(msg.data) != n * 20 or msg.point_step != 20 or msg.is_bigendian
        ):
            raise ValueError("cloud layout changed")
        received[sensor, stamp] = (now, n)
        last[sensor] = msg

    def imu(sensor, msg):
        now = time.monotonic_ns()
        ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if (sensor, ns) in inertial or msg.orientation_covariance[0] != -1:
            raise ValueError("duplicate or truth-oriented IMU")
        inertial[sensor, ns] = now

    for i in range(args.sensors):
        node.create_subscription(
            CustomMsg if args.format == "livox" else PointCloud2,
            f"/transport/sensor_{i}/"
            + ("livox" if args.format == "livox" else "points"),
            lambda m, i=i: cloud(i, m),
            qos,
        )
        node.create_subscription(
            Imu, f"/transport/sensor_{i}/imu", lambda m, i=i: imu(i, m), qos
        )
    binary = ROOT / "target/release/examples/transport"
    before_cpu = resource.getrusage(resource.RUSAGE_CHILDREN)
    log = (out / "native.stderr.log").open("w")
    child = subprocess.Popen(
        [
            str(binary),
            "--frames",
            str(args.frames + args.warmup),
            "--stride",
            str(args.stride),
            "--sensors",
            str(args.sensors),
            "--format",
            args.format,
            "--motion",
            args.motion,
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=log,
        text=True,
        bufsize=1,
    )
    lines = queue.Queue()

    def read_lines():
        for line in child.stdout:
            lines.put(line.rstrip())
        lines.put("EOF")

    reader = threading.Thread(target=read_lines, daemon=True)
    reader.start()
    events, transcript = [], []

    def line(timeout=12):
        value = lines.get(timeout=timeout)
        transcript.append(value)
        return value

    try:
        if line() != "READY":
            raise RuntimeError("native discovery failed; inspect native.stderr.log")
        calibration = []
        for _ in range(12):
            begin = time.monotonic_ns()
            child.stdin.write("PING\n")
            child.stdin.flush()
            value = line()
            end = time.monotonic_ns()
            if not value.startswith("PONG "):
                raise ValueError("clock handshake failed")
            calibration.append((begin, end, int(value.split()[1])))
        offset, uncertainty = clock_interval(calibration)
        start, cpu_start = time.monotonic(), time.process_time()
        child.stdin.write("GO\n")
        child.stdin.flush()
        deadline = start + (args.frames + args.warmup) * 0.25 + 15
        target = (args.frames + args.warmup) * args.sensors
        eof = False
        eof_at = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.001)
            while not lines.empty():
                value = line()
                if value == "EOF":
                    eof = True
                    eof_at = time.monotonic()
                elif value.startswith("EVENT "):
                    events.append(tuple(map(int, value.split()[1:])))
                else:
                    raise ValueError("unexpected native trace: " + value)
            if eof and len(received) >= target and len(inertial) >= target * 20:
                break
            if eof_at is not None and time.monotonic() - eof_at > 1:
                break
            if eof and child.poll() not in (None, 0):
                break
        wall, cpu = time.monotonic() - start, time.process_time() - cpu_start
        child.wait(timeout=3)
        after_cpu = resource.getrusage(resource.RUSAGE_CHILDREN)
        if child.returncode != 0:
            raise RuntimeError("native publisher failed or hung")
        # All validation/serialization below is outside receive timing and CPU.
        timing = {
            k: []
            for k in [
                "acquisition_ms",
                "packet_conversion_ms",
                "imu_burst_publish_ms",
                "cloud_publish_call_ms",
                "deadline_lateness_ms",
                "publish_begin_to_callback_ms",
                "acquired_to_callback_ms",
                "imu_publish_begin_to_callback_ms",
            ]
        }
        missing_clouds = missing_imu = 0
        for (
            sensor,
            frame,
            stamp,
            count,
            acq,
            acquired,
            converted,
            imu_begin,
            begin,
            done,
            scheduled,
        ) in events:
            data = received.get((sensor, stamp))
            expected = (
                20_000 * (frame + 1)
                - (20_000 * frame + args.stride - 1) // args.stride * args.stride
                + args.stride
                - 1
            ) // args.stride
            if count != expected or data is not None and data[1] != count:
                raise ValueError("retained cloud count changed")
            if data is None:
                missing_clouds += 1
            sample_times = [
                1_000_000_000 + frame * 100_000_000 + i * 5_000_000
                for i in range(1, 21)
            ]
            missing_imu += sum((sensor, t) not in inertial for t in sample_times)
            if frame < args.warmup:
                continue
            for k, v in [
                ("acquisition_ms", acquired - acq),
                ("packet_conversion_ms", converted - acquired),
                ("imu_burst_publish_ms", begin - imu_begin),
                ("cloud_publish_call_ms", done - begin),
                ("deadline_lateness_ms", acq - scheduled),
            ]:
                timing[k].append(v / 1e6)
            if data is not None:
                latency = data[0] - offset - begin
                if latency < -uncertainty:
                    raise ValueError(
                        "callback preceded publication beyond calibration uncertainty"
                    )
                timing["publish_begin_to_callback_ms"].append(latency / 1e6)
                timing["acquired_to_callback_ms"].append(
                    (data[0] - offset - acquired) / 1e6
                )
            timing["imu_publish_begin_to_callback_ms"].extend(
                (inertial[sensor, t] - offset - imu_begin) / 1e6
                for t in sample_times
                if (sensor, t) in inertial
            )
        for sensor, msg in last.items():
            if args.format == "pointcloud":
                if [(f.name, f.offset, f.datatype, f.count) for f in msg.fields] != [
                    ("x", 0, 7, 1),
                    ("y", 4, 7, 1),
                    ("z", 8, 7, 1),
                    ("offset_time", 12, 6, 1),
                    ("reflectivity", 16, 2, 1),
                    ("tag", 17, 2, 1),
                    ("line", 18, 2, 1),
                ]:
                    raise ValueError("point fields changed")
                for n in {0, msg.width // 2, msg.width - 1}:
                    x, y, z, dt, _, tag, lane = struct.unpack_from(
                        "<fffIBBBx", msg.data, n * 20
                    )
                    if (
                        not all(math.isfinite(v) for v in [x, y, z])
                        or not 0 <= dt < 100_000_000
                        or tag != 0x10
                        or not 0 <= lane < 4
                    ):
                        raise ValueError("bad binary point")
        result = {
            "status": "pass"
            if len(events) == target and not missing_clouds and not missing_imu
            else "loss_or_timeout",
            "clouds_received": len(received),
            "imu_received": len(inertial),
            "clouds_expected": target,
            "imu_expected": target * 20,
            "clouds_missing": missing_clouds,
            "imu_missing": missing_imu,
            "events": len(events),
            "wall_s_including_warmup": wall,
            "client_cpu_s": cpu,
            "client_cpu_cores": cpu / wall,
            "native_cpu_s_including_startup": after_cpu.ru_utime
            + after_cpu.ru_stime
            - before_cpu.ru_utime
            - before_cpu.ru_stime,
            "clock_offset_uncertainty_us": uncertainty / 1000,
            "stage_ms": {k: distribution(v) for k, v in timing.items()},
            "nominal_cloud_cdr_MiB_s": sum(
                len(serialize_message(msg)) for msg in last.values()
            )
            * 10
            / 2**20,
            "last_cloud_cdr_bytes": {
                str(i): len(serialize_message(msg)) for i, msg in last.items()
            },
            "native_binary_sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
            "scope": "separate native rclrs publisher/rclpy receiver, best effort, 100ms bundles; publisher call includes dynamic ROS encoding/RMW; no claim to isolate DDS wire or RMW queue; clocks calibrated by monotonic interval intersection",
        }
        (out / "native.trace.json").write_text(
            json.dumps(
                {
                    "clock_samples": calibration,
                    "events": events,
                    "receiver_clouds": [
                        [i, stamp, when, n]
                        for (i, stamp), (when, n) in sorted(received.items())
                    ],
                    "receiver_imu": [
                        [i, stamp, when]
                        for (i, stamp), when in sorted(inertial.items())
                    ],
                    "transcript": transcript,
                }
            )
            + "\n"
        )
        return result
    finally:
        if child.poll() is None:
            child.kill()
        child.wait(timeout=5)
        child.stdin.close()
        child.stdout.close()
        log.close()
        reader.join(timeout=1)
        node.destroy_node()
        rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mode", choices=["http", "ros"], required=True)
    p.add_argument("--output", type=Path, required=True, help="new directory")
    p.add_argument("--frames", type=int, default=60)
    p.add_argument("--warmup", type=int, default=5)
    p.add_argument("--stride", type=int, default=1)
    p.add_argument("--sensors", type=int, default=1)
    p.add_argument(
        "--motion", choices=["stationary", "yaw", "articulated"], default="articulated"
    )
    p.add_argument("--carrier", choices=["Chassis", "Gimbal"], default="Gimbal")
    p.add_argument("--format", choices=["livox", "pointcloud"], default="pointcloud")
    p.add_argument(
        "--rmw",
        choices=["rmw_cyclonedds_cpp", "rmw_fastrtps_cpp"],
        default="rmw_cyclonedds_cpp",
    )
    p.add_argument("--poll-ms", type=float, default=10.0)
    p.add_argument("--http-concurrency", type=int, choices=[1, 2, 4], default=1)
    p.add_argument(
        "--discovery-range",
        choices=["SYSTEM_DEFAULT", "LOCALHOST"],
        default="SYSTEM_DEFAULT",
    )
    p.add_argument("--watchdog-seconds", type=float, default=None)
    p.add_argument("--load-workers", type=int, default=0)
    p.add_argument("--codec-repeats", type=int, default=15)
    p.add_argument("--rm-checkout", type=Path)
    p.add_argument("--cad-assets", type=Path)
    p.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    p.add_argument("--burn", action="store_true", help=argparse.SUPPRESS)
    args = p.parse_args()
    if args.burn:
        block = b"x" * 65536
        while True:
            block = hashlib.sha256(block).digest() * 2048
    if args.watchdog_seconds is not None and not (
        math.isfinite(args.watchdog_seconds) and 0.05 <= args.watchdog_seconds <= 600
    ):
        p.error("watchdog must be finite, 0.05–600 seconds")
    if not (
        6 <= args.frames <= 600
        and 0 <= args.warmup <= 30
        and args.frames + args.warmup <= 600
        and 1 <= args.stride <= 1001
        and 1 <= args.sensors <= 4
        and 0 <= args.load_workers <= 8
        and 1 <= args.codec_repeats <= 100
        and math.isfinite(args.poll_ms)
        and 0 <= args.poll_ms <= 500
    ):
        p.error("invalid bounded workload")
    if args.mode == "http" and (args.rm_checkout is None or args.cad_assets is None):
        p.error("HTTP benchmark needs an external --rm-checkout and --cad-assets")
    if not args.worker:
        args.output.mkdir(parents=True, exist_ok=False)
        env = dict(
            os.environ,
            RMW_IMPLEMENTATION=args.rmw,
            ROS_DOMAIN_ID="82",
            ROS_AUTOMATIC_DISCOVERY_RANGE=args.discovery_range,
            CYCLONEDDS_URI="file://" + str(ROOT / "config/cyclonedds-loopback.xml"),
        )
        env.pop("ROS_LOCALHOST_ONLY", None)
        with (args.output / "worker.log").open("w") as log:
            child = subprocess.Popen(
                [
                    sys.executable,
                    str(Path(__file__).resolve()),
                    *sys.argv[1:],
                    "--worker",
                ],
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            timed_out = False
            try:
                child.wait(
                    timeout=args.watchdog_seconds
                    or ((args.frames + args.warmup) * 0.6 + 100)
                )
            except subprocess.TimeoutExpired:
                timed_out = True
            finally:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                child.wait()
        report_path = args.output / "report.json"
        if not report_path.exists() or timed_out:
            report = {
                "configuration": {
                    k: str(v) if isinstance(v, Path) else v
                    for k, v in vars(args).items()
                },
                "result": {
                    "status": "watchdog_timeout" if timed_out else "worker_error",
                    "worker_exit_code": child.returncode,
                },
            }
            report_path.write_text(json.dumps(report, indent=2) + "\n")
        elif child.returncode != 0:
            report = json.loads(report_path.read_text())
            if report["result"]["status"] == "pass":
                report["result"]["status"] = "worker_error_after_report"
                report_path.write_text(json.dumps(report, indent=2) + "\n")
        print((args.output / "worker.log").read_text())
        if timed_out:
            raise SystemExit("FAIL: transport process group exceeded watchdog")
        raise SystemExit(child.returncode)
    loads = []
    try:
        for _ in range(args.load_workers):
            loads.append(
                subprocess.Popen(
                    [
                        sys.executable,
                        str(Path(__file__).resolve()),
                        "--mode",
                        args.mode,
                        "--output",
                        str(args.output),
                        "--burn",
                    ]
                )
            )
        load_before = os.getloadavg()
        result = (http_run if args.mode == "http" else ros_run)(args, args.output)
        load_after = os.getloadavg()
        report = {
            "configuration": {
                k: str(v) if isinstance(v, Path) else v
                for k, v in vars(args).items()
                if k not in ("worker", "burn")
            },
            "platform": platform.platform(),
            "python": platform.python_version(),
            "cpu_count": os.cpu_count(),
            "load_average_before": load_before,
            "load_average_after": load_after,
            "source_files_sha256": {
                name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                for name in [
                    "examples/transport.rs",
                    "examples/transport/benchmark.py",
                    "examples/transport/codec.py",
                    "examples/synchronized/fixture.rs",
                    "src/publisher.rs",
                    "src/sensor.rs",
                ]
            },
            "discovery_range": os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE"),
            "cyclonedds_uri": os.environ.get("CYCLONEDDS_URI"),
            "cpu_model": subprocess.check_output(
                ["sysctl", "-n", "machdep.cpu.brand_string"], text=True
            ).strip()
            if sys.platform == "darwin"
            else platform.processor(),
            "source_commit": subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
            ).strip(),
            "source_dirty": bool(
                subprocess.check_output(
                    ["git", "status", "--porcelain"], cwd=ROOT, text=True
                )
            ),
            "result": result,
        }
        (args.output / "report.json").write_text(
            json.dumps(report, indent=2, allow_nan=False) + "\n"
        )
        print(json.dumps(report, indent=2))
        if result["status"] != "pass":
            raise SystemExit(1)
    finally:
        for child in loads:
            child.kill()
            child.wait(timeout=5)


if __name__ == "__main__":
    main()
