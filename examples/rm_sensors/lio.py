#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Run external TriLIO with native fixture or a fresh external RM CAD host.

No estimator source/dependency is bundled. Truth is used only for scoring.
Native DDS teardown is bounded by an outer process-group watchdog.
"""

import argparse
import bisect
import hashlib
import json
import math
import os
import re
import signal
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

from live import post
from ros import RosStream, inverse, multiply, pose_from_rm, rotate


def interpolate(rows, ns):
    """Interpolate metres and shortest-arc unit xyzw; never extrapolate truth."""
    j = bisect.bisect_left([r[0] for r in rows], ns)
    if j < len(rows) and rows[j][0] == ns:
        return rows[j][1:4], rows[j][4:8]
    if j == 0 or j == len(rows):
        raise ValueError("estimate outside recorded truth")
    a, b = rows[j - 1], rows[j]
    u = (ns - a[0]) / (b[0] - a[0])
    p = [(1 - u) * a[i] + u * b[i] for i in range(1, 4)]
    qa, qb = a[4:8], b[4:8]
    dot = sum(x * y for x, y in zip(qa, qb))
    if dot < 0:
        qb, dot = [-x for x in qb], -dot
    if dot > 0.9995:
        q = [(1 - u) * x + u * y for x, y in zip(qa, qb)]
        norm = math.sqrt(sum(x * x for x in q))
        q = [x / norm for x in q]
    else:
        angle = math.acos(min(1.0, dot))
        q = [
            (math.sin((1 - u) * angle) * x + math.sin(u * angle) * y) / math.sin(angle)
            for x, y in zip(qa, qb)
        ]
    return p, q


def score(truth, estimates):
    """One first-pose SE3 alignment; no scale, time shift or trajectory fitting."""
    if len(truth) < 2 or any(b[0] <= a[0] for a, b in zip(truth, truth[1:])):
        raise ValueError("truth needs increasing acquisition stamps")
    if not all(math.isfinite(v) for row in truth for v in row):
        raise ValueError("nonfinite truth")
    if any(
        abs(sum(v * v for v in row[4:8]) - 1.0) > 1e-6 for row in [*truth, *estimates]
    ):
        raise ValueError("pose orientation is not a unit quaternion")
    if len(estimates) < 20:
        raise ValueError("estimator produced fewer than 20 corrected poses")
    if any(b[0] <= a[0] for a, b in zip(estimates, estimates[1:])):
        raise ValueError("nonmonotonic estimator output")
    if not all(math.isfinite(v) for row in estimates for v in row):
        raise ValueError("nonfinite estimator output")
    p0, q0 = interpolate(truth, estimates[0][0])
    e0 = estimates[0]
    alignment = multiply(q0, inverse(e0[4:8]))
    distances, angles = [], []
    for e in estimates:
        p, q = interpolate(truth, e[0])
        shifted = rotate(alignment, [e[i] - e0[i] for i in range(1, 4)])
        distances.append(
            math.sqrt(sum((shifted[i] + p0[i] - p[i]) ** 2 for i in range(3)))
        )
        error = multiply(inverse(q), multiply(alignment, e[4:8]))
        angles.append(math.degrees(2 * math.acos(min(1.0, abs(error[3])))))
    return {
        "poses": len(estimates),
        "position_rmse_m": math.sqrt(sum(x * x for x in distances) / len(distances)),
        "position_max_m": max(distances),
        "rotation_rmse_deg": math.sqrt(sum(x * x for x in angles) / len(angles)),
        "rotation_max_deg": max(angles),
        "first_ns": estimates[0][0],
        "last_ns": estimates[-1][0],
        "alignment": "first output pose only; no scale/time/trajectory fit",
    }


def state(host):
    with urllib.request.urlopen(host + "/api/state", timeout=3) as response:
        return json.load(response)["field"]


def start_rm(args, output):
    checkout = args.rm_checkout.resolve()
    binary = checkout / "target/nix/debug/rm-simulator-server"
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        host = "http://127.0.0.1:" + str(sock.getsockname()[1])
    log = (output / "rm.log").open("w")
    child = subprocess.Popen(
        [
            str(binary),
            "--cad-assets",
            str(args.cad_assets.resolve()),
            "--no-referee",
            "--no-lobby",
            "--listen",
            "127.0.0.1:0",
            "--http",
            host.removeprefix("http://"),
        ],
        cwd=checkout,
        stdout=log,
        stderr=subprocess.STDOUT,
    )
    log.close()
    try:
        deadline = time.monotonic() + 45
        while True:
            try:
                state(host)
                break
            except OSError:
                if child.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("RM host failed; inspect rm.log")
                time.sleep(0.1)
        post(
            host,
            "/api/command",
            {
                "SpawnSentry": {
                    "team": "Red",
                    "position_m": [4.0, -3.0, 1.0],
                    "yaw_deg": 0.0,
                }
            },
        )
        robot = max(c["id"] for c in state(host)["chassis"])
        post(host, "/api/command", {"StopSentry": {"chassis": robot}})
        time.sleep(0.7)
        return child, host, robot
    except BaseException:
        child.kill()
        child.wait(timeout=5)
        raise


def run(args):
    import rclpy
    from nav_msgs.msg import Odometry
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage

    rclpy.init(args=[])
    node = rclpy.create_node("mid360_lio_regression")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    estimates = []
    joints = []

    def receive(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        estimates.append(
            [
                msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                p.x,
                p.y,
                p.z,
                q.x,
                q.y,
                q.z,
                q.w,
            ]
        )

    node.create_subscription(Odometry, "/lio/odometry", receive, 2000)
    node.create_subscription(
        TFMessage,
        "/tf",
        lambda msg: joints.extend(
            t for t in msg.transforms if t.child_frame_id == "rm_gimbal"
        ),
        100,
    )
    flush = node.create_client(Trigger, "/trident_lio_rust/flush")
    # Fixture's T_imu_lidar is inverse(T_lidar_imu), not the forward TF edge.
    pitch = -0.1 if args.source == "fixture" else 0.0
    offset = (
        rotate(
            [0.0, math.sin(pitch / 2), 0.0, math.cos(pitch / 2)], [-0.04, 0.02, -0.015]
        )
        if args.source == "fixture"
        else [0.0, 0.0, 0.0]
    )
    parameters = {
        "imu_topic": "/synchronized/imu",
        "lidar_topic": "/synchronized/points",
        "accel_scale": 1.0,
        "base_offset_z": 0.0,
        "extrinsic_pitch": pitch,
        "extrinsic_x": offset[0],
        "extrinsic_y": offset[1],
        "extrinsic_z": offset[2],
        "exit_after_flush": True,
        "use_sim_time": True,
        "gravity_magnitude_m_s2": 9.80665 if args.source == "fixture" else 9.81,
    }
    command = [str(args.estimator.resolve()), "--ros-args"]
    for name, value in parameters.items():
        command += [
            "-p",
            name
            + ":="
            + (str(value).lower() if isinstance(value, bool) else str(value)),
        ]
    children, logs = [], []
    truth, stream = [], None
    try:
        log = (output / "estimator.log").open("w")
        logs.append(log)
        estimator = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        children.append(estimator)
        deadline = time.monotonic() + 15
        while not flush.wait_for_service(timeout_sec=0.1):
            if estimator.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("estimator failed to start; inspect estimator.log")
        if args.source == "fixture":
            log = (output / "fixture.log").open("w")
            logs.append(log)
            fixture = subprocess.Popen(
                [
                    str(
                        Path(__file__).resolve().parents[2]
                        / "target/release/examples/synchronized"
                    ),
                    "--ros",
                    "--pointcloud",
                    "--tf",
                    "--reliable",
                    "--motion",
                    "lio",
                    "--frames",
                    str(args.frames),
                    "--stride",
                    str(args.stride),
                    "--truth",
                    str(output / "truth.tum"),
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            children.append(fixture)
            deadline = time.monotonic() + args.frames * 0.2 + 20
            while fixture.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
            if fixture.poll() != 0:
                raise RuntimeError("fixture failed or hung; inspect fixture.log")
            truth = [
                [int(v) if i == 0 else float(v) for i, v in enumerate(line.split())]
                for line in (output / "truth.tum").read_text().splitlines()
            ]
        else:
            server, host, robot = start_rm(args, output)
            children.append(server)
            config = {"robot": robot, "stride": args.stride, "carrier": args.carrier}
            stream = RosStream(node, config, publish_clock=True)
            discovery_deadline = time.monotonic() + 5
            while not (
                stream.imu.get_subscription_count()
                and stream.cloud.get_subscription_count()
            ):
                if (
                    estimator.poll() is not None
                    or time.monotonic() > discovery_deadline
                ):
                    raise RuntimeError(
                        "RM bridge did not discover both estimator inputs"
                    )
                rclpy.spin_once(node, timeout_sec=0.01)
            stream.publish_mounts()
            attachment = post(host, "/api/sensors/attach", config)["id"]
            try:
                deadline = time.monotonic() + args.frames * 0.3 + 20
                moving = False
                first_time = None
                while (
                    stream.validator.frames < args.frames
                    and time.monotonic() < deadline
                ):
                    snapshot = state(host)
                    c = next(c for c in snapshot["chassis"] if c["id"] == robot)
                    body, head = pose_from_rm(c["pose"]), pose_from_rm(c["turret"])
                    ns = snapshot["time_ns"]
                    stream.publish_joint(ns, body, head)
                    carrier = head if args.carrier == "Gimbal" else body
                    pos = rotate(carrier[1], [0.0, 0.0, 0.4])
                    row = [ns, *[carrier[0][i] + pos[i] for i in range(3)], *carrier[1]]
                    if not truth or ns > truth[-1][0]:
                        truth.append(row)
                    batch = post(host, "/api/sensors/poll", attachment)
                    stream.publish(batch)
                    if batch["frames"] and first_time is None:
                        first_time = batch["frames"][0]["start_ns"]
                    # Quiet initialization, then actual dynamics-driven rotation.
                    if (
                        first_time is not None
                        and ns - first_time > 3_000_000_000
                        and not moving
                    ):
                        post(
                            host,
                            "/api/command",
                            {
                                "Chassis": {
                                    "chassis": robot,
                                    "command": {
                                        "forward_m_s": args.forward_m_s,
                                        "left_m_s": 0.0,
                                        "yaw_rate_rad_s": 0.5,
                                    },
                                }
                            },
                        )
                        moving = True
                    rclpy.spin_once(node, timeout_sec=0.001)
                if (
                    stream.validator.frames < args.frames
                    or stream.dropped
                    or stream.validator.sequence_gaps
                ):
                    raise RuntimeError(
                        "incomplete/lost RM measurements: "
                        + json.dumps(
                            {**stream.validator.report(), "drops": stream.dropped}
                        )
                    )
                if args.carrier == "Chassis" and stream.validator.angular_peak < 0.1:
                    raise RuntimeError("RM motion failed")
                # Bracket the final emitted measurement, without extrapolating.
                snapshot = state(host)
                c = next(c for c in snapshot["chassis"] if c["id"] == robot)
                carrier = pose_from_rm(
                    c["turret"] if args.carrier == "Gimbal" else c["pose"]
                )
                pos = rotate(carrier[1], [0.0, 0.0, 0.4])
                if snapshot["time_ns"] > truth[-1][0]:
                    truth.append(
                        [
                            snapshot["time_ns"],
                            *[carrier[0][i] + pos[i] for i in range(3)],
                            *carrier[1],
                        ]
                    )
                (output / "truth.tum").write_text(
                    "".join(" ".join(map(str, row)) + "\n" for row in truth)
                )
            finally:
                post(host, "/api/sensors/detach", attachment)
        # Drain callbacks, flush the estimator's reorder tail, and bound teardown.
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
        future = flush.call_async(Trigger.Request())
        end = time.monotonic() + 15
        while estimator.poll() is None and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
        if estimator.poll() != 0 or not future.done() or not future.result().success:
            raise RuntimeError("estimator flush/shutdown failed")
        counts = re.search(
            r"Finished: (\d+) points, (\d+) corrections, (\d+) IMUs, (\d+) packets",
            (output / "estimator.log").read_text(),
        )
        expected_frames = stream.validator.frames if stream else args.frames
        if (
            not counts
            or int(counts[2]) == 0
            or (int(counts[3]), int(counts[4]))
            != (expected_frames * 20, expected_frames)
        ):
            raise ValueError(
                "estimator did not consume all clouds/IMUs or perform corrections"
            )
        if (
            args.source == "rm"
            and args.carrier == "Gimbal"
            and (
                len(joints) < 20
                or joints[0].transform.rotation == joints[-1].transform.rotation
            )
        ):
            raise ValueError("moving gimbal TF was missing or frozen")
        if (
            args.source == "rm"
            and args.forward_m_s != 0.0
            and max(math.dist(row[1:4], truth[0][1:4]) for row in truth) < 0.1
        ):
            raise ValueError("RM forward request did not produce 0.1 m displacement")
        metrics = score(truth, estimates)
        duration = (truth[-1][0] - truth[0][0]) * 1e-9
        if (metrics["last_ns"] - metrics["first_ns"]) * 1e-9 < duration - 2.0:
            raise ValueError("estimator output does not cover the run")
        if (
            metrics["position_rmse_m"] > args.max_position_rmse
            or metrics["rotation_rmse_deg"] > args.max_rotation_rmse
        ):
            raise ValueError(
                "LIO error exceeded regression limits: " + json.dumps(metrics)
            )
        report = {
            "source": args.source,
            "estimator_input_counts": {
                "imu": int(counts[3]),
                "clouds": int(counts[4]),
                "corrections": int(counts[2]),
            },
            "joint_tf_samples": len(joints),
            "carrier": args.carrier,
            "forward_request_m_s": args.forward_m_s,
            "frames": args.frames,
            "stride": args.stride,
            "rm_commit": subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=args.rm_checkout, text=True
            ).strip()
            if args.source == "rm"
            else None,
            "estimator_sha256": hashlib.sha256(args.estimator.read_bytes()).hexdigest(),
            "parameters": parameters,
            "metrics": metrics,
            "truth_usage": "post-hoc scoring only; physical relative extrinsics/joints are separate",
            "truth_max_displacement_m": max(
                math.dist(row[1:4], truth[0][1:4]) for row in truth
            ),
            "truth_max_spacing_ms": max(b[0] - a[0] for a, b in zip(truth, truth[1:]))
            / 1e6,
            "measurements": stream.validator.report() if stream else None,
        }
        (output / "estimated.tum").write_text(
            "".join(" ".join(map(str, row)) + "\n" for row in estimates)
        )
        (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
    finally:
        for child in reversed(children):
            if child.poll() is None:
                child.kill()
            child.wait(timeout=5)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--estimator", type=Path, required=True)
    parser.add_argument(
        "--output", type=Path, required=True, help="new output directory"
    )
    parser.add_argument("--source", choices=["fixture", "rm"], default="fixture")
    parser.add_argument("--rm-checkout", type=Path)
    parser.add_argument("--cad-assets", type=Path)
    parser.add_argument("--carrier", choices=["Chassis", "Gimbal"], default="Chassis")
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--stride", type=int, default=3)
    parser.add_argument(
        "--forward-m-s",
        type=float,
        default=0.0,
        help="RM forward request after quiet interval, metres/s; |value| <=0.5",
    )
    parser.add_argument("--max-position-rmse", type=float, default=0.15)
    parser.add_argument("--max-rotation-rmse", type=float, default=3.0)
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not all(
        math.isfinite(v) and v > 0
        for v in (args.max_position_rmse, args.max_rotation_rmse)
    ):
        parser.error("regression limits must be finite and positive")
    if not math.isfinite(args.forward_m_s) or abs(args.forward_m_s) > 0.5:
        parser.error("forward request must be finite, within +/-0.5 m/s")
    if not 80 <= args.frames <= 600 or not 1 <= args.stride <= 1001:
        parser.error("frames must be 80–600; stride 1–1001")
    if args.source == "rm" and (args.rm_checkout is None or args.cad_assets is None):
        parser.error("RM evaluation needs --rm-checkout and --cad-assets")
    if not args.worker:
        root = Path(__file__).resolve().parents[2]
        env = dict(
            os.environ,
            RMW_IMPLEMENTATION="rmw_cyclonedds_cpp",
            ROS_DOMAIN_ID="81",
            ROS_AUTOMATIC_DISCOVERY_RANGE="SYSTEM_DEFAULT",
            CYCLONEDDS_URI="file://" + str(root / "config/cyclonedds-loopback.xml"),
        )
        env.pop("ROS_LOCALHOST_ONLY", None)
        child = subprocess.Popen(
            [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:], "--worker"],
            env=env,
            start_new_session=True,
        )
        try:
            child.wait(timeout=args.frames * 0.4 + 100)
        except subprocess.TimeoutExpired:
            raise SystemExit("LIO process group exceeded watchdog")
        finally:
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            child.wait()
        raise SystemExit(child.returncode)
    run(args)


if __name__ == "__main__":
    main()
