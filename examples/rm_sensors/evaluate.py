#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Evaluate a fresh external CAD host; never change an existing running match."""

import argparse
import json
from pathlib import Path
import socket
import subprocess
import tempfile
import time
import urllib.error
import urllib.request
from live import Validator, collect, post


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rm-checkout", type=Path, required=True)
    parser.add_argument("--cad-assets", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if not 6 <= args.frames <= 100:
        parser.error("evaluation frames must be 6–100")
    checkout = args.rm_checkout.resolve()
    binary = checkout / "target/nix/debug/rm-simulator-server"
    if not binary.is_file():
        parser.error(
            "build the external host with nix develop -c cargo build -p rm-simulator-server --locked"
        )
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        address = f"http://127.0.0.1:{sock.getsockname()[1]}"
    reports = {}
    with tempfile.TemporaryFile(mode="w+") as log:
        server = subprocess.Popen(
            [
                str(binary),
                "--cad-assets",
                str(args.cad_assets.resolve()),
                "--no-referee",
                "--no-lobby",
                "--listen",
                "127.0.0.1:0",
                "--http",
                address.removeprefix("http://"),
            ],
            cwd=checkout,
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        try:
            deadline = time.monotonic() + 45
            while True:
                try:
                    post(address, "/api/sensors/attach", {"robot": 999999})
                except urllib.error.HTTPError as e:
                    if e.code == 400:
                        break
                    raise
                except OSError:
                    if server.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError("external CAD host failed to start")
                    time.sleep(0.1)
            post(
                address,
                "/api/command",
                {
                    "SpawnSentry": {
                        "team": "Red",
                        "position_m": [4.0, -3.0, 1.0],
                        "yaw_deg": 0.0,
                    }
                },
            )
            # Sentry is the second chassis in this fresh host. Read its addressed ID
            # through the operator state only for attachment, never sensor synthesis.
            with urllib.request.urlopen(address + "/api/state", timeout=3) as response:
                robot = max(c["id"] for c in json.load(response)["field"]["chassis"])
            post(address, "/api/command", {"StopSentry": {"chassis": robot}})
            time.sleep(0.5)
            for moving, stride, carrier in [
                (False, 1, "Chassis"),
                (True, 1, "Chassis"),
                (True, 3, "Chassis"),
                (True, 3, "Gimbal"),
            ]:
                post(
                    address,
                    "/api/command",
                    {
                        "Chassis": {
                            "chassis": robot,
                            "command": {
                                "forward_m_s": 0.0,
                                "left_m_s": 0.0,
                                "yaw_rate_rad_s": 0.5 if moving else 0.0,
                            },
                        }
                    },
                )
                config = {"robot": robot, "stride": stride, "carrier": carrier}
                attachment = post(address, "/api/sensors/attach", config)["id"]
                try:
                    result = collect(
                        address,
                        attachment,
                        config,
                        args.frames,
                        timeout=max(15, args.frames * 0.4),
                    )
                    if not 3 < result["median_specific_force_m_s2"] < 30:
                        raise ValueError(
                            "specific force does not have SI stationary-gravity scale"
                        )
                    if (
                        moving
                        and carrier == "Chassis"
                        and result["peak_angular_rate_rad_s"] < 0.1
                    ):
                        raise ValueError(
                            "requested moving case did not produce measured rotation"
                        )
                    if carrier == "Gimbal" and result["peak_angular_rate_rad_s"] > 0.1:
                        raise ValueError(
                            "world-stabilized gimbal unexpectedly follows body spin"
                        )
                    key = f"{'moving' if moving else 'stationary'}_{carrier.lower()}_stride_{stride}"
                    reports[key] = result
                finally:
                    post(address, "/api/sensors/detach", attachment)
            # A changing requested head angle must reach the measured actual carrier.
            post(
                address,
                "/api/command",
                {
                    "Chassis": {
                        "chassis": robot,
                        "command": {
                            "forward_m_s": 0.0,
                            "left_m_s": 0.0,
                            "yaw_rate_rad_s": 0.5,
                            "aim_yaw_rad": 2.0,
                        },
                    }
                },
            )
            config = {"robot": robot, "stride": 3, "carrier": "Gimbal"}
            attachment = post(address, "/api/sensors/attach", config)["id"]
            try:
                result = collect(address, attachment, config, 6)
                assert result["peak_angular_rate_rad_s"] > 0.1, (
                    "moving gimbal did not reach the IMU carrier"
                )
                reports["moving_gimbal"] = result
            finally:
                post(address, "/api/sensors/detach", attachment)
            config = {"robot": robot, "stride": 3}
            attachment = post(address, "/api/sensors/attach", config)["id"]
            try:
                continuous = Validator(config)
                collect(address, attachment, config, 6, validator=continuous)
                post(address, "/api/command", {"Pause": {"paused": True}})
                time.sleep(0.1)
                for _ in range(3):
                    frozen = post(address, "/api/sensors/poll", attachment)
                    time.sleep(0.03)
                for _ in range(5):
                    batch = post(address, "/api/sensors/poll", attachment)
                    assert (
                        batch["time_ns"] == frozen["time_ns"] and not batch["frames"]
                    ), "pause generated advancing measurements"
                    time.sleep(0.03)
                post(address, "/api/command", {"Pause": {"paused": False}})
                reports["resume"] = collect(
                    address, attachment, config, 6, validator=continuous
                )
                # Deliberately stop polling while the same world advances. Output
                # must remain bounded and report loss, rather than block the host.
                time.sleep(0.85)
                backlog = post(address, "/api/sensors/poll", attachment)
                assert len(backlog["frames"]) <= 2 and backlog["dropped_intervals"] > 0
                validator = Validator(config)
                for frame in backlog["frames"]:
                    validator.accept(frame, backlog["time_ns"])
                reports["pressure"] = {
                    "queued_frames": len(backlog["frames"]),
                    "dropped_intervals": backlog["dropped_intervals"],
                    "host_time_ns": backlog["time_ns"],
                }
            finally:
                post(address, "/api/sensors/detach", attachment)
            try:
                post(address, "/api/sensors/poll", attachment)
                raise AssertionError("detached stream remained accessible")
            except urllib.error.HTTPError as e:
                assert e.code == 404
            attachment = post(address, "/api/sensors/attach", config)["id"]
            try:
                continuous = Validator(config)
                collect(address, attachment, config, 6, validator=continuous)
                post(
                    address,
                    "/api/command",
                    {"SpawnBot": {"team": "Blue", "spin_rad_s": 0.0}},
                )
                reports["topology_recovery"] = collect(
                    address,
                    attachment,
                    config,
                    6,
                    allow_drops=True,
                    validator=continuous,
                )
                assert reports["topology_recovery"]["input_discontinuities"] > 1
                assert reports["topology_recovery"]["dropped_intervals"] > 0
                assert reports["topology_recovery"]["packet_gaps"] > 0
            finally:
                post(address, "/api/sensors/detach", attachment)
            post(address, "/api/command", {"Pause": {"paused": True}})
            attachment = post(address, "/api/sensors/attach", config)["id"]
            time.sleep(5.3)
            try:
                post(address, "/api/sensors/poll", attachment)
                raise AssertionError("abandoned paused attachment did not expire")
            except urllib.error.HTTPError as e:
                assert e.code == 404
            reports["expiry_while_paused"] = "passed"
            reports["pause_resume_and_detach"] = "passed"
            reports["rm_commit"] = subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=checkout, text=True
            ).strip()
            result = json.dumps(reports, indent=2, allow_nan=False)
            print(result)
            if args.report:
                args.report.parent.mkdir(parents=True, exist_ok=True)
                args.report.write_text(result + "\n")
        finally:
            if server.poll() is None:
                server.kill()
                server.wait(timeout=5)
            log.seek(0)
            print(log.read())


if __name__ == "__main__":
    main()
