# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Extract bounded sensor-frame samples from LVX2 or ROS1 Livox CustomMsg.

LVX2 layout follows Livox's LVX2 Specifications v1.0 (2023-06), pp. 4-8:
https://www.livoxtech.com/downloads
No source recording is redistributed. LVX2 timing inside a complete 96-point
packet is an explicit nominal 5 us assumption; the container has no interval.
ROS messages instead retain their actual offset_time and line fields.
"""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import struct

import numpy as np

POINTS = np.dtype([("xyz", "<i4", (3,)), ("reflect", "u1"), ("tag", "u1")])
PACKAGE = struct.Struct("<BIBBQHBIB4s")


def read_exact(stream, count):
    data = stream.read(count)
    if len(data) != count:
        raise ValueError("truncated capture")
    return data


def lvx2_packets(path, report):
    """Yield original packet timestamps and local returns; reject mixed sensors."""
    with path.open("rb") as stream:
        signature, version, magic = struct.unpack("<16s4sI", read_exact(stream, 24))
        if (signature, version, magic) != (
            b"livox_tech" + b"\0" * 6,
            b"\x02\0\0\0",
            0xAC0EA767,
        ):
            raise ValueError("unsupported LVX2 header")
        frame_ms, devices = struct.unpack("<IB", read_exact(stream, 5))
        if devices != 1:
            raise ValueError(
                "select a single sensor before analyzing multi-device LVX2"
            )
        info = read_exact(stream, 63)
        device_id, _, kind, enabled, *extrinsics = struct.unpack_from(
            "<IBBB6f", info, 32
        )
        if kind != 9 or any(extrinsics):
            raise ValueError("requires MID-360 with identity extrinsics")
        report.update(
            frame_duration_ms=frame_ms,
            device_type=kind,
            extrinsic_enabled=bool(enabled),
            extrinsics=extrinsics,
            timing_policy="nominal 5 us per point, inferred from complete 96-point packets; no interval field in LVX2",
            lane_policy="point position modulo 4 in each 96-point packet (inferred)",
        )
        file_size = path.stat().st_size
        previous_frame = None
        while stream.tell() < file_size:
            offset = stream.tell()
            current, following, frame_index = struct.unpack(
                "<QQQ", read_exact(stream, 24)
            )
            if current != offset or not current + 24 <= following <= file_size:
                raise ValueError("invalid LVX2 frame offsets")
            if previous_frame is not None and frame_index != previous_frame + 1:
                raise ValueError("discontinuous LVX2 frame indices")
            previous_frame = frame_index
            while stream.tell() < following:
                if following - stream.tell() < PACKAGE.size:
                    raise ValueError("truncated LVX2 package header")
                version, lidar, _, clock, stamp, counter, dtype, size, _, _ = (
                    PACKAGE.unpack(read_exact(stream, PACKAGE.size))
                )
                if version != 0 or lidar != device_id or dtype != 1 or size != 96 * 14:
                    raise ValueError(
                        "requires complete 96-point millimetre LVX2 packets"
                    )
                if stream.tell() + size > following:
                    raise ValueError("LVX2 package crosses frame boundary")
                points = np.frombuffer(read_exact(stream, size), dtype=POINTS)
                yield stamp, np.arange(96, dtype=np.int64) * 5000, points["xyz"].astype(
                    float
                ), points["tag"], points["reflect"], np.arange(96) % 4, clock
        report["frames"] = 0 if previous_frame is None else previous_frame + 1


def reconstruct_ros_offsets(offsets):
    """Reverse the declared divide-by-96 hypothesis, rejecting ambiguous groups."""
    if len(offsets) % 96:
        raise ValueError("interval reconstruction requires complete 96-point groups")
    groups = offsets.reshape(-1, 96)
    relative = groups - groups[:, :1]
    step = relative[:, 1]
    if not np.all(relative == step[:, None] * np.arange(96)):
        raise ValueError("not a constant-integer-step 96-point group")
    total = ((step * 96 + 99) // 100) * 100
    if np.any(total >= (step + 1) * 96):
        raise ValueError("no 100 ns interval consistent with divide-by-96 hypothesis")
    return (groups[:, :1] + (total[:, None] * np.arange(96) + 47) // 95).ravel()


def ros1_packets(path, report):
    """Read embedded ROS1 definitions without a sourced ROS installation."""
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg

    store = get_typestore(Stores.ROS1_NOETIC)
    with Reader(path) as reader:
        connections = [c for c in reader.connections if c.topic == "/livox/lidar"]
        if len(connections) != 1 or not connections[0].msgtype.endswith("/CustomMsg"):
            raise ValueError("need exactly one /livox/lidar CustomMsg connection")
        connection = connections[0]
        store.register(get_types_from_msg(connection.msgdef, connection.msgtype))
        report.update(
            topics=[
                dict(topic=c.topic, type=c.msgtype, messages=c.msgcount)
                for c in reader.connections
            ],
            timing_policy="original CustomMsg.timebase + CustomPoint.offset_time",
            lane_policy="original CustomPoint.line",
            frame_ids=[],
            lidar_ids=[],
        )
        frame_ids, lidar_ids = set(), set()
        timing_layouts = Counter()
        # Deserialize through the embedded schema. Retaining zero returns is
        # important: subsequent sampling never renumbers emissions after misses.
        for _, _, raw in reader.messages(connections=connections):
            msg = store.deserialize_ros1(raw, connection.msgtype)
            frame_ids.add(msg.header.frame_id)
            lidar_ids.add(int(msg.lidar_id))
            if len(lidar_ids) != 1 or len(frame_ids) != 1:
                raise ValueError("mixed lidar IDs or frames")
            p = msg.points
            if len(p) != msg.point_num or not p:
                raise ValueError("invalid CustomMsg point count")
            offsets = np.fromiter(
                (v.offset_time for v in p), dtype=np.int64, count=len(p)
            )
            xyz = np.array([(v.x, v.y, v.z) for v in p]) * 1000
            tags = np.fromiter((v.tag for v in p), dtype=np.uint8, count=len(p))
            reflect = np.fromiter(
                (v.reflectivity for v in p), dtype=np.uint8, count=len(p)
            )
            lane = np.fromiter((v.line for v in p), dtype=np.uint8, count=len(p))
            # Diagnose the known driver interpolation convention without changing
            # source offsets or pretending the bag carries original UDP headers.
            if len(offsets) % 96 == 0:
                within = offsets.reshape(-1, 96) - offsets.reshape(-1, 96)[:, :1]
                for step in [4947, 5000]:
                    timing_layouts[str(step)] += int(
                        np.sum(np.all(within == np.arange(96) * step, axis=1))
                    )
            if report.get("requested_ros_interval_reconstruction", False):
                offsets = reconstruct_ros_offsets(offsets)
                report["timing_policy"] = (
                    "explicit driver hypothesis: recover a 100 ns total interval consistent with each complete 96-point group's integer step=floor(interval/96); interpolate interval/95; original file untouched"
                )
            yield int(msg.timebase), offsets, xyz, tags, reflect, lane, "ROS CustomMsg"
        report["within_96_point_group_spacing_ns"] = dict(timing_layouts)
        report["frame_ids"] = sorted(frame_ids)
        report["lidar_ids"] = sorted(lidar_ids)


def extract(path, output, ros_reconstruct_interval=False):
    output.mkdir(parents=True, exist_ok=False)
    report = dict(
        source_file=path.name,
        source_bytes=path.stat().st_size,
        requested_ros_interval_reconstruction=ros_reconstruct_interval,
    )
    with path.open("rb") as stream:
        digest = hashlib.sha256()
        while data := stream.read(1024 * 1024):
            digest.update(data)
    report["source_sha256"] = digest.hexdigest()
    packets = lvx2_packets if path.suffix.lower() == ".lvx2" else ros1_packets
    chunks = {
        k: [] for k in ["time_ns", "xyz_mm", "tag", "reflectivity", "lane", "block"]
    }
    counts, tags_seen, lanes, clocks, steps = (
        Counter(),
        Counter(),
        Counter(),
        set(),
        Counter(),
    )
    first = last = previous = None
    for packet_index, (stamp, offsets, xyz, tags, reflect, lane, clock) in enumerate(
        packets(path, report)
    ):
        if (
            np.any(offsets < 0)
            or np.any(np.diff(offsets) < 0)
            or not np.isfinite(xyz).all()
        ):
            raise ValueError("invalid point offsets or coordinates")
        if first is None:
            first = stamp + int(offsets[0])
        times = stamp + offsets
        if previous is not None:
            steps[int(times[0]) - previous] += 1
        previous = int(times[-1])
        last = max(last or previous, previous)
        clocks.add(clock)
        valid = np.any(xyz != 0, axis=1)
        counts.update(
            messages=1,
            emissions=len(xyz),
            valid=int(valid.sum()),
            zero=int((~valid).sum()),
        )
        tags_seen.update(dict(zip(*np.unique(tags, return_counts=True))))
        lanes.update(dict(zip(*np.unique(lane, return_counts=True))))
        # Dense first four seconds; afterwards retain one packet in 20 for
        # LVX2, or a deterministic 1/20 of point groups inside each ROS scan.
        keep = valid.copy()
        if stamp - first >= 4_000_000_000:
            if path.suffix.lower() == ".lvx2":
                keep &= packet_index % 20 == 0
            else:
                keep &= ((offsets // 20000) % 20) == 0
        chunks["time_ns"].append((times - first)[keep])
        chunks["xyz_mm"].append(xyz[keep])
        chunks["tag"].append(tags[keep])
        chunks["reflectivity"].append(reflect[keep])
        chunks["lane"].append(lane[keep])
        # Distinct 0.48ms acquisition windows, independent of ROS batching.
        chunks["block"].append(((times - first) // 480000)[keep])
    if first is None:
        raise ValueError("empty capture")
    arrays = {k: np.concatenate(v) for k, v in chunks.items()}
    order = np.argsort(arrays["time_ns"], kind="stable")
    np.savez(output / "samples.npz", **{k: a[order] for k, a in arrays.items()})
    report.update(
        counts,
        first_sensor_time_ns=first,
        last_sensor_time_ns=last,
        duration_s=(last - first) / 1e9,
        sampled_valid_points=len(order),
        tags={str(k): int(v) for k, v in tags_seen.items()},
        lanes={str(k): int(v) for k, v in lanes.items()},
        clock_types=sorted(clocks, key=str),
        message_boundary_spacing_ns={str(k): int(v) for k, v in steps.most_common(20)},
        sampling="all valid first 4s, then 1/20 packets (LVX2) or 20us groups (ROS); timestamps and lane labels preserved",
        integrity="SHA-256 identifies downloaded bytes; LVX2/ROS parsing does not verify original UDP CRCs",
    )
    (output / "decode-report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--ros-reconstruct-interval",
        action="store_true",
        help="Explicit divide-by-96 driver timing reconstruction hypothesis; never default",
    )
    args = parser.parse_args()
    extract(args.capture, args.output, args.ros_reconstruct_interval)
