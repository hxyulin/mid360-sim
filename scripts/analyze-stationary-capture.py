# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
# TriLIO Decoder is imported externally, never copied into this repository.
import sys, struct, time, json, hashlib
from pathlib import Path
import numpy as np
import argparse

parser = argparse.ArgumentParser(
    description="Validate a native Livox capture using an external TriLIO Decoder and retain bounded analysis samples."
)
parser.add_argument("capture", type=Path)
parser.add_argument("--decoder-tools", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument(
    "--allow-zero-crc",
    action="store_true",
    help="Accept missing checksums explicitly; this does not verify them",
)
args = parser.parse_args()
root = args.output
root.mkdir(parents=True, exist_ok=False)
sys.path.insert(0, str(args.decoder_tools.resolve()))
from livox_capture import Decoder

p = args.capture
decoder = Decoder(allow_zero_crc=args.allow_zero_crc)
start = time.monotonic()
indexes = []
times = []
xyzs = []
tags = []
reflect = []
imu = []
imuts = []
slot = 0
packets = 0
first = None
previous = None
gaps = 0
intervals = set()
counts = set()
clocktypes = set()
last = None
with p.open("rb") as f:
    assert f.read(8) == b"TLVX0001"
    while head := f.read(16):
        if len(head) != 16:
            raise ValueError("truncated record")
        arrival, handle, n = struct.unpack("<QII", head)
        b = f.read(n)
        if len(b) != n:
            raise ValueError("truncated packet")
        result = decoder.decode(arrival, handle, b)
        ts = struct.unpack_from("<Q", b, 28)[0]
        clocktypes.add(b[11])
        if b[10] == 0:
            imu.append(struct.unpack_from("<6f", b, 36))
            imuts.append(ts)
            continue
        _, length, interval, count = struct.unpack_from("<BHHH", b)
        if interval != 4750 or count != 96:
            raise ValueError("analysis requires 96 returns at a nominal 5 us cadence")
        intervals.add(interval)
        counts.add(count)
        if first is None:
            first = ts
        if previous is not None and ts - previous != 480000:
            raise ValueError(
                "discontinuous point packets: cannot assign contiguous emission indices"
            )
        previous = ts
        last = ts + (interval * 100)
        # All emissions in the first four seconds; one complete packet in 40 thereafter.
        if ts - first < 4_000_000_000 or packets % 40 == 0:
            raw = np.frombuffer(
                b,
                dtype=[("xyz", "<i4", (3,)), ("reflect", "u1"), ("tag", "u1")],
                count=count,
                offset=36,
            )
            keep = np.any(raw["xyz"] != 0, axis=1)
            ids = np.arange(count, dtype=np.uint64)
            indexes.append((slot + ids)[keep])
            times.append((ts - first + ids * 5000)[keep])
            xyzs.append(raw["xyz"][keep].copy())
            tags.append(raw["tag"][keep].copy())
            reflect.append(raw["reflect"][keep].copy())
        slot += count
        packets += 1
        if packets % 100000 == 0:
            print(
                packets, "packets", round(time.monotonic() - start, 1), "s", flush=True
            )
np.savez(
    root / "samples.npz",
    index=np.concatenate(indexes),
    time_ns=np.concatenate(times),
    xyz_mm=np.concatenate(xyzs),
    tag=np.concatenate(tags),
    reflectivity=np.concatenate(reflect),
    imu=np.array(imu),
    imu_time_ns=np.array(imuts),
)
with p.open("rb") as stream:
    digest = hashlib.sha256()
    while block := stream.read(1024 * 1024):
        digest.update(block)
    capture_digest = digest.hexdigest()
report = dict(
    decoder.counts,
    analysis_sampling="All first 4 seconds, then 1 of every 40 complete point packets; zero returns excluded from direction analysis only",
    point_packets=packets,
    emissions=slot,
    first_sensor_time_ns=first,
    last_sensor_time_ns=last,
    duration_s=(last - first) / 1e9,
    non_480us_point_intervals=gaps,
    packet_intervals_100ns=sorted(intervals),
    points_per_packet=sorted(counts),
    clock_types=sorted(clocktypes),
    decoder_sha256=hashlib.sha256(
        (args.decoder_tools / "livox_capture.py").read_bytes()
    ).hexdigest(),
    capture_sha256=capture_digest,
    elapsed_s=time.monotonic() - start,
)
(root / "decode-report.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2), flush=True)
