# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Prepare private benchmark-only triangle arrays from caller-provided RM GLBs.

No source CAD or generated geometry is distributed. Preserves all indexed
triangles and graph placements; concatenation changes grouping only. NumPy is
provided by the simulator/Python adapter environment. This intentionally narrow
reader rejects skins, animation, required extensions and sparse accessors.
"""

import argparse
import hashlib
import json
from pathlib import Path
import struct
import numpy as np


def rotation(q):
    """Column-vector rotation for a glTF xyzw unit quaternion."""
    x, y, z, w = q
    if not np.isclose(np.linalg.norm(q), 1):
        raise ValueError("nonunit source quaternion")
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def load_glb(path):
    """Read dense triangle GLB geometry, retaining each scene occurrence."""
    data = path.read_bytes()
    if struct.unpack_from("<III", data) != (0x46546C67, 2, len(data)):
        raise ValueError("not GLB 2")
    chunks, offset = {}, 12
    while offset < len(data):
        length, kind = struct.unpack_from("<II", data, offset)
        chunks[kind] = data[offset + 8 : offset + 8 + length]
        offset += length + 8
    doc, binary = json.loads(chunks[0x4E4F534A]), chunks[0x004E4942]
    if doc.get("extensionsRequired") or doc.get("skins") or doc.get("animations"):
        raise ValueError("unsupported required extension, skin or animation")

    def accessor(index):
        a = doc["accessors"][index]
        if a.get("sparse") or a.get("normalized"):
            raise ValueError("sparse/normalized accessor unsupported")
        v = doc["bufferViews"][a["bufferView"]]
        if v.get("buffer", 0) != 0:
            raise ValueError("external buffer unsupported")
        dtype = np.dtype(
            {5121: "u1", 5123: "<u2", 5125: "<u4", 5126: "<f4"}[a["componentType"]]
        )
        width = {"SCALAR": 1, "VEC3": 3}[a["type"]]
        begin = v.get("byteOffset", 0) + a.get("byteOffset", 0)
        stride = v.get("byteStride", dtype.itemsize * width)
        return np.ndarray(
            (a["count"], width),
            dtype=dtype,
            buffer=binary,
            offset=begin,
            strides=(stride, dtype.itemsize),
        ).copy()

    vertices, triangles, names, stack = [], [], [], set()
    vertex_count = 0

    def visit(index, parent):
        nonlocal vertex_count
        if index in stack:
            raise ValueError("cyclic graph")
        stack.add(index)
        node = doc["nodes"][index]
        if "matrix" in node:
            local = np.array(node["matrix"], dtype=float).reshape(4, 4, order="F")
        else:
            local = np.eye(4)
            local[:3, :3] = rotation(node.get("rotation", [0, 0, 0, 1])) @ np.diag(
                node.get("scale", [1, 1, 1])
            )
            local[:3, 3] = node.get("translation", [0, 0, 0])
        transform = parent @ local
        if "mesh" in node:
            for primitive in doc["meshes"][node["mesh"]]["primitives"]:
                if primitive.get("mode", 4) != 4 or primitive.get("targets"):
                    raise ValueError("only fixed triangle geometry supported")
                points = accessor(primitive["attributes"]["POSITION"]).astype(float)
                indices = (
                    accessor(primitive["indices"]).ravel()
                    if "indices" in primitive
                    else np.arange(len(points))
                )
                if len(indices) % 3 or int(indices.max()) >= len(points):
                    raise ValueError("invalid triangles")
                points = points @ transform[:3, :3].T + transform[:3, 3]
                vertices.append(points)
                triangles.append(indices.reshape(-1, 3).astype(np.int64) + vertex_count)
                names.append(node.get("name", str(index)))
                vertex_count += len(points)
        for child in node.get("children", []):
            visit(child, transform)
        stack.remove(index)

    for root in doc["scenes"][doc.get("scene", 0)]["nodes"]:
        visit(root, np.eye(4))
    points, faces = np.concatenate(vertices), np.concatenate(triangles)
    if not np.isfinite(points).all():
        raise ValueError("nonfinite source geometry")
    return points, faces, len(names)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--field", type=Path, required=True)
    parser.add_argument("--robot", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads((args.field / "manifest.json").read_text())
    if manifest["season"] != "2027" or manifest["units"] != "metres":
        parser.error("expected metre RM2027 package")
    # Source-arena FLU axes with floor at zero, matching the supplied package.
    offset = np.array([0.0, 1.6243436, manifest["floor_top_source_z_m"]])
    arrays, records = {}, []
    index = 0
    for subdir in (Path("."), Path("equipment")):
        doc = json.loads((args.field / subdir / "manifest.json").read_text())
        for name, entry in doc["assets"].items():
            path = args.field / subdir / entry["visual"]
            sha = hashlib.sha256(path.read_bytes()).hexdigest()
            if sha != entry["visual_sha256"]:
                raise ValueError(f"checksum mismatch: {path.name}")
            points, faces, occurrences = load_glb(path)
            placements = entry["placements_in_source_arena_frame"] or [
                dict(translation_m=[0, 0, 0], rotation_xyzw=[0, 0, 0, 1])
            ]
            for placement in placements:
                world = (
                    points @ rotation(placement["rotation_xyzw"]).T
                    + placement["translation_m"]
                    - offset
                )
                arrays[f"p{index}"] = world.astype(np.float32)
                arrays[f"t{index}"] = faces.astype(np.int32)
                records.append(
                    dict(
                        name=name,
                        index=index,
                        source_file=str(path.name),
                        sha256=sha,
                        triangles=len(faces),
                        source_occurrences=occurrences,
                        bounds_m=[
                            world.min(axis=0).tolist(),
                            world.max(axis=0).tolist(),
                        ],
                    )
                )
                index += 1
    robot_manifest = json.loads((args.robot.parent / "manifest.json").read_text())
    robot_entry = next(
        (v for v in robot_manifest["files"].values() if v["file"] == args.robot.name),
        None,
    )
    if (
        robot_entry is None
        or robot_entry["sha256"] != hashlib.sha256(args.robot.read_bytes()).hexdigest()
    ):
        raise ValueError("robot does not match its package manifest")
    points, faces, occurrences = load_glb(args.robot)
    # Bound robot runtime packages document X-left/Y-up/Z-forward -> FLU.
    points = points[:, [2, 0, 1]]
    ground_offset = -float(points[:, 2].min())
    points[:, 2] += ground_offset
    arrays["robot_p"], arrays["robot_t"] = points.astype(np.float32), faces.astype(
        np.int32
    )
    report = dict(
        field_provenance=dict(
            source_file=manifest.get("source_file"),
            source_sha256=manifest.get("source_sha256"),
            notice=(args.field / "CAD-NOTICE.md").read_text(),
        ),
        static=records,
        static_triangles=sum(r["triangles"] for r in records),
        robot=dict(
            file=args.robot.name,
            sha256=hashlib.sha256(args.robot.read_bytes()).hexdigest(),
            triangles=len(faces),
            source_occurrences=occurrences,
            root_to_ground_translation_m=ground_offset,
            attribution=json.loads(
                (args.robot.parent / "manifest.json").read_text()
            ).get("source", {}),
            bounds_m=[points.min(axis=0).tolist(), points.max(axis=0).tolist()],
        ),
        policy="visual triangles; complete equipment at rest; robots merged exactly in fixed neutral posture; not redistributed",
    )
    arrays["metadata"] = np.array(json.dumps(report))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output, **arrays)
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
