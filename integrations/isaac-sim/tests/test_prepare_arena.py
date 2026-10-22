# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""CPU-only checks of benchmark GLB graph placement and indexed triangles."""

import json
from pathlib import Path
import struct
import tempfile
import unittest
import numpy as np
from prepare_arena import load_glb


class PreparationTests(unittest.TestCase):
    def test_shared_mesh_trs_matrix_reflection_and_interleaved_vertices(self):
        points = np.array([[0, 0, 0, 99], [1, 0, 0, 99], [0, 1, 0, 99]], dtype="<f4")
        binary = points.tobytes() + np.array([0, 1, 2], dtype="<u2").tobytes() + b"\0\0"
        matrix = np.eye(4)
        matrix[0, 0], matrix[1, 3] = -1, 2
        doc = dict(
            asset=dict(version="2.0"),
            buffers=[dict(byteLength=len(binary))],
            bufferViews=[
                dict(buffer=0, byteOffset=0, byteLength=48, byteStride=16),
                dict(buffer=0, byteOffset=48, byteLength=6),
            ],
            accessors=[
                dict(bufferView=0, componentType=5126, count=3, type="VEC3"),
                dict(bufferView=1, componentType=5123, count=3, type="SCALAR"),
            ],
            meshes=[dict(primitives=[dict(attributes=dict(POSITION=0), indices=1)])],
            nodes=[
                dict(
                    translation=[2, 0, 0],
                    rotation=[0, 0, 2**-0.5, 2**-0.5],
                    children=[1, 2],
                ),
                dict(mesh=0, translation=[1, 0, 0]),
                dict(mesh=0, matrix=matrix.ravel(order="F").tolist()),
            ],
            scenes=[dict(nodes=[0])],
            scene=0,
        )
        raw = json.dumps(doc).encode()
        raw += b" " * (-len(raw) % 4)
        content = (
            struct.pack("<II", len(raw), 0x4E4F534A)
            + raw
            + struct.pack("<II", len(binary), 0x004E4942)
            + binary
        )
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "fixture.glb"
            path.write_bytes(
                struct.pack("<III", 0x46546C67, 2, 12 + len(content)) + content
            )
            vertices, faces, occurrences = load_glb(path)
        np.testing.assert_allclose(
            vertices,
            [[2, 1, 0], [2, 2, 0], [1, 1, 0], [0, 0, 0], [0, -1, 0], [-1, 0, 0]],
            atol=1e-7,
        )
        np.testing.assert_array_equal(faces, [[0, 1, 2], [3, 4, 5]])
        self.assertEqual(occurrences, 2)


if __name__ == "__main__":
    unittest.main()
