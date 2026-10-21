# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Warp ray-query adapter over actual USD meshes, not a second dynamics world.

Caller supplies time, sensor poses and stage synchronization. Geometry transforms
are captured once per scan; deforming/topology-changing meshes require rebuilding.
No optical material/noise/IMU calibration is claimed by this geometry-only backend.
"""

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
import warp as wp
from pxr import Usd, UsdGeom

MODEL = json.loads(Path(__file__).with_name("nominal.json").read_text())
CYCLE_GROUPS = wp.constant(MODEL["groups_per_cycle"])
SWEEP_NUMERATOR = wp.constant(MODEL["sweep_numerator"])
NUM_TERMS = wp.constant(len(MODEL["terms"]))
NUM_COEFFICIENTS = wp.constant(len(MODEL["coefficients"][0]))


@wp.func
def nominal_direction(
    index: wp.int64,
    terms: wp.array(dtype=wp.int32),
    coefficients: wp.array(dtype=wp.float32),
):
    group = (index // wp.int64(4)) % wp.int64(CYCLE_GROUPS)
    spin = float((group * wp.int64(181)) % wp.int64(50000)) * (2.0 * wp.pi / 50000.0)
    sweep = float((group * wp.int64(SWEEP_NUMERATOR)) % wp.int64(CYCLE_GROUPS)) * (
        2.0 * wp.pi / float(CYCLE_GROUPS)
    )
    lane = int(index % wp.int64(4))
    base = lane * NUM_COEFFICIENTS * 2
    az = coefficients[base]
    el = coefficients[base + 1]
    for term in range(NUM_TERMS):
        phase = float(terms[2 * term]) * spin + float(terms[2 * term + 1]) * sweep
        c = wp.cos(phase)
        s = wp.sin(phase)
        ci = base + (1 + term) * 2
        si = base + (1 + NUM_TERMS + term) * 2
        az += coefficients[ci] * c + coefficients[si] * s
        el += coefficients[ci + 1] * c + coefficients[si + 1] * s
    az = spin + az * (wp.pi / 720.0)
    el = el * (wp.pi / 720.0)
    return wp.vec3(wp.cos(el) * wp.cos(az), wp.cos(el) * wp.sin(az), wp.sin(el))


@wp.kernel
def update_world_points(
    local: wp.array(dtype=wp.vec3),
    objects: wp.array(dtype=wp.int32),
    transforms: wp.array(dtype=wp.mat44),
    world: wp.array(dtype=wp.vec3),
):
    i = wp.tid()
    world[i] = wp.transform_point(transforms[objects[i]], local[i])


@wp.kernel
def query_scan(
    first: wp.int64,
    stride: int,
    count: int,
    terms: wp.array(dtype=wp.int32),
    coefficients: wp.array(dtype=wp.float32),
    mesh: wp.uint64,
    pose_start: wp.transform,
    pose_end: wp.transform,
    min_range: float,
    max_range: float,
    directions: wp.array(dtype=wp.vec3),
    ranges: wp.array(dtype=wp.float32),
):
    i = wp.tid()
    d = nominal_direction(first + wp.int64(i) * wp.int64(stride), terms, coefficients)
    directions[i] = d
    fraction = float(i) / float(wp.max(count - 1, 1))
    position = wp.lerp(
        wp.transform_get_translation(pose_start),
        wp.transform_get_translation(pose_end),
        fraction,
    )
    rotation = wp.quat_slerp(
        wp.transform_get_rotation(pose_start),
        wp.transform_get_rotation(pose_end),
        fraction,
    )
    direction = wp.quat_rotate(rotation, d)
    result = wp.mesh_query_ray(mesh, position, direction, max_range)
    ranges[i] = -1.0
    if result.result and result.t >= min_range:
        ranges[i] = result.t


@dataclass
class ScanFrame:
    """Local emitted rays and ranges; misses are -1, timestamps are nanoseconds.

    XYZ is expressed in the sensor frame at each emission, requiring consumer
    deskewing. Reflectivity/tag outputs are explicit synthetic placeholders.
    """

    first_index: int
    stride: int
    epoch_ns: int
    directions: np.ndarray
    ranges_m: np.ndarray

    @property
    def indices(self):
        return (
            self.first_index
            + np.arange(len(self.ranges_m), dtype=np.uint64) * self.stride
        )

    @property
    def timestamps_ns(self):
        return self.epoch_ns + self.indices * 5000

    @property
    def valid(self):
        return self.ranges_m >= 0

    def livox_fields(self):
        """Transport-free CustomPoint-compatible arrays, with original miss offsets.

        Packet timebase is the first scheduled emission including misses, not
        the first surviving return. No ROS dependency or privileged world pose.
        """
        count = len(self.ranges_m)
        if count == 0 or (count - 1) * self.stride * 5000 > np.iinfo(np.uint32).max:
            raise ValueError("empty scan or offset_time exceeds uint32")
        keep = self.valid
        return dict(
            timebase=self.epoch_ns + self.first_index * 5000,
            offset_time=(np.arange(count, dtype=np.uint64) * self.stride * 5000)[
                keep
            ].astype(np.uint32),
            xyz_m=(self.directions * self.ranges_m[:, None])[keep],
            line=(self.indices[keep] % 4).astype(np.uint8),
            tag=np.full(int(keep.sum()), 0x10, dtype=np.uint8),
            reflectivity=np.full(int(keep.sum()), 100, dtype=np.uint8),
        )


def _triangles(mesh):
    if (
        mesh.GetSubdivisionSchemeAttr().Get() != UsdGeom.Tokens.none
        or mesh.GetHoleIndicesAttr().Get()
    ):
        raise ValueError(f"bake subdivision/holes before querying: {mesh.GetPath()}")
    points = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float32)
    counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int32)
    indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int32)
    if len(points) == 0 or sum(counts) != len(indices) or np.any(counts < 3):
        raise ValueError(f"invalid mesh {mesh.GetPath()}")
    # USD mesh faces may be concave. Requiring triangles avoids silently
    # changing their geometry via an invalid fan; triangulate upstream first.
    if np.any(counts != 3):
        raise ValueError(f"triangulate USD mesh first: {mesh.GetPath()}")
    if (
        np.any(indices < 0)
        or np.any(indices >= len(points))
        or not np.isfinite(points).all()
    ):
        raise ValueError(f"invalid mesh coordinates/indices: {mesh.GetPath()}")
    return points, indices


def _cube(cube):
    size = float(cube.GetSizeAttr().Get()) / 2
    points = np.array(
        [
            [x, y, z]
            for x in [-size, size]
            for y in [-size, size]
            for z in [-size, size]
        ],
        dtype=np.float32,
    )
    indices = np.array(
        [
            0,
            1,
            3,
            0,
            3,
            2,
            4,
            6,
            7,
            4,
            7,
            5,
            0,
            4,
            5,
            0,
            5,
            1,
            2,
            3,
            7,
            2,
            7,
            6,
            0,
            2,
            6,
            0,
            6,
            4,
            1,
            5,
            7,
            1,
            7,
            3,
        ],
        dtype=np.int32,
    )
    return points, indices


class Mid360Sensor:
    """Caller-stepped GPU LiDAR over selected USD mesh/cube geometry.

    Coordinates are sensor FLU and a Z-up, metre USD stage. Rigid transforms are
    refreshed per scan, sensor pose is interpolated per ray. Geometry is frozen
    within a scan; use physics-step-sized scans for moving obstacles. Triangle
    meshes and Cube primitives are supported; unsupported Gprims are rejected
    rather than silently omitted. Supply narrow scene roots/exclusions explicitly.
    """

    def __init__(
        self,
        stage,
        roots=("/World",),
        exclude=(),
        min_range_m=0.1,
        max_range_m=70.0,
        device="cuda:0",
    ):
        if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z or not np.isclose(
            UsdGeom.GetStageMetersPerUnit(stage), 1.0
        ):
            raise ValueError("requires a Z-up, metre USD stage")
        if (
            not np.isfinite([min_range_m, max_range_m]).all()
            or not 0 <= min_range_m < max_range_m
        ):
            raise ValueError("invalid ranges")
        self.stage, self.device = stage, device
        self.min_range_m, self.max_range_m = min_range_m, max_range_m
        self.paths = []
        local_points, triangles, object_ids = [], [], []
        offset = 0
        roots, exclude = tuple(roots), tuple(exclude)
        if not roots or any(not stage.GetPrimAtPath(p).IsValid() for p in roots):
            raise ValueError("missing scene root")
        for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
            path = str(prim.GetPath())
            if not any(path == r or path.startswith(r + "/") for r in roots) or any(
                path == r or path.startswith(r + "/") for r in exclude
            ):
                continue
            if prim.IsA(UsdGeom.PointInstancer):
                raise ValueError(f"expand USD PointInstancer first: {path}")
            if not prim.IsA(UsdGeom.Gprim):
                continue
            if UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible:
                continue
            if prim.IsA(UsdGeom.Mesh):
                points, indices = _triangles(UsdGeom.Mesh(prim))
            elif prim.IsA(UsdGeom.Cube):
                points, indices = _cube(UsdGeom.Cube(prim))
            else:
                raise ValueError(
                    f"unsupported optical geometry {path}: {prim.GetTypeName()}"
                )
            local_points.append(points)
            triangles.append(indices + offset)
            object_ids.append(np.full(len(points), len(self.paths), dtype=np.int32))
            offset += len(points)
            if offset > np.iinfo(np.int32).max:
                raise ValueError("scene exceeds int32 mesh indexing")
            self.paths.append(path)
        if not self.paths:
            raise ValueError("no supported optical geometry")
        self.local_points = wp.array(
            np.concatenate(local_points), dtype=wp.vec3, device=device
        )
        self.object_ids = wp.array(
            np.concatenate(object_ids), dtype=wp.int32, device=device
        )
        self.transforms = wp.empty(len(self.paths), dtype=wp.mat44, device=device)
        self.world_points = wp.empty(offset, dtype=wp.vec3, device=device)
        self._last_transforms = None
        self.mesh = None
        self.sync_transforms()
        self.mesh = wp.Mesh(
            points=self.world_points,
            indices=wp.array(np.concatenate(triangles), dtype=wp.int32, device=device),
        )
        self.terms = wp.array(
            np.array(MODEL["terms"], dtype=np.int32).ravel(),
            dtype=wp.int32,
            device=device,
        )
        self.coefficients = wp.array(
            np.array(MODEL["coefficients"], dtype=np.float32).ravel(),
            dtype=wp.float32,
            device=device,
        )
        self.capacity = 0

    def sync_transforms(self):
        """Capture current USD transforms; caller must synchronize physics to USD."""
        cache = UsdGeom.XformCache()
        matrices = []
        for path in self.paths:
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise ValueError("scene topology changed; rebuild the sensor")
            m = cache.GetLocalToWorldTransform(prim)
            if abs(m.GetDeterminant()) < 1e-12:
                raise ValueError("singular geometry transform")
            matrices.append(np.array(m.GetTranspose(), dtype=np.float32))
        matrices = np.array(matrices, dtype=np.float32)
        if self._last_transforms is None or not np.array_equal(
            matrices, self._last_transforms
        ):
            self.transforms.assign(matrices)
            wp.launch(
                update_world_points,
                dim=len(self.world_points),
                inputs=[
                    self.local_points,
                    self.object_ids,
                    self.transforms,
                    self.world_points,
                ],
                device=self.device,
            )
            if self.mesh is not None:
                self.mesh.refit()
            self._last_transforms = matrices

    def scan(
        self,
        first_index,
        count,
        pose_start=((0, 0, 0), (0, 0, 0, 1)),
        pose_end=None,
        epoch_ns=0,
        stride=1,
    ):
        """Query explicit emissions; poses are (metre translation, xyzw quaternion).

        Poses correspond to first/last selected emissions, not frame boundaries.
        No wall clock or hidden acquisition index is used. Noise is currently off.
        """
        if (
            not all(isinstance(v, int) for v in [first_index, count, stride, epoch_ns])
            or first_index < 0
            or epoch_ns < 0
            or not 1 <= count <= 200000
            or not 1 <= stride <= 1001
        ):
            raise ValueError("invalid acquisition window")
        last = first_index + (count - 1) * stride
        if epoch_ns + last * 5000 > np.iinfo(np.uint64).max:
            raise ValueError("timestamp overflow")
        poses = []
        for position, rotation in [
            pose_start,
            pose_start if pose_end is None else pose_end,
        ]:
            if (
                len(position) != 3
                or len(rotation) != 4
                or not np.isfinite([*position, *rotation]).all()
                or not np.isclose(np.linalg.norm(rotation), 1.0, atol=1e-5)
            ):
                raise ValueError("invalid pose; need a unit xyzw quaternion")
            poses.append(wp.transform(position, rotation))
        self.sync_transforms()
        if count > self.capacity:
            self.capacity = count
            self.directions = wp.empty(count, dtype=wp.vec3, device=self.device)
            self.ranges = wp.empty(count, dtype=wp.float32, device=self.device)
        wp.launch(
            query_scan,
            dim=count,
            inputs=[
                wp.int64(first_index),
                stride,
                count,
                self.terms,
                self.coefficients,
                self.mesh.id,
                *poses,
                self.min_range_m,
                self.max_range_m,
                self.directions,
                self.ranges,
            ],
            device=self.device,
        )
        # CUDA .numpy() owns a synchronized host copy. CPU arrays need copying.
        directions = self.directions.numpy()[:count]
        ranges = self.ranges.numpy()[:count]
        if self.device == "cpu":
            directions, ranges = directions.copy(), ranges.copy()
        return ScanFrame(
            first_index,
            stride,
            epoch_ns,
            directions,
            ranges,
        )
