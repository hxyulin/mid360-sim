# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Warp ray-query adapter over actual USD meshes, not a second dynamics world.

Caller supplies time, sensor poses and stage synchronization. Geometry transforms
are captured once per scan; deforming/topology-changing meshes require rebuilding.
No optical material/noise/IMU calibration is claimed by this geometry-only backend.
"""

from dataclasses import dataclass
import json
import math
import weakref
from pathlib import Path

import numpy as np
import warp as wp
from pxr import Usd, UsdGeom, UsdPhysics, Tf, Sdf

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
    output_offset: int,
    terms: wp.array(dtype=wp.int32),
    coefficients: wp.array(dtype=wp.float32),
    mesh: wp.uint64,
    pose_start: wp.transform,
    pose_end: wp.transform,
    fraction_start: float,
    fraction_end: float,
    min_range: float,
    max_range: float,
    directions: wp.array(dtype=wp.vec3),
    ranges: wp.array(dtype=wp.float32),
):
    i = wp.tid()
    d = nominal_direction(first + wp.int64(i) * wp.int64(stride), terms, coefficients)
    directions[output_offset + i] = d
    fraction = wp.lerp(
        fraction_start, fraction_end, float(i) / float(wp.max(count - 1, 1))
    )
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
    ranges[output_offset + i] = -1.0
    if result.result and result.t >= min_range:
        ranges[output_offset + i] = result.t


@wp.struct
class _PoseSegment:
    """One acquisition segment, packed for one CPU-to-GPU metadata transfer."""

    count: int
    offset: int
    start: wp.transform
    end: wp.transform
    fraction: wp.vec2


@wp.kernel
def query_segments(
    first: wp.int64,
    stride: int,
    segments: wp.array(dtype=_PoseSegment),
    terms: wp.array(dtype=wp.int32),
    coefficients: wp.array(dtype=wp.float32),
    mesh: wp.uint64,
    min_range: float,
    max_range: float,
    directions: wp.array(dtype=wp.vec3),
    ranges: wp.array(dtype=wp.float32),
):
    i, segment = wp.tid()
    record = segments[segment]
    if i < record.count:
        output = record.offset + i
        d = nominal_direction(
            first + wp.int64(output) * wp.int64(stride), terms, coefficients
        )
        directions[output] = d
        fraction = wp.lerp(
            record.fraction[0],
            record.fraction[1],
            float(i) / float(wp.max(record.count - 1, 1)),
        )
        position = wp.lerp(
            wp.transform_get_translation(record.start),
            wp.transform_get_translation(record.end),
            fraction,
        )
        rotation = wp.quat_slerp(
            wp.transform_get_rotation(record.start),
            wp.transform_get_rotation(record.end),
            fraction,
        )
        result = wp.mesh_query_ray(
            mesh, position, wp.quat_rotate(rotation, d), max_range
        )
        ranges[output] = -1.0
        if result.result and result.t >= min_range:
            ranges[output] = result.t


@wp.kernel
def merge_nearest_ranges(
    static_ranges: wp.array(dtype=wp.float32),
    dynamic_ranges: wp.array(dtype=wp.float32),
    min_range: float,
):
    i = wp.tid()
    a = static_ranges[i]
    b = dynamic_ranges[i]
    nearest = b
    if a >= 0.0 and (b < 0.0 or a < b):
        nearest = a
    dynamic_ranges[i] = -1.0
    if nearest >= min_range:
        dynamic_ranges[i] = nearest


@wp.kernel
def query_rigid_history(
    first: wp.int64,
    stride: int,
    terms: wp.array(dtype=wp.int32),
    coefficients: wp.array(dtype=wp.float32),
    static_mesh: wp.uint64,
    segments: wp.array(dtype=_PoseSegment),
    inverse_history: wp.array(dtype=wp.mat44),
    mesh_ids: wp.array(dtype=wp.uint64),
    object_count: int,
    min_range: float,
    max_range: float,
    directions: wp.array(dtype=wp.vec3),
    ranges: wp.array(dtype=wp.float32),
):
    i, segment = wp.tid()
    record = segments[segment]
    if i < record.count:
        output = record.offset + i
        fraction = wp.lerp(
            record.fraction[0],
            record.fraction[1],
            float(i) / float(wp.max(record.count - 1, 1)),
        )
        position = wp.lerp(
            wp.transform_get_translation(record.start),
            wp.transform_get_translation(record.end),
            fraction,
        )
        rotation = wp.quat_slerp(
            wp.transform_get_rotation(record.start),
            wp.transform_get_rotation(record.end),
            fraction,
        )
        local = nominal_direction(
            first + wp.int64(output) * wp.int64(stride), terms, coefficients
        )
        directions[output] = local
        direction = wp.quat_rotate(rotation, local)
        static_hit = wp.mesh_query_ray(static_mesh, position, direction, max_range)
        nearest = float(-1.0)
        if static_hit.result:
            nearest = static_hit.t
        # A static hit below min range already blocks everything behind it.
        if nearest < 0.0 or nearest >= min_range:
            for obj in range(object_count):
                inverse = inverse_history[segment * object_count + obj]
                local_origin = wp.transform_point(inverse, position)
                local_direction = wp.transform_vector(inverse, direction)
                scale = wp.length(local_direction)
                limit = max_range
                if nearest >= 0.0:
                    limit = nearest
                hit = wp.mesh_query_ray(
                    mesh_ids[obj], local_origin, local_direction / scale, limit * scale
                )
                if hit.result:
                    distance = hit.t / scale
                    if nearest < 0.0 or distance < nearest:
                        nearest = distance
        ranges[output] = -1.0
        if nearest >= min_range:
            ranges[output] = nearest


def _host_frame(first, stride, epoch, directions, ranges):
    """Copy populated device slices into an owned, transport-free host frame."""
    host_directions, host_ranges = directions.numpy(), ranges.numpy()
    if str(directions.device) == "cpu":
        host_directions, host_ranges = host_directions.copy(), host_ranges.copy()
    return ScanFrame(first, stride, epoch, host_directions, host_ranges)


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
        self._transforms_dirty = True
        self._scene_revision = 0

        # Instance-prototype edits need broader dependency tracking; retain the
        # conservative per-query capture for instance proxies or physics-driven
        # ancestors: PhysX writeback is not required to emit ordinary USD notices.
        def needs_live_capture(path):
            prim = stage.GetPrimAtPath(path)
            if prim.IsInstanceProxy():
                return True
            while prim.IsValid():
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    return True
                prim = prim.GetParent()
            return False

        self._cache_transforms = not any(needs_live_capture(p) for p in self.paths)
        self._transform_ancestors = set()
        for path in self.paths:
            path = Sdf.Path(path)
            while path.pathElementCount > 0:
                self._transform_ancestors.add(path)
                path = path.GetParentPath()
        self._transform_ancestors.add(Sdf.Path.absoluteRootPath)
        owner_ref = weakref.ref(self)

        def scene_changed(notice, sender):
            owner = owner_ref()
            if owner is not None:
                for path in (
                    *notice.GetResyncedPaths(),
                    *notice.GetChangedInfoOnlyPaths(),
                ):
                    if path.GetPrimPath() in owner._transform_ancestors:
                        owner._transforms_dirty = True
                        owner._scene_revision += 1
                        break

        self._scene_changed_callback = scene_changed
        self._scene_notice = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged, scene_changed, stage
        )
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
        self._pending_count = 0

    def sync_transforms(self):
        """Capture current USD transforms; caller must synchronize physics to USD."""
        if self._cache_transforms and not self._transforms_dirty:
            return
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
        self._transforms_dirty = False

    def scan(
        self,
        first_index,
        count,
        pose_start=((0, 0, 0), (0, 0, 0, 1)),
        pose_end=None,
        epoch_ns=0,
        stride=1,
        defer_readback=False,
        pose_fraction=(0.0, 1.0),
    ):
        """Query explicit emissions; poses are (metre translation, xyzw quaternion).

        Poses correspond to first/last selected emissions, not frame boundaries.
        pose_fraction selects a subinterval of the supplied pose endpoints
        (default 0..1). This lets physics callbacks pass boundary poses directly
        without first interpolating poses on the CPU.
        No wall clock or hidden acquisition index is used. Noise is currently off.
        With defer_readback=True, queue queries on the GPU and return None;
        flush() returns one owned host frame for contiguous queued emissions.
        Each query still captures this step's geometry and interpolated poses.
        At most 200,000 rays may be pending. Never mix immediate scans with a
        pending batch; flush it first. No scene/pose history is approximated.
        """
        if (
            not all(isinstance(v, int) for v in [first_index, count, stride, epoch_ns])
            or first_index < 0
            or epoch_ns < 0
            or not 1 <= count <= 200000
            or not 1 <= stride <= 1001
        ):
            raise ValueError("invalid acquisition window")
        if (
            len(pose_fraction) != 2
            or not all(math.isfinite(v) for v in pose_fraction)
            or not 0 <= pose_fraction[0] <= pose_fraction[1] <= 1
        ):
            raise ValueError("invalid pose interpolation fractions")
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
                or not all(math.isfinite(v) for v in (*position, *rotation))
                or not math.isclose(
                    sum(v * v for v in rotation), 1.0, rel_tol=2e-5, abs_tol=2e-5
                )
            ):
                raise ValueError("invalid pose; need a unit xyzw quaternion")
            poses.append(wp.transform(position, rotation))
        pending = self._pending_count
        if pending and (
            not defer_readback
            or first_index != self._pending_first + pending * self._pending_stride
            or stride != self._pending_stride
            or epoch_ns != self._pending_epoch
        ):
            raise ValueError(
                "flush pending contiguous emissions before changing windows"
            )
        needed = pending + count
        if needed > 200000:
            raise ValueError("pending GPU batch exceeds 200000 rays; flush first")
        self.sync_transforms()
        if needed > self.capacity:
            capacity = max(
                needed,
                min(200000, max(self.capacity * 2, 20000 if defer_readback else count)),
            )
            directions = wp.empty(capacity, dtype=wp.vec3, device=self.device)
            ranges = wp.empty(capacity, dtype=wp.float32, device=self.device)
            if pending:
                wp.copy(directions, self.directions, count=pending)
                wp.copy(ranges, self.ranges, count=pending)
            self.capacity = capacity
            self.directions, self.ranges = directions, ranges
        wp.launch(
            query_scan,
            dim=count,
            inputs=[
                wp.int64(first_index),
                stride,
                count,
                pending,
                self.terms,
                self.coefficients,
                self.mesh.id,
                *poses,
                *pose_fraction,
                self.min_range_m,
                self.max_range_m,
                self.directions,
                self.ranges,
            ],
            device=self.device,
        )
        if not pending:
            self._pending_first, self._pending_stride, self._pending_epoch = (
                first_index,
                stride,
                epoch_ns,
            )
        self._pending_count = needed
        return None if defer_readback else self.flush()

    def flush(self):
        """Read queued rays into an owned host frame; None if no rays are pending.

        Query launches, refits and readback share Warp's current stream. Scene
        mutation must not race that stream. Readback preserves step captures.
        """
        frame = self._drain_device()
        return None if frame is None else _host_frame(*frame)

    def _drain_device(self):
        """Borrow populated device outputs until the next scan on this backend."""
        count = self._pending_count
        if not count:
            return None
        frame = (
            self._pending_first,
            self._pending_stride,
            self._pending_epoch,
            self.directions[:count],
            self.ranges[:count],
        )
        self._pending_count = 0
        return frame


class StaticSceneBatch:
    """Batch GPU tracing of recorded moving-sensor poses in static optical geometry.

    This is opt-in: all selected optical geometry must stay fixed while a batch
    is pending. Source/ancestor USD edits invalidate the batch and raise before
    tracing. Instance proxies and physics-driven ancestors are rejected because
    their updates need not emit the USD notices tracked by this batch. Moving sensor poses are retained per segment, including
    rotations within and between segments. No second physics world is created.
    """

    def __init__(self, sensor):
        if sensor._pending_count or not sensor._cache_transforms:
            raise ValueError(
                "need an idle backend without instance proxies or physics-driven geometry"
            )
        self.sensor = sensor
        self.segments = []
        self.count = 0
        self.history = None
        self.capacity = 0

    def scan(
        self,
        first_index,
        count,
        pose_start,
        pose_end=None,
        epoch_ns=0,
        stride=1,
        defer_readback=True,
        pose_fraction=(0.0, 1.0),
    ):
        """Record contiguous segment emissions; tracing waits for flush().

        At most 100 segments and 200,000 rays may be queued. Returns None.
        Scene changes during a pending batch are rejected, never retroactively
        applied to the recorded poses. Flush before changing stride/epoch.
        """
        sensor = self.sensor
        if self.history is not None:
            raise ValueError("flush the bulk pose history before appending segments")
        if not defer_readback:
            raise ValueError("static segments require explicit deferred readback")
        if (
            not all(isinstance(v, int) for v in (first_index, count, epoch_ns, stride))
            or first_index < 0
            or epoch_ns < 0
            or not 1 <= count <= 200000
            or not 1 <= stride <= 1001
            or epoch_ns + (first_index + (count - 1) * stride) * 5000
            > np.iinfo(np.uint64).max
        ):
            raise ValueError("invalid static acquisition window")
        if (
            len(pose_fraction) != 2
            or not all(math.isfinite(v) for v in pose_fraction)
            or not 0 <= pose_fraction[0] <= pose_fraction[1] <= 1
        ):
            raise ValueError("invalid pose interpolation fractions")
        pose_end = pose_start if pose_end is None else pose_end
        for position, rotation in (pose_start, pose_end):
            if (
                len(position) != 3
                or len(rotation) != 4
                or not all(math.isfinite(v) for v in (*position, *rotation))
                or not math.isclose(
                    sum(v * v for v in rotation), 1.0, rel_tol=2e-5, abs_tol=2e-5
                )
            ):
                raise ValueError("invalid pose; need a unit xyzw quaternion")
        if self.count + count > 200000 or len(self.segments) >= 100:
            raise ValueError("static batch capacity exceeded; flush first")
        if self.segments:
            if self.revision != sensor._scene_revision:
                raise ValueError("optical geometry changed during static tracing batch")
            if (
                first_index != self.first + self.count * self.stride
                or stride != self.stride
                or epoch_ns != self.epoch
            ):
                raise ValueError("flush before changing contiguous segment windows")
        else:
            sensor.sync_transforms()
            self.revision = sensor._scene_revision
            self.first, self.stride, self.epoch = first_index, stride, epoch_ns
        # Own each pose: callers may reuse/mutate input arrays after this call.
        self.segments.append(
            (
                count,
                tuple(map(tuple, pose_start)),
                tuple(map(tuple, pose_end)),
                tuple(pose_fraction),
            )
        )
        self.count += count

    def scan_history(self, first_index, counts, poses, epoch_ns=0, stride=1):
        """Queue an owned bulk history for static geometry, then flush explicitly.

        counts contains 1..100 positive segment emission counts; poses has
        shape (segments+1, 7): metres followed by unit xyzw quaternions.
        Consecutive poses delimit each segment, whose final selected emission
        precedes its end boundary by one emission interval. This is equivalent
        to scan(..., pose_fraction=(0,(count-1)/count)) for each segment.
        Maximum 200,000 rays. Cannot mix with pending segments. Geometry must
        stay static; no future robot/geometry poses are invented here.
        """
        if self.count:
            raise ValueError("flush before queuing another history")
        raw_counts = np.asarray(counts)
        raw_poses = np.asarray(poses, dtype=np.float64)
        if (
            raw_counts.ndim != 1
            or not 1 <= len(raw_counts) <= 100
            or not np.issubdtype(raw_counts.dtype, np.integer)
            or np.any(raw_counts < 1)
            or np.any(raw_counts > 200000)
        ):
            raise ValueError("invalid history emission counts")
        total = int(raw_counts.sum())
        if (
            total > 200000
            or raw_poses.shape != (len(raw_counts) + 1, 7)
            or not np.isfinite(raw_poses).all()
            or not np.allclose(
                np.sum(raw_poses[:, 3:] ** 2, axis=1), 1.0, rtol=2e-5, atol=2e-5
            )
        ):
            raise ValueError("invalid bounded pose history")
        if (
            not all(isinstance(v, int) for v in (first_index, epoch_ns, stride))
            or first_index < 0
            or epoch_ns < 0
            or not 1 <= stride <= 1001
            or epoch_ns + (first_index + (total - 1) * stride) * 5000
            > np.iinfo(np.uint64).max
        ):
            raise ValueError("invalid history acquisition window")
        counts = raw_counts.astype(np.int32, copy=True)
        owned_poses = np.array(raw_poses, dtype=np.float32, copy=True)
        fractions = np.column_stack(
            [np.zeros(len(counts)), (counts - 1) / counts]
        ).astype(np.float32)
        self.sensor.sync_transforms()
        self.revision = self.sensor._scene_revision
        self.first, self.stride, self.epoch = first_index, stride, epoch_ns
        self.history = counts, owned_poses, fractions
        self.count = total

    def flush(self):
        """Trace pending static segments into an owned host frame; None if empty.

        Scene changes fail closed. Use per-step tracing for moving geometry.
        """
        frame = self._drain_device()
        return None if frame is None else _host_frame(*frame)

    def _drain_device(self, trace=True):
        """Prepare outputs, optionally trace; borrow until the next batch.

        The rigid union path sets trace=False and fills both outputs in its
        fused kernel before any host readback. Public flush always traces.
        """
        if not self.segments and self.history is None:
            return None
        sensor = self.sensor
        if self.revision != sensor._scene_revision:
            raise ValueError("optical geometry changed during static tracing batch")
        if self.history is not None:
            counts, poses, fractions = self.history
            starts, ends = poses[:-1], poses[1:]
        else:
            counts = np.array([s[0] for s in self.segments], dtype=np.int32)
            starts = np.array(
                [(*s[1][0], *s[1][1]) for s in self.segments], dtype=np.float32
            )
            ends = np.array(
                [(*s[2][0], *s[2][1]) for s in self.segments], dtype=np.float32
            )
            fractions = np.array([s[3] for s in self.segments], dtype=np.float32)
        # One structured allocation/upload replaces five small transfers. The
        # moving-object pass consumes this same immutable device metadata.
        packed = np.empty(len(counts), dtype=_PoseSegment.numpy_dtype())
        packed["count"] = counts
        packed["offset"][0] = 0
        np.cumsum(counts[:-1], out=packed["offset"][1:])
        packed["start"], packed["end"], packed["fraction"] = starts, ends, fractions
        self._device_segments = wp.array(
            packed, dtype=_PoseSegment, device=sensor.device
        )
        if self.count > self.capacity:
            self.capacity = max(self.count, min(200000, self.capacity * 2))
            self.directions = wp.empty(
                self.capacity, dtype=wp.vec3, device=sensor.device
            )
            self.ranges = wp.empty(
                self.capacity, dtype=wp.float32, device=sensor.device
            )
        directions, ranges = self.directions[: self.count], self.ranges[: self.count]
        if trace:
            wp.launch(
                query_segments,
                dim=(int(counts.max()), len(counts)),
                inputs=[
                    wp.int64(self.first),
                    self.stride,
                    self._device_segments,
                    sensor.terms,
                    sensor.coefficients,
                    sensor.mesh.id,
                    sensor.min_range_m,
                    sensor.max_range_m,
                    directions,
                    ranges,
                ],
                device=sensor.device,
            )
        frame = (self.first, self.stride, self.epoch, directions, ranges)
        self.segments.clear()
        self.history = None
        self.count = 0
        return frame


class HybridSensor:
    """Query disjoint static and moving optical partitions on one Warp stream.

    Static rays retain per-step sensor poses and are traced at flush. Moving
    geometry is captured/traced on every scan. The closest hits are merged on
    the GPU before minimum-range filtering, preserving near-surface occlusion.
    Callers own a complete disjoint partition; these are two optical BVHs of
    the caller's scene, not separate physics worlds. All host output is owned.
    """

    def __init__(self, static_sensor, dynamic_sensor, min_range_m=0.1):
        if (
            static_sensor.device != dynamic_sensor.device
            or static_sensor.max_range_m != dynamic_sensor.max_range_m
            or static_sensor.min_range_m != 0
            or dynamic_sensor.min_range_m != 0
            or static_sensor.stage != dynamic_sensor.stage
            or dynamic_sensor._pending_count
        ):
            raise ValueError(
                "hybrid backends need one stage/device, equal max range and zero min range"
            )
        if set(static_sensor.paths) & set(dynamic_sensor.paths):
            raise ValueError("hybrid optical partitions overlap")
        if (
            not math.isfinite(min_range_m)
            or not 0 <= min_range_m < static_sensor.max_range_m
        ):
            raise ValueError("invalid hybrid minimum range")
        self.static = StaticSceneBatch(static_sensor)
        self.dynamic = dynamic_sensor
        self.min_range_m = min_range_m
        self.failed = False

    def scan(
        self,
        first_index,
        count,
        pose_start,
        pose_end=None,
        epoch_ns=0,
        stride=1,
        defer_readback=True,
        pose_fraction=(0.0, 1.0),
    ):
        """Queue step-local moving hits and static pose history; returns None.

        At most 100 segments / 200,000 rays may be pending. flush() returns
        merged hits. On any acquisition failure this instance must be discarded;
        it cannot emit a partial or mismatched union frame.
        """
        if self.failed:
            raise ValueError("discard failed hybrid acquisition")
        if not defer_readback:
            raise ValueError("hybrid acquisition requires explicit flush")
        try:
            self.static.scan(
                first_index,
                count,
                pose_start,
                pose_end,
                epoch_ns,
                stride,
                pose_fraction=pose_fraction,
            )
            self.dynamic.scan(
                first_index,
                count,
                pose_start,
                pose_end,
                epoch_ns,
                stride,
                defer_readback=True,
                pose_fraction=pose_fraction,
            )
        except Exception:
            self.failed = True
            raise

    def flush(self):
        """Merge nearest unfiltered GPU hits, then return one owned host frame.

        Hits below min_range_m suppress farther geometry and become misses.
        Static/moving ties need no material winner: output carries ranges only.
        """
        if self.failed:
            raise ValueError("discard failed hybrid acquisition")
        try:
            static, dynamic = self.static._drain_device(), self.dynamic._drain_device()
            if static is None and dynamic is None:
                return None
            if (
                static is None
                or dynamic is None
                or static[:3] != dynamic[:3]
                or len(static[4]) != len(dynamic[4])
            ):
                raise ValueError("hybrid acquisition windows diverged")
            wp.launch(
                merge_nearest_ranges,
                dim=len(dynamic[4]),
                inputs=[static[4], dynamic[4], self.min_range_m],
                device=self.dynamic.device,
            )
            return _host_frame(*dynamic)
        except Exception:
            self.failed = True
            raise


@dataclass(frozen=True)
class SceneTransformSnapshot:
    """Owned read-only inverse matrices for one caller-identified scene capture.

    Matrices map world metres into fixed mesh-local coordinates. The history
    retains their stage, ordered partition and generation identity. Retaining a
    snapshot retains its storage independently of any sensor's flush schedule.
    """

    history: "SceneTransformHistory"
    capture_id: object
    inverses: np.ndarray


class _SceneGeometryGuard:
    """Share structural notices, never captured transforms, for one partition."""

    _instances = weakref.WeakValueDictionary()

    @classmethod
    def for_partition(cls, stage, paths):
        key = (stage, paths)
        guard = cls._instances.get(key)
        if guard is None or guard.invalid:
            guard = cls(stage, paths)
            cls._instances[key] = guard
        return guard

    def __init__(self, stage, paths):
        self.stage = stage
        self.prims = tuple(stage.GetPrimAtPath(path) for path in paths)
        self.invalid = False
        ancestors = {Sdf.Path.absoluteRootPath}
        for path in paths:
            path = Sdf.Path(path)
            while path.pathElementCount:
                ancestors.add(path)
                path = path.GetParentPath()
        owner_ref = weakref.ref(self)

        def changed(notice, sender):
            owner = owner_ref()
            if owner is None:
                return
            # Resync can replace a prim at the same path, making path-only
            # validity checks insufficient. Ordinary pose notices are not a cache.
            for path in notice.GetResyncedPaths():
                if path.GetPrimPath() in ancestors:
                    owner.invalid = True
            for path in notice.GetChangedInfoOnlyPaths():
                if (
                    path.name
                    in (
                        "points",
                        "faceVertexCounts",
                        "faceVertexIndices",
                        "holeIndices",
                        "subdivisionScheme",
                        "size",
                    )
                    and path.GetPrimPath() in ancestors
                ):
                    owner.invalid = True

        self._changed_callback = changed
        self._notice = Tf.Notice.Register(Usd.Notice.ObjectsChanged, changed, stage)


class SceneTransformHistory:
    """Caller-owned, bounded rigid-scene capture for an identical mesh partition.

    Create a history with an explicit generation identity per <=100 captures.
    Call capture() after actual physics-to-USD writeback, with a unique hashable
    capture identity per scene snapshot (not a wall-clock time or USD notice).
    Repeating that identity returns the same immutable capture, even after later
    pose edits. Stage and ordered paths must match every consuming sensor.
    No object interpolation or synchronization is performed here. Rebuild this
    history and the sensors after removal, replacement, resync or mesh edits.
    """

    def __init__(self, stage, paths, generation):
        hash(generation)
        self.stage, self.paths, self.generation = stage, tuple(paths), generation
        if not 1 <= len(self.paths) <= 128 or len(set(self.paths)) != len(self.paths):
            raise ValueError("need 1..128 distinct ordered moving source paths")
        # The weak registry shares only invalidation state. Histories retain
        # independent captures and keep their guard alive for pending snapshots.
        self._geometry = _SceneGeometryGuard.for_partition(stage, self.paths)
        self._prims = self._geometry.prims
        self._storage = None
        self._snapshots = weakref.WeakValueDictionary()
        self._indices = {}
        self._validate()

    def _validate(self, *, check_prims=True):
        if self._geometry.invalid or (
            check_prims and any(not prim.IsValid() for prim in self._prims)
        ):
            raise ValueError("moving source changed; rebuild sensor and history")

    def _capture_into(self, destination):
        # Both callers validate prims before entering; a capture checks each
        # source matrix below before any inverse can become a snapshot.
        cache = UsdGeom.XformCache()
        for index, prim in enumerate(self._prims):
            matrix = cache.GetLocalToWorldTransform(prim)
            source = np.asarray(matrix, dtype=np.float64)
            if (
                not np.isfinite(source).all()
                or source[0, 3] != 0
                or source[1, 3] != 0
                or source[2, 3] != 0
                or source[3, 3] != 1
            ):
                raise ValueError("nonfinite or non-affine moving source transform")
            determinant = matrix.GetDeterminant()
            if not math.isfinite(determinant) or abs(determinant) < 1e-12:
                raise ValueError("singular moving source transform")
            # Retain Gf's double-precision affine inverse, including nested
            # shear/reflection; pose decomposition is not equivalent.
            # Copy the transposed double view directly into retained float32
            # storage, avoiding temporary transposed Gf and NumPy matrices.
            destination[index] = np.asarray(matrix.GetInverse()).T
        if not np.isfinite(destination).all():
            raise ValueError("nonfinite moving source inverse")

    def capture(self, capture_id):
        """Capture once per explicit identity; matrices use world metres.

        At most 100 distinct captures are retained. Start a new generation/history
        for another batch; old snapshots remain valid while consumers retain them.
        Callers must serialize scene writes and capture on their acquisition thread.
        """
        self._validate()
        if capture_id in self._snapshots:
            return self._snapshots[capture_id]
        if capture_id in self._indices:
            index = self._indices[capture_id]
        else:
            index = len(self._indices)
            if index >= 100:
                raise ValueError("scene transform history capacity exceeded")
            if self._storage is None:
                self._storage = np.empty((100, len(self.paths), 4, 4), dtype=np.float32)
            self._capture_into(self._storage[index])
            self._indices[capture_id] = index
        # A read-only buffer prevents consumers enabling ndarray writes. Rows
        # are never overwritten, and each snapshot keeps its entire owner alive.
        inverses = np.frombuffer(
            memoryview(self._storage[index]).toreadonly(), dtype=np.float32
        ).reshape(len(self.paths), 4, 4)
        snapshot = SceneTransformSnapshot(self, capture_id, inverses)
        self._snapshots[capture_id] = snapshot
        return snapshot


class RigidHybridSensor(HybridSensor):
    """Batch static and moving queries using per-step object-transform histories.

    Each moving source mesh has a fixed local-space BVH. scan() records its
    inverse world matrix; flush() transforms rays into each object's captured
    local frame. This retains ordinary step-frozen moving geometry without
    per-step GPU launches or BVH refits. Static and moving nearest-hit queries
    share one kernel and one sensor-pose interpolation per emission. Affine
    nonuniform scale is supported.
    Source meshes must remain undeformed and topology must stay fixed; rebuild
    after edits, as with the ordinary backend. At most 128 moving source meshes
    and 100 step captures may be pending. This is opt-in and caller-partitioned.
    """

    def __init__(self, static_sensor, dynamic_sensor, min_range_m=0.1):
        super().__init__(static_sensor, dynamic_sensor, min_range_m)
        if len(dynamic_sensor.paths) > 128:
            raise ValueError("rigid history supports at most 128 moving source meshes")
        self.local_meshes = []
        for path in dynamic_sensor.paths:
            prim = dynamic_sensor.stage.GetPrimAtPath(path)
            if prim.IsA(UsdGeom.Mesh):
                points, indices = _triangles(UsdGeom.Mesh(prim))
            else:
                points, indices = _cube(UsdGeom.Cube(prim))
            self.local_meshes.append(
                wp.Mesh(
                    points=wp.array(
                        points, dtype=wp.vec3, device=dynamic_sensor.device
                    ),
                    indices=wp.array(
                        indices, dtype=wp.int32, device=dynamic_sensor.device
                    ),
                )
            )
        self.mesh_ids = wp.array(
            np.array([m.id for m in self.local_meshes], dtype=np.uint64),
            dtype=wp.uint64,
            device=dynamic_sensor.device,
        )
        self._scene_history = SceneTransformHistory(
            dynamic_sensor.stage, dynamic_sensor.paths, generation=0
        )
        self._inverse_history = np.empty(
            (100, len(dynamic_sensor.paths), 4, 4), dtype=np.float32
        )
        self._history_count = 0
        self._shared_histories = set()
        self._capture_mode = None

    def scan(
        self,
        first_index,
        count,
        pose_start,
        pose_end=None,
        epoch_ns=0,
        stride=1,
        defer_readback=True,
        pose_fraction=(0.0, 1.0),
        scene_snapshot=None,
    ):
        """Record sensor endpoints and this step's moving-object inverse matrices.

        Geometry is frozen within each segment, exactly as in ordinary scans.
        Invalid/removed/singular geometry fails the whole acquisition; discard
        the instance on error. No GPU work is launched for moving meshes here.
        Optional scene_snapshot explicitly shares one caller-owned capture from
        SceneTransformHistory; poses/fractions and flush schedules remain local.
        Generations may overlap independent flush schedules within the existing
        100-segment bound. Do not mix sharing and self-capture in one batch.
        """
        if self.failed:
            raise ValueError("discard failed rigid hybrid acquisition")
        if not defer_readback:
            raise ValueError("rigid hybrid requires explicit flush")
        try:
            self.static.scan(
                first_index,
                count,
                pose_start,
                pose_end,
                epoch_ns,
                stride,
                pose_fraction=pose_fraction,
            )
            destination = self._inverse_history[self._history_count]
            if scene_snapshot is None:
                if self._capture_mode == "shared":
                    raise ValueError("cannot mix shared and self-captured histories")
                self._scene_history._validate()
                self._scene_history._capture_into(destination)
                self._capture_mode = "self"
            else:
                # Capture validated prims, and flush checks them again. Between
                # those boundaries retain notice-based structural invalidation
                # without rechecking every prim for every consuming sensor.
                self._scene_history._validate(check_prims=False)
                history = scene_snapshot.history
                history._validate(check_prims=False)
                if (
                    history.stage is not self.dynamic.stage
                    or history.paths != tuple(self.dynamic.paths)
                    or self._capture_mode == "self"
                    or history._snapshots.get(scene_snapshot.capture_id)
                    is not scene_snapshot
                ):
                    raise ValueError(
                        "scene snapshot stage/partition/generation mismatch"
                    )
                self._shared_histories.add(history)
                self._capture_mode = "shared"
                destination[:] = scene_snapshot.inverses
            self._history_count += 1
        except Exception:
            self.failed = True
            raise

    def flush(self):
        """Trace both partitions from captured histories and return one owned frame.

        Local ray lengths are converted back to world metres before the nearest
        hit comparison and min-range filter. Static edits fail closed. No moving
        object is queried at its final pose on behalf of earlier emissions.
        """
        if self.failed:
            raise ValueError("discard failed rigid hybrid acquisition")
        if not self._history_count:
            return None
        try:
            self._scene_history._validate()
            for history in self._shared_histories:
                history._validate()
            segments = list(self.static.segments)
            frame = self.static._drain_device(trace=False)
            if frame is None or len(segments) != self._history_count:
                raise ValueError("rigid hybrid histories diverged")
            device = self.dynamic.device
            matrices = self._inverse_history[: self._history_count].reshape(-1, 4, 4)
            wp.launch(
                query_rigid_history,
                dim=(max(s[0] for s in segments), len(segments)),
                inputs=[
                    wp.int64(frame[0]),
                    frame[1],
                    self.static.sensor.terms,
                    self.static.sensor.coefficients,
                    self.static.sensor.mesh.id,
                    self.static._device_segments,
                    wp.array(matrices, dtype=wp.mat44, device=device),
                    self.mesh_ids,
                    len(self.local_meshes),
                    self.min_range_m,
                    self.dynamic.max_range_m,
                    frame[3],
                    frame[4],
                ],
                device=device,
            )
            result = _host_frame(*frame)
            self._history_count = 0
            self._shared_histories.clear()
            self._capture_mode = None
            return result
        except Exception:
            self.failed = True
            raise
