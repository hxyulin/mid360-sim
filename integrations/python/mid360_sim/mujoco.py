# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""MuJoCo 3.7+ CPU ray adapter over the caller's live mjModel/mjData.

No stepping, wall clock or second dynamics world. scan() holds the mounted pose;
scan_rolling() interpolates caller-supplied sensor poses. Both query the current
geometry; schedule coherent short captures from the physics loop.
"""

import mujoco
import numpy as np
from .pattern import ScanFrame, directions, validate_window


class MujocoSensor:
    """Site-mounted nominal LiDAR using mj_multiRay and existing MuJoCo geoms.

    Call mj_forward/mj_kinematics after changing qpos before querying. Geom
    group filters select optical geoms; contype/conaffinity are not this filter.
    body_exclude=-1 retains robot self-occlusion. MuJoCo's alpha/visibility and
    mesh-ray semantics apply, so retain an appropriate optical geom set.
    """

    def __init__(
        self,
        model,
        data,
        site_name,
        min_range_m=0.1,
        max_range_m=70.0,
        geom_groups=None,
        body_exclude=-1,
    ):
        self.model, self.data = model, data
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if self.site_id < 0:
            raise ValueError("missing sensor mount site")
        if (
            not np.isfinite([min_range_m, max_range_m]).all()
            or not 0 <= min_range_m < max_range_m
        ):
            raise ValueError("invalid ranges")
        if not isinstance(body_exclude, int) or not -1 <= body_exclude < model.nbody:
            raise ValueError("invalid excluded body")
        self.min_range_m, self.max_range_m = min_range_m, max_range_m
        self.body_exclude = body_exclude
        self.geom_groups = None
        if geom_groups is not None:
            groups = np.asarray(geom_groups)
            if groups.shape != (6,) or not np.isin(groups, [0, 1]).all():
                raise ValueError("geom_groups must contain six 0/1 entries")
            self.geom_groups = groups.astype(np.uint8)
        self.capacity = 0

    def scan(self, first_index, count, epoch_ns=0, stride=1):
        """Sample the current mounted pose; original per-ray times remain explicit.

        Returned arrays own their data and survive subsequent queries. Callers
        own scheduling/reset and must not concurrently mutate the same mjData.
        """
        validate_window(first_index, count, stride, epoch_ns)
        local = directions(first_index, count, stride)
        rotation = self.data.site_xmat[self.site_id].reshape(3, 3)
        world = np.ascontiguousarray(local @ rotation.T)
        origin = np.ascontiguousarray(self.data.site_xpos[self.site_id])
        if count > self.capacity:
            self.capacity = count
            self._ranges = np.empty(count, dtype=np.float64)
            self._ids = np.empty(count, dtype=np.int32)
        ranges, ids = self._ranges[:count], self._ids[:count]
        mujoco.mj_multiRay(
            self.model,
            self.data,
            origin,
            world.ravel(),
            self.geom_groups,
            1,
            self.body_exclude,
            ids,
            ranges,
            None,
            count,
            self.max_range_m,
        )
        invalid = (
            (ranges < self.min_range_m)
            | (ranges > self.max_range_m)
            | ~np.isfinite(ranges)
        )
        ranges[invalid] = -1
        ids[invalid] = -1
        return ScanFrame(
            first_index, stride, epoch_ns, local, ranges.copy(), ids.copy()
        )

    def scan_rolling(
        self,
        first_index,
        count,
        pose_start,
        pose_end,
        epoch_ns=0,
        stride=1,
        origin_tolerance_m=0.0005,
        pose_fraction=(0.0, 1.0),
    ):
        """Interpolate caller-supplied sensor poses without advancing physics.

        Poses are (metre translation, xyzw unit quaternion). They correspond
        to first/last emissions unless pose_fraction selects a subinterval
        of boundary poses. Rotations use shortest-arc SLERP per emission.
        Native mj_multiRay shares one origin: split translation into bounded
        groups, using their midpoint origins. origin_tolerance_m bounds that
        origin displacement, NOT range error at grazing surfaces. Scene/body
        geometry is the current mjData throughout; callers own its history.
        Defaults remain unchanged for ordinary scan().
        """
        validate_window(first_index, count, stride, epoch_ns)
        if not np.isfinite(origin_tolerance_m) or origin_tolerance_m <= 0:
            raise ValueError("origin_tolerance_m must be positive and finite")
        fraction = np.asarray(pose_fraction, dtype=float)
        if (
            fraction.shape != (2,)
            or not np.isfinite(fraction).all()
            or not 0 <= fraction[0] <= fraction[1] <= 1
        ):
            raise ValueError("invalid pose subinterval")
        poses = []
        for pose in (pose_start, pose_end):
            if len(pose) != 2:
                raise ValueError("pose must contain translation and xyzw quaternion")
            p, q = (np.asarray(v, dtype=float) for v in pose)
            if (
                p.shape != (3,)
                or q.shape != (4,)
                or not np.isfinite(p).all()
                or not np.isfinite(q).all()
            ):
                raise ValueError("invalid pose")
            norm = np.linalg.norm(q)
            if abs(norm - 1) > 1e-3:
                raise ValueError("need unit xyzw quaternion")
            poses.append((p, q / norm))
        (p0, q0), (p1, q1) = poses
        dot = float(np.dot(q0, q1))
        if dot < 0:
            q1, dot = -q1, -dot
        t = np.linspace(fraction[0], fraction[1], count)
        if dot > 0.99999999:
            q = q0 + t[:, None] * (q1 - q0)
            q /= np.linalg.norm(q, axis=1)[:, None]
        else:
            angle = np.arccos(np.clip(dot, -1.0, 1.0))
            q = (
                np.sin((1 - t) * angle)[:, None] * q0 + np.sin(t * angle)[:, None] * q1
            ) / np.sin(angle)
        local = directions(first_index, count, stride)
        # Unit-quaternion vector rotation; no SciPy dependency or per-ray loop.
        # Expand the two quaternion cross products component-wise. This keeps
        # the same per-emission rotation without np.cross's axis handling and
        # temporary Nx3 arrays in every short physics-step query.
        x, y, z, w = q.T
        dx, dy, dz = local.T
        tx = 2 * (y * dz - z * dy)
        ty = 2 * (z * dx - x * dz)
        tz = 2 * (x * dy - y * dx)
        world = np.empty_like(local)
        world[:, 0] = dx + w * tx + y * tz - z * ty
        world[:, 1] = dy + w * ty + z * tx - x * tz
        world[:, 2] = dz + w * tz + x * ty - y * tx
        distance = float(np.linalg.norm(p1 - p0)) * float(fraction[1] - fraction[0])
        if not np.isfinite(distance):
            raise ValueError("nonfinite pose displacement")
        groups = max(1, int(np.ceil(min(count, distance / (2 * origin_tolerance_m)))))
        if count > self.capacity:
            self.capacity = count
            self._ranges = np.empty(count, dtype=np.float64)
            self._ids = np.empty(count, dtype=np.int32)
        for group in range(groups):
            a, b = group * count // groups, (group + 1) * count // groups
            origin = np.ascontiguousarray(p0 + (p1 - p0) * ((t[a] + t[b - 1]) * 0.5))
            rays = np.ascontiguousarray(world[a:b])
            mujoco.mj_multiRay(
                self.model,
                self.data,
                origin,
                rays.ravel(),
                self.geom_groups,
                1,
                self.body_exclude,
                self._ids[a:b],
                self._ranges[a:b],
                None,
                b - a,
                self.max_range_m,
            )
        ranges, ids = self._ranges[:count], self._ids[:count]
        invalid = (
            (ranges < self.min_range_m)
            | (ranges > self.max_range_m)
            | ~np.isfinite(ranges)
        )
        ranges[invalid] = -1
        ids[invalid] = -1
        return ScanFrame(
            first_index, stride, epoch_ns, local, ranges.copy(), ids.copy()
        )
