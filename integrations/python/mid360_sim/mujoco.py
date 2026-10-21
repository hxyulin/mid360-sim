# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""MuJoCo 3.7+ CPU ray adapter over the caller's live mjModel/mjData.

No stepping, wall clock or second dynamics world. Geometry and mounted pose are
frozen during each query batch; schedule short scans from the physics loop.
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
