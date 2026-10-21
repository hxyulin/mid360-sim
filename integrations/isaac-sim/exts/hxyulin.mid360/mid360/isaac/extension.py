# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Kit extension lifecycle and opt-in USD sensor markers."""

import math
import omni.ext
import omni.physx
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, UsdGeom

from .backend import Mid360Sensor

_ACTIVE = None


def create_sensor(stage, path="/World/Mid360", roots=("/World",), exclude=()):
    """Create an opt-in sensor Xform; configure before starting the timeline.

    Attaching the prim beneath a robot mounts it to that robot. Optical roots
    select actual visible triangle meshes/cubes, not PhysX collision proxies.
    """
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    for key, typ, value in [
        ("mid360:enabled", Sdf.ValueTypeNames.Bool, True),
        ("mid360:roots", Sdf.ValueTypeNames.StringArray, list(roots)),
        ("mid360:exclude", Sdf.ValueTypeNames.StringArray, list(exclude)),
        ("mid360:minRangeM", Sdf.ValueTypeNames.Float, 0.1),
        ("mid360:maxRangeM", Sdf.ValueTypeNames.Float, 70.0),
    ]:
        prim.CreateAttribute(key, typ, custom=True).Set(value)
    return prim


def latest_frame(path="/World/Mid360"):
    """Latest caller-readable frame, or None before acquisition; no history queue."""
    return None if _ACTIVE is None else _ACTIVE.frames.get(path)


def _pose(stage, path):
    m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
    transform = Gf.Transform(m)
    if not Gf.IsClose(transform.GetScale(), Gf.Vec3d(1), 1e-6):
        raise ValueError("sensor mount must have unit scale")
    q = transform.GetRotation().GetQuat()
    return (tuple(transform.GetTranslation()), (*q.GetImaginary(), q.GetReal()))


class Extension(omni.ext.IExt):
    """One GPU backend per opt-in sensor; simulation controls all acquisition."""

    def on_startup(self, ext_id):
        global _ACTIVE
        _ACTIVE = self
        self.sensors = {}
        self.frames = {}
        self.previous = {}
        self.elapsed_s = 0.0
        self.next_index = 0
        self._step = omni.physx.get_physx_interface().subscribe_physics_on_step_events(
            self._on_step, False, 100
        )
        self._timeline = (
            omni.timeline.get_timeline_interface()
            .get_timeline_event_stream()
            .create_subscription_to_pop(self._on_timeline)
        )

        self._stage_events = (
            omni.usd.get_context()
            .get_stage_event_stream()
            .create_subscription_to_pop(self._on_stage)
        )

    def _reset(self):
        self.sensors.clear()
        self.frames.clear()
        self.previous.clear()
        self.elapsed_s = 0.0
        self.next_index = 0

    def _on_stage(self, event):
        if event.type in [
            int(omni.usd.StageEventType.CLOSING),
            int(omni.usd.StageEventType.OPENED),
        ]:
            self._reset()

    def _on_timeline(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self._reset()

    def _on_step(self, dt):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        if not math.isfinite(dt) or dt <= 0:
            return
        if not self.sensors:
            for prim in stage.Traverse():
                attr = prim.GetAttribute("mid360:enabled")
                if attr and attr.Get():
                    path = str(prim.GetPath())
                    self.sensors[path] = Mid360Sensor(
                        stage,
                        roots=prim.GetAttribute("mid360:roots").Get(),
                        exclude=prim.GetAttribute("mid360:exclude").Get(),
                        min_range_m=float(prim.GetAttribute("mid360:minRangeM").Get()),
                        max_range_m=float(prim.GetAttribute("mid360:maxRangeM").Get()),
                    )
                    self.previous[path] = _pose(stage, path)
        begin_ns = round(self.elapsed_s * 1e9)
        self.elapsed_s += dt
        end_ns = round(self.elapsed_s * 1e9)
        until = (end_ns + 4999) // 5000
        count = until - self.next_index
        for path, sensor in self.sensors.items():
            current = _pose(stage, path)
            # Interpolate poses at the selected emission endpoints, not merely
            # the physics step boundaries. Geometry stays at this step's capture.
            old = self.previous[path]
            poses = []
            for t in [self.next_index * 5000, (until - 1) * 5000]:
                f = max(0.0, min(1.0, (t - begin_ns) / max(1, end_ns - begin_ns)))
                position = tuple(a + (b - a) * f for a, b in zip(old[0], current[0]))
                qa = Gf.Quatd(old[1][3], Gf.Vec3d(*old[1][:3]))
                qb = Gf.Quatd(current[1][3], Gf.Vec3d(*current[1][:3]))
                q = Gf.Slerp(f, qa, qb)
                poses.append((position, (*q.GetImaginary(), q.GetReal())))
            if count > 0:
                self.frames[path] = sensor.scan(
                    self.next_index, count, poses[0], poses[1]
                )
            self.previous[path] = current
        self.next_index = until

    def on_shutdown(self):
        global _ACTIVE
        self._step = None
        self._timeline = None
        self._stage_events = None
        self.sensors.clear()
        self.frames.clear()
        self.previous.clear()
        _ACTIVE = None
