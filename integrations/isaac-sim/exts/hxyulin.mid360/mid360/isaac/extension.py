# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Kit extension lifecycle and opt-in USD sensor markers."""

import math
import omni.ext
import omni.physx
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, UsdGeom

from .backend import Mid360Sensor, StaticSceneBatch, HybridSensor, RigidHybridSensor

_ACTIVE = None


def create_sensor(
    stage,
    path="/World/Mid360",
    roots=("/World",),
    exclude=(),
    batch_steps=1,
    static_trace=False,
    dynamic_roots=(),
    rigid_trace=False,
):
    """Create an opt-in sensor Xform; configure before starting the timeline.

    Attaching the prim beneath a robot mounts it to that robot. Optical roots
    select actual visible triangle meshes/cubes, not PhysX collision proxies.
    batch_steps (1..100) delays host output. static_trace additionally delays
    GPU tracing, retaining segment poses; all selected optical geometry must
    remain static during each batch. dynamic_roots instead partitions moving
    geometry; rigid_trace records fixed-mesh object transforms for batched
    moving queries. The default traces the full scene each step.
    """
    if not isinstance(batch_steps, int) or not 1 <= batch_steps <= 100:
        raise ValueError("batch_steps must be in 1..100")
    if rigid_trace and not dynamic_roots:
        raise ValueError("rigid_trace requires dynamic_roots")
    if static_trace and dynamic_roots:
        raise ValueError("static_trace and dynamic_roots are separate execution modes")
    if any(
        not any(d == r or d.startswith(r + "/") for r in roots) for d in dynamic_roots
    ):
        raise ValueError("dynamic roots must be within optical roots")
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    for key, typ, value in [
        ("mid360:rigidTrace", Sdf.ValueTypeNames.Bool, rigid_trace),
        ("mid360:dynamicRoots", Sdf.ValueTypeNames.StringArray, list(dynamic_roots)),
        ("mid360:staticTrace", Sdf.ValueTypeNames.Bool, static_trace),
        ("mid360:batchSteps", Sdf.ValueTypeNames.Int, batch_steps),
        ("mid360:enabled", Sdf.ValueTypeNames.Bool, True),
        ("mid360:roots", Sdf.ValueTypeNames.StringArray, list(roots)),
        ("mid360:exclude", Sdf.ValueTypeNames.StringArray, list(exclude)),
        ("mid360:minRangeM", Sdf.ValueTypeNames.Float, 0.1),
        ("mid360:maxRangeM", Sdf.ValueTypeNames.Float, 70.0),
    ]:
        prim.CreateAttribute(key, typ, custom=True).Set(value)
    return prim


def latest_frame(path="/World/Mid360"):
    """Latest completed host frame, or None before readback; no history queue."""
    return None if _ACTIVE is None else _ACTIVE.frames.get(path)


def _pose(stage, path):
    m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
    transform = Gf.Transform(m)
    if not Gf.IsClose(transform.GetScale(), Gf.Vec3d(1), 1e-6):
        raise ValueError("sensor mount must have unit scale")
    q = transform.GetRotation().GetQuat()
    return (tuple(transform.GetTranslation()), (*q.GetImaginary(), q.GetReal()))


class Extension(omni.ext.IExt):
    """One GPU backend per opt-in sensor; simulation controls all acquisition.

    Optional pose_reader(stage, sensor_path) returns a metre/xyzw world pose.
    A caller using native physics must apply sensor extrinsics and synchronize
    that pose to the callback's step. The default reader uses USD writeback."""

    def __init__(self, pose_reader=None):
        super().__init__()
        # Trusted in-process adapters may supply authoritative physics poses.
        # The reader receives (stage, sensor_path) and returns metre/xyzw pose.
        self.pose_reader = _pose if pose_reader is None else pose_reader

    def on_startup(self, ext_id):
        global _ACTIVE
        _ACTIVE = self
        self.sensors = {}
        self.frames = {}
        self.previous = {}
        self.batch_steps = {}
        self.batch_counts = {}
        self.elapsed_ns = 0
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
        self.batch_steps.clear()
        self.batch_counts.clear()
        self.elapsed_ns = 0
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
                    attr = prim.GetAttribute("mid360:dynamicRoots")
                    dynamic_roots = tuple(attr.Get()) if attr else ()
                    roots = tuple(prim.GetAttribute("mid360:roots").Get())
                    exclude = tuple(prim.GetAttribute("mid360:exclude").Get())
                    minimum = float(prim.GetAttribute("mid360:minRangeM").Get())
                    maximum = float(prim.GetAttribute("mid360:maxRangeM").Get())
                    static_trace = prim.GetAttribute("mid360:staticTrace")
                    static_trace = bool(static_trace and static_trace.Get())
                    if static_trace and dynamic_roots:
                        raise ValueError(
                            "static_trace and dynamic_roots are separate execution modes"
                        )
                    if any(
                        not any(d == r or d.startswith(r + "/") for r in roots)
                        for d in dynamic_roots
                    ):
                        raise ValueError("dynamic roots must be within optical roots")
                    backend = Mid360Sensor(
                        stage,
                        roots=roots,
                        exclude=exclude + dynamic_roots,
                        min_range_m=0.0 if dynamic_roots else minimum,
                        max_range_m=maximum,
                    )
                    rigid_trace = prim.GetAttribute("mid360:rigidTrace")
                    rigid_trace = bool(rigid_trace and rigid_trace.Get())
                    if rigid_trace and not dynamic_roots:
                        raise ValueError("rigid_trace requires dynamic_roots")
                    if dynamic_roots:
                        moving = Mid360Sensor(
                            stage,
                            roots=dynamic_roots,
                            exclude=exclude,
                            min_range_m=0.0,
                            max_range_m=maximum,
                        )
                        factory = RigidHybridSensor if rigid_trace else HybridSensor
                        backend = factory(backend, moving, min_range_m=minimum)
                    elif static_trace:
                        backend = StaticSceneBatch(backend)
                    self.sensors[path] = backend
                    self.previous[path] = self.pose_reader(stage, path)
                    attr = prim.GetAttribute("mid360:batchSteps")
                    batch_steps = attr.Get() if attr else 1
                    if not isinstance(batch_steps, int) or not 1 <= batch_steps <= 100:
                        raise ValueError("mid360:batchSteps must be in 1..100")
                    self.batch_steps[path] = batch_steps
                    self.batch_counts[path] = 0
        begin_ns = self.elapsed_ns
        self.elapsed_ns += round(dt * 1e9)
        end_ns = self.elapsed_ns
        until = (end_ns + 4999) // 5000
        count = until - self.next_index
        for path, sensor in self.sensors.items():
            current = self.pose_reader(stage, path)
            # GPU interpolation uses each ray's fraction of this physics step.
            # Avoid composing two CPU endpoint interpolations with GPU SLERP.
            old = self.previous[path]
            if count > 0:
                fractions = tuple(
                    max(0.0, min(1.0, (t - begin_ns) / max(1, end_ns - begin_ns)))
                    for t in (self.next_index * 5000, (until - 1) * 5000)
                )
                sensor.scan(
                    self.next_index,
                    count,
                    old,
                    current,
                    pose_fraction=fractions,
                    defer_readback=True,
                )
                self.batch_counts[path] += 1
                if self.batch_counts[path] >= self.batch_steps[path]:
                    self.frames[path] = sensor.flush()
                    self.batch_counts[path] = 0
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
        self.batch_steps.clear()
        self.batch_counts.clear()
        _ACTIVE = None
