# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Alternate native Warp history backends using identical caller-owned motion.

Run with Isaac's Python. A baseline is an external backend.py and its adjacent
nominal.json; no baseline source or recorded data is bundled. No physics,
rendering, XYZ packaging or transport is timed. Rigid mode includes USD edits
and per-step moving-object captures. This is not a full-simulator benchmark.
"""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--input", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--baseline", type=Path)
parser.add_argument("--mode", choices=("static", "rigid"), default="static")
parser.add_argument("--objects", type=int, choices=(1, 8, 32), default=8)
parser.add_argument("--sensors", type=int, choices=(1, 2), default=1)
parser.add_argument("--share-captures", action="store_true")
parser.add_argument("--trials", type=int, choices=(3, 5, 7), default=7)
args = parser.parse_args()
if args.mode != "rigid" and (args.sensors != 1 or args.share_captures):
    parser.error("multiple sensors and shared captures require rigid mode")
args.output.parent.mkdir(parents=True, exist_ok=True)
from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "multi_gpu": False})
try:
    import numpy as np
    import omni.usd
    from pxr import Gf, UsdGeom
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("omni.warp.core")
    sys.path.insert(
        0, str(Path(__file__).resolve().parents[1] / "exts" / "hxyulin.mid360")
    )
    from mid360.isaac import backend

    modules = {"candidate": backend}
    if args.baseline is not None:
        spec = importlib.util.spec_from_file_location("mid360_baseline", args.baseline)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        modules = {"baseline": module, **modules}
    fixture = np.load(args.input, allow_pickle=False)
    positions, rotations = fixture["boundary_p"], fixture["boundary_q"]
    metadata = json.loads(str(fixture["metadata"]))
    if metadata["step_ns"] != 1000000 or positions.shape != (2001, 3):
        raise ValueError("need the shared two-second, 1 ms MoCap fixture")
    poses = np.column_stack([positions, rotations])
    counts = np.full(100, 200, dtype=np.int32)
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, "/World/Static")
    room = UsdGeom.Cube.Define(stage, "/World/Static/Room")
    room.CreateSizeAttr(20.0)
    movers = []
    if args.mode == "rigid":
        UsdGeom.Xform.Define(stage, "/World/Moving")
        for index in range(args.objects):
            cube = UsdGeom.Cube.Define(stage, f"/World/Moving/Part{index}")
            cube.CreateSizeAttr(0.6)
            movers.append((cube.AddTranslateOp(), cube.AddRotateXYZOp()))

    def make(module):
        static = module.Mid360Sensor(stage, roots=("/World/Static",), min_range_m=0)
        if args.mode == "static":
            return module.StaticSceneBatch(static)
        dynamic = module.Mid360Sensor(stage, roots=("/World/Moving",), min_range_m=0)
        return module.RigidHybridSensor(static, dynamic)

    sensors = {
        name: [make(module) for _ in range(args.sensors)]
        for name, module in modules.items()
    }

    def replay(name):
        consumers = sensors[name]
        sensor = consumers[0]
        frames = []
        start = time.perf_counter()
        if args.mode == "static":
            for batch in range(20):
                sensor.scan_history(
                    batch * 20000, counts, poses[batch * 100 : batch * 100 + 101]
                )
                frames.append(sensor.flush().ranges_m)
        else:
            for step in range(2000):
                if step % 100 == 0 and args.share_captures and name == "candidate":
                    history = backend.SceneTransformHistory(
                        stage, sensor.dynamic.paths, generation=step // 100
                    )
                for index, (translation, rotation) in enumerate(movers):
                    phase = step * 0.001 + index
                    translation.Set(
                        Gf.Vec3d(
                            2.0 + 0.2 * np.sin(phase),
                            -3.5 + 7 * index / max(1, args.objects - 1),
                            0.8,
                        )
                    )
                    rotation.Set(Gf.Vec3f(step * 0.03, index * 7.0, step * 0.07))
                snapshot = (
                    history.capture(step)
                    if args.share_captures and name == "candidate"
                    else None
                )
                for index, consumer in enumerate(consumers):
                    kwargs = {} if snapshot is None else {"scene_snapshot": snapshot}
                    consumer.scan(
                        step * 200,
                        200,
                        (positions[step] + (0, index * 0.1, 0), rotations[step]),
                        (
                            positions[step + 1] + (0, index * 0.1, 0),
                            rotations[step + 1],
                        ),
                        pose_fraction=(0, 199 / 200),
                        **kwargs,
                    )
                    if step % 100 == 99:
                        frames.append(consumer.flush().ranges_m)
        elapsed = time.perf_counter() - start
        return elapsed, np.concatenate(frames)

    for name in sensors:
        replay(name)
    timings, outputs = {name: [] for name in sensors}, {}
    for trial in range(args.trials):
        order = list(sensors) if trial % 2 == 0 else list(reversed(sensors))
        for name in order:
            elapsed, outputs[name] = replay(name)
            timings[name].append(elapsed)
    if args.baseline is not None:
        np.testing.assert_array_equal(
            outputs["baseline"] >= 0, outputs["candidate"] >= 0
        )
        np.testing.assert_allclose(
            outputs["baseline"], outputs["candidate"], rtol=0, atol=2e-5
        )
        difference = float(np.max(abs(outputs["baseline"] - outputs["candidate"])))
    else:
        difference = None
    report = dict(
        mode=args.mode,
        backend_sha256={
            name: hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest()
            for name, module in modules.items()
        },
        source_metadata=metadata,
        warmup_replays_per_mode=1,
        objects=len(movers),
        sensors=args.sensors,
        shared_candidate_captures=args.share_captures,
        rays=400000 * args.sensors,
        baseline=str(args.baseline) if args.baseline is not None else None,
        median_s={name: float(np.median(values)) for name, values in timings.items()},
        trials_s=timings,
        max_range_difference_m=difference,
        valid={
            name: int(np.count_nonzero(values >= 0)) for name, values in outputs.items()
        },
        scope="same-process alternating native acquisition, owned range readback; rigid adds USD edits/captures; no physics, rendering, XYZ packaging or transport",
    )
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print("HISTORY_BENCHMARK", json.dumps(report), flush=True)
except BaseException:
    import traceback

    args.output.with_suffix(".failure.txt").write_text(traceback.format_exc())
    raise
finally:
    app.close()
