# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Shared custom RTX test profile; import after SimulationApp starts."""
import numpy as np
from pxr import Sdf, Vt
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor


def create_profile(profile, az, el, tick_rate=10.0):
    """Create repeating idealized LiDAR profile and owned GMO reader.

    Rotary uses 32 fixed elevations and 6,250 firing groups/s. Nominal uses
    20,000 explicit directions at 10 Hz and remains experimental.
    tick_rate sets native sensor scheduling, independently of 10 Hz scan output.
    """
    n = 20000 if profile == "nominal" else 32
    # Instantiating the schema directly avoids network asset dependencies.
    lidar = Lidar("/World/Lidar", aux_output_level="BASIC", tick_rate=tick_rate)
    prim = lidar.prims[0]

    def attr(name, value, typ=None):
        a = prim.GetAttribute(name)
        if not a:
            assert typ is not None, name
            a = prim.CreateAttribute(name, typ, custom=False)
        assert a.Set(value), name

    for key, value in {
        "scanType": "SOLID_STATE" if profile == "nominal" else "ROTARY",
        "rayType": "IDEALIZED",
        "nearRangeM": 0.1,
        "farRangeM": 70.0,
        "scanRateBaseHz": 10,
        "patternFiringRateHz": 10 if profile == "nominal" else 6250,
        "numberOfEmitters": n,
        "numberOfChannels": n,
        "numLines": 1,
        "numRaysPerLine": Vt.UIntArray([n]),
        "rangeCount": 1,
        "rangesMinM": Vt.FloatArray([0.1]),
        "rangesMaxM": Vt.FloatArray([70.0]),
        "stateResolutionStep": 1,
        "maxReturns": 1,
        "accumulateOutputs": True,
        "minReflectance": 0.00001,
        "minReflectionRangeM": 1000.0,
        "skipDroppingInvalidPoints": True,
        "rangeAccuracyM": 0.0,
        "rangeResolutionM": 0.0001,
        "azimuthErrorMean": 0.0,
        "azimuthErrorStd": 0.0,
        "elevationErrorMean": 0.0,
        "elevationErrorStd": 0.0,
        "outputFrameOfReference": "SENSOR",
        "outputMotionCompensationState": "NONCOMPENSATED",
    }.items():
        attr("omni:sensor:Core:" + key, value)
    prim.ApplyAPI("OmniSensorGenericLidarCoreEmitterStateAPI", "s001")
    for key, values, typ in [
        ("azimuthDeg", Vt.FloatArray(az.tolist()), Sdf.ValueTypeNames.FloatArray),
        ("elevationDeg", Vt.FloatArray(el.tolist()), Sdf.ValueTypeNames.FloatArray),
        (
            "fireTimeNs",
            Vt.UIntArray((np.arange(n) * 5000).tolist()),
            Sdf.ValueTypeNames.UIntArray,
        ),
        (
            "channelId",
            Vt.UIntArray(list(range(1, n + 1))),
            Sdf.ValueTypeNames.UIntArray,
        ),
        ("rangeId", Vt.UIntArray([0] * n), Sdf.ValueTypeNames.UIntArray),
    ]:
        attr("omni:sensor:Core:emitterState:s001:" + key, values, typ)
    sensor = LidarSensor(lidar, annotators=["generic-model-output"])
    return lidar, sensor
