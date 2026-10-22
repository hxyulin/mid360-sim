# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Exact model/provenance checks with bounded libm variation in reference rays."""

import math
import re

# Unit-vector components only: at most 4e-13 m per coordinate at 40 m, not a sensor fit tolerance.
RAY_ABS_TOLERANCE = 1e-14


def compare_rays(expected, actual):
    assert len(expected) == len(actual), "Reference ray count differs"
    for a, b in zip(expected, actual):
        assert set(a) == set(b) == {"index", "direction"}
        assert a["index"] == b["index"], "Reference emission index differs"
        assert len(a["direction"]) == len(b["direction"]) == 3
        for x, y in zip(a["direction"], b["direction"]):
            assert math.isfinite(x) and math.isfinite(y)
            assert abs(x - y) <= RAY_ABS_TOLERANCE, "Reference direction differs"


def compare_model(expected, actual):
    """All model values exact; only computed reference directions allow libm roundoff."""
    assert {k: v for k, v in expected.items() if k != "reference_rays"} == {
        k: v for k, v in actual.items() if k != "reference_rays"
    }, "Optical coefficients, clocks or schema differ"
    compare_rays(expected["reference_rays"], actual["reference_rays"])


def header_parts(header):
    prefix, found, rest = header.partition("inline constexpr Fixture Fixtures[]={")
    assert found, "Missing C++ fixtures"
    body, found, suffix = rest.partition("};\n")
    assert found, "Unterminated C++ fixtures"
    pattern = r"\{(\d+), \{([^{}]+)\}\},"
    rays = [{"index": int(i), "direction": [float(x) for x in v.split(",")]}
            for i, v in re.findall(pattern, body)]
    assert not re.sub(pattern, "", body).strip(), "Unexpected C++ fixture syntax"
    return prefix, rays, suffix


def compare_header(expected, actual):
    """Keep generated C++ constants, format and notices exact outside fixture floats."""
    a, ar, az = header_parts(expected)
    b, br, bz = header_parts(actual)
    assert a == b and az == bz, "C++ model constants, structure or notice differ"
    compare_rays(ar, br)
