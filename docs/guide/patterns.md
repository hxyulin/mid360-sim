<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Scan patterns

The Rust library and room demo expose four profiles. Optional simulator
adapters currently use `nominal` only.

| Profile | Purpose | Limits |
|---|---|---|
| `nominal` | Default representative hardware-derived optics | 10 harmonics, quarter-degree coefficients, 18.2-hour clock cycle |
| `official-table` | Regression against the published Livox table | Table fit, not a calibrated hardware model |
| `hardware-mean` | Unsimplified learned optics with mean clocks | Does not predict an individual device's future phase |
| `hardware-capture` | Sparse measured-clock reconstruction | Bounded to the 299.996640 s source recording |

The nominal clock choices are 181 Hz spin and 642697/65521 Hz sweep. Their exact
common cycle is 65,521 s. Integer modular phase avoids long-running float drift.
A four-lane group shares harmonic work without changing emission timestamps.

The [hardware fit](../hardware-fit.md) describes conditional fit accuracy.
[Independent captures](../external-captures.md) show transfer error; that error
must not be turned into an added angular-noise setting. The emitted direction
and the reported local point must stay coherent.

Use `--pattern` in the ROS demo. The [official reference audit](../official-reference.md)
explains why reproducing its direction table alone cannot reproduce real motion.
