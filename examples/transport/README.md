<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Transport measurements

See the [transport benchmark guide](../../docs/guide/transport-benchmarks.md)
for the native ROS workload, external RM HTTP baseline, local binary codec
experiment, timing boundaries and reproduction commands. Results and limitations
live in the [evaluation report](../../docs/transport-evaluation.md).

The binary codec is a benchmark prototype, not a supported server protocol.
Sensor timestamps remain acquisition timestamps; host clocks are separate.
