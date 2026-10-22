<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# External RM sensor stream

See the [synchronized sensor guide](../../docs/guide/synchronized-sensors.md)
for the live consumer, isolated CAD evaluator, compatible host revision and
measurement contract. The original consumer/evaluator use only Python's standard library.
The optional [navigation demo](../../docs/guide/navigation-demo.md) adds
`ros.py` and `lio.py` for PointCloud2, physical TF and an external ROS estimator. No RM
library, CAD or ROS runtime is bundled here.
