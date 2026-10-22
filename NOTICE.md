<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Provenance

Extracted from rm-simulator at f9e0c89 (MIT OR Apache-2.0), by hxyulin.
The coefficient table derives from Livox-SDK/livox_laser_simulation revision
1cce1073633a062b92e30243a4c2920e45551bb5, scan_mode/mid360.csv, SHA-256
aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a.
The 16.8 MB source CSV is not redistributed. Fitting scripts retain provenance.
CustomMsg/CustomPoint definitions come from Livox-SDK/livox_ros_driver2 revision
21445540f0d100dc86a7e6df312dd70bbdb4afdf, copyright (c) 2022 Livox, MIT;
see interfaces/livox_ros_driver2/LICENSE. Only trailing whitespace was normalized.
rclrs 0.8.0 is unmodified, Apache-2.0. Its async-std dependency has a scoped
maintenance advisory exception in deny.toml (RUSTSEC-2025-0052).

```text
MIT License

Copyright (c) 2021 livox

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

The fitted hardware optics/clock parameters in `src/hardware_data.rs` derive from
an owner-provided stationary MID-360 capture with SHA-256
`82ef4b05ad71132999e522aae7b6f927db58a5570876b865a407a9d0faaa5b69`.
The raw recording and private decoder are not redistributed. Original measurements
retain their ownership. The fitter/runtime are local implementations; the
hardware profile does not copy the official Livox coefficient table. See
`docs/hardware-fit.md` for filtering, withheld data, phase calibration and limits.


## Public ETH3D worked example

`examples/eth3d/` contains original MIT OR Apache-2.0 example code. No source
point cloud or generated mesh is bundled. The geometry illustrations in
`docs/public/eth3d/office-geometry.webp`, `nominal-points.webp` and
`coverage-error.svg` and `range-panorama.webp` derive from
ETH3D Office, by Thomas Schöps, Johannes L. Schönberger, Silvano Galliani,
Torsten Sattler, Konrad Schindler, Marc Pollefeys and Andreas Geiger,
"A Multi-View Stereo Benchmark with High-Resolution Images and Multi-Camera
Videos", CVPR 2017. Original data ownership is retained.

Source: https://eth3d.ethz.ch/. Dataset and derived scene illustrations:
Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International,
https://creativecommons.org/licenses/by-nc-sa/4.0/.
Changes: voxelization, triangulation, synthetic shading, raycasting and plotting.
These data-derived illustrations are separate from the code's permissive license;
see `docs/public/eth3d/ATTRIBUTION.txt` and the scanned-geometry guide.

## Optional Isaac Sim adapter

The adapter in `integrations/isaac-sim/` is original MIT OR Apache-2.0 code
using NVIDIA Isaac Sim, OpenUSD and Warp APIs supplied by the user's Isaac
installation. No NVIDIA runtime, implementation source, raw recording or
sensor CAD is vendored. Those installed products retain their own licenses.
The small generated `nominal.json` contains the same derived optical parameters
as the Rust nominal model, plus explicit simulation clock choices and test
rays. Original measurements retain their ownership. This adapter performs
geometric ray queries, not the proprietary RTX sensor material/return model.


## Optional MuJoCo and Gazebo adapters

The original adapter code under `integrations/python/` and
`integrations/gazebo/` uses separately installed MuJoCo, NumPy and Gazebo SDKs.
No engine implementation code or recorded datasets are vendored. MuJoCo 3.7.0,
NumPy 2.5.4 and Gazebo Harmonic were used for native validation; installed
products retain their own licenses. Generated tiny model files contain the same
derived nominal optical parameters and explicit simulation clocks as the Rust
core. Original measurements retain their ownership. Docker recipes install
Gazebo packages only into an opt-in development container.

## Synchronized sensor examples

The optional `ImuPublisher`, smoke-test lifecycle handling and Cyclone loopback
profile were adapted from rm-simulator revision
`c3784e168cf21167f15c8728ba8e8edb8d1d80a9` (MIT OR Apache-2.0,
Copyright (c) 2026 hxyulin <hxyulin@proton.me>). The examples contain original
MIT OR Apache-2.0 kinematic fixture and protocol-consumer code. No RM physics
implementation, CAD package or runtime is bundled, and no reverse dependency is
added. Caller-provided IMU values do not establish calibrated device behavior.
