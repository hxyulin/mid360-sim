<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# ETH3D Office reconstruction example

See the [worked guide](../../docs/guide/scanned-geometry.md) for source attribution,
setup, measured results and limits. Run from the repository root with Python 3.12:

```sh
python3.12 -m venv local-assets/eth3d-venv
. local-assets/eth3d-venv/bin/activate
python -m pip install -r examples/eth3d/requirements.txt './integrations/python[mujoco]'
python -m unittest discover -s examples/eth3d -v
OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 python examples/eth3d/run.py \
  --download --data local-assets/eth3d-office-source \
  --output local-assets/eth3d-office-run
```

The explicit `--download` fetches approximately 729 MB from a pinned mirror.
Existing files must match the pinned sizes and SHA-256 hashes. The output directory
must not already exist. It contains the mesh, a loadable MuJoCo scene, measured-ray
validation, two million nominal emissions, figures and source/license provenance.
An existing verified source directory can be supplied without `--download`.

Only scan1 builds geometry; all scan2 returns remain withheld. No GPU, ROS, vendor
SDK, private recording or decoder is required. Inputs and generated assets stay
local. This is an offline scene example, not a second dynamics world in an adapter.

ETH3D Office is by Schöps et al., *A Multi-View Stereo Benchmark with
High-Resolution Images and Multi-Camera Videos*, CVPR 2017.
[Dataset](https://eth3d.ethz.ch/) and derived scene illustrations retain
[CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/).
The original example code is MIT OR Apache-2.0. The dataset is not bundled.
