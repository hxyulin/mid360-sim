<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- Copyright (c) 2026 hxyulin <hxyulin@proton.me> -->
# Turn public scans into a LiDAR scene

This example combines **both ETH3D Office scanner views** into one optical mesh,
loads it into MuJoCo, and runs the existing MID360 adapter. Points from each scan
are reserved before reconstruction and used to check the resulting surfaces.

The useful result is a repeatable **scene-conversion test**. ETH3D's scans come
from a Faro survey scanner, so the range errors below describe reconstruction
of those surfaces—not calibrated MID360 hardware accuracy.

![Observed Office surfaces reconstructed from the first scan, with synthetic grey shading. Dark areas are missing geometry.](/eth3d/office-geometry.webp)

*ETH3D Office, Schöps et al., CVPR 2017. Derived illustration under
[CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/).
Voxelized, triangulated and rendered with synthetic lighting; source RGB is not used.*

## Run it

Use a source checkout, Python **3.12**, and several GB of available RAM. The
optional example dependencies stay outside the Rust core and ordinary adapters.
No GPU, ROS installation, Livox SDK or private decoder is required.

```sh
python3.12 -m venv local-assets/eth3d-venv
. local-assets/eth3d-venv/bin/activate
python -m pip install -r examples/eth3d/requirements.txt './integrations/python[mujoco]'
python -m unittest discover -s examples/eth3d -v

OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 python examples/eth3d/run.py \
  --download \
  --data local-assets/eth3d-office-source \
  --output local-assets/eth3d-office-run
```

`--download` explicitly fetches approximately **729 MB**. It downloads only
`scan1.ply`, `scan2.ply` and `scan_alignment.mlp` from the
[pinned mirror revision](https://huggingface.co/datasets/PeterDAI/eth3d/tree/fb2ddc77390926c335c48dace97533f519fdd571/office/scan_clean).
Every file is checked against its pinned byte count and SHA-256 before parsing.
The upstream server was unavailable during this experiment; mirror hashes
identify the downloaded bytes but do not independently verify an upstream archive.

Already have those exact files? Point `--data` at their directory and omit
`--download`. Choose a new output directory for each run; existing results are
never overwritten. Inputs and generated geometry remain under ignored
`local-assets/`, outside Git and releases.

The [example source](https://github.com/hxyulin/mid360-sim/tree/main/examples/eth3d)
and [machine-readable report](/eth3d/report.json) contain the full settings,
hashes, sample seed, adapter source hashes and native replay results.

## Keep geometry and validation apart

The [ETH3D format documentation](https://www.eth3d.net/documentation) specifies
binary PLY scan points and scan-to-world transforms in `scan_alignment.mlp`.
Each scanner origin is its local coordinate origin. Keep these origins when
transforming the points; a merged point cloud alone loses visibility information.
These are ETH3D's already-cleaned scans, not unprocessed sensor packets.

The example applies this sequence:

1. Place finite nonzero-range points within 20 m using the supplied transforms.
2. Split **each scan before processing**: filtered indices `i % 5 == 0` are
   validation; the other 80% supply geometry. Reserve all validation points,
   including those not selected for the 200,000-ray evaluation sample per view.
3. Average each training view into **1 cm voxels**. Estimate normals using a
   4 cm neighbourhood, at most 32 neighbours, oriented toward that view's scanner.
4. Merge scan2 representatives into nearby scan1 representatives only at
   **≤5 mm separation and normal dot ≥0.95**. Average compatible groups and keep
   unmatched or opposing surfaces separate. These are reconstruction heuristics,
   not sensor specifications. No ICP is fitted to validation points.
5. Ball-pivot the **joint point cloud once**, at **1.5 / 2.5 / 4 cm** radii.
   Remove triangles with an edge over **6 cm** and degenerate triangles. Leave
   unknown areas unfilled; do not overlay two independently finished meshes.
6. Raycast 200,000 reserved points per scan from their actual scanner origins,
   seed `20261022`, and repeat those queries through native MuJoCo.

There are no manual boundary patches or blanket free-space pruning. Preserving
source-view origins helps distinguish surface support, occlusion and unknown
coverage; a nearer mesh intersection alone is not a complete fusion policy.

**Both viewpoints now supply training data.** The tests measure reconstruction
of excluded observations at those scanner positions, not generalization to an
unseen third viewpoint. Input partitions and sampled index hashes are in the
report; local output also retains the exact sampled indices.

The exported and simulated mesh has **948,293 vertices and
1,864,268 triangles**, built from the two training partitions.

## Results: count missed rays too

For each recorded point, cast its actual direction into the reconstructed mesh.
Compare the first-hit range to the recorded range. The table uses **native
MuJoCo `mj_multiRay`**; the report also contains the closely agreeing Open3D result.

| Reserved observations | Mesh hit rate | All rays within 2 cm | Median error | p95 error | RMS error |
|---|---:|---:|---:|---:|---:|
| Scan1 | 99.57% | 99.26% | 0.46 mm | 2.35 mm | 38.08 mm |
| Scan2 | 99.57% | 99.23% | 0.45 mm | 2.13 mm | 43.63 mm |

**Median, p95 and RMS include hits only.** The hit rate and “all rays within
2 cm” columns include every evaluated ray, so missing geometry cannot disappear
from the score. Small typical errors reproduce dense survey observations; they
are not a precision specification for a virtual or real MID360.

![Cumulative absolute range error for reserved points from both source views, with misses included in the denominator.](/eth3d/coverage-error.svg)

![Matched-ray scan2 panorama: held-out Faro reference, joint reconstruction, absolute per-ray errors and missing fractions.](/eth3d/range-panorama.webp)

*ETH3D Office, Schöps et al., CVPR 2017. Derived figures, CC BY-NC-SA 4.0.
The panorama uses 2° bins with at least two reference samples; grey means
insufficient display support. Display binning does not change numerical scores.*

Remaining tails can include holes, depth boundaries, overlapping surface evidence,
registration uncertainty and reference measurement errors. A ray passing through
a foreground gap may hit a surface far behind it. The example does not classify
these residuals as windows or replace recorded finite ranges with infinity.

## Generate an actual nominal scan

The script writes `scene.xml` and `room.obj`, creates one MuJoCo scene, and places
a virtual `lidar` site at scan2's supplied pose. The production
`mid360_sim.mujoco.MujocoSensor` adapter generates **2,000,000 emissions over
10 simulated seconds**, with explicit 5 μs acquisition increments and `-1` misses.

![Faro reference points and a virtual MID360 nominal scan in the same scanner coordinate frame; different sampling and fields of view.](/eth3d/nominal-points.webp)

*Derived ETH3D illustration, CC BY-NC-SA 4.0. Display points are thinned and
coloured by height. These are different sampling patterns and fields of view,
not paired emissions. The spatial crop is identical for the two panels.*

`nominal-scan.npz` contains local `directions`, `ranges_m` and `timestamps_ns`.
Load it without a running simulator:

```python
import numpy as np

scan = np.load("local-assets/eth3d-office-run/nominal-scan.npz")
valid = scan["ranges_m"] >= 0
points_local_m = scan["directions"][valid] * scan["ranges_m"][valid, None]
times_ns = scan["timestamps_ns"][valid]
```

The recorded runtime in the report is **headless acquisition only**, one local
trial. It excludes scene compilation, physics stepping, rendering and ROS/DDS
publication; it is not a whole-simulator performance guarantee.

To continue querying this scene interactively or from a navigation harness:

```python
import mujoco
from mid360_sim.mujoco import MujocoSensor

model = mujoco.MjModel.from_xml_path("local-assets/eth3d-office-run/scene.xml")
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
sensor = MujocoSensor(model, data, "lidar", min_range_m=.1, max_range_m=30.)
frame = sensor.scan(first_index=0, count=200)  # one millisecond at full rate
```

For sensor motion, callers own pose history and scheduling; follow the
[MuJoCo rolling acquisition guide](../install/mujoco.md). This stationary example
does not validate real motion distortion or supply IMU measurements.

## Apply the procedure to your own scans

Keep per-view poses, scanner origins and timestamps. With moving captures,
establish pose/time alignment and deskew before accumulating surfaces. With
stationary captures, use time-separated observations to reject unstable ranges.
Save independent views or time windows for evaluation before fitting anything.

Reliable input coverage comes before local mesh repair. A single scan can miss
foreground surfaces that a different viewpoint needs; a merged PCD should keep
its source origins and alignment provenance. Reserve observations from every
source scan before fitting, and use a third view when testing unseen-view
generalization. More coverage does not make meshing or registration errors vanish.

For multi-view data, distinguish **observed free space**, **surface support**,
**occlusion**, and **unknown space**. A surface behind another recorded return
is occluded; that alone is not a reason to delete it. Missing returns do not
prove a region empty. Prefer corroborated additions to an already validated
surface over a union of every reconstructed sheet. Small local repairs should
be checked against training visibility and then against withheld rays.

Inspect optical geometry independently of collision proxies. The example mesh
geom has `contype="0" conaffinity="0"` to keep it out of contacts; it still
participates in native ray queries. No invisible fallback floor or room closure
is added. Physics contacts need a separate, explicitly designed collision policy.

There is no required simulator-core change for this workflow. Better observed
coverage and reliable scene fusion remain the main procedural opportunities.
This example does not calibrate range noise, dropout, beam footprint, infrared
reflectance, or a physical device's future scan phase. Visual colour is not
905 nm reflectance. See [validation and limits](validation.md) before treating
the output as hardware ground truth.

## Attribution and distribution

ETH3D Office is by Thomas Schöps, Johannes L. Schönberger, Silvano Galliani,
Torsten Sattler, Konrad Schindler, Marc Pollefeys and Andreas Geiger:
*A Multi-View Stereo Benchmark with High-Resolution Images and Multi-Camera
Videos*, CVPR 2017. [Dataset and license](https://eth3d.ethz.ch/).

The dataset and derived scene illustrations on this page retain
**[CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/)**.
Changes include voxelization, triangulation, synthetic shading, raycasting and
plotting. They are separate from the simulator code's MIT OR Apache-2.0 license.
Source scans and generated meshes are downloaded/generated locally, not bundled
with the simulator. See the [asset attribution](/eth3d/ATTRIBUTION.txt).
