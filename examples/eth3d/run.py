#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Public ETH3D Office → optical mesh → held-out rays → native MID360 scan.

The optional dependencies are isolated in requirements.txt. Dataset inputs and
generated assets retain ETH3D's CC BY-NC-SA-4.0 terms; they are not bundled here.
"""

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

REVISION = "fb2ddc77390926c335c48dace97533f519fdd571"
FILES = {
    "scan1.ply": (
        361952514,
        "2d657e015b5111c7df1662677a6c0816e94859b331ca9718bb1f60b7f7def3e8",
    ),
    "scan2.ply": (
        366947229,
        "27132032ae29cd6c9acf50df81464e0e331fadfa1453d0545121bd8cb3e72044",
    ),
    "scan_alignment.mlp": (
        509,
        "b9b4c539b04c0a7493a3a8d013871f83daaf2895f007432e9ba4562061eba53c",
    ),
}
SEED = 20261022


def write_json(path, value):
    """Write an inspectable report, rejecting nonfinite numeric JSON values."""
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def verify(path, size, sha256):
    """Validate exact source bytes before parsing or reusing a download."""
    if path.stat().st_size != size:
        raise ValueError(f"Wrong size: {path.name}")
    with path.open("rb") as stream:
        digest = hashlib.file_digest(stream, "sha256").hexdigest()
    if digest != sha256:
        raise ValueError(f"Wrong SHA-256: {path.name}")


def acquire(directory, download):
    """Fetch only the pinned Office files when explicitly requested."""
    directory.mkdir(parents=True, exist_ok=True)
    for name, (size, digest) in FILES.items():
        path = directory / name
        if not path.exists():
            if not download:
                raise FileNotFoundError(
                    f"{path}; pass --download or supply verified files"
                )
            url = f"https://huggingface.co/datasets/PeterDAI/eth3d/resolve/{REVISION}/office/scan_clean/{name}?download=true"
            temporary = directory / (name + ".partial")
            print(f"Downloading {name} ({size:,} bytes)", flush=True)
            with urllib.request.urlopen(url, timeout=60) as response, temporary.open(
                "wb"
            ) as stream:
                while chunk := response.read(1024 * 1024):
                    stream.write(chunk)
            verify(temporary, size, digest)
            temporary.replace(path)
        verify(path, size, digest)


def load_transforms(path):
    """Read named scan-to-world matrices without following paths from XML."""
    transforms = {}
    for item in ET.parse(path).iter("MLMesh"):
        name = Path(item.attrib["filename"]).name
        matrix = np.fromstring(item.find("MLMatrix44").text, sep=" ").reshape(4, 4)
        if not np.isfinite(matrix).all() or not np.allclose(matrix[3], [0, 0, 0, 1]):
            raise ValueError("Invalid scan transform")
        if (
            not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=2e-6)
            or np.linalg.det(matrix[:3, :3]) < 0
        ):
            raise ValueError("Nonrigid scan transform")
        transforms[name] = matrix
    if set(transforms) != {"scan1.ply", "scan2.ply"}:
        raise ValueError("Expected the two Office scan transforms")
    return transforms


def load_scan(path, transform):
    """Place finite source points/colours within 20 m of their source origin."""
    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points)
    placed = np.einsum("ij,kj->ik", points, transform[:3, :3]) + transform[:3, 3]
    ranges = np.linalg.norm(points, axis=1)
    keep = np.isfinite(placed).all(axis=1) & (ranges > 0) & (ranges < 20)
    return placed[keep], np.asarray(cloud.colors)[keep]


def split_indices(count, samples, rng):
    """Reserve every fifth filtered point before processing either view.

    Return a training mask and sampled validation indices. Unselected validation
    points also remain excluded from geometry; no viewpoint is wholly withheld.
    """
    if count <= 0 or samples <= 0:
        raise ValueError("Positive point and sample counts required")
    train = np.arange(count) % 5 != 0
    reserved = np.flatnonzero(~train)
    if samples > len(reserved):
        raise ValueError("Not enough reserved points for the requested sample")
    return train, rng.choice(reserved, samples, replace=False)


def prepare_cloud(points, colors, origin):
    """Voxelize one training view and orient normals toward its scanner origin."""
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    cloud.colors = o3d.utility.Vector3dVector(colors)
    cloud = cloud.voxel_down_sample(0.01)
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.04, max_nn=32))
    cloud.orient_normals_towards_camera_location(origin)
    return cloud


def fuse_clouds(reference, additional):
    """Merge nearby, normal-compatible representatives before joint meshing.

    A 5 mm proximity and normal dot >=0.95 are reconstruction heuristics.
    Incompatible/opposing surfaces stay separate. Multiple additional points
    assigned to one reference are averaged as one group, independent of order.
    Inputs retain their original positions, colours and source-oriented normals.
    """
    p1, p2 = np.asarray(reference.points), np.asarray(additional.points)
    n1, n2 = np.asarray(reference.normals), np.asarray(additional.normals)
    if not len(p1) or not len(p2):
        raise ValueError("Two nonempty prepared clouds required")
    distance, nearest = cKDTree(p1).query(p2)
    merge = (distance <= 0.005) & (np.einsum("ij,ij->i", n2, n1[nearest]) >= 0.95)
    counts = 1 + np.bincount(nearest[merge], minlength=len(p1))
    fused = o3d.geometry.PointCloud()
    for attribute in ("points", "normals", "colors"):
        first = np.asarray(getattr(reference, attribute))
        second = np.asarray(getattr(additional, attribute))
        average = np.column_stack(
            [
                (
                    first[:, j]
                    + np.bincount(
                        nearest[merge], weights=second[merge, j], minlength=len(p1)
                    )
                )
                / counts
                for j in range(3)
            ]
        )
        values = np.concatenate([average, second[~merge]])
        if attribute == "normals":
            values /= np.maximum(np.linalg.norm(values, axis=1)[:, None], 1e-15)
        setattr(fused, attribute, o3d.utility.Vector3dVector(values))
    return fused, dict(
        reference_representatives=len(p1),
        additional_representatives=len(p2),
        merged_additional_representatives=int(merge.sum()),
        fused_representatives=len(fused.points),
    )


def reconstruct(cloud):
    """Mesh the fused training cloud without hole filling or free-space pruning."""
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        cloud, o3d.utility.DoubleVector([0.015, 0.025, 0.04])
    )
    q = np.asarray(mesh.vertices)[np.asarray(mesh.triangles)]
    maximum_edge = np.maximum.reduce(
        [np.linalg.norm(q[:, j] - q[:, (j + 1) % 3], axis=1) for j in range(3)]
    )
    mesh.remove_triangles_by_mask(maximum_edge > 0.06)
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    return mesh


def optical_scene(mesh):
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    return scene


def score(ranges, truth):
    """Report misses separately from range errors conditional on finite hits."""
    ranges, truth = np.asarray(ranges), np.asarray(truth)
    if (
        ranges.ndim != 1
        or ranges.shape != truth.shape
        or not len(truth)
        or not np.isfinite(truth).all()
        or np.any(truth <= 0)
    ):
        raise ValueError("Expected nonempty matched ranges and positive finite truth")
    hit = np.isfinite(ranges) & (ranges >= 0)
    error = ranges - truth
    absolute = np.abs(error[hit])
    return dict(
        rays=len(truth),
        hit_fraction=float(hit.mean()),
        within_20mm_fraction=float(np.mean(hit & (np.abs(error) <= 0.02))),
        early_over_20mm_fraction=float(np.mean(hit & (error < -0.02))),
        late_over_20mm_fraction=float(np.mean(hit & (error > 0.02))),
        median_mm=float(np.median(absolute) * 1000) if len(absolute) else None,
        p95_mm=float(np.percentile(absolute, 95) * 1000) if len(absolute) else None,
        rms_mm=float(np.sqrt(np.mean(absolute**2)) * 1000) if len(absolute) else None,
    )


def query(scene, points, origin):
    delta = points - origin
    truth = np.linalg.norm(delta, axis=1)
    rays = np.c_[np.broadcast_to(origin, delta.shape), delta / truth[:, None]].astype(
        np.float32
    )
    return scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy(), truth


def native_scan(mesh, transforms, tests, output):
    """Run the shipped MuJoCo adapter in the same source-derived optical scene."""
    import mujoco

    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "integrations/python"))
    from mid360_sim.mujoco import MujocoSensor

    o3d.io.write_triangle_mesh(str(output / "room.obj"), mesh, write_triangle_uvs=False)
    pose = transforms["scan2.ply"]
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, np.ascontiguousarray(pose[:3, :3]).ravel())
    position = " ".join(map(str, pose[:3, 3]))
    quat = " ".join(map(str, quaternion))
    xml = f'<mujoco><asset><mesh name="room" file="room.obj" maxhullvert="64"/></asset><worldbody><geom name="optical_room" type="mesh" mesh="room" contype="0" conaffinity="0"/><site name="lidar" pos="{position}" quat="{quat}" size=".01"/></worldbody></mujoco>'
    (output / "scene.xml").write_text(xml + "\n")
    started = time.perf_counter()
    model = mujoco.MjModel.from_xml_path(str(output / "scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    compile_s = time.perf_counter() - started
    replay = {}
    for name, points, origin in tests:
        delta = points - origin
        truth = np.linalg.norm(delta, axis=1)
        ranges = np.empty(len(points))
        ids = np.empty(len(points), np.int32)
        mujoco.mj_multiRay(
            model,
            data,
            np.ascontiguousarray(origin),
            np.ascontiguousarray(delta / truth[:, None]).ravel(),
            None,
            1,
            -1,
            ids,
            ranges,
            None,
            len(points),
            30,
        )
        replay[name] = score(ranges, truth)
    sensor = MujocoSensor(model, data, "lidar", min_range_m=0.1, max_range_m=30)
    sensor.scan(0, 20000)
    started = time.perf_counter()
    frames = [sensor.scan(i * 200000, 200000) for i in range(10)]
    elapsed = time.perf_counter() - started
    ranges = np.concatenate([f.ranges_m for f in frames])
    stamps = np.concatenate([f.timestamps_ns for f in frames])
    assert len(ranges) == 2000000 and np.all(np.diff(stamps) == 5000)
    np.savez_compressed(
        output / "nominal-scan.npz",
        directions=np.concatenate([f.directions for f in frames]),
        ranges_m=ranges,
        timestamps_ns=stamps,
    )
    return dict(
        backend="production mid360_sim.mujoco.MujocoSensor / mj_multiRay",
        mujoco_version=mujoco.__version__,
        compile_s=compile_s,
        measured_ray_replay=replay,
        nominal=dict(
            emissions=len(ranges),
            simulated_s=10,
            acquisition_wall_s=elapsed,
            realtime_factor=10 / elapsed,
            valid_fraction=float(np.mean(ranges >= 0)),
            mount="scan2 source pose",
            phase_matched=False,
        ),
        scope="Stationary virtual MID360 in Faro-derived geometry; not hardware MID360 returns. Timing excludes compilation, rendering, physics stepping and publication.",
    )


def preview_rays(origin, width=1200, height=800):
    """Render toward -X with world +Z at the top of the image."""
    # Open3D 0.19's helper uses the image-down axis for this argument.
    return o3d.t.geometry.RaycastingScene.create_rays_pinhole(
        fov_deg=90,
        center=origin + [-1.0, 0, 0],
        eye=origin,
        up=[0, 0, -1],
        width_px=width,
        height_px=height,
    )


def range_panorama(scene, test, output):
    """Show reference, reconstruction, residuals and misses on identical rays."""
    import matplotlib.pyplot as plt
    from PIL import Image

    name, points, origin = test
    ranges, truth = query(scene, points, origin)
    delta = points - origin
    azimuth = np.degrees(np.arctan2(delta[:, 1], delta[:, 0]))
    elevation = np.degrees(
        np.arctan2(delta[:, 2], np.linalg.norm(delta[:, :2], axis=1))
    )
    x = np.floor((azimuth + 180) / 2).astype(int) % 180
    y = np.floor((elevation + 60) / 2).astype(int)
    inside = (y >= 0) & (y < 70)
    cells = y * 180 + x
    counts = np.bincount(cells[inside], minlength=12600).reshape(70, 180)

    def median(values, selected):
        result = np.full(12600, np.nan)
        selected = inside & selected
        order = np.argsort(cells[selected])
        ids, values = cells[selected][order], values[selected][order]
        keys, starts, sizes = np.unique(ids, return_index=True, return_counts=True)
        for key, start, size in zip(keys, starts, sizes):
            result[key] = np.median(values[start : start + size])
        return np.where(counts >= 2, result.reshape(70, 180), np.nan)

    hit = np.isfinite(ranges)
    missing = np.bincount(cells[inside & ~hit], minlength=12600).reshape(70, 180)
    missing = np.where(counts >= 2, missing / np.maximum(counts, 1) * 100, np.nan)
    panels = [
        (
            median(truth, np.ones(len(truth), bool)),
            "turbo",
            10,
            "Held-out Faro reference",
            "Range (m)",
        ),
        (
            median(ranges, hit),
            "turbo",
            10,
            "Joint two-view reconstruction",
            "Range (m)",
        ),
        (
            median(np.abs(ranges - truth) * 1000, hit),
            "magma",
            100,
            "Median absolute per-ray error · hits only",
            "Error (mm)",
        ),
        (
            missing,
            "magma",
            100,
            "Missing first intersections · all reference rays",
            "Missing (%)",
        ),
    ]
    fig, axes = plt.subplots(4, 1, figsize=(12, 9), layout="constrained")
    for ax, (values, cmap, limit, title, label) in zip(axes, panels):
        colors = plt.colormaps[cmap].copy()
        colors.set_bad("#d7dce1")
        image = ax.imshow(
            values,
            origin="lower",
            extent=(-180, 180, -60, 80),
            aspect="auto",
            cmap=colors,
            vmin=0,
            vmax=limit,
        )
        ax.set(
            title=title, xlabel="World-relative azimuth (deg)", ylabel="Elevation (deg)"
        )
        fig.colorbar(image, ax=ax, fraction=0.02, pad=0.01, label=label)
    fig.suptitle(name.replace("_", " ") + " · identical measured rays")
    fig.supxlabel(
        "2° bins, at least two reference returns/bin; grey = insufficient support. ETH3D Office · Schöps et al., CVPR 2017 · CC BY-NC-SA 4.0.",
        fontsize=8,
    )
    fig.savefig(output / "range-panorama.png", dpi=160)
    plt.close(fig)
    Image.open(output / "range-panorama.png").save(
        output / "range-panorama.webp", quality=90
    )


def figures(scene, mesh, tests, output):
    """Write attributed geometry and miss-aware CDF figures, without source RGB."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image

    fig, ax = plt.subplots(figsize=(8, 4.5), layout="constrained")
    for name, points, origin in tests:
        ranges, truth = query(scene, points, origin)
        error = np.sort(
            np.abs(ranges[np.isfinite(ranges)] - truth[np.isfinite(ranges)]) * 1000
        )
        ax.plot(
            error,
            np.arange(1, len(error) + 1) / len(truth) * 100,
            label=name.replace("_", " "),
        )
    ax.set(
        xscale="log",
        xlim=(0.05, 3000),
        ylim=(0, 100),
        xlabel="Absolute range error (mm)",
        ylabel="All evaluation rays within error (%)",
    )
    ax.axvline(20, color="gray", linestyle="--", linewidth=1)
    ax.grid(alpha=0.2)
    ax.legend()
    fig.savefig(output / "coverage-error.svg", metadata={"Date": None})
    svg = output / "coverage-error.svg"
    svg.write_text(
        "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
    )
    plt.close(fig)
    range_panorama(scene, tests[1], output)
    origin = tests[0][2]
    image = preview_rays(origin)
    hit = scene.cast_rays(image)
    depth = hit["t_hit"].numpy()
    normal = hit["primitive_normals"].numpy()
    direction = image.numpy()[:, :, 3:]
    direction /= np.linalg.norm(direction, axis=2)[:, :, None]
    intensity = 0.35 + 0.65 * np.abs(np.einsum("ijk,ijk->ij", normal, direction))
    rgb = np.full((800, 1200, 3), [19, 23, 29], np.uint8)
    valid = np.isfinite(depth)
    rgb[valid] = np.clip(intensity[valid, None] * [220, 226, 234], 0, 255).astype(
        np.uint8
    )
    Image.fromarray(rgb).save(output / "office-geometry.webp", quality=88)
    scan = np.load(output / "nominal-scan.npz")
    valid = scan["ranges_m"] >= 0
    simulated = scan["directions"][valid] * scan["ranges_m"][valid, None]
    # Re-express held-out Faro returns in the virtual mount's local frame.
    # The caller supplies the source pose through the saved mount rotation.
    rotation = np.load(output / "mount-rotation.npy")
    reference = np.einsum("ij,jk->ik", tests[1][1] - tests[1][2], rotation)
    fig = plt.figure(figsize=(10, 4.5), layout="constrained")
    rng = np.random.default_rng(SEED)
    for index, (title, points) in enumerate(
        [
            ("Faro reference · full field of view", reference),
            ("Virtual MID360 · nominal pattern", simulated),
        ]
    ):
        points = points[rng.choice(len(points), min(40000, len(points)), replace=False)]
        ax = fig.add_subplot(1, 2, index + 1, projection="3d")
        ax.scatter(
            *points.T,
            c=points[:, 2],
            s=0.2,
            vmin=-2,
            vmax=2,
            cmap="viridis",
            rasterized=True,
        )
        ax.set(
            xlim=(-4, 4),
            ylim=(-4, 4),
            zlim=(-2, 2),
            xlabel="x (m)",
            ylabel="y (m)",
            zlabel="z (m)",
        )
        ax.set_box_aspect([8, 8, 4])
        ax.view_init(25, -65)
        ax.set_title(title, fontsize=10)
    fig.savefig(output / "nominal-points.png", dpi=160)
    plt.close(fig)
    Image.open(output / "nominal-points.png").save(
        output / "nominal-points.webp", quality=88
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data",
        type=Path,
        required=True,
        help="Directory with the three verified Office files",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="New local directory; never overwritten",
    )
    parser.add_argument(
        "--download",
        action="store_true",
        help="Explicitly download ~729 MB from the pinned mirror",
    )
    args = parser.parse_args()
    if args.output.exists():
        parser.error(
            "Output already exists; choose a new directory to preserve reports"
        )
    acquire(args.data, args.download)
    args.output.mkdir(parents=True)
    transforms = load_transforms(args.data / "scan_alignment.mlp")
    started = time.perf_counter()
    rng = np.random.default_rng(SEED)
    tests, clouds, splits = [], [], {}
    for name in ("scan1", "scan2"):
        pose = transforms[name + ".ply"]
        points, colors = load_scan(args.data / (name + ".ply"), pose)
        train, sample = split_indices(len(points), 200000, rng)
        # Both views reserve points BEFORE voxelization, normals or fusion.
        tests.append((name + "_heldout", points[sample], pose[:3, 3]))
        clouds.append(prepare_cloud(points[train], colors[train], pose[:3, 3]))
        splits[name] = dict(
            filtered_points=len(points),
            training_points=int(train.sum()),
            reserved_points=int((~train).sum()),
            sampled_validation_points=len(sample),
            training_mask_sha256=hashlib.sha256(train.tobytes()).hexdigest(),
            validation_indices_sha256=hashlib.sha256(sample.tobytes()).hexdigest(),
        )
        np.save(args.output / (name + "-validation-indices.npy"), sample)
        del points, colors
    cloud, fusion = fuse_clouds(*clouds)
    mesh = reconstruct(cloud)
    del cloud, clouds
    scene = optical_scene(mesh)
    results = {name: score(*query(scene, points, eye)) for name, points, eye in tests}
    o3d.io.write_triangle_mesh(str(args.output / "training-mesh.ply"), mesh)
    for name, points, eye in tests:
        np.savez_compressed(args.output / (name + ".npz"), points=points, origin=eye)
    native = native_scan(mesh, transforms, tests, args.output)
    np.save(args.output / "mount-rotation.npy", transforms["scan2.ply"][:3, :3])
    figures(scene, mesh, tests, args.output)
    adapter_root = (
        Path(__file__).resolve().parents[2] / "integrations/python/mid360_sim"
    )
    adapter_hashes = {
        name: hashlib.sha256((adapter_root / name).read_bytes()).hexdigest()
        for name in ["mujoco.py", "pattern.py", "nominal.json"]
    }
    report = dict(
        source=dict(
            dataset="ETH3D Office clean Faro scans",
            upstream="https://eth3d.ethz.ch/datasets",
            mirror_revision=REVISION,
            files={k: dict(bytes=v[0], sha256=v[1]) for k, v in FILES.items()},
            license="CC-BY-NC-SA-4.0",
            mirror_hashes_not_upstream_archive_verification=True,
        ),
        seed=SEED,
        settings=dict(
            voxel_m=0.01,
            normal_radius_m=0.04,
            normal_neighbors=32,
            pivot_radii_m=[0.015, 0.025, 0.04],
            maximum_edge_m=0.06,
            range_filter_m=20,
            train="both scans: filtered indices i%5 != 0",
            validation="both scans: filtered indices i%5 == 0",
            merge_max_distance_m=0.005,
            merge_min_normal_dot=0.95,
            evaluation_rays_per_view=200000,
            scan2_used_for_geometry=True,
            free_space_pruning=False,
            hole_filling=False,
        ),
        splits=splits,
        fusion=fusion,
        geometry=dict(vertices=len(mesh.vertices), triangles=len(mesh.triangles)),
        evaluation=results,
        native=native,
        adapter_sha256=adapter_hashes,
        elapsed_s=time.perf_counter() - started,
        caveats=[
            "Faro scene reconstruction accuracy, not a calibrated MID360 comparison.",
            "Range errors conditional on hits; coverage includes misses.",
            "Both views build geometry; excluded points do not test unseen-view generalization.",
            "Remaining errors are not semantically classified as windows or infinite ranges.",
            "No motion, material-response, dropout or real device phase validation.",
        ],
    )
    write_json(args.output / "report.json", report)
    (args.output / "ATTRIBUTION.txt").write_text(
        "ETH3D Office, Schops et al., CVPR 2017. Source https://eth3d.ethz.ch/. Dataset and derived scene illustrations: CC BY-NC-SA 4.0 https://creativecommons.org/licenses/by-nc-sa/4.0/. Changes: source cleaning by ETH3D; local voxelization, meshing and synthetic shading. Source dataset is not part of the simulator's code license.\n"
    )
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
