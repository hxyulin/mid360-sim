#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Generate documentation figures from the nominal model and recorded results.

Uses NumPy/Matplotlib from requirements-analysis.txt; never reruns a simulator or
claims new measurements. --check verifies evidence values and committed hashes.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / 'docs/public/figures'
TEAL, BLUE, INK, MUTED, GREY = '#0f766e', '#2563eb', '#172b45', '#53657b', '#a2adba'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def row(source, prefix):
    matches = [line for line in (ROOT / source).read_text().splitlines() if line.startswith(prefix)]
    if len(matches) != 1:
        raise ValueError(f'Expected exactly one evidence row: {prefix}')
    return [cell.strip().replace('**', '') for cell in matches[0].split('|')[1:-1]]


def verify(data):
    """Keep plotted values tied to the named baseline rows, not later optimizations."""
    assert data['schema'] == 1
    source = data['motion']['source']
    prefixes = ['| Ours MuJoCo, 1 ms held poses |', '| Ours MuJoCo, rolling poses,',
                '| Ours Isaac/Warp, bulk history, headless']
    assert data['motion']['rms_mm'] == [float(row(source, p)[3].split()[0]) for p in prefixes]
    official = row(source, '| Official Livox / Gazebo 9,')
    warp = row(source, prefixes[2])
    assert data['runtime']['whole_loop_ms'] == [float(r[1].split()[0]) for r in [official, warp]]
    for value in data['runtime']['sensor_interval_ms']:
        assert f'{value:.2f} ms' in (ROOT / source).read_text()
    for label, p50, p99 in zip(data['transport']['labels'], data['transport']['p50_ms'], data['transport']['p99_ms']):
        cells = row(data['transport']['source'], f'| {label} |')
        assert [p50, p99] == [float(cells[3]), float(cells[4])]
    text = (ROOT / data['transport']['source']).read_text()
    assert '305 clouds and 6,100 IMU messages each' in text
    assert data['transport']['clouds_received_per_format'] == 305
    assert data['transport']['imu_received_per_format'] == 6100
    model = json.loads((ROOT / data['pattern']['source']).read_text())
    assert 1_000_000_000 // model['point_interval_ns'] == data['pattern']['emissions_per_second']


def generate(data, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np

    plt.rcParams.update({
        'font.family': ['DejaVu Sans', 'sans-serif'], 'font.size': 12, 'text.color': INK,
        'axes.labelcolor': MUTED, 'xtick.color': MUTED, 'ytick.color': INK,
        'axes.edgecolor': '#dbe3ec', 'axes.spines.top': False,
        'axes.spines.right': False, 'axes.spines.left': False,
        'axes.titleweight': 'bold', 'svg.fonttype': 'none',
        'svg.hashsalt': 'mid360-sim-figures-v1', 'figure.facecolor': 'white',
        'savefig.facecolor': 'white',
    })
    output.mkdir(parents=True, exist_ok=True)
    metadata = {'Creator': 'mid360-sim / scripts/generate-doc-figures.py', 'Date': None}
    descriptions = {}

    def save(fig, name, title, description):
        path = output / name
        if path.suffix == '.svg':
            fig.savefig(path, metadata={**metadata, 'Title': title, 'Description': description,
                                        'Rights': 'MIT OR Apache-2.0; see NOTICE.md for model provenance.'})
        elif path.suffix == '.webp':
            fig.savefig(path, dpi=120, pil_kwargs={'lossless': True, 'method': 6})
        else:
            fig.savefig(path, dpi=160, metadata={'Title': title, 'Description': description,
                                               'Software': 'mid360-sim documentation figures'})
        descriptions[name] = {'title': title, 'description': description}
        plt.close(fig)

    # Load the committed adapter directly: no installed simulator or dataset.
    spec = importlib.util.spec_from_file_location('docs_pattern', ROOT / 'integrations/python/mid360_sim/pattern.py')
    pattern = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = pattern
    spec.loader.exec_module(pattern)
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.5))
    fig.subplots_adjust(left=.075, right=.98, bottom=.29, top=.86, wspace=.22)
    colors = np.array([TEAL, BLUE, '#8b5bb7', '#be731a'])
    for ax, duration in zip(axes, data['pattern']['windows_s']):
        count = round(duration * data['pattern']['emissions_per_second'])
        directions = pattern.directions(0, count)
        assert np.allclose(np.linalg.norm(directions, axis=1), 1, rtol=0, atol=2e-14)
        x = np.degrees(np.arctan2(directions[:, 1], directions[:, 0]))
        y = np.degrees(np.arcsin(np.clip(directions[:, 2], -1, 1)))
        ax.scatter(x, y, s=.45, c=colors[np.arange(count) % 4], alpha=.65, linewidths=0, rasterized=True)
        ax.set(xlim=(-180, 180), ylim=(-10, 55), xlabel='Azimuth (degrees)',
               xticks=[-180, -90, 0, 90, 180], yticks=[0, 15, 30, 45])
        ax.set_title(f'{duration:g} s  /  {count:,} emissions', loc='left', fontsize=12, pad=12)
        ax.grid(alpha=.18); ax.set_axisbelow(True)
    axes[0].set_ylabel('Elevation (degrees)')
    fig.text(.075, .09, 'Nominal scan · 200,000 emissions/s · colour = lane · both windows start at emission zero', fontsize=10, color=MUTED)
    save(fig, 'scan-pattern.webp', 'Nominal scan coverage at 100 ms and one second', data['pattern']['scope'])

    fig, ax = plt.subplots(figsize=(8, 3.1))
    fig.subplots_adjust(left=.39, right=.91, top=.94, bottom=.32)
    values = data['motion']['rms_mm']
    bars = ax.barh([0, 1, 2], values, height=.48, color=[GREY, TEAL, BLUE])
    ax.set_yticks([0, 1, 2], data['motion']['labels']); ax.invert_yaxis()
    ax.set_xlim(0, 6.7); ax.set_xticks([0, 2, 4, 6]); ax.set_xlabel('Range RMS (mm)')
    ax.xaxis.grid(alpha=.18); ax.set_axisbelow(True); ax.tick_params(axis='y', length=0, pad=10)
    for bar, value in zip(bars, values):
        ax.text(value+.12, bar.get_y()+bar.get_height()/2, f'{value:.3f}', va='center', weight='bold', fontsize=13)
    fig.text(.05, .06, 'Motion replay · 400,000 emissions · reference residuals, not hardware precision', fontsize=9.5, color=MUTED, linespacing=1.6)
    save(fig, 'motion-accuracy.svg', 'Measured motion replay: held and rolling acquisition', data['motion']['scope'])

    fig, axes = plt.subplots(1, 2, figsize=(10, 3.1))
    fig.subplots_adjust(left=.07, right=.97, top=.84, bottom=.35, wspace=.26)
    for ax, name, label in zip(axes, ['whole_loop_ms', 'sensor_interval_ms'], ['Whole loop (physics included)', 'Sensor / callback interval']):
        values = data['runtime'][name]
        bars = ax.barh([0, 1], values, height=.44, color=[GREY, TEAL])
        ax.set_yticks([0, 1], ['Official', 'Warp']); ax.invert_yaxis(); ax.set_xlim(0, 170)
        ax.set_title(label, loc='left', fontsize=12, pad=16)
        ax.set_xticks([0, 50, 100, 150]); ax.xaxis.grid(alpha=.18); ax.set_axisbelow(True)
        ax.tick_params(axis='y', length=0)
        for bar, value in zip(bars, values):
            ax.text(value+3, bar.get_y()+bar.get_height()/2, f'{value:.2f}', va='center', weight='bold')
        ax.set_xlabel('Wall time (ms)')
    fig.text(.07, .12, '2 s simulation · static room · Ryzen 9 9950X3D / RTX 5090', fontsize=10, color=MUTED)
    fig.text(.07, .055, 'Warp: bulk history, headless, USD off · scan patterns and timing boundaries differ', fontsize=9.5, color=MUTED)
    save(fig, 'runtime.svg', 'Original static-room runtime comparison', data['runtime']['scope'])

    fig, ax = plt.subplots(figsize=(8, 3.3))
    fig.subplots_adjust(left=.26, right=.96, top=.83, bottom=.35)
    y = np.arange(2)
    ax.barh(y-.15, data['transport']['p50_ms'], height=.25, label='p50', color=TEAL)
    ax.barh(y+.15, data['transport']['p99_ms'], height=.25, label='p99', color=BLUE)
    ax.set_yticks(y, data['transport']['labels']); ax.invert_yaxis(); ax.set_xlim(0, 49)
    ax.set_xticks([0, 10, 20, 30, 40]); ax.xaxis.grid(alpha=.18); ax.set_axisbelow(True)
    ax.tick_params(axis='y', length=0, pad=10); ax.set_xlabel('Publish → Python callback (ms)')
    for n, values in enumerate([data['transport']['p50_ms'], data['transport']['p99_ms']]):
        for i, value in enumerate(values):
            ax.text(value+.65, i+(-.15 if n==0 else .15), f'{value:.3f}', va='center', fontsize=11)
    fig.legend(*ax.get_legend_handles_labels(), loc='upper right', bbox_to_anchor=(.96, .99), frameon=False, ncol=2, fontsize=10)
    fig.text(.05, .12, '30 s · one stream · Cyclone / macOS arm64 / M5 Max · 305 clouds + 6,100 IMUs per format', fontsize=9, color=MUTED)
    fig.text(.05, .055, 'Includes Python conversion and scheduling · loaded runs also had missing deliveries', fontsize=9, color=MUTED)
    save(fig, 'ros-callbacks.svg', 'ROS Python callback intervals for two message formats', data['transport']['scope'])

    inputs = {data[k]['source'] for k in ['pattern', 'motion', 'runtime', 'transport']}
    manifest = {'schema': 1, 'evidence_source_revision': data['source_revision'],
                'generator_sha256': digest(Path(__file__)),
                'generator': 'scripts/generate-doc-figures.py',
                'versions': {'numpy': np.__version__, 'matplotlib': matplotlib.__version__},
                'measurements_sha256': digest(DEFAULT/'measurements.json'),
                'inputs': {name: digest(ROOT/name) for name in sorted(inputs)},
                'figures': {name: {**value, 'sha256': digest(output/name)} for name, value in descriptions.items()}}
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=DEFAULT)
    parser.add_argument('--check', action='store_true', help='Verify values, provenance and figure hashes without plotting dependencies')
    args = parser.parse_args()
    data = json.loads((DEFAULT/'measurements.json').read_text())
    verify(data)
    if args.check:
        manifest = json.loads((args.output/'manifest.json').read_text())
        assert manifest['measurements_sha256'] == digest(DEFAULT/'measurements.json')
        assert manifest['generator_sha256'] == digest(Path(__file__)), 'Generator changed; review and regenerate'
        for name, checksum in manifest['inputs'].items():
            assert digest(ROOT/name) == checksum, f'Figure input changed: {name}; review and regenerate'
        for name, meta in manifest['figures'].items():
            assert digest(args.output/name) == meta['sha256'], f'Figure differs: {name}'
        print('Figure values, input provenance and four asset hashes agree.')
    else:
        generate(data, args.output)
        print(f'Generated four documentation figures in {args.output}')


if __name__ == '__main__':
    main()
