// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
import { defineConfig } from 'vitepress';

export default defineConfig({
  title: 'mid360-sim',
  description: 'Rolling MID-360 LiDAR acquisition for Rust, ROS 2, Isaac Sim, MuJoCo and Gazebo.',
  base: '/mid360-sim/',
  // Keep copyright headers at the beginning of Markdown files. Home metadata
  // lives here because Markdown frontmatter must otherwise be the first bytes.
  transformPageData(page) {
    if (page.relativePath === 'index.md') Object.assign(page.frontmatter, {
      layout: 'home',
      hero: {
        name: 'mid360-sim', text: 'Rolling LiDAR for your simulator.',
        tagline: 'A compact MID-360 scan approximation with explicit acquisition times, simulator adapters and optional ROS 2 publishing.',
        actions: [
          { theme: 'brand', text: 'Choose your simulator', link: '/guide/getting-started' },
          { theme: 'alt', text: 'Understand the model', link: '/guide/acquisition' },
        ],
      },
      features: [
        { title: 'Keep motion in the measurements', details: 'Per-emission timestamps and local points let navigation stacks test their own deskewing. Acquisition and publication schedules stay separate.' },
        { title: 'Query your existing scene', details: 'Rust scene queries, Isaac Warp meshes, MuJoCo CPU rays and Gazebo physics rays. No second dynamics world.' },
        { title: 'Know what is approximate', details: 'Hardware-derived optics, representative clocks and reproducible motion comparisons, with measured limits kept separate from simulation choices.' },
      ],
    });
  },
  cleanUrls: false,
  lastUpdated: true,
  themeConfig: {
    nav: [
      { text: 'Guide', link: '/guide/getting-started' },
      { text: 'Install', link: '/install/isaac-sim' },
      { text: 'Validation', link: '/guide/validation' },
      { text: 'Releases', link: '/distribution' },
    ],
    sidebar: [
      { text: 'Start here', items: [
        { text: 'Choose your simulator', link: '/guide/getting-started' },
        { text: 'Acquisition and motion', link: '/guide/acquisition' },
        { text: 'Scan patterns', link: '/guide/patterns' },
        { text: 'Validation and limits', link: '/guide/validation' },
      ] },
      { text: 'Install and integrate', items: [
        { text: 'Rust', link: '/install/rust' },
        { text: 'ROS 2 Jazzy', link: '/install/ros2' },
        { text: 'Isaac Sim', link: '/install/isaac-sim' },
        { text: 'MuJoCo', link: '/install/mujoco' },
        { text: 'Gazebo Harmonic', link: '/install/gazebo' },
      ] },
      { text: 'Release and contribute', items: [
        { text: 'Distribution', link: '/distribution' },
        { text: 'Development', link: '/development' },
        { text: 'Changelog', link: '/changelog' },
        { text: 'Provenance', link: '/provenance' },
      ] },
      { text: 'Evidence and research', collapsed: true, items: [
        { text: 'Realism assessment', link: '/realism' },
        { text: 'Hardware capture', link: '/hardware-validation' },
        { text: 'Hardware fit', link: '/hardware-fit' },
        { text: 'Independent captures', link: '/external-captures' },
        { text: 'Controlled motion', link: '/motion-distortion' },
        { text: 'Motion ground truth', link: '/motion-ground-truth' },
        { text: 'Recorded-motion results', link: '/motion-comparison-results' },
        { text: 'Official reference', link: '/official-reference' },
      ] },
    ],
    search: { provider: 'local' },
    socialLinks: [{ icon: 'github', link: 'https://github.com/hxyulin/mid360-sim' }],
    editLink: { pattern: 'https://github.com/hxyulin/mid360-sim/edit/main/docs/:path' },
    footer: { message: 'MIT OR Apache-2.0 · Livox and measurement provenance retained.' },
  },
});
