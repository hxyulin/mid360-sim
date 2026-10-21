// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
// Check actual production HTML, including sidebar links and emitted asset paths.
import { readdir, readFile, stat } from 'node:fs/promises';
import path from 'node:path';
const root = path.resolve('docs/.vitepress/dist');
const base = '/mid360-sim/';
async function walk(dir) {
  const entries = await readdir(dir, { withFileTypes: true });
  return (await Promise.all(entries.map(e => e.isDirectory() ? walk(path.join(dir, e.name)) : [path.join(dir, e.name)]))).flat();
}
const files = await walk(root);
let checked = 0;
for (const file of files.filter(f => f.endsWith('.html'))) {
  const html = await readFile(file, 'utf8');
  if (html.includes('/Users/') || html.includes('/home/hxyulin/')) throw new Error(`Private machine path in ${file}`);
  for (const match of html.matchAll(/(?:href|src)="([^"#]+)"/g)) {
    const link = match[1].split(/[?#]/)[0];
    if (/^(?:[a-z]+:|\/\/)/i.test(link) || !link) continue;
    if (link.startsWith('/') && !link.startsWith(base)) throw new Error(`Wrong site base: ${link}`);
    let target = link.startsWith(base) ? path.join(root, link.slice(base.length)) : path.resolve(path.dirname(file), link);
    if ((await stat(target).catch(() => null))?.isDirectory()) target = path.join(target, 'index.html');
    if (!(await stat(target).catch(() => null))?.isFile()) throw new Error(`Broken link ${link} in ${file}`);
    checked++;
  }
}
for (const page of ['index.html', 'install/isaac-sim.html', 'install/mujoco.html', 'install/gazebo.html', 'install/ros2.html', 'install/rust.html', 'distribution.html']) {
  if (!files.includes(path.join(root, page))) throw new Error(`Missing ${page}`);
}
console.log(`Checked ${checked} production links/assets across ${files.filter(f => f.endsWith('.html')).length} HTML pages.`);
