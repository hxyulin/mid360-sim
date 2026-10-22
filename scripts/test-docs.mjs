// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
// Exercise production navigation/search at the actual GitHub Pages base path.
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { mkdir } from 'node:fs/promises';
import { chromium } from 'playwright';

const server = spawn(process.execPath, ['node_modules/vitepress/bin/vitepress.js', 'preview', 'docs', '--host', '127.0.0.1', '--port', '4178', '--strictPort'], { stdio: 'inherit' });
const origin = 'http://127.0.0.1:4178/mid360-sim/';
let browser;
try {
  let ready = false;
  for (let i = 0; i < 100; i++) {
    if (server.exitCode !== null) throw new Error('Preview server exited');
    ready = await fetch(origin).then(r => r.ok).catch(() => false);
    if (ready) break;
    await new Promise(resolve => setTimeout(resolve, 100));
  }
  assert(ready, 'Preview server did not start');
  browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
  const errors = [];
  page.on('pageerror', e => errors.push(e.message));
  await page.goto(origin);
  await page.getByRole('link', { name: 'Choose your simulator', exact: true }).click();
  await page.getByRole('heading', { name: /Getting started/, level: 1 }).waitFor();
  await page.goto(origin + 'install/mujoco.html');
  await page.getByRole('heading', { level: 1 }).waitFor();
  assert((await page.locator('main').innerText()).includes('mid360_sim_adapters-0.1.0'));
  await page.getByRole('button', { name: /Search/ }).first().click();
  await page.locator('#localsearch-input').fill('acquisition');
  await page.locator('.VPFeatures, .VPLocalSearchBox').last().waitFor();
  await page.locator('.VPLocalSearchBox .result').first().waitFor();
  await page.keyboard.press('Escape');
  await page.goto(origin + 'guide/scanned-geometry.html');
  await page.getByRole('heading', { name: 'Turn public scans into a LiDAR scene', level: 1 }).waitFor();
  assert.equal(await page.locator('main img').count(), 4);
  await page.waitForFunction(() => [...document.querySelectorAll('main img')].every(img => img.complete && img.naturalWidth > 0));
  const sceneReport = await page.request.get(origin + 'eth3d/report.json');
  assert(sceneReport.ok(), 'ETH3D report is unavailable');
  const report = await sceneReport.json();
  assert.equal(report.settings.scan2_used_for_geometry, true);
  assert.equal(report.settings.validation, 'both scans: filtered indices i%5 == 0');
  assert(report.evaluation.scan1_heldout && report.evaluation.scan2_heldout);
  await mkdir('local-assets/docs-review', { recursive: true });
  await page.goto(origin);
  await page.screenshot({ path: 'local-assets/docs-review/desktop.png', fullPage: true });
  await page.setViewportSize({ width: 390, height: 844 });
  await page.goto(origin + 'install/isaac-sim.html');
  await page.getByRole('heading', { level: 1 }).waitFor();
  assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth + 1), 'Mobile page overflows horizontally');
  await page.screenshot({ path: 'local-assets/docs-review/mobile.png', fullPage: false });
  await page.goto(origin + 'guide/scanned-geometry.html');
  await page.getByRole('heading', { level: 1 }).waitFor();
  assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth + 1), 'Scanned-geometry guide overflows on mobile');
  await page.screenshot({ path: 'local-assets/docs-review/scanned-geometry-mobile.png', fullPage: true });
  await page.goto(origin + 'guide/results.html');
  await page.getByRole('heading', { name: 'Results at a glance', level: 1 }).waitFor();
  assert.equal(await page.locator('main img').count(), 4);
  await page.waitForFunction(() => [...document.querySelectorAll('main img')].every(img => img.complete && img.naturalWidth > 0));
  assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth + 1), 'Visual results guide overflows on mobile');
  const figureData = await page.request.get(origin + 'figures/measurements.json');
  assert(figureData.ok(), 'Figure source data is unavailable');
  assert.equal((await figureData.json()).motion.rms_mm[1], 0.294);
  await page.screenshot({ path: 'local-assets/docs-review/results-mobile.png', fullPage: true });
  assert.deepEqual(errors, []);
  console.log('Docs navigation, local search, mobile layout and console checks pass.');
} finally {
  await browser?.close();
  server.kill();
}
