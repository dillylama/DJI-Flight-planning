import { test } from 'node:test';
import assert from 'node:assert/strict';
import { planLines, buildRoute, applyHeights, planTransit, coverage, type LonLat } from '../src/index.ts';

const lat0 = -33.2, lon0 = 22.0, m = 1 / 111320, k = Math.cos(lat0 * Math.PI / 180);
const P = (x: number, y: number): LonLat => [lon0 + x * m / k, lat0 + y * m];
const poly = [P(-1500, -1500), P(1500, -1500), P(1500, 1500), P(-1500, 1500)];
const plan = planLines(poly, { aglM: 300 });
const flat = applyHeights(plan, buildRoute(plan), () => 100);
const home = P(-3000, -3000);

test('safely: vertical climb to route height, level transit, clearance = AGL over flat ground', () => {
  const t = planTransit(plan, flat, () => 100, home, { flyToMode: 'safely', takeoffSecurityM: 50 });
  assert.equal(t.homeElev, 100);
  assert.equal(t.path[0].h, 100);
  assert.equal(t.path[1].h, 400);                     // max(100+50, WP1 at 100+300)
  assert.equal(t.path[2].h, 400);
  assert.ok(Math.abs(t.minClearanceM - 300) < 1e-6);
});

test('pointToPoint: climb to security height then slope up to WP1', () => {
  const t = planTransit(plan, flat, () => 100, home, { flyToMode: 'pointToPoint', takeoffSecurityM: 50 });
  assert.equal(t.path[1].h, 150);
  assert.equal(t.path.at(-1)!.h, 400);
  assert.ok(Math.abs(t.minClearanceM - 50) < 1e-6);   // lowest right after leaving the climb point
});

test('a hill between home and the block raises the recommended RTH and is caught on transit', () => {
  const hill = (lon: number, lat: number) => {
    const x = (lon - lon0) / m * k, y = (lat - lat0) / m;
    return Math.hypot(x + 2200, y + 2200) < 300 ? 700 : 100;   // 600 m hill on the home→block line
  };
  const f = applyHeights(plan, buildRoute(plan), hill);
  const t = planTransit(plan, f, hill, home, { flyToMode: 'pointToPoint', takeoffSecurityM: 50, minClearanceM: 60 });
  assert.ok(t.minClearanceM < 0, 'sloping transit hits the hill');
  assert.ok(t.rthRecommendedM >= 700 + 60 - 100, `recommended RTH ${t.rthRecommendedM}`);
  assert.ok(t.rthWorstClearanceM >= 60 - 1e-6);
  const low = planTransit(plan, f, hill, home, { rthHeightM: 100 });
  assert.ok(low.rthWorstClearanceM < 0);
});

test('coverage: flat ground gives exactly the planned sidelap', () => {
  const c = coverage(plan, flat);
  assert.ok(Math.abs(c.achievedMinPct - 50) < 1e-6 && Math.abs(c.achievedMaxPct - 50) < 1e-6);
  assert.equal(c.swaths.length, plan.lines.length);
  assert.ok(Math.abs(c.swathMinM - plan.swath) < 1e-6);
});


import { gsdCm, aglForGsd, photoPlan, cameraFovDeg, lidarDensity, lensFovDeg, frontlapAtInterval, CAMERAS, L3_PULSE, checkLimits, type Camera } from '../src/index.ts';
// 36 mm / 9000 px = 4 µm pixels; 40 mm lens → GSD = 4e-3 mm × 100 m / 40 mm = 1.0 cm at 100 m
const cam: Camera = { id: 't', name: 'test', imgW: 9000, imgH: 6000, hfovDeg: lensFovDeg(36, 40), vfovDeg: lensFovDeg(24, 40), minIntervalS: 1 };

test('photogrammetry: GSD, inverse, footprint, interval, blur, FOV', () => {
  assert.ok(Math.abs(gsdCm(cam, 100) - 1) < 1e-9);
  assert.ok(Math.abs(aglForGsd(cam, 2) - 200) < 1e-9);
  const p = photoPlan(cam, 100, 10, 80, 1 / 1000);
  assert.ok(Math.abs(p.footprintAcrossM - 90) < 1e-9 && Math.abs(p.footprintAlongM - 60) < 1e-9);
  assert.ok(Math.abs(p.photoSpacingM - 12) < 1e-9);          // 60 m × (1 − 0.8)
  assert.ok(Math.abs(p.intervalS - 1.2) < 1e-9);
  assert.ok(Math.abs(p.maxSpeedMs - 12) < 1e-9);             // 12 m per 1 s min interval
  assert.ok(Math.abs(p.blurPx - 1) < 1e-9);                  // 10 m/s × 1 ms = 1 cm = 1 px
  assert.ok(Math.abs(cameraFovDeg(cam) - 2 * Math.atan(36 / 80) * 180 / Math.PI) < 1e-9);
});

test('lidar density: per strip and with sidelap', () => {
  const d = lidarDensity(1e6, 10, 500, 250);
  assert.equal(d.perStrip, 200);
  assert.equal(d.total, 400);
});

test('P1 presets match DJI published GSD (H/55, H/80, H/114 cm/px)', () => {
  for (const [id, k] of [['p1-24', 55], ['p1-35', 80], ['p1-50', 114]] as const) {
    const c = CAMERAS.find(c => c.id === id)!;
    const g = gsdCm(c, 100);
    assert.ok(Math.abs(g - 100 / k) / (100 / k) < 0.01, `${id}: ${g} vs ${100 / k}`);
  }
});

test('L3 RGB frontlap at 1 s interval', () => {
  const c = CAMERAS.find(c => c.id === 'l3-100')!;
  const along = 2 * 300 * Math.tan(20.6 * Math.PI / 180);
  assert.ok(Math.abs(frontlapAtInterval(c, 300, 15, 1) - 100 * (1 - 15 / along)) < 1e-9);
});

test('limits: pulse-rate AGL and climb-rate checks', () => {
  const plan500 = planLines(poly, { aglM: 500 });
  const f = applyHeights(plan500, buildRoute(plan500), () => 100);
  const codes = (khz: number) => checkLimits(plan500, f, { sensor: 'lidar', pulse: L3_PULSE.find(p => p.khz === khz)!, sortieMin: 30, totalMin: 10 }).map(i => i.severity + ':' + i.code);
  assert.ok(!codes(100).some(c => c.startsWith('error')), codes(100).join());
  assert.ok(codes(350).includes('error:L3_AGL'));
  // The default 15 % gradient at 12 m/s is only 1.8 m/s vertical: no climb warning.
  const step = (lon: number) => { const x = (lon - lon0) / m * k; return x > 0 && x < 600 ? 1500 : 100; };   // ridge: climb then descent
  const gentle = applyHeights(plan, buildRoute(plan, { speedMs: 12 }), step);
  assert.ok(!checkLimits(plan, gentle, { sensor: 'photo', sortieMin: 30, totalMin: 10 }).some(i => i.code === 'CLIMB'));
  // A 50 % gradient at 12 m/s = 6 m/s+ vertical: over the conservative limits.
  const plan50 = planLines(poly, { aglM: 300, maxGradient: 0.6 });
  const steep = applyHeights(plan50, buildRoute(plan50, { speedMs: 12 }), step);
  const c2 = checkLimits(plan50, steep, { sensor: 'photo', sortieMin: 30, totalMin: 10 }).map(i => i.severity + ':' + i.code);
  assert.ok(c2.includes('error:CLIMB') && c2.includes('error:DESCENT'), c2.join());
  assert.ok(checkLimits(plan50, steep, { sensor: 'photo', sortieMin: 30, totalMin: 95 }).some(i => i.code === 'SORTIES'));
});
