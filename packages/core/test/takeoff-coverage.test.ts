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
