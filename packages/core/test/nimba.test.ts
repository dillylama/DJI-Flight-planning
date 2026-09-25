import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import { planLines, buildRoute, applyHeights, stats, type LonLat } from '../src/index.ts';

// Synthetic ~3,900 ha irregular block near Nimba, synthetic ridge terrain 440–1,225 m.
const lat0 = 7.55, lon0 = -8.55, m2d = 1 / 111320;
const cosLat = Math.cos(lat0 * Math.PI / 180);
const polyM = [[-3300, -3000], [3100, -3300], [3400, 800], [1500, 3200], [-2800, 3000], [-3500, 200]];
const poly: LonLat[] = polyM.map(([x, y]) => [lon0 + x * m2d / cosLat, lat0 + y * m2d]);
const elev = (lon: number, lat: number) => {
  const x = (lon - lon0) / m2d * cosLat, y = (lat - lat0) / m2d;
  return 440 + 785 * Math.exp(-((x - 800) ** 2 / 1.2e6 + (y - 500) ** 2 / 1.2e7)) + 60 * Math.sin(x / 400) * Math.cos(y / 550);
};

const ts = { planLines, buildRoute, applyHeights, stats };
const legacy: typeof ts = createRequire(import.meta.url)('../../../route-engine.js');

function run(E: typeof ts, opts: { fromLine?: number; speedMs?: number } = {}) {
  const plan = E.planLines(poly, { courseDeg: 20, verticalMode: 'raise', alignStraightS: 0, corridorM: 75, demSampleM: 75 });   // legacy: raise mode, 2.5 r approach, 3 corridor samples, no end figure-8
  const route = E.applyHeights(plan, E.buildRoute(plan, { ...opts, fig8End: false }), elev);
  return { route, s: E.stats(plan, route) };
}

test('full job matches known baseline', () => {
  const { s } = run(ts);
  assert.equal(s.lines, 23);
  assert.equal(s.waypoints, 902);
  assert.ok(Math.abs(s.routeKm - 137.373) < 0.01);
  assert.ok(s.aglOnLineMin >= 500, 'every line waypoint clears terrain by ≥ AGL');
});

for (const [name, opts] of [['full', {}], ['resume line 9 @14 m/s', { fromLine: 9, speedMs: 14 }]] as const) {
  test(`parity with legacy route-engine.js: ${name}`, () => {
    const a = run(ts, opts), b = run(legacy, opts);
    for (const [k, v] of Object.entries(b.s)) assert.ok(Math.abs(((a.s as unknown as Record<string, number>)[k]) - (v as number)) < 1e-9, `stat ${k}`);   // legacy stats keys only
    assert.equal(a.route.startIdx, b.route.startIdx);
    assert.equal(a.route.wps.length, b.route.wps.length);
    a.route.wps.forEach((w, i) => {
      const l = b.route.wps[i];
      assert.equal(w.role, l.role);
      assert.deepEqual(w.actions, l.actions);
      assert.equal(w.line, l.line);
      for (const k of ['lat', 'lon', 'h', 'hWrite', 'dampingM', 'speed'] as const) assert.equal(w[k], l[k], `wp ${i} ${k}`);
    });
  });
}

test('resume restarts one line early with a fresh figure-8 and recording on', () => {
  const { route } = run(ts, { fromLine: 9, speedMs: 14 });
  assert.equal(route.startIdx, 8);
  assert.equal(route.wps[0].role, 'approach');
  assert.deepEqual(route.wps[0].actions, ['START_RECORD']);
  assert.ok(route.wps.slice(1, 25).every(w => w.role === 'fig8'));
  assert.deepEqual(route.wps.at(-1)!.actions, ['STOP_RECORD']);
  assert.ok(route.wps.every(w => w.speed === 14));
});
