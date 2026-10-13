import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import { planLines, buildRoute, applyHeights, stats, type LonLat, type PlanOptions } from '../src/index.ts';

// Synthetic ~3,900 ha irregular block near Nimba, synthetic ridge terrain 440–1,225 m.
const lat0 = 7.55, lon0 = -8.55, m2d = 1 / 111320;
const cosLat = Math.cos(lat0 * Math.PI / 180);
const polyM = [[-3300, -3000], [3100, -3300], [3400, 800], [1500, 3200], [-2800, 3000], [-3500, 200]];
const poly: LonLat[] = polyM.map(([x, y]) => [lon0 + x * m2d / cosLat, lat0 + y * m2d]);
const elev = (lon: number, lat: number) => {
  const x = (lon - lon0) / m2d * cosLat, y = (lat - lat0) / m2d;
  return 440 + 785 * Math.exp(-((x - 800) ** 2 / 1.2e6 + (y - 500) ** 2 / 1.2e7)) + 60 * Math.sin(x / 400) * Math.cos(y / 550);
};

// The original JS engine (route-engine.js) is the reference for the route GEOMETRY. Heights have since been
// made deliberately more conservative (wider terrain search, level figure-8, strict gaps), so the port must
// never plan LOWER than the original did.
const legacy = createRequire(import.meta.url)('../../../route-engine.js');
// Settings that reproduce the original layout: gradient-limited heights, 2.5 r approach, no end figure-8.
const legacyLike: Partial<PlanOptions> = { courseDeg: 20, verticalMode: 'raise', alignStraightS: 0, fig8MinRadiusM: 0 };

function runTs(opts: { fromLine?: number; speedMs?: number } = {}) {
  const plan = planLines(poly, legacyLike, 'sphere');
  const route = applyHeights(plan, buildRoute(plan, { ...opts, fig8End: false }), elev);
  return { plan, route, s: stats(plan, route) };
}
function runLegacy(opts: { fromLine?: number; speedMs?: number } = {}) {
  const plan = legacy.planLines(poly, { courseDeg: 20 });
  const route = legacy.applyHeights(plan, legacy.buildRoute(plan, opts), elev);
  return { route, s: legacy.stats(plan, route) };
}

test('full job matches the known layout', () => {
  const { s } = runTs();
  assert.equal(s.lines, 23);
  assert.equal(s.waypoints, 902);
  assert.ok(Math.abs(s.routeKm - 137.373) < 0.01);
  assert.ok(s.aglOnLineMin >= 500, 'every line waypoint clears terrain by ≥ AGL');
});

for (const [name, opts] of [['full', {}], ['resume line 9 @14 m/s', { fromLine: 9, speedMs: 14 }]] as const) {
  test(`same geometry as the original engine, never lower: ${name}`, () => {
    const a = runTs(opts), b = runLegacy(opts);
    assert.equal(a.route.startIdx, b.route.startIdx);
    assert.equal(a.route.wps.length, b.route.wps.length);
    for (const k of ['areaHa', 'lines', 'swathM', 'spacingM', 'waypoints', 'routeKm', 'dataKm', 'fig8RadiusM'] as const) {
      assert.ok(Math.abs(a.s[k] - b.s[k]) < 1e-9, `stat ${k}`);
    }
    a.route.wps.forEach((w, i) => {
      const l = b.route.wps[i];
      assert.equal(w.role, l.role);
      assert.deepEqual(w.actions, l.actions);
      assert.equal(w.line, l.line);
      for (const k of ['lat', 'lon', 'speed'] as const) assert.equal(w[k], l[k], `wp ${i} ${k}`);
      assert.ok(w.h >= l.h - 1e-6, `wp ${i}: planned ${w.h} m, the original planned ${l.h} m`);
      assert.ok(w.dampingM <= l.dampingM + 1e-9, `wp ${i} damping`);
    });
    // and not wildly higher either: the extra search distance is 75 m along-track
    const extra = a.route.wps.map((w, i) => w.h - b.route.wps[i].h);
    assert.ok(Math.max(...extra) < 250, `max extra height ${Math.max(...extra).toFixed(0)} m`);
  });
}

test('resume restarts one line early with a fresh figure-8 and recording on', () => {
  const { route } = runTs({ fromLine: 9, speedMs: 14 });
  assert.equal(route.startIdx, 8);
  assert.equal(route.wps[0].role, 'approach');
  assert.deepEqual(route.wps[0].actions, ['START_RECORD']);
  assert.ok(route.wps.slice(1, 25).every(w => w.role === 'fig8'));
  assert.deepEqual(route.wps.at(-1)!.actions, ['STOP_RECORD']);
  assert.ok(route.wps.every(w => w.speed === 14));
});

test('alignment manoeuvres are flown level', () => {
  const plan = planLines(poly, { courseDeg: 20 });
  for (const mode of ['slow', 'raise'] as const) {
    const p = { ...plan, o: { ...plan.o, verticalMode: mode } };
    const f = applyHeights(p, buildRoute(p), elev).wps;
    const firstRunIn = f.findIndex(w => w.role === 'runin'), lastRunOut = f.map(w => w.role).lastIndexOf('runout');
    const start = f.slice(0, firstRunIn + 1), end = f.slice(lastRunOut);
    assert.ok(start.length > 20 && end.length > 20);
    assert.ok(start.every(w => Math.abs(w.h - start[0].h) < 1e-9), `${mode}: approach + figure-8 + run-in share one height`);
    assert.ok(end.every(w => Math.abs(w.h - end[0].h) < 1e-9), `${mode}: run-out + figure-8 + exit share one height`);
    assert.ok(start.every(w => w.h >= w.terrainMax + plan.o.aglM - 1e-9));
  }
});
