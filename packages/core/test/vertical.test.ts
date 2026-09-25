import { test } from 'node:test';
import assert from 'node:assert/strict';
import { planLines, buildRoute, applyHeights, stats, validate, coverage, legSpeed, dist, type LonLat } from '../src/index.ts';

const lat0 = -33.2, lon0 = 22.0, m = 1 / 111320, k = Math.cos(lat0 * Math.PI / 180);
const P = (x: number, y: number): LonLat => [lon0 + x * m / k, lat0 + y * m];
const poly = [P(-1500, -1500), P(1500, -1500), P(1500, 1500), P(-1500, 1500)];
// Lines run N–S (course 0). A 300 m high ridge across the middle (x = −200…200) that every line crosses.
const ridge = (lon: number) => { const x = (lon - lon0) / m * k; return Math.abs(x) < 200 ? 250 : 100; };   // 150 m step: needs 2.7 m/s at the 4 m/s climb limit, above the 1 m/s floor

test('slow mode: AGL stays nominal on the flat and legs onto the ridge are slowed to the climb limit', () => {
  const plan = planLines(poly, { aglM: 120, speedMs: 12, courseDeg: 90, wpSpacingM: 100, climbMs: 4, descentMs: 3 });   // course 90: lines E–W cross the ridge
  const f = applyHeights(plan, buildRoute(plan, { fig8: false }), ridge);
  const st = stats(plan, f);
  assert.ok(st.aglOnLineMin >= 120 - 1e-6);
  assert.ok(st.slowedLegs > 0, 'some legs slowed');
  assert.ok(st.lineSpeedMin < 12 && st.lineSpeedMax === 12, `${st.lineSpeedMin}–${st.lineSpeedMax}`);
  // every slowed leg sits exactly at the limit; no unslowed leg exceeds it
  for (let i = 0; i < f.wps.length - 1; i++) {
    const d = dist(f.wps[i].xy, f.wps[i + 1].xy), dh = f.wps[i + 1].h - f.wps[i].h;
    if (d < 1e-6) continue;
    const vz = Math.abs(dh) / d * legSpeed(f.wps, i);
    const lim = dh > 0 ? 4 : 3;
    assert.ok(vz <= lim + 1e-6, `leg ${i}: ${vz} > ${lim}`);
  }
  // at least one slowed leg sits exactly on the limit (others may be held lower by the ramp cap of a neighbour)
  assert.ok(f.wps.some((w, i) => w.slowed && i < f.wps.length - 1 && Math.abs(Math.abs(f.wps[i + 1].h - w.h) / dist(w.xy, f.wps[i + 1].xy) * legSpeed(f.wps, i) - (f.wps[i + 1].h > w.h ? 4 : 3)) < 1e-6));
  assert.ok(!validate(plan, f).some(i => i.code === 'GRADIENT' || i.code === 'TOO_STEEP'));
  assert.ok(validate(plan, f).some(i => i.code === 'SLOWED'));
  // flight time counts the slow legs
  const uniform = f.wps.slice(1).reduce((t, w, i) => t + dist(w.xy, f.wps[i].xy), 0) / 12 / 60;
  assert.ok(st.flightMin > uniform);
});

test('raise mode keeps line speed and lifts waypoints instead', () => {
  const plan = planLines(poly, { aglM: 120, speedMs: 12, courseDeg: 90, wpSpacingM: 100, verticalMode: 'raise', maxGradient: 0.15 });
  const f = applyHeights(plan, buildRoute(plan, { fig8: false }), ridge);
  const st = stats(plan, f);
  assert.equal(st.slowedLegs, 0);
  assert.equal(st.lineSpeedMin, 12);
  assert.ok(st.aglOnLineMax > 120 + 50, 'waypoints raised well above nominal');
});

test('a vertical wall inside one leg is flagged as too steep', () => {
  const wall = (lon: number) => { const x = (lon - lon0) / m * k; return x > 0 && x < 40 ? 2000 : 100; };
  const plan = planLines(poly, { aglM: 100, speedMs: 12, courseDeg: 90, wpSpacingM: 50, climbMs: 4, descentMs: 3 });
  const f = applyHeights(plan, buildRoute(plan, { fig8: false }), wall);
  assert.ok(validate(plan, f).some(i => i.code === 'TOO_STEEP'));
});

test('figure-8 at the end: after the last run-out, then an exit point that stops recording', () => {
  const plan = planLines(poly, { aglM: 120 });
  const r = buildRoute(plan);                                   // fig8 + fig8End default on
  const roles = r.wps.map(w => w.role);
  const lastRunout = roles.lastIndexOf('runout');
  assert.ok(roles.slice(lastRunout + 1, -1).every(x => x === 'fig8'), 'figure-8 after the last run-out');
  assert.equal(roles.at(-1), 'exit');
  assert.deepEqual(r.wps.at(-1)!.actions, ['STOP_RECORD']);
  assert.equal(r.wps.filter(w => w.actions.includes('STOP_RECORD')).length, 1);
  const noEnd = buildRoute(plan, { fig8End: false });
  assert.equal(noEnd.wps.at(-1)!.role, 'runout');
  assert.ok(r.wps.length > noEnd.wps.length);
});

test('overlap bands between consecutive lines: flat ground gives the planned sidelap', () => {
  const plan = planLines(poly, { aglM: 300 });
  const f = applyHeights(plan, buildRoute(plan), () => 100);
  const c = coverage(plan, f);
  assert.equal(c.overlaps.length, plan.lines.length - 1);
  for (const o of c.overlaps) {
    assert.ok(Math.abs(o.pctMin - 50) < 1e-6 && Math.abs(o.pctMax - 50) < 1e-6, `${o.pctMin}–${o.pctMax}`);
    assert.ok(Math.abs(o.widthMinM - plan.swath / 2) < 1e-6);
    assert.ok(o.poly.length >= 4);
  }
});
