import { test } from 'node:test';
import assert from 'node:assert/strict';
import { planLines, buildRoute, applyHeights, planSorties, type LonLat } from '../src/index.ts';

const lat0 = -33.2, lon0 = 22.0, m = 1 / 111320, k = Math.cos(lat0 * Math.PI / 180);
const P = (x: number, y: number): LonLat => [lon0 + x * m / k, lat0 + y * m];
const poly = [P(-4000, -3000), P(4000, -3000), P(4000, 3000), P(-4000, 3000)];
const plan = planLines(poly, { aglM: 300, speedMs: 12 });
const flat = () => 200;
const home = P(-4500, -3500);

test('range build: startLine/endLine fly exactly those lines with a figure-8', () => {
  const r = buildRoute(plan, { startLine: 3, endLine: 5 });
  const lines = [...new Set(r.wps.filter(w => w.line != null).map(w => w.line))];
  assert.deepEqual(lines, [3, 4, 5]);
  assert.equal(r.wps[0].role, 'approach');
  assert.ok(r.wps.some(w => w.role === 'fig8'));
});

test('sorties cover every line once, in order, each within the budget', () => {
  const sp = planSorties(plan, flat, home, {}, { usableMin: 35 });
  assert.ok(sp.sorties.length > 1, `sorties: ${sp.sorties.length}`);
  assert.equal(sp.sorties[0].fromLine, 0);
  assert.equal(sp.sorties.at(-1)!.toLine, plan.lines.length - 1);
  for (let i = 1; i < sp.sorties.length; i++) assert.equal(sp.sorties[i].fromLine, sp.sorties[i - 1].toLine + 1);
  for (const so of sp.sorties) assert.ok(!so.overBudget, `sortie ${so.index + 1}: ${so.time.total}`);
  for (const so of sp.sorties) assert.ok(so.transit && so.time.transit > 0 && so.time.rth > 0);
  // each sortie starts recording and ends it
  for (const so of sp.sorties) {
    assert.deepEqual(so.route.wps[0].actions, ['START_RECORD']);
    assert.deepEqual(so.route.wps.at(-1)!.actions, ['STOP_RECORD']);
  }
});

test('a bigger budget means fewer sorties; overlap re-flies one line', () => {
  const a = planSorties(plan, flat, home, {}, { usableMin: 35 }).sorties.length;   // one 6 km line + RTH ≈ 20 min
  const b = planSorties(plan, flat, home, {}, { usableMin: 70 }).sorties.length;
  assert.ok(b < a, `${b} < ${a}`);
  const ov = planSorties(plan, flat, home, {}, { usableMin: 35, overlapLines: 1 }).sorties;
  // a sortie re-flies the previous sortie's last line, unless that sortie was a single line (no progress otherwise)
  for (let i = 1; i < ov.length; i++) {
    const prev = ov[i - 1];
    assert.equal(ov[i].fromLine, prev.toLine > prev.fromLine ? prev.toLine : prev.toLine + 1);
  }
});

test('waypoint cap limits sortie size', () => {
  const sp = planSorties(plan, flat, home, {}, { usableMin: 500, maxWaypoints: 150 });
  assert.ok(sp.sorties.every(so => so.route.wps.length <= 150));
  assert.ok(sp.sorties.length > 1);
});

test('a single line longer than the budget is reported as an error', () => {
  const sp = planSorties(plan, flat, home, {}, { usableMin: 1 });
  assert.ok(sp.issues.some(i => i.code === 'SORTIE_TIME' && i.severity === 'error'));
});

test('without home, sorties are timed on the route alone', () => {
  const sp = planSorties(plan, flat, null, {}, { usableMin: 35 });
  assert.ok(sp.sorties.every(so => so.transit === null && so.time.transit === 0));
  assert.ok(sp.issues.some(i => i.code === 'SORTIE_NO_HOME'));
});

test('resume start: sorties begin at firstLine', () => {
  const sp = planSorties(plan, flat, home, {}, { usableMin: 35, firstLine: 7 });
  assert.equal(sp.sorties[0].fromLine, 7);
  const h = applyHeights(plan, buildRoute(plan), flat);
  assert.ok(h.wps.length > 0);
});
