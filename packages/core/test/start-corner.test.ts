import { test } from 'node:test';
import assert from 'node:assert/strict';
import { planLines, buildRoute, type LonLat, type PlanOptions } from '../src/index.ts';

const lat0 = -33.2, lon0 = 22.0, m = 1 / 110900, k = Math.cos(lat0 * Math.PI / 180);
const P = (x: number, y: number): LonLat => [lon0 + x * m / k, lat0 + y * m];
const poly = [P(-500, -300), P(500, -300), P(500, 300), P(-500, 300)];
const base: Partial<PlanOptions> = { aglM: 100, speedMs: 10, courseDeg: 90, runInM: 50, runOutM: 50 };

// course 90: lines run east–west; 'left' of east is north, 'near' end is the west end.
const firstLineWp = (o: Partial<PlanOptions>) => {
  const plan = planLines(poly, { ...base, ...o });
  const route = buildRoute(plan, { fig8: false });
  const w = route.wps.find(x => x.role === 'line')!;
  return { x: w.xy[0], y: w.xy[1], lines: plan.lines };
};

test('default start: north side, west end', () => {
  const { x, y } = firstLineWp({});
  assert.ok(y > 200 && x < -400, `${x.toFixed(0)}, ${y.toFixed(0)}`);
});
test('right side starts on the south line', () => {
  const { x, y } = firstLineWp({ startSide: 'right' });
  assert.ok(y < -200 && x < -400, `${x.toFixed(0)}, ${y.toFixed(0)}`);
});
test('far end starts at the east end', () => {
  const { x, y } = firstLineWp({ startEnd: 'far' });
  assert.ok(y > 200 && x > 400, `${x.toFixed(0)}, ${y.toFixed(0)}`);
});
test('right side, far end: south-east corner', () => {
  const { x, y } = firstLineWp({ startSide: 'right', startEnd: 'far' });
  assert.ok(y < -200 && x > 400, `${x.toFixed(0)}, ${y.toFixed(0)}`);
});
test('the line set is the same whichever corner starts it', () => {
  const a = planLines(poly, base).lines.map(l => l.v.toFixed(3)).sort();
  const b = planLines(poly, { ...base, startSide: 'right', startEnd: 'far' }).lines.map(l => l.v.toFixed(3)).sort();
  assert.deepEqual(a, b);
  const dirs = planLines(poly, { ...base, startEnd: 'far' }).lines.map(l => l.dir);
  assert.equal(dirs[0], -1); assert.equal(dirs[1], 1);
});
