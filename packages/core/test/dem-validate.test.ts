import { test } from 'node:test';
import assert from 'node:assert/strict';
import { rasterElev, bufferedBbox, openTopoUrl, planLines, buildRoute, applyHeights, validate, type Raster, type LonLat } from '../src/index.ts';

// 3×3 raster, 0.001° pixels, value = 100·col + 10·row, with one nodata cell at (2,2)
const r: Raster = {
  width: 3, height: 3, west: 10, north: 5, dLon: 0.001, dLat: 0.001,
  data: [0, 100, 200, 10, 110, 210, 20, 120, -9999], nodata: -9999,
};
const elev = rasterElev(r);

test('bilinear sampling on pixel centres', () => {
  const near = (a: number | null, b: number) => assert.ok(a != null && Math.abs(a - b) < 1e-6, `${a} ≉ ${b}`);
  near(elev(10.0005, 4.9995), 0);                         // centre of (0,0)
  near(elev(10.0015, 4.9985), 110);                       // centre of (1,1)
  near(elev(10.001, 4.999), 55);                          // midway between the four top-left centres
});

test('outside the raster or touching nodata returns null', () => {
  assert.equal(elev(9.99, 4.9995), null);
  assert.equal(elev(10.0024, 4.9976), null);
});

test('buffered bbox and OpenTopography URL', () => {
  const b = bufferedBbox([[10, 5], [10.01, 5.01]], 1113.2);
  assert.ok(Math.abs(b.south - 4.99) < 1e-9 && Math.abs(b.north - 5.02) < 1e-9);
  const u = new URL(openTopoUrl(b, 'KEY'));
  assert.equal(u.searchParams.get('demtype'), 'COP30');
  assert.equal(u.searchParams.get('outputFormat'), 'GTiff');
});

test('validate flags high AGL, unknown WP cap, and nothing blocking on a flat block', () => {
  const lat0 = 7.55, lon0 = -8.55, m = 1 / 111320;
  const poly: LonLat[] = [[-2000, -2000], [2000, -2000], [2000, 2000], [-2000, 2000]].map(([x, y]) => [lon0 + x * m, lat0 + y * m]);
  const plan = planLines(poly, {});
  const flat = applyHeights(plan, buildRoute(plan), () => 500);
  const codes = validate(plan, flat).map(i => i.code);
  assert.ok(!validate(plan, flat).some(i => i.severity === 'error'), codes.join());
  assert.ok(codes.includes('WP_CAP_UNKNOWN'));
  assert.ok(validate(plan, flat, { maxWaypoints: 10 }).some(i => i.code === 'WP_CAP'));

  const ridge = applyHeights(plan, buildRoute(plan), (lon) => ((lon - lon0) / m > 20 && (lon - lon0) / m < 60 ? 900 : 500));
  assert.ok(validate(plan, ridge).some(i => i.code === 'AGL_HIGH'));
});
