import { test } from 'node:test';
import assert from 'node:assert/strict';
import { planLines, buildRoute, applyHeights, flightRoute, thinRoute, rasterElev, writeWpml, verifyRouteFiles, isStop, dist, type LonLat, type PlanOptions } from '../src/index.ts';

const lat0 = -33.2, lon0 = 22.0, m = 1 / 110900, k = Math.cos(lat0 * Math.PI / 180);
const P = (x: number, y: number): LonLat => [lon0 + x * m / k, lat0 + y * m];
const poly = [P(-900, -300), P(900, -300), P(900, 300), P(-900, 300)];
// Flat ground with one sharp 80 m knoll at x = 300 on the middle line. (A slope would limit thinning by itself:
// every leg is held level with the highest ground along it, so on a slope the excess grows with leg length.)
const ground = (lon: number, lat: number) => {
  const x = (lon - lon0) / m * k, y = (lat - lat0) / m;
  return 500 + 80 * Math.exp(-(((x - 300) / 60) ** 2 + ((y - 0) / 120) ** 2));
};
const PX = 1 / 3600, WEST = lon0 - 0.03, NORTH = lat0 + 0.02, W = 240, H = 160;
const terrain = rasterElev({
  width: W, height: H, west: WEST, north: NORTH, dLon: PX, dLat: PX, nodata: null, pixelIsPoint: true,
  data: Float32Array.from({ length: W * H }, (_, i) => ground(WEST + ((i % W) + 0.5) * PX, NORTH - (Math.floor(i / W) + 0.5) * PX)),
});
const base: Partial<PlanOptions> = { aglM: 100, speedMs: 10, courseDeg: 90, wpSpacingM: 50, corridorM: 40 };

function build(o: Partial<PlanOptions>) {
  const plan = planLines(poly, o);
  const route = buildRoute(plan, { fig8: false });
  return { plan, route, ...flightRoute(plan, route, terrain) };
}

test('tolerance 0 changes nothing', () => {
  const { plan, route, flight, dense } = build({ ...base, thinToleranceM: 0 });
  assert.equal(flight.wps.length, dense.wps.length);
  assert.equal(flight.wps.length, applyHeights(plan, route, terrain).wps.length);
  assert.deepEqual(flight.thinned, { before: route.wps.length, after: route.wps.length });
});

test('flat ground: line waypoints collapse to the ends; the knoll keeps its waypoints', () => {
  const { flight, dense } = build({ ...base, thinToleranceM: 5 });
  assert.ok(flight.wps.length < dense.wps.length / 2, `${flight.wps.length} of ${dense.wps.length} left`);
  // a line well away from the knoll keeps no interior line waypoints at all
  // (the knoll's flank reaches lines up to ~250 m away through the ±40 m corridor)
  const far = flight.wps.filter(w => w.role === 'line' && Math.abs(w.xy[1]) > 255);
  const farDense = dense.wps.filter(w => w.role === 'line' && Math.abs(w.xy[1]) > 255);
  assert.ok(farDense.length > 10, 'no flat line in the fixture');
  assert.ok(far.length <= farDense.length / 5, `${far.length} of ${farDense.length} line waypoints left on the flat lines`);
  // every non-line waypoint, action carrier and stop turn survives
  const keyDense = dense.wps.filter(w => w.role !== 'line' || w.actions.length || isStop(w)).length;
  const keyThin = flight.wps.filter(w => w.role !== 'line' || w.actions.length || isStop(w)).length;
  assert.equal(keyThin, keyDense);
  // the lines over the knoll keep a waypoint on its top (the corridor and its 40 m extension make the requirement a
  // plateau of a few waypoints around the summit; any of them counts)
  const nearTop = flight.wps.filter(w => w.role === 'line' && Math.abs(w.xy[1]) < 80 && Math.abs(w.xy[0] - 300) < 120);
  assert.ok(nearTop.length >= 1, 'knoll waypoint dropped');
});

test('the thinned route never sits below the dense requirement and never more than the tolerance above it', () => {
  for (const tol of [3, 10, 30, 100]) {
    const { plan, route, flight, dense } = build({ ...base, thinToleranceM: tol });
    assert.equal(flight.thinned?.after, flight.wps.length);
    // walk the thinned legs over the dense waypoints
    const key = (w: { xy: [number, number] }) => w.xy.map(v => v.toFixed(3)).join(',');
    const pos = new Map(flight.wps.map((w, i) => [key(w), i]));
    const idx = dense.wps.map(w => pos.get(key(w)) ?? -1);
    let last = -1, lastD = 0;
    for (let i = 0; i < dense.wps.length; i++) {
      if (idx[i] >= 0) { last = i; continue; }
      let next = i + 1; while (idx[next] < 0) next++;
      const a = dense.wps[last], b = dense.wps[next];
      const L = dist(a.xy, b.xy), f = L > 0 ? dist(a.xy, dense.wps[i].xy) / L : 0;
      const h = flight.wps[idx[last]].h + f * (flight.wps[idx[next]].h - flight.wps[idx[last]].h);
      assert.ok(h >= dense.wps[i].h - 1e-6, `tol ${tol}: below the requirement at ${i} by ${(dense.wps[i].h - h).toFixed(2)} m`);
      assert.ok(h - dense.wps[i].h <= tol + 1e-6, `tol ${tol}: ${(h - dense.wps[i].h).toFixed(2)} m above at ${i}`);
      lastD = L;
    }
    void lastD;
    // and the exported file passes the independent check exactly like an un-thinned one
    const files = writeWpml(flight, { takeoff: { flyToMode: 'safely', takeoffSecurityM: 60, transitSpeedMs: 15 } });
    const rep = verifyRouteFiles(files.templateKml, files.waylinesWpml, { elev: terrain, aglM: plan.o.aglM, corridorM: plan.o.corridorM, maxClimbMs: plan.o.climbMs, maxDescentMs: plan.o.descentMs, maxSpeedMs: 20, lidar: false });
    assert.ok(rep.ok, `tol ${tol}: ${rep.issues.map(i => i.message).join(' | ')}`);
    assert.ok(rep.minClearanceM >= plan.o.aglM - 0.01);
  }
});

test('a bigger tolerance never keeps more waypoints', () => {
  let prev = Infinity;
  for (const tol of [2, 5, 10, 20, 50]) {
    const { flight } = build({ ...base, thinToleranceM: tol });
    assert.ok(flight.wps.length <= prev, `tol ${tol}: ${flight.wps.length} > ${prev}`);
    prev = flight.wps.length;
  }
});

test('thinRoute keeps transit and figure-8 waypoints', () => {
  const plan = planLines(poly, { ...base, thinToleranceM: 50 });
  const route = buildRoute(plan, { fig8: true, transit: { home: plan.proj.fwd(...P(-1500, -800)), speedMs: 15, clearanceM: 60 } });
  const dense = applyHeights(plan, route, terrain);
  const thin = thinRoute(plan, route, dense, terrain);
  for (const role of ['transit', 'fig8', 'approach', 'exit', 'runin', 'runout'] as const)
    assert.equal(thin.wps.filter(w => w.role === role).length, dense.wps.filter(w => w.role === role).length, role);
});
