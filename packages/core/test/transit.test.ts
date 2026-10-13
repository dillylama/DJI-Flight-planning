import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  planLines, buildRoute, applyHeights, writeWpml, verifyRouteFiles, parseRouteFile, planTransit, planSorties, validate, toUgcsCsv,
  isStop, dist, rasterElev, STOP_TURN, type LonLat, type PlanOptions,
} from '../src/index.ts';

const lat0 = -33.2, lon0 = 22.0, m = 1 / 110900, k = Math.cos(lat0 * Math.PI / 180);
const P = (x: number, y: number): LonLat => [lon0 + x * m / k, lat0 + y * m];
const poly = [P(-700, -500), P(700, -500), P(700, 500), P(-700, 500)];
const home = P(-2600, -1800);
// A 400 m ridge between home and the block, and rolling ground under the block.
const ground = (lon: number, lat: number) => {
  const x = (lon - lon0) / m * k, y = (lat - lat0) / m;
  return 600 + 400 * Math.exp(-(((x + 1500) / 350) ** 2)) + 60 * Math.sin(x / 300) * Math.cos(y / 260);
};
// The same ground as a 1 arc-second raster, which is what the planner's clearance guarantee is stated for.
const PX = 1 / 3600, WEST = lon0 - 0.06, NORTH = lat0 + 0.05, W = 440, H = 360;
const terrain = rasterElev({
  width: W, height: H, west: WEST, north: NORTH, dLon: PX, dLat: PX, nodata: null, pixelIsPoint: true,
  data: Float32Array.from({ length: W * H }, (_, i) => ground(WEST + ((i % W) + 0.5) * PX, NORTH - (Math.floor(i / W) + 0.5) * PX)),
});
const at = (p: LonLat) => terrain(p[0], p[1])!;
const L3 = { samplingRate: 350000, returnMode: 'sedecupleReturn', scanningMode: 'repetitive', modelColoring: true } as const;
const opts: Partial<PlanOptions> = { aglM: 150, speedMs: 12, courseDeg: 90 };

function build(clearanceM = 60, o = opts) {
  const plan = planLines(poly, o);
  const transit = { home: plan.proj.fwd(home[0], home[1]), speedMs: 15, clearanceM };
  const route = buildRoute(plan, { transit });
  const flight = applyHeights(plan, route, terrain);
  return { plan, route, flight, transit };
}

test('in-route transit: the route starts and ends above home, with waypoints all the way', () => {
  const { plan, route, transit } = build();
  const w = route.wps, n = w.length;
  assert.deepEqual(w[0].xy, transit.home);
  assert.ok(dist(w[n - 1].xy, transit.home) < 1e-6);
  const roles = w.map(x => x.role).filter((r, i, all) => r !== all[i - 1]).join(',');
  assert.ok(/^transit,approach,fig8,runin,.*,fig8,exit,transit$/.test(roles), roles);
  for (let i = 0; i < n - 1; i++) if (w[i].role === 'transit' || w[i + 1].role === 'transit') {
    assert.ok(dist(w[i].xy, w[i + 1].xy) <= plan.o.wpSpacingM + 1e-6, `transit leg ${i} is ${dist(w[i].xy, w[i + 1].xy).toFixed(0)} m`);
  }
  // recording starts and stops on stop turns, as in Pilot 2's own L3 routes; everything else flies through
  const a = w.findIndex(x => x.role === 'approach'), e = w.findIndex(x => x.role === 'exit');
  assert.deepEqual(w[a].actions, ['START_RECORD']); assert.deepEqual(w[e].actions, ['STOP_RECORD']);
  assert.deepEqual(w.map((x, i) => (isStop(x) ? i : -1)).filter(i => i >= 0), [a, e]);
  assert.ok(w[a].dampingM === 0 && w[e].dampingM === 0);
  assert.ok(w.filter(x => x.role === 'transit').every(x => x.speed === 15) && w[e].speed === 15);
  // without the option nothing changes
  const plain = buildRoute(plan);
  assert.equal(plain.wps[0].role, 'approach');
  assert.ok(plain.wps.every(x => !isStop(x) && x.role !== 'transit'));
  // corners between lines are coordinated turns; everything else flies through on straight legs
  for (let i = 1; i < plain.wps.length - 1; i++) {
    const corner = (plain.wps[i].role === 'runout' && plain.wps[i + 1].role === 'runin') || (plain.wps[i].role === 'runin' && plain.wps[i - 1].role === 'runout');
    assert.equal(plain.wps[i].turnMode, corner ? 'coordinateTurn' : 'toPointAndPassWithContinuityCurvature');
  }
});

test('in-route transit follows the terrain over a ridge that the direct transit would hit', () => {
  const { plan, flight } = build(60);
  // direct: climb to the first waypoint's height, then fly level across the ridge
  const direct = applyHeights(plan, buildRoute(plan), terrain);
  const td = planTransit(plan, direct, terrain, home, { flyToMode: 'safely', takeoffSecurityM: 60, minClearanceM: 60 });
  assert.ok(td.minClearanceM < 0 && !td.inRoute, `direct transit clearance ${td.minClearanceM.toFixed(0)} m`);
  // in route: every waypoint, transit included, sits at least AGL above the highest ground of its legs
  assert.ok(flight.wps.every(w => w.h >= w.terrainMax + 150 - 1e-9));
  const top = Math.max(...flight.wps.filter(w => w.role === 'transit').map(w => w.h));
  assert.ok(top > 1000 + 150 - 1, `transit climbs over the ridge to ${top.toFixed(0)} m`);
  const t = planTransit(plan, flight, terrain, home, { minClearanceM: 60 });
  assert.ok(t.inRoute);
  assert.ok(Math.abs(t.wp1AboveHomeM - (flight.wps[0].h - at(home))) < 1e-9);
  assert.ok(t.rthWorstClearanceM >= 60 - 1e-6 && t.topAboveHomeM >= top - at(home) - 1e-9);
  assert.ok(!validate(plan, flight).some(i => i.severity === 'error'));
  // alignment manoeuvres are still level, counted from the approach point
  const a = flight.wps.findIndex(w => w.role === 'approach'), r = flight.wps.findIndex(w => w.role === 'runin');
  assert.ok(flight.wps.slice(a, r + 1).every(w => Math.abs(w.h - flight.wps[a].h) < 1e-9));
  // nothing is rounded and nothing lags at a stop turn
  assert.ok(flight.wps.filter(isStop).every(w => w.roundingM === 0 && w.lagM === 0 && w.turnM === 0));
});

test('a transit clearance above the AGL is held on the transit legs only', () => {
  const { flight } = build(250);
  for (const w of flight.wps) assert.ok(w.h >= w.terrainMax + (w.role === 'transit' ? 250 : 150) - 1e-9, w.role);
  assert.ok(flight.wps.some(w => w.role === 'line' && w.h < w.terrainMax + 200));
});

test('the files of a route with in-route transit pass the independent check; record actions sit on stop turns', () => {
  const { plan, flight } = build();
  const files = writeWpml(flight, { lidar: L3, rgbPhotoSpacingM: 40 });
  // Pilot 2's IMU calibration belongs to the first and last waypoint of a route: not offered with the transit in it
  assert.throws(() => writeWpml(flight, { lidar: L3, djiImuCalibration: true }), /IMU calibration/);
  const vo = { elev: terrain, aglM: 150, corridorM: plan.o.corridorM, maxClimbMs: 4, maxDescentMs: 3, maxSpeedMs: 20, lidar: true };
  const rep = verifyRouteFiles(files.templateKml, files.waylinesWpml, vo);
  assert.deepEqual(rep.issues.map(i => i.message), []);
  assert.ok(rep.minClearanceM >= 150 - 0.05);
  const w = parseRouteFile(files.waylinesWpml, 'waylines').waypoints;
  const a = flight.wps.findIndex(x => x.role === 'approach'), e = flight.wps.findIndex(x => x.role === 'exit');
  const stops = w.map((p, i) => (p.turnMode === STOP_TURN ? i : -1)).filter(i => i >= 0);
  assert.deepEqual(stops, [0, a, e, w.length - 1]);
  assert.deepEqual(w[a].groups[0].actions.map(x => x.func), ['gimbalRotate', 'recordPointCloud']);
  assert.deepEqual(w[e].groups[0].actions.map(x => x.func), ['recordPointCloud']);
  assert.ok(w[0].groups.length === 0 && w[w.length - 1].groups.length === 0);
  // recording that starts on a fly-through waypoint is refused
  let n = -1;
  const pass = (xml: string) => { n = -1; return xml.replace(/<wpml:waypointTurnMode>[^<]*</g, all => (++n === a ? '<wpml:waypointTurnMode>toPointAndPassWithContinuityCurvature<' : all)); };
  const bad = verifyRouteFiles(pass(files.templateKml), pass(files.waylinesWpml), vo);
  assert.ok(bad.issues.some(i => i.code === 'V_ACTIONS'), bad.issues.map(i => i.code).join());
});

test('sorties with in-route transit: each one starts and ends above home and verifies', () => {
  const big = [P(-1200, -900), P(1200, -900), P(1200, 900), P(-1200, 900)];
  const plan = planLines(big, opts);
  const tk = { transitSpeedMs: 15, minClearanceM: 60 };
  const direct = planSorties(plan, terrain, home, tk, { usableMin: 30 });
  const sp = planSorties(plan, terrain, home, tk, { usableMin: 30, transitInRoute: true });
  assert.ok(sp.sorties.length >= 2, `sorties: ${sp.sorties.length}`);
  assert.equal(sp.sorties.at(-1)!.toLine, plan.lines.length - 1);
  const hx = plan.proj.fwd(home[0], home[1]);
  for (const so of sp.sorties) {
    const w = so.route.wps;
    assert.ok(dist(w[0].xy, hx) < 1e-6 && dist(w.at(-1)!.xy, hx) < 1e-6, `sortie ${so.index + 1}`);
    assert.ok(so.transit!.inRoute && so.time.transit < 1e-6 && so.time.rth < 1e-6);
    assert.ok(!so.overBudget || so.fromLine === so.toLine);
    const files = writeWpml(so.route, { lidar: L3 });
    const rep = verifyRouteFiles(files.templateKml, files.waylinesWpml, { elev: terrain, aglM: 150, corridorM: 75, maxClimbMs: 4, maxDescentMs: 3, maxSpeedMs: 20, lidar: true });
    assert.deepEqual(rep.issues.map(i => i.message), [], `sortie ${so.index + 1}`);
  }
  assert.ok(direct.sorties.every(so => so.route.wps[0].role === 'approach'));
});

test('UgCS CSV: one row per waypoint, AMSL heights, stop and fly-through turn types', () => {
  const { flight } = build();
  const lines = toUgcsCsv(flight).trimEnd().split('\r\n');
  assert.equal(lines[0], 'WP,Latitude,Longitude,AltitudeAMSL,Speed,TurnType');
  assert.equal(lines.length, flight.wps.length + 1);
  lines.slice(1).forEach((row, i) => {
    const [wp, lat, lon, h, v, turn] = row.split(',').map(Number);
    const w = flight.wps[i];
    assert.equal(wp, i + 1);
    assert.ok(Math.abs(lat - w.lat) < 1e-7 && Math.abs(lon - w.lon) < 1e-7 && Math.abs(h - w.h) < 0.006);
    assert.ok(v > 0 && v <= w.speed + 1e-9 && w.speed - v < 0.01);
    assert.equal(turn, i === 0 || i === flight.wps.length - 1 || isStop(w) ? 1 : 2);
  });
});
