import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { egm96ToEllipsoid } from 'egm96-universal';
import {
  planLines, buildRoute, applyHeights, writeWpml, verifyRouteFiles, parseRouteFile, rasterElev, validate, clearanceBudget,
  planTransit, planSorties, SAFETY,
  type Raster, type LonLat, type PlanOptions, type VerifyOptions, type BuildOptions, type ElevFn, type RouteWp,
} from '../src/index.ts';

// ── A synthetic 1″ raster the size of a real DEM request (about 11 × 13 km near Prince Albert) ──────
const W = 420, H = 420, PX = 1 / 3600, WEST = 22.0, NORTH = -33.2;
function raster(fn: (c: number, r: number) => number, w = W, h = H, px = PX): Raster {
  const data = new Float32Array(w * h);
  for (let r = 0; r < h; r++) for (let c = 0; c < w; c++) data[r * w + c] = fn(c, r);
  return { width: w, height: h, west: WEST, north: NORTH, dLon: px, dLat: px, data, nodata: null, pixelIsPoint: true };
}
const hash = (c: number, r: number) => { let x = Math.imul((c * 73856093) ^ (r * 19349663), 0x9e3779b1); x ^= x >>> 15; return ((x >>> 0) % 10000) / 10000; };
// Ridges and valleys with ±400 m of relief, slopes to 45°, and 40 m of cell-to-cell roughness.
const rough = raster((c, r) => 900 + 250 * Math.sin(c / 23) * Math.cos(r / 31) + 120 * Math.sin((c + r) / 9) + 40 * (hash(c, r) - 0.5));
const roughElev = rasterElev(rough);

const C0 = { lon: WEST + (W / 2) * PX, lat: NORTH - (H / 2) * PX };
// The test's own metres ↔ degrees (WGS84 radii at this latitude), independent of the planner's projection.
const M_LAT = 110904, M_LON = 111412.84 * Math.cos(C0.lat * Math.PI / 180) - 93.5 * Math.cos(3 * C0.lat * Math.PI / 180);
const at = (x: number, y: number): LonLat => [C0.lon + x / M_LON, C0.lat + y / M_LAT];
function block(cx: number, cy: number, w: number, h: number, rotDeg = 0): LonLat[] {
  const t = rotDeg * Math.PI / 180, cs = Math.cos(t), sn = Math.sin(t);
  return [[-w / 2, -h / 2], [w / 2, -h / 2], [w / 2, h / 2], [-w / 2, h / 2]].map(([x, y]) => at(cx + x * cs - y * sn, cy + x * sn + y * cs));
}

const L3 = { samplingRate: 350000, returnMode: 'sedecupleReturn', scanningMode: 'repetitive', modelColoring: true } as const;
function pipeline(poly: LonLat[], o: Partial<PlanOptions>, elev: ElevFn, build: BuildOptions = {}) {
  const plan = planLines(poly, o);
  const flight = applyHeights(plan, buildRoute(plan, build), elev);
  const files = writeWpml(flight, { lidar: L3, djiImuCalibration: true, rgbPhotoSpacingM: 40, gimbalStartGroup: true });
  const slow = plan.o.verticalMode === 'slow';
  const vopts: VerifyOptions = { elev, aglM: plan.o.aglM, corridorM: plan.o.corridorM, maxClimbMs: slow ? plan.o.climbMs : 6, maxDescentMs: slow ? plan.o.descentMs : 5, maxSpeedMs: 20, lidar: true };
  return { plan, flight, files, vopts, report: verifyRouteFiles(files.templateKml, files.waylinesWpml, vopts) };
}

// Brute force, straight from the raster cells and the waypoints parsed out of the file: every cell whose
// centre lies within `reach` of a leg (ends included) must be at least `agl` below the leg at that point.
function worstCellClearance(r: Raster, wps: { lon: number; lat: number; hEgm96: number }[], reach: number): number {
  let worst = Infinity;
  for (let i = 0; i < wps.length - 1; i++) {
    const a = wps[i], b = wps[i + 1];
    const bx = (b.lon - a.lon) * M_LON, by = (b.lat - a.lat) * M_LAT, L2 = bx * bx + by * by;
    const c0 = Math.floor((Math.min(a.lon, b.lon) - reach / M_LON - r.west) / r.dLon) - 1, c1 = Math.ceil((Math.max(a.lon, b.lon) + reach / M_LON - r.west) / r.dLon) + 1;
    const r0 = Math.floor((r.north - Math.max(a.lat, b.lat) - reach / M_LAT) / r.dLat) - 1, r1 = Math.ceil((r.north - Math.min(a.lat, b.lat) + reach / M_LAT) / r.dLat) + 1;
    for (let row = Math.max(0, r0); row <= Math.min(r.height - 1, r1); row++) for (let col = Math.max(0, c0); col <= Math.min(r.width - 1, c1); col++) {
      const px = (r.west + (col + 0.5) * r.dLon - a.lon) * M_LON, py = (r.north - (row + 0.5) * r.dLat - a.lat) * M_LAT;
      const t = L2 > 0 ? Math.min(1, Math.max(0, (px * bx + py * by) / L2)) : 0;
      if (Math.hypot(px - t * bx, py - t * by) > reach) continue;
      worst = Math.min(worst, a.hEgm96 + t * (b.hEgm96 - a.hEgm96) - r.data[row * r.width + col]);
    }
  }
  return worst;
}

const good = pipeline(block(300, -200, 1500, 1300, 25), { aglM: 150, speedMs: 17, courseDeg: 65 }, roughElev);
const codesOf = (t: string, w: string, o: Partial<VerifyOptions> = {}) => {
  const r = verifyRouteFiles(t, w, { ...good.vopts, ...o });
  assert.equal(r.ok, r.issues.length === 0);
  return [...new Set(r.issues.map(i => i.code))];
};

test('a planned route passes the independent check of its own files, with the planner’s figures', () => {
  const { report, plan, flight, files } = good;
  assert.deepEqual(report.issues, []);
  assert.ok(report.ok);
  assert.equal(report.waypoints, flight.wps.length);
  assert.ok(report.minClearanceM >= 150 - 0.05, `min clearance ${report.minClearanceM}`);
  assert.ok(report.minClearanceM < 150 + 60, 'the route is not simply flown far too high');
  assert.ok(report.samples > 100000);
  assert.ok(report.straightLegs);
  assert.equal(report.finishAction, 'goHome');
  assert.equal(report.rcLostAction, 'goBack');
  // straight from the raster cells: nothing within 90 % of the corridor is closer than the AGL
  const parsed = parseRouteFile(files.waylinesWpml, 'waylines').waypoints;
  const brute = worstCellClearance(rough, parsed, 0.9 * plan.o.corridorM);
  assert.ok(brute >= 150 - 0.05, `brute-force clearance over raw cells: ${brute}`);
  // the allowances it reads back from the file agree with the planner's budget
  const b = clearanceBudget(plan, flight, 20);
  assert.ok(Math.abs(report.maxPathErrorM - b.pathM) < 0.3, `${report.maxPathErrorM} vs ${b.pathM}`);
  assert.ok(report.maxRoundingM <= plan.o.roundingMaxM + 0.05 && report.maxTurnM <= 0.9 * plan.o.corridorM + 0.05);
  assert.ok(report.maxClimbMs <= 4.01 && report.maxDescentMs <= 3.01 && report.minSpeedMs >= 1);
  assert.ok(report.minSpeedMs < 17 && report.maxSpeedMs === 17, 'this route has slowed legs, which the fault tests below rely on');
  assert.ok(Math.abs(report.topEgm96M - Math.max(...flight.wps.map(w => w.h))) < 1e-4);
});

test('the verifier shares no code with the planner', () => {
  const src = readFileSync(new URL('../src/verify.ts', import.meta.url), 'utf8');
  const imports = [...src.matchAll(/^import\s+(type\s+)?[^;]*?from\s+'(\.[^']+)';/gm)];
  assert.ok(imports.length >= 2);
  for (const m of imports) assert.ok(m[1], `verify.ts imports code from ${m[2]}; only types are allowed`);
});

// ── fault injection: every one of these must be refused ───────────────────────────────────────────
const tagRe = (tag: string) => new RegExp(`<wpml:${tag}>([^<]*)</wpml:${tag}>`, 'g');
const every = (xml: string, tag: string, f: (v: string, k: number) => string) => { let k = -1; return xml.replace(tagRe(tag), (_all, v) => `<wpml:${tag}>${f(v, ++k)}</wpml:${tag}>`); };
const nth = (xml: string, tag: string, n: number, f: (v: string) => string) => every(xml, tag, (v, k) => (k === n ? f(v) : v));
const T = good.files.templateKml, Wl = good.files.waylinesWpml;
const N0 = egm96ToEllipsoid(C0.lat, C0.lon, 0);

test('faults in heights and datum are caught', () => {
  assert.ok(N0 > 25 && N0 < 40);
  const leg = good.report.minClearanceAt!.leg;
  // one waypoint 5 m low in the file that is flown
  assert.deepEqual(codesOf(T, nth(nth(Wl, 'executeHeight', leg, v => String(+v - 5)), 'executeHeight', leg + 1, v => String(+v - 5))).sort(), ['V_CLEARANCE', 'V_TEMPLATE']);
  // the same in both files, so they still agree with each other: only the terrain check can see it
  const low = (xml: string, tag: string) => nth(nth(xml, tag, leg, v => String(+v - 5)), tag, leg + 1, v => String(+v - 5));
  assert.deepEqual(codesOf(low(low(T, 'height'), 'ellipsoidHeight'), low(Wl, 'executeHeight')), ['V_CLEARANCE']);
  // EGM96 heights written where ellipsoidal ones belong (no geoid conversion): everything 32 m low
  assert.ok(codesOf(T, every(Wl, 'executeHeight', v => String(+v - N0))).includes('V_CLEARANCE'));
  assert.deepEqual(codesOf(every(every(T, 'height', v => String(+v - N0)), 'ellipsoidHeight', v => String(+v - N0)), every(Wl, 'executeHeight', v => String(+v - N0))), ['V_CLEARANCE']);
  // geoid applied with the wrong sign: 64 m low
  assert.ok(codesOf(T, every(Wl, 'executeHeight', v => String(+v - 2 * N0))).includes('V_CLEARANCE'));
  // template height that does not match its own ellipsoid height
  assert.deepEqual(codesOf(nth(T, 'height', 7, v => String(+v + 3)), Wl), ['V_TEMPLATE']);
  assert.deepEqual(codesOf(nth(T, 'ellipsoidHeight', 7, v => String(+v - 3)), Wl), ['V_TEMPLATE']);
  // wrong height mode: the same numbers would be flown relative to the take-off point
  assert.deepEqual(codesOf(T, Wl.replace('<wpml:executeHeightMode>WGS84<', '<wpml:executeHeightMode>relativeToStartPoint<')), ['V_HEIGHT_MODE']);
  assert.deepEqual(codesOf(T.replace('<wpml:heightMode>EGM96<', '<wpml:heightMode>relativeToStartPoint<'), Wl), ['V_HEIGHT_MODE']);
  // a global height below the route (the fallback if per-waypoint heights were ignored)
  assert.deepEqual(codesOf(every(T, 'globalHeight', v => String(+v - 50)), Wl), ['V_TEMPLATE']);
  // missing or broken height
  assert.deepEqual(codesOf(T, nth(Wl, 'executeHeight', 3, () => 'NaN')), ['V_HEIGHT']);
  assert.deepEqual(codesOf(T, nth(Wl, 'executeHeight', 3, () => '')), ['V_HEIGHT']);
  // the terrain is 50 m higher than the plan assumed (wrong DEM, or a DEM in another datum)
  const raised: ElevFn = (lon, lat) => { const v = roughElev(lon, lat); return v == null ? null : v + 50; };
  raised.upper = (lon, lat) => { const v = roughElev.upper!(lon, lat); return v == null ? null : v + 50; };
  raised.cell = roughElev.cell;
  assert.deepEqual(codesOf(T, Wl, { elev: raised }), ['V_CLEARANCE']);
});

test('faults in position are caught', () => {
  const shift = (xml: string, dLon: number, dLat: number) => xml.replace(/(<coordinates>\s*)([-\d.eE]+),([-\d.eE]+)/g, (_a, p, lon, lat) => `${p}${+lon + dLon},${+lat + dLat}`);
  // the whole route 400 m east, in both files
  assert.deepEqual(codesOf(shift(T, 0.0043, 0), shift(Wl, 0.0043, 0)), ['V_CLEARANCE']);
  // 60 m north: two DEM cells
  assert.deepEqual(codesOf(shift(T, 0, 0.00054), shift(Wl, 0, 0.00054)), ['V_CLEARANCE']);
  // latitude and longitude swapped: nowhere near the DEM
  const swap = (xml: string) => xml.replace(/(<coordinates>\s*)([-\d.eE]+),([-\d.eE]+)/g, (_a, p, lon, lat) => `${p}${lat},${lon}`);
  assert.deepEqual(codesOf(swap(T), swap(Wl)), ['V_NODATA']);
  // one waypoint moved in the flown file only
  let k = -1;
  const one = Wl.replace(/(<coordinates>\s*)([-\d.eE]+),([-\d.eE]+)/g, (all, p, lon, lat) => (++k === 30 ? `${p}${+lon + 0.002},${lat}` : all));
  assert.ok(codesOf(T, one).includes('V_TEMPLATE'));
  // garbage coordinates
  assert.ok(codesOf(T, Wl.replace(/(<coordinates>\s*)[-\d.eE]+,/, '$1abc,')).includes('V_COORD'));
  const outOfRange = codesOf(T, Wl.replace(/(<coordinates>\s*)[-\d.eE]+,[-\d.eE]+/, '$1200,95'));
  assert.ok(outOfRange.includes('V_COORD') || outOfRange.includes('V_FORMAT'), outOfRange.join());
  // duplicate waypoint
  const pts = [...Wl.matchAll(/<coordinates>\s*([-\d.eE]+,[-\d.eE]+)/g)].map(m => m[1]);
  k = -1;
  const dup = (xml: string) => { k = -1; return xml.replace(/(<coordinates>\s*)([-\d.eE]+,[-\d.eE]+)/g, (all, p) => (++k === 41 ? `${p}${pts[40]}` : all)); };
  assert.ok(codesOf(dup(T), dup(Wl)).includes('V_LEG'));
  // a hole in the DEM under the route
  const hole = good.flight.wps[60];
  const holed: ElevFn = (lon, lat) => (Math.hypot((lon - hole.lon) * M_LON, (lat - hole.lat) * M_LAT) < 40 ? null : roughElev(lon, lat));
  holed.upper = (lon, lat) => (Math.hypot((lon - hole.lon) * M_LON, (lat - hole.lat) * M_LAT) < 40 ? null : roughElev.upper!(lon, lat));
  holed.cell = roughElev.cell;
  assert.deepEqual(codesOf(T, Wl, { elev: holed }), ['V_NODATA']);
});

test('faults in speed, turns and damping are caught', () => {
  for (const bad of ['0', '-3', 'abc', '', '45']) assert.deepEqual(codesOf(T, nth(Wl, 'waypointSpeed', 5, () => bad)), ['V_SPEED'], `speed "${bad}"`);
  // line speed on every leg: the steep ones now demand more climb or descent than the limits
  const fast = (xml: string) => every(xml, 'waypointSpeed', () => '17');
  const c = codesOf(fast(T), fast(Wl));
  assert.ok(c.includes('V_CLIMB') || c.includes('V_DESCENT'), c.join());
  // the global speed must be the slowest waypoint speed (fallback if per-waypoint speeds were ignored)
  assert.deepEqual(codesOf(every(T, 'autoFlightSpeed', () => '17'), every(Wl, 'autoFlightSpeed', () => '17')), ['V_TEMPLATE']);
  assert.deepEqual(codesOf(T, every(Wl, 'autoFlightSpeed', () => '0')), ['V_HEADER']);
  // damping longer than the leg; damping that swings the turn out of the corridor
  const d = codesOf(T, nth(Wl, 'waypointTurnDampingDist', 10, () => '500'));
  assert.ok(d.includes('V_DAMPING') && d.includes('V_TEMPLATE'), d.join());
  const turnWp = good.flight.wps.findIndex((w, i) => w.role === 'runout' && good.flight.wps[i + 1]?.role === 'runin');
  const wide = (xml: string) => nth(xml, 'waypointTurnDampingDist', turnWp, () => '73');
  assert.deepEqual(codesOf(wide(T), wide(Wl)), ['V_TURN']);
  const zero = (xml: string) => nth(xml, 'waypointTurnDampingDist', 10, () => '0');
  assert.deepEqual(codesOf(zero(T), zero(Wl)), ['V_DAMPING']);
  assert.deepEqual(codesOf(T, nth(Wl, 'waypointTurnDampingDist', 10, () => '-1')), ['V_DAMPING']);
  // curved path between waypoints: clearance cannot be checked
  const curved = (xml: string) => every(every(xml, 'useStraightLine', () => '0'), 'globalUseStraightLine', () => '0');
  assert.deepEqual(codesOf(curved(T), curved(Wl)), ['V_TURN']);
  // first waypoint not a stop; unknown turn mode
  assert.ok(codesOf(T, Wl.replace('toPointAndStopWithDiscontinuityCurvature', 'toPointAndPassWithContinuityCurvature')).includes('V_TURN'));
  assert.deepEqual(codesOf(T, nth(Wl, 'waypointTurnMode', 9, () => 'sideways')), ['V_TURN']);
});

test('faults in the header, indices and actions are caught', () => {
  const both = (f: (xml: string) => string) => codesOf(f(T), f(Wl));
  assert.deepEqual(both(x => x.replace('<wpml:droneEnumValue>103<', '<wpml:droneEnumValue>89<')), ['V_HEADER']);
  assert.deepEqual(both(x => x.replace('<wpml:payloadEnumValue>117<', '<wpml:payloadEnumValue>84<')), ['V_HEADER']);
  assert.deepEqual(both(x => x.replace('wpmz/1.0.6', 'wpmz/1.0.2')), ['V_HEADER']);
  assert.deepEqual(both(x => x.replace('<wpml:takeOffSecurityHeight>60<', '<wpml:takeOffSecurityHeight>0<')), ['V_HEADER']);
  assert.deepEqual(both(x => x.replace('<wpml:executeRCLostAction>goBack<', '<wpml:executeRCLostAction>carryOn<')), ['V_HEADER']);
  assert.deepEqual(both(x => x.replace('<wpml:flyToWaylineMode>safely<', '<wpml:flyToWaylineMode>fast<')), ['V_HEADER']);
  assert.deepEqual(codesOf(T, Wl.replace('<wpml:finishAction>goHome<', '<wpml:finishAction>noAction<')), ['V_TEMPLATE']);
  assert.deepEqual(codesOf(T, nth(Wl, 'index', 3, () => '7')), ['V_INDEX']);
  assert.deepEqual(codesOf(T, Wl.slice(0, Wl.length - 400)), ['V_FORMAT']);
  assert.deepEqual(codesOf('', Wl), ['V_FORMAT']);
  assert.deepEqual(codesOf(T, '<kml><Document></Document></kml>'), ['V_FORMAT']);
  // recording never stopped, or not started on the first waypoint
  assert.deepEqual(both(x => x.replace('>stopRecord<', '>startRecord<')), ['V_ACTIONS']);
  assert.deepEqual(both(x => x.replace(/<wpml:actionGroup>(?:(?!<\/wpml:actionGroup>)[\s\S])*recordPointCloud[\s\S]*?<\/wpml:actionGroup>\n/, '')).filter(c => c !== 'V_ACTIONS'), []);
  assert.ok(both(x => x.replace(/<wpml:actionGroup>(?:(?!<\/wpml:actionGroup>)[\s\S])*recordPointCloud[\s\S]*?<\/wpml:actionGroup>\n/, '')).includes('V_ACTIONS'));
  // group ids out of order, a group pointing outside the route
  assert.deepEqual(both(x => nth(x, 'actionGroupId', 1, () => '9')), ['V_ACTIONS']);
  assert.deepEqual(both(x => nth(x, 'actionGroupEndIndex', 2, () => '99999')), ['V_ACTIONS']);
  // template with a waypoint missing
  assert.deepEqual(codesOf(T.replace(/<Placemark>[\s\S]*?<\/Placemark>\n/, ''), Wl).filter(c => c !== 'V_TEMPLATE'), []);
  // unusable limits handed to the verifier
  assert.deepEqual(codesOf(T, Wl, { aglM: NaN }), ['V_FORMAT']);
});

// ── seeded random routes over rough terrain ────────────────────────────────────────────────────────
function rng(seed: number) { return () => { seed |= 0; seed = (seed + 0x6d2b79f5) | 0; let t = Math.imul(seed ^ (seed >>> 15), 1 | seed); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }

test('random routes over rough terrain: valid plans always verify, and clear every raw DEM cell in the corridor', () => {
  const rnd = rng(20261003);
  const between = (a: number, b: number) => a + (b - a) * rnd();
  let valid = 0, tight = 0;
  for (let run = 0; run < 28; run++) {
    const o: Partial<PlanOptions> = {
      aglM: Math.round(between(100, 400)), speedMs: Math.round(between(5, 20)), corridorM: Math.round(between(30, 110)),
      courseDeg: Math.round(between(0, 359)), wpSpacingM: Math.round(between(40, 150)), sidelapPct: Math.round(between(30, 60)),
      verticalMode: rnd() < 0.7 ? 'slow' : 'raise', runInM: Math.round(between(20, 200)), runOutM: Math.round(between(20, 200)),
    };
    const poly = block(between(-1800, 1800), between(-1800, 1800), between(500, 1500), between(500, 1500), between(0, 90));
    const fig8 = rnd() < 0.7;
    const p = pipeline(poly, o, roughElev, { fig8, fig8End: fig8 && rnd() < 0.7 });
    const tag = `run ${run} ${JSON.stringify(o)}`;
    const planErrors = validate(p.plan, p.flight).filter(i => i.severity === 'error').map(i => i.code);
    // whatever else is wrong with a plan, its legs always clear the terrain
    const brute = worstCellClearance(rough, parseRouteFile(p.files.waylinesWpml, 'waylines').waypoints, 0.9 * o.corridorM!);
    assert.ok(brute >= o.aglM! - 0.05, `${tag}: raw cells cleared by only ${brute.toFixed(2)} m`);
    assert.ok(p.report.minClearanceM >= o.aglM! - 0.05 || p.report.issues.some(i => i.code !== 'V_CLEARANCE'), `${tag}: verifier clearance ${p.report.minClearanceM}`);
    assert.ok(!p.report.issues.some(i => i.code === 'V_CLEARANCE' || i.code === 'V_NODATA' || i.code === 'V_TEMPLATE' || i.code === 'V_TURN'), `${tag}: ${p.report.issues.map(i => i.message).join(' | ')}`);
    if (planErrors.length === 0) {
      valid++;
      assert.deepEqual(p.report.issues.map(i => i.code + ': ' + i.message), [], tag);
      assert.ok(Math.abs(p.report.maxPathErrorM - clearanceBudget(p.plan, p.flight, 20).pathM) < 0.5, tag);
      if (p.report.minClearanceM < o.aglM! + 5) tight++;
    } else {
      // a plan the planner refuses (too steep for the waypoint spacing) is refused by the verifier as well
      assert.ok(planErrors.every(c => c === 'TOO_STEEP' || c === 'DAMPING' || c === 'SHORT_LEG'), `${tag}: ${planErrors.join()}`);
      assert.ok(!p.report.ok, tag);
    }
  }
  assert.ok(valid >= 20, `only ${valid} of 28 random plans were valid`);
  assert.ok(tight >= 15, `only ${tight} valid plans come within 5 m of their AGL somewhere: the test would not notice a planner that flies too high`);
});

test('a single-cell spike is never missed: every cell around a small route, one at a time', () => {
  const poly = block(0, 0, 420, 260, 15);
  const o: Partial<PlanOptions> = { aglM: 120, speedMs: 12, courseDeg: 40, runInM: 60, runOutM: 60 };
  const plan = planLines(poly, o);
  const route = buildRoute(plan, { fig8: false });
  const flat = raster(() => 500), data = flat.data as Float32Array, elev = rasterElev(flat);
  const base = applyHeights(plan, route, elev).wps;
  assert.ok(base.every(w => Math.abs(w.h - 620) < 1e-9));
  const lons = base.map(w => w.lon), lats = base.map(w => w.lat);
  const c0 = Math.floor((Math.min(...lons) - WEST) / PX) - 5, c1 = Math.ceil((Math.max(...lons) - WEST) / PX) + 5;
  const r0 = Math.floor((NORTH - Math.max(...lats)) / PX) - 5, r1 = Math.ceil((NORTH - Math.min(...lats)) / PX) + 5;
  let cells = 0, raised = 0;
  for (let row = r0; row <= r1; row++) for (let col = c0; col <= c1; col++) {
    data[row * W + col] = 800;
    const wps = applyHeights(plan, route, elev).wps.map(w => ({ lon: w.lon, lat: w.lat, hEgm96: w.h }));
    const brute = worstCellClearance(flat, wps, 0.9 * 75);
    data[row * W + col] = 500;
    assert.ok(brute >= 120 - 1e-6, `spike at cell ${col},${row}: cleared by ${brute.toFixed(1)} m`);
    cells++; if (wps.some(w => w.hEgm96 > 700)) raised++;
  }
  assert.ok(cells > 400 && raised > 0.4 * cells && raised < cells, `${raised} of ${cells} spike positions lifted the route`);
});

test('RTH: a one-cell peak between the route and home is never missed, from anywhere on the route', () => {
  const rnd = rng(11);
  const poly = block(0, 0, 900, 700, 15);
  const home = at(-2600, -1900);
  for (const cfg of [{ wpSpacingM: 150, corridorM: 75 }, { wpSpacingM: 900, corridorM: 30 }]) {
  const po: Partial<PlanOptions> = { aglM: 120, speedMs: 15, courseDeg: 40, ...cfg };
  let mattered = 0;
  const flatPlan = planLines(poly, po);
  const flatWps = applyHeights(flatPlan, buildRoute(flatPlan), () => 500).wps;
  for (let run = 0; run < 60; run++) {
    // a peak somewhere on the straight line from a random point of the route to home
    const i = Math.floor(rnd() * (flatWps.length - 1)), f = rnd(), g = 0.08 + 0.84 * rnd();
    const oLon = flatWps[i].lon + (flatWps[i + 1].lon - flatWps[i].lon) * f, oLat = flatWps[i].lat + (flatWps[i + 1].lat - flatWps[i].lat) * f;
    const sLon = oLon + (home[0] - oLon) * g, sLat = oLat + (home[1] - oLat) * g;
    const sc = Math.floor((sLon - WEST) / PX), sr = Math.floor((NORTH - sLat) / PX);
    const spiky = raster((c, r) => (c === sc && r === sr ? 900 : 500));
    const elev = rasterElev(spiky);
    const plan = planLines(poly, po);
    const flight = applyHeights(plan, buildRoute(plan), elev);
    const t = planTransit(plan, flight, elev, home, { minClearanceM: 60 });
    assert.equal(t.homeElev, 500);
    assert.ok(t.rthWorstClearanceM >= 60 - 1e-6);
    // every point of every leg (each 12 m), straight home at max(own height, home + recommended RTH height):
    // the ground right under that line, read cell by cell, is at least 60 m below
    const pkLon = WEST + (sc + 0.5) * PX, pkLat = NORTH - (sr + 0.5) * PX;
    const hx = (home[0] - pkLon) * M_LON, hy = (home[1] - pkLat) * M_LAT;
    let worst = Infinity;
    for (let i = 0; i < flight.wps.length - 1; i++) {
      const a = flight.wps[i], b = flight.wps[i + 1];
      const L = Math.hypot((b.lon - a.lon) * M_LON, (b.lat - a.lat) * M_LAT), n = Math.max(1, Math.ceil(L / 12));
      for (let k = 0; k <= n; k++) {
        const f = k / n, ox = (a.lon + (b.lon - a.lon) * f - pkLon) * M_LON, oy = (a.lat + (b.lat - a.lat) * f - pkLat) * M_LAT;
        const alt = Math.max(a.h + (b.h - a.h) * f, 500 + t.rthRecommendedM);
        // distance from the peak cell's centre to the segment origin → home
        const dx = hx - ox, dy = hy - oy, u = Math.min(1, Math.max(0, -(ox * dx + oy * dy) / (dx * dx + dy * dy)));
        const d = Math.hypot(ox + u * dx, oy + u * dy);
        if (d <= 20) worst = Math.min(worst, alt - 900);      // the line passes over the peak cell
      }
    }
    if (worst < Infinity) { mattered++; assert.ok(worst >= 60 - 1e-6, `peak at cell ${sc},${sr}: an RTH line clears it by ${worst.toFixed(1)} m (RTH ${t.rthRecommendedM} m)`); }
  }
  assert.ok(mattered >= 55, `${JSON.stringify(cfg)}: the peak was under an RTH line in only ${mattered} of 60 runs: test too weak`);
  }
});

test('RTH from the middle of one long leg: a peak that only that line home crosses is seen', () => {
  // Two waypoints 2 km apart, home 3 km to the side. The lines home from the two waypoints pass far from the
  // peak, so only origins ALONG the leg can see it.
  const plan = planLines(block(0, 0, 900, 700), { aglM: 120, corridorM: 30 });   // for the projection and the corridor
  const A = at(-1000, 0), B = at(1000, 0), home = at(100, -3000);
  const mk = (p: LonLat): RouteWp => ({ xy: plan.proj.fwd(p[0], p[1]), role: 'line', speed: 10, actions: [], dampingM: 10, turnMode: 'toPointAndPassWithContinuityCurvature' });
  const flight = applyHeights(plan, { wps: [mk(A), mk(B)], startIdx: 0, fig8RadiusM: 0, speedMs: 10 }, () => 500);
  assert.ok(flight.wps.every(w => w.h === 620));
  for (const [f, g] of [[0.5, 0.5], [0.35, 0.2], [0.07, 0.6], [0.93, 0.35], [0.61, 0.8], [0.2, 0.05]]) {
    const o = [A[0] + (B[0] - A[0]) * f, A[1] + (B[1] - A[1]) * f], s = [o[0] + (home[0] - o[0]) * g, o[1] + (home[1] - o[1]) * g];
    const sc = Math.floor((s[0] - WEST) / PX), sr = Math.floor((NORTH - s[1]) / PX);
    const elev = rasterElev(raster((c, r) => (c === sc && r === sr ? 900 : 500)));
    const t = planTransit(plan, flight, elev, home, { minClearanceM: 60 });
    assert.ok(t.rthRecommendedM >= 900 + 60 - 500, `peak ${g} of the way home from ${f} along the leg: recommended RTH ${t.rthRecommendedM} m`);
    assert.ok(t.rthWorstClearanceM >= 60 - 1e-6);
    assert.ok(planTransit(plan, flight, elev, home, { minClearanceM: 60, rthHeightM: 200 }).rthWorstClearanceM < 0, 'a lower RTH height is shown to hit it');
  }
});

test('transit: a one-cell peak on the way to the first waypoint shows in the transit clearance', () => {
  const poly = block(0, 0, 900, 700, 15);
  const home = at(-2600, -1900);
  const plan = planLines(poly, { aglM: 120, speedMs: 15, courseDeg: 40 });
  const base = applyHeights(plan, buildRoute(plan), () => 500);
  const wp1 = base.wps[0];
  for (const f of [0.2, 0.37, 0.5, 0.81]) {
    const lon = home[0] + (wp1.lon - home[0]) * f, lat = home[1] + (wp1.lat - home[1]) * f;
    const sc = Math.floor((lon - WEST) / PX), sr = Math.floor((NORTH - lat) / PX);
    const elev = rasterElev(raster((c, r) => (c === sc && r === sr ? 700 : 500)));
    const flight = applyHeights(plan, buildRoute(plan), elev);
    const t = planTransit(plan, flight, elev, home, { flyToMode: 'safely', takeoffSecurityM: 60 });
    assert.ok(Math.abs(t.minClearanceM - (620 - 700)) < 1e-6, `peak ${f} of the way: transit clearance ${t.minClearanceM}`);
    assert.ok(Math.abs(t.wp1AboveHomeM - 120) < 1e-6 && Math.abs(t.topAboveHomeM - 120) < 1e-6);
  }
});

test('every sortie of a split job verifies on its own', () => {
  const poly = block(-200, 300, 2600, 2200, 10);
  const plan = planLines(poly, { aglM: 200, speedMs: 15, courseDeg: 100 });
  const home = at(-2500, -900);
  const sp = planSorties(plan, roughElev, home, {}, { usableMin: 20 });
  assert.ok(sp.sorties.length >= 3, `sorties: ${sp.sorties.length}`);
  for (const so of sp.sorties) {
    const files = writeWpml(so.route, { lidar: L3, djiImuCalibration: true, gimbalStartGroup: true });
    const r = verifyRouteFiles(files.templateKml, files.waylinesWpml, { elev: roughElev, aglM: 200, corridorM: 75, maxClimbMs: 4, maxDescentMs: 3, maxSpeedMs: 20, lidar: true });
    assert.deepEqual(r.issues.map(i => i.message), [], `sortie ${so.index + 1}`);
    assert.ok(r.minClearanceM >= 200 - 0.05);
    assert.ok(so.transit && so.transit.rthWorstClearanceM >= 60 - 1e-6);
  }
});

test('on a DEM finer than its own 10 m lattice the verifier samples at the DEM’s resolution', () => {
  // 5 m cells. The route is planned without the spike; then one 5 m cell under a leg is raised by 200 m.
  const px = 5 / 110904, n = 500;
  const fine = (spike: [number, number] | null) => raster((c, r) => (spike && c === spike[0] && r === spike[1] ? 700 : 500), n, n, px);
  const centre: LonLat = [WEST + (n / 2) * px, NORTH - (n / 2) * px];
  const mLon = M_LON, sq = (x: number, y: number): LonLat => [centre[0] + x / mLon, centre[1] + y / M_LAT];
  const poly = [sq(-150, -150), sq(150, -150), sq(150, 150), sq(-150, 150)];
  const flatElev = rasterElev(fine(null));
  const plan = planLines(poly, { aglM: 100, speedMs: 8, corridorM: 30, runInM: 40, runOutM: 40 });
  const flight = applyHeights(plan, buildRoute(plan, { fig8: false }), flatElev);
  const files = writeWpml(flight, { lidar: L3 });
  const opts: VerifyOptions = { elev: flatElev, aglM: 100, corridorM: 30, maxClimbMs: 4, maxDescentMs: 3, maxSpeedMs: 20, lidar: true };
  assert.deepEqual(verifyRouteFiles(files.templateKml, files.waylinesWpml, opts).issues, []);
  const lineWps = flight.wps.filter(w => w.role === 'line');
  for (const f of [0.03, 0.21, 0.5, 0.77]) {
    const a = lineWps[0], b = lineWps[1];
    const lon = a.lon + (b.lon - a.lon) * f + 7 / mLon, lat = a.lat + (b.lat - a.lat) * f;   // 7 m beside the leg
    const spike: [number, number] = [Math.floor((lon - WEST) / px), Math.floor((NORTH - lat) / px)];
    const r = verifyRouteFiles(files.templateKml, files.waylinesWpml, { ...opts, elev: rasterElev(fine(spike)) });
    assert.deepEqual([...new Set(r.issues.map(i => i.code))], ['V_CLEARANCE'], `spike ${f} along the leg`);
    assert.ok(Math.abs(r.minClearanceM - (-100)) < 1e-3);
    // and the planner, given that DEM, climbs over it
    const p2 = applyHeights(plan, buildRoute(plan, { fig8: false }), rasterElev(fine(spike)));
    assert.ok(Math.max(...p2.wps.map(w => w.h)) >= 800 - 1e-6);
  }
  assert.ok(SAFETY.corridorMinM === 30);
});
