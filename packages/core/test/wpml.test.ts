import { test } from 'node:test';
import assert from 'node:assert/strict';
import { existsSync, readFileSync } from 'node:fs';
import JSZip from 'jszip';
import { writeWpml, writeKmz, planLines, buildRoute, applyHeights, makeProj, parseRouteFile, type FlightWp, type Route, type LonLat } from '../src/index.ts';

// Flatten XML into [path, value] leaf pairs so two files can be compared field by field.
function leaves(xml: string): [string, string][] {
  const out: [string, string][] = [], stack: string[] = [];
  const re = /<(\/?)([\w:]+)[^>]*?(\/?)>|([^<]+)/g;
  let m: RegExpExecArray | null, text = '';
  while ((m = re.exec(xml))) {
    if (m[4] != null) { text += m[4].trim(); continue; }
    const [, close, tag, self] = m;
    if (tag.startsWith('?') || tag === 'xml') continue;
    if (close) { if (text) out.push([stack.join('/'), text]); text = ''; stack.pop(); }
    else if (self) out.push([[...stack, tag].join('/'), '']);
    else { stack.push(tag); text = ''; }
  }
  return out;
}

// Pilot 2 samples stay out of the public repo (real site coordinates): these tests run locally only.
const haveSamples = existsSync(new URL('../../../samples/extracted/waypoint/wpmz/waylines.wpml', import.meta.url));
const sampleTest = haveSamples ? test : test.skip;
const sample = (f: string) => readFileSync(new URL(`../../../samples/extracted/waypoint/wpmz/${f}`, import.meta.url), 'utf8');

// Rebuild the Pilot 2 sample route: 5 WPs, 150 m EGM96, 10 m/s, fly-through damping 10, no actions.
function sampleRoute(): Route<FlightWp> {
  const pts: LonLat[] = [...sample('waylines.wpml').matchAll(/<coordinates>\s*([\d.-]+),([\d.-]+)/g)].map(m => [+m[1], +m[2]]);
  const proj = makeProj(pts[0][1], pts[0][0]);
  const wps: FlightWp[] = pts.map(([lon, lat]) => ({
    xy: proj.fwd(lon, lat), lon, lat, role: 'line', speed: 10, actions: [], dampingM: 10,
    turnMode: 'toPointAndPassWithContinuityCurvature', h: 150, terrainMax: 0, terrainUnderWp: 0, roundingM: 0, lagM: 0, turnM: 0,
  }));
  return { wps, startIdx: 0, fig8RadiusM: 0, speedMs: 10 };
}

sampleTest('waylines.wpml reproduces the Pilot 2 sample field by field', () => {
  // The sample was saved with useStraightLine 0 ("fully curved"); straightLegs: false reproduces that.
  const ours = leaves(writeWpml(sampleRoute(), { straightLegs: false, takeoff: { flyToMode: 'safely', takeoffSecurityM: 60, transitSpeedMs: 15 } }).waylinesWpml);
  const theirs = leaves(sample('waylines.wpml'));
  const skip = new Set(['kml/Document/Folder/wpml:distance', 'kml/Document/Folder/wpml:duration']);   // Pilot 2 uses its own curve length
  const a = ours.filter(([p]) => !skip.has(p)), b = theirs.filter(([p]) => !skip.has(p));
  assert.equal(a.length, b.length, 'same number of fields');
  a.forEach(([p, v], i) => {
    assert.equal(p, b[i][0], `field ${i} path`);
    const x = Number(v), y = Number(b[i][1]);
    if (Number.isFinite(x) && Number.isFinite(y) && b[i][1] !== '') assert.ok(Math.abs(x - y) < 1e-4, `${p}: ${v} vs ${b[i][1]}`);
    else assert.equal(v.replace(/\s+/g, ''), b[i][1].replace(/\s+/g, ''), p);
  });
});

sampleTest('template.kml header and heights match the sample (EGM96 height + ellipsoidHeight)', () => {
  const t = writeWpml(sampleRoute(), { takeoff: { takeoffSecurityM: 60 } }).templateKml;
  const theirs = sample('template.kml');
  for (const tag of ['droneEnumValue', 'payloadEnumValue', 'payloadPositionIndex', 'heightMode', 'coordinateMode', 'templateType', 'flyToWaylineMode', 'executeRCLostAction']) {
    const get = (x: string) => x.match(new RegExp(`<wpml:${tag}>([^<]*)<`))![1];
    assert.equal(get(t), get(theirs), tag);
  }
  const ell = (x: string) => [...x.matchAll(/<wpml:ellipsoidHeight>([^<]*)</g)].map(m => +m[1]);
  ell(t).forEach((e, i) => assert.ok(Math.abs(e - ell(theirs)[i]) < 1e-3, `ellipsoidHeight ${i}: ${e} vs ${ell(theirs)[i]}`));
});

sampleTest('our reader parses the Pilot 2 sample the way Pilot 2 wrote it', () => {
  const w = parseRouteFile(sample('waylines.wpml'), 'waylines'), t = parseRouteFile(sample('template.kml'), 'template');
  assert.equal(w.drone, 103); assert.equal(w.payload, 117); assert.equal(w.heightMode, 'WGS84'); assert.equal(t.heightMode, 'EGM96');
  assert.equal(w.waypoints.length, 5);
  for (const p of w.waypoints) assert.ok(Math.abs(p.hEgm96 - 150) < 1e-3, `150 m EGM96 read back as ${p.hEgm96}`);
  for (const p of t.waypoints) { assert.equal(p.hEgm96, 150); assert.equal(p.speed, 10); }   // global height and speed
});

const lat0 = -33.28, lon0 = 22.1, m = 1 / 111320, k = Math.cos(lat0 * Math.PI / 180);
const P = (x: number, y: number): LonLat => [lon0 + x * m / k, lat0 + y * m];
function survey() {
  const plan = planLines([P(-800, -800), P(800, -800), P(800, 800), P(-800, 800)], { aglM: 120, speedMs: 10 });
  return { plan, route: applyHeights(plan, buildRoute(plan), () => 300) };
}
const L3 = { samplingRate: 350000, returnMode: 'sedecupleReturn', scanningMode: 'repetitive', modelColoring: true } as const;

test('L3 survey route: recording, DJI IMU calibration, RGB shooting per line, group ids', async () => {
  const { plan, route } = survey();
  const files = writeWpml(route, { lidar: L3, djiImuCalibration: true, rgbPhotoSpacingM: 30, gimbalStartGroup: true });
  const w = files.waylinesWpml;
  const ops = [...w.matchAll(/<wpml:recordPointCloudOperate>(\w+)</g)].map(x => x[1]);
  assert.deepEqual(ops, ['startRecord', 'stopRecord']);
  assert.equal([...w.matchAll(/aircraftCalibration/g)].length, 2);
  assert.equal([...w.matchAll(/startContinuousShooting/g)].length, plan.lines.length);
  const ids = [...w.matchAll(/<wpml:actionGroupId>(\d+)</g)].map(x => +x[1]);
  assert.deepEqual(ids, ids.map((_, i) => i), 'group ids 0..n-1 in order');
  assert.ok(w.includes('<wpml:startActionGroup>'));

  const parsed = parseRouteFile(w, 'waylines');
  const n = parsed.waypoints.length;
  // first waypoint: gimbal down, calibrate, then start recording; last: stop recording, then calibrate
  assert.deepEqual(parsed.waypoints[0].groups[0].actions.map(a => a.func), ['gimbalRotate', 'aircraftCalibration', 'recordPointCloud']);
  assert.deepEqual(parsed.waypoints[n - 1].groups[0].actions.map(a => a.func), ['recordPointCloud', 'aircraftCalibration']);
  // one arrival group per waypoint at most
  for (const p of parsed.waypoints) assert.ok(p.groups.filter(g => g.trigger === 'reachPoint').length <= 1);
  // continuous shooting stops on the RUN-OUT waypoint (outside the block), never on a data-line waypoint
  parsed.waypoints.forEach((p, i) => {
    if (p.groups.some(g => g.actions.some(a => a.func === 'stopContinuousShooting'))) assert.equal(route.wps[i].role, 'runout', `stop shooting on waypoint ${i}`);
    if (route.wps[i].role === 'line') assert.ok(!p.groups.some(g => g.trigger === 'reachPoint'), 'no arrival actions on data-line waypoints');
  });
  // KMZ zips both files under wpmz/
  const zip = await JSZip.loadAsync(await writeKmz(writeWpml(route)));
  assert.deepEqual(Object.keys(zip.files).filter(f => !zip.files[f].dir).sort(), ['wpmz/template.kml', 'wpmz/waylines.wpml']);
});

test('straight legs by default; first and last waypoint are stop turns; fallbacks fail safe', () => {
  const { route } = survey();
  const files = writeWpml(route, { lidar: L3 });
  const w = parseRouteFile(files.waylinesWpml, 'waylines'), t = parseRouteFile(files.templateKml, 'template');
  const n = w.waypoints.length;
  assert.ok(w.waypoints.every(p => p.straight === 1) && t.waypoints.every(p => p.straight === 1), 'useStraightLine 1 everywhere');
  assert.ok(files.templateKml.includes('<wpml:globalUseStraightLine>1</wpml:globalUseStraightLine>'));
  for (const i of [0, n - 1]) { assert.equal(w.waypoints[i].turnMode, 'toPointAndStopWithDiscontinuityCurvature'); assert.equal(w.waypoints[i].damping, 0); }
  for (let i = 1; i < n - 1; i++) { assert.equal(w.waypoints[i].turnMode, 'toPointAndPassWithContinuityCurvature'); assert.ok(w.waypoints[i].damping > 0); }
  // every waypoint carries its own height and speed in the template, and the global height is the HIGHEST one
  assert.equal([...files.templateKml.matchAll(/<wpml:useGlobalHeight>0</g)].length, n);
  assert.equal([...files.templateKml.matchAll(/<wpml:useGlobalSpeed>0</g)].length, n);
  assert.ok(t.globalHeight >= Math.max(...route.wps.map(p => p.h)) - 1e-6);
  // heights round-trip: EGM96 in, ellipsoidal in the file, EGM96 back out
  w.waypoints.forEach((p, i) => assert.ok(Math.abs(p.hEgm96 - route.wps[i].h) < 1e-5, `waypoint ${i}`));
  t.waypoints.forEach((p, i) => { assert.ok(Math.abs(p.hEgm96 - route.wps[i].h) < 1e-5); assert.ok(Math.abs(p.hEllipsoid - w.waypoints[i].hEllipsoid) < 1e-9); });
});

test('speeds and damping are rounded down, never up', () => {
  const { route } = survey();
  route.wps[5].speed = 2.6689; route.wps[6].dampingM = 7.999;
  const w = parseRouteFile(writeWpml(route).waylinesWpml, 'waylines');
  assert.equal(w.waypoints[5].speed, 2.66);
  assert.equal(w.waypoints[6].damping, 7.99);
});

test('unverified L3 values and broken numbers are refused', () => {
  const { route } = survey();
  assert.throws(() => writeWpml(route, { lidar: { ...L3, samplingRate: 123 } }));
  const bad = { ...route, wps: route.wps.map((p, i) => (i === 3 ? { ...p, h: NaN } : p)) };
  assert.throws(() => writeWpml(bad), /non-numeric/);
  const zero = { ...route, wps: route.wps.map((p, i) => (i === 3 ? { ...p, speed: 0 } : p)) };
  assert.throws(() => writeWpml(zero), /speed/);
});
