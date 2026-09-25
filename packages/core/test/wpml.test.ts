import { test } from 'node:test';
import assert from 'node:assert/strict';
import { existsSync, readFileSync } from 'node:fs';
import JSZip from 'jszip';
import { writeWpml, writeKmz, planLines, buildRoute, applyHeights, makeProj, type FlightWp, type Route, type LonLat } from '../src/index.ts';

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
    turnMode: 'toPointAndPassWithContinuityCurvature', h: 150, hWrite: 150, terrainMax: 0, terrainUnderWp: 0,
  }));
  return { wps, startIdx: 0, fig8RadiusM: 0, speedMs: 10 };
}

sampleTest('waylines.wpml reproduces the Pilot 2 sample field by field', () => {
  const ours = leaves(writeWpml(sampleRoute(), { takeoff: { flyToMode: 'safely', takeoffSecurityM: 60, transitSpeedMs: 15 } }).waylinesWpml);
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

test('L3 survey route: start/stop recording, DJI IMU calibration, RGB shooting per line, unique group ids', async () => {
  const lat0 = -33.28, lon0 = 22.1, m = 1 / 111320, k = Math.cos(lat0 * Math.PI / 180);
  const P = (x: number, y: number): LonLat => [lon0 + x * m / k, lat0 + y * m];
  const plan = planLines([P(-800, -800), P(800, -800), P(800, 800), P(-800, 800)], { aglM: 120, speedMs: 10 });
  const route = applyHeights(plan, buildRoute(plan), () => 300);
  const w = writeWpml(route, {
    lidar: { samplingRate: 350000, returnMode: 'sedecupleReturn', scanningMode: 'repetitive', modelColoring: true },
    djiImuCalibration: true, rgbPhotoSpacingM: 30, gimbalStartGroup: true,
  }).waylinesWpml;
  const ops = [...w.matchAll(/<wpml:recordPointCloudOperate>(\w+)</g)].map(x => x[1]);
  assert.deepEqual(ops, ['startRecord', 'stopRecord']);
  assert.equal([...w.matchAll(/aircraftCalibration/g)].length, 2);
  assert.equal([...w.matchAll(/startContinuousShooting/g)].length, plan.lines.length + 0 /* per line */);
  const ids = [...w.matchAll(/<wpml:actionGroupId>(\d+)</g)].map(x => +x[1]);
  assert.deepEqual(ids, ids.map((_, i) => i), 'group ids 0..n-1 in order');
  assert.ok(w.includes('<wpml:startActionGroup>'));
  // the first action group sits on the approach waypoint (index 0) and starts recording after calibration
  const first = w.slice(w.indexOf('<wpml:actionGroup>'), w.indexOf('</wpml:actionGroup>'));
  assert.ok(first.indexOf('aircraftCalibration') < first.indexOf('startRecord'));
  // KMZ zips both files under wpmz/
  const zip = await JSZip.loadAsync(await writeKmz(writeWpml(route)));
  assert.deepEqual(Object.keys(zip.files).filter(n => !zip.files[n].dir).sort(), ['wpmz/template.kml', 'wpmz/waylines.wpml']);
});

test('unverified L3 values are refused', () => {
  const lat0 = -33.28, lon0 = 22.1, m = 1 / 111320;
  const plan = planLines([[lon0, lat0], [lon0 + 800 * m, lat0], [lon0 + 800 * m, lat0 + 800 * m], [lon0, lat0 + 800 * m]], { aglM: 120 });
  const r = applyHeights(plan, buildRoute(plan), () => 300);
  assert.throws(() => writeWpml(r, { lidar: { samplingRate: 123, returnMode: 'sedecupleReturn', scanningMode: 'repetitive', modelColoring: true } }));
});
