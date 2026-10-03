// Generates RC import-test KMZs (IMPORT / SIMULATOR ONLY, DO NOT FLY) near the Pilot 2 sample site.
//   node tools/make-import-tests.ts            (needs VITE_OPENTOPO_KEY in apps/web/.env.development.local)
// Output: samples/generated/*.kmz + dem.tif + README.md describing each file. Every file is read back and
// checked by the independent verifier before it is written; a file that fails is not written at all.
import { mkdirSync, readFileSync, readdirSync, unlinkSync, writeFileSync } from 'node:fs';
import {
  planLines, buildRoute, applyHeights, readGeoTiff, rasterElev, bufferedBbox, openTopoUrl, writeWpml, writeKmz,
  validate, safetyChecks, clearanceBudget, verifyRouteFiles,
  type LonLat, type PlanOptions, type VerifyReport,
} from '../packages/core/src/index.ts';

const root = new URL('..', import.meta.url);
const env = readFileSync(new URL('apps/web/.env.development.local', root), 'utf8');
const key = env.match(/^VITE_OPENTOPO_KEY=(.+)$/m)?.[1].trim();
if (!key) throw new Error('VITE_OPENTOPO_KEY missing');

// Centre = the first waypoint of the local (gitignored) Pilot 2 sample, so no site coordinates live in the repo.
const sampleWpml = readFileSync(new URL('samples/extracted/waypoint/wpmz/waylines.wpml', root), 'utf8');
const [lon0, lat0] = sampleWpml.match(/<coordinates>\s*([\d.-]+),([\d.-]+)/)!.slice(1).map(Number);
const m = 1 / 110900, k = Math.cos(lat0 * Math.PI / 180);
const P = (x: number, y: number): LonLat => [lon0 + x * m / k, lat0 + y * m];
const rect = (w: number, h: number): LonLat[] => [P(-w / 2, -h / 2), P(w / 2, -h / 2), P(w / 2, h / 2), P(-w / 2, h / 2)];

const out = new URL('samples/generated/', root);
mkdirSync(out, { recursive: true });
for (const f of readdirSync(out)) if (/\.(kmz|tif)$/i.test(f) || f === 'README.md') unlinkSync(new URL(f, out));   // no stale files from an older writer

const res = await fetch(openTopoUrl(bufferedBbox(rect(6000, 6000), 2500), key, 'COP30'));
if (!res.ok) throw new Error('OpenTopography ' + res.status);
const demBuf = await res.arrayBuffer();
const elev = rasterElev(await readGeoTiff(demBuf));
writeFileSync(new URL('dem.tif', out), new Uint8Array(demBuf));   // the terrain these files were checked against

const L3 = { samplingRate: 350000, returnMode: 'sedecupleReturn' as const, scanningMode: 'repetitive' as const, modelColoring: true };
const rows: string[] = [];
async function emit(name: string, poly: LonLat[], o: Partial<PlanOptions>, note: string) {
  const plan = planLines(poly, o);
  const route = applyHeights(plan, buildRoute(plan), elev);
  const problems = [...validate(plan, route), ...safetyChecks(plan, route, { uncertaintyM: 20 })].filter(i => i.severity === 'error');
  if (problems.length) throw new Error(`${name}: ${problems.map(i => i.message).join(' | ')}`);
  const files = writeWpml(route, {
    takeoff: { flyToMode: 'safely', takeoffSecurityM: 60, transitSpeedMs: 15 },
    lidar: L3, djiImuCalibration: true, rgbPhotoSpacingM: 25, gimbalStartGroup: true,
  });
  const rep: VerifyReport = verifyRouteFiles(files.templateKml, files.waylinesWpml, {
    elev, aglM: plan.o.aglM, corridorM: plan.o.corridorM, maxClimbMs: plan.o.climbMs, maxDescentMs: plan.o.descentMs, maxSpeedMs: 20, lidar: true,
  });
  if (!rep.ok) throw new Error(`${name} failed the file check: ${rep.issues.map(i => i.message).join(' | ')}`);
  writeFileSync(new URL(name, out), await writeKmz(files));
  const hs = route.wps.map(w => w.h), b = clearanceBudget(plan, route, 20);
  rows.push(`| \`${name}\` | ${route.wps.length} | ${(files.distanceM / 1000).toFixed(1)} km | ${Math.min(...hs).toFixed(0)}–${Math.max(...hs).toFixed(0)} m | ${rep.minSpeedMs}–${rep.maxSpeedMs} m/s | ${rep.minClearanceM.toFixed(1)} m (worst case ${b.worstCaseM.toFixed(0)} m) | ${note} |`);
  console.log(`${name}: ${route.wps.length} WP, file check passed, min clearance ${rep.minClearanceM.toFixed(1)} m`);
}
// Side of a square block that gives at least `target` waypoints (geometry only: no terrain needed to count).
function sideFor(target: number, o: Partial<PlanOptions>): number {
  for (let side = 300; side <= 6000; side += 50) {
    const plan = planLines(rect(side, side), o);
    if (buildRoute(plan).wps.length >= target) return side;
  }
  throw new Error(`no block up to 6 km gives ${target} waypoints`);
}

// 1. Small real survey: 600 × 400 m, 100 m AGL, 10 m/s, figure-8 at both ends, 350 kHz, DJI IMU calibration, RGB photos.
const small = { aglM: 100, courseDeg: 0, wpSpacingM: 100 };
await emit('test-01-small-l3-survey.kmz', rect(600, 400), { ...small, speedMs: 10 },
  'The reference file. Import it; check where it draws, the heights, the actions on the first and last waypoint. Then save it in Pilot 2 and EXPORT it again for the comparison.');
// 2. The same route at 15 and 17 m/s: does Pilot 2 accept waypoint speeds above 15 m/s on the M400, or clamp them?
await emit('test-02-speed-15.kmz', rect(600, 400), { ...small, speedMs: 15 }, 'Same block at 15 m/s. Note the speed Pilot 2 shows.');
await emit('test-03-speed-17.kmz', rect(600, 400), { ...small, speedMs: 17 }, 'Same block at 17 m/s (our default line speed). Does it import? Does Pilot 2 show 17 m/s or cut it to 15?');
// 3. Waypoint-cap probes: square blocks with 50 m waypoint spacing, sized to reach each count. The gradient-limited
//    profile is used so that a big block over steep ground always gives a valid route (these are import probes).
const cap: Partial<PlanOptions> = { aglM: 100, speedMs: 10, courseDeg: 0, wpSpacingM: 50, verticalMode: 'raise' };
for (const target of [250, 500, 1000, 2000, 5000]) {
  const side = sideFor(target, cap);
  await emit(`test-cap-${String(target).padStart(4, '0')}.kmz`, rect(side, side), cap, `Waypoint-cap probe (≥ ${target}). Import only: does Pilot 2 accept it?`);
}

writeFileSync(new URL('README.md', out), `# Import tests: IMPORT / SIMULATOR ONLY, DO NOT FLY

Generated by \`node tools/make-import-tests.ts\` on ${new Date().toISOString().slice(0, 10)} from the planner's WPML writer,
centred on the first waypoint of \`m400-l3-waypoint-reference.kmz\`, on real Copernicus GLO-30 terrain (\`dem.tif\`, kept here).
Heights are EGM96 in template.kml and WGS84 ellipsoidal in waylines.wpml, exactly as Pilot 2 writes them.
Every file was read back and checked against the terrain by the independent verifier before it was written.

These files have no home point, RTH height or Max Altitude worked out for them. They are for the import test
and the simulator, not for the air. A route to fly comes from the planner, with home set.

| File | Waypoints | Length | Height (EGM96) | Speed | Min clearance over the DEM | What to check |
|---|---|---|---|---|---|---|
${rows.join('\n')}

## Gate 1: import into Pilot 2 (no flight, no props)
1. Copy the .kmz files to the RC (USB → File transfer) → Pilot 2 → Flight Route → **Import route** (KMZ).
2. \`test-01\`: does it import (note any message)? Does it draw on the right place? Open the route and note:
   - the height Pilot 2 shows for waypoint 1 and for the highest waypoint, and whether it says ASL / EGM96 / ALT;
   - the speed it shows, and whether the legs between waypoints are drawn straight. The route's global speed is
     deliberately the SLOWEST waypoint speed and the global height the HIGHEST waypoint (safe fallbacks), so the
     route summary may look slow and high: what counts is the speed and height on each waypoint;
   - the actions on waypoint 1 (gimbal, IMU calibration, start point cloud recording) and on the last waypoint.
3. **Save test-01 in Pilot 2 without changing anything, then export it** (Flight Route → select → export KMZ) and copy that
   file back to the PC as \`samples/pilot2-reexport-test-01.kmz\`. Comparing what Pilot 2 writes with what we wrote is the
   single most useful check: \`node tools/compare-kmz.ts samples/generated/test-01-small-l3-survey.kmz samples/pilot2-reexport-test-01.kmz\`.
4. \`test-02\`, \`test-03\`: note the speed Pilot 2 shows for each (15 and 17 m/s expected).
5. Cap probes: the largest file that imports cleanly is our **max waypoints per route**. Note the number and any message.

## Gate 2: simulator (props off, aircraft on the bench)
Only after Gate 1. Run \`test-01\` in the DJI simulator, started from the ground near the route, and note:
whether the L3 starts and stops recording, the height shown at waypoint 1, whether the aircraft slows or stops at
waypoints, and what it does on "RTH" pressed mid-line.
`);
console.log('\n' + rows.join('\n'));
