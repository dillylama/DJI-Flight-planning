// Compares two route KMZs waypoint by waypoint: normally ours and the same route after Pilot 2 has imported,
// saved and exported it again. Shows what Pilot 2 changed, and (with --dem) runs the terrain check on its version.
//   node tools/compare-kmz.ts <ours.kmz> <pilot2.kmz> [--dem samples/generated/dem.tif] [--agl 100] [--corridor 75]
import { readFileSync } from 'node:fs';
import JSZip from 'jszip';
import { parseRouteFile, verifyRouteFiles, readGeoTiff, rasterElev, type ParsedRoute, type FileWaypoint } from '../packages/core/src/index.ts';

const args = process.argv.slice(2);
const flag = (name: string) => { const i = args.indexOf('--' + name); return i >= 0 ? args.splice(i, 2)[1] : undefined; };
const demPath = flag('dem'), agl = Number(flag('agl') ?? 100), corridor = Number(flag('corridor') ?? 75);
const [oursPath, theirsPath] = args;
if (!oursPath || !theirsPath) { console.error('usage: node tools/compare-kmz.ts <ours.kmz> <pilot2.kmz> [--dem dem.tif] [--agl 100] [--corridor 75]'); process.exit(2); }

async function load(path: string) {
  const zip = await JSZip.loadAsync(readFileSync(path));
  const find = (re: RegExp) => Object.keys(zip.files).find(f => re.test(f));
  const t = find(/template\.kml$/i), w = find(/waylines\.wpml$/i);
  if (!t || !w) throw new Error(`${path}: template.kml or waylines.wpml missing (found ${Object.keys(zip.files).join(', ')})`);
  const template = await zip.files[t].async('string'), waylines = await zip.files[w].async('string');
  return { template, waylines, t: parseRouteFile(template, 'template'), w: parseRouteFile(waylines, 'waylines') };
}
const ours = await load(oursPath), theirs = await load(theirsPath);

let differences = 0;
const diff = (what: string, a: unknown, b: unknown) => { differences++; console.log(`  DIFFERENT  ${what}: ours ${JSON.stringify(a)}  |  Pilot 2 ${JSON.stringify(b)}`); };
const HEAD: (keyof ParsedRoute)[] = ['namespace', 'drone', 'payload', 'payloadPosition', 'flyToWaylineMode', 'finishAction', 'exitOnRCLost', 'rcLostAction', 'takeOffSecurityHeight', 'transitSpeed', 'heightMode', 'autoFlightSpeed', 'globalHeight'];

function compare(kind: string, a: ParsedRoute, b: ParsedRoute) {
  console.log(`\n── ${kind}`);
  const before = differences;
  for (const k of HEAD) if (JSON.stringify(a[k]) !== JSON.stringify(b[k]) && !(Number.isNaN(a[k] as number) && Number.isNaN(b[k] as number))) diff(String(k), a[k], b[k]);
  if (JSON.stringify(a.startActions) !== JSON.stringify(b.startActions)) diff('start actions', a.startActions, b.startActions);
  if (a.waypoints.length !== b.waypoints.length) { diff('waypoint count', a.waypoints.length, b.waypoints.length); return; }
  const worst = { pos: 0, h: 0, hEll: 0 };
  const counts: Record<string, number> = {};
  const note = (what: string, i: number, x: unknown, y: unknown) => { counts[what] = (counts[what] ?? 0) + 1; if (counts[what] <= 3) diff(`waypoint ${i + 1} ${what}`, x, y); };
  a.waypoints.forEach((p: FileWaypoint, i: number) => {
    const q = b.waypoints[i];
    const dPos = Math.hypot((p.lon - q.lon) * 111320 * Math.cos(p.lat * Math.PI / 180), (p.lat - q.lat) * 110900);
    worst.pos = Math.max(worst.pos, dPos); worst.h = Math.max(worst.h, Math.abs(p.hEgm96 - q.hEgm96)); worst.hEll = Math.max(worst.hEll, Math.abs(p.hEllipsoid - q.hEllipsoid));
    if (dPos > 0.05) note('position (m apart)', i, 0, +dPos.toFixed(3));
    if (Math.abs(p.hEgm96 - q.hEgm96) > 0.1) note('EGM96 height', i, +p.hEgm96.toFixed(2), +q.hEgm96.toFixed(2));
    if (Math.abs(p.hEllipsoid - q.hEllipsoid) > 0.1) note('ellipsoid height', i, +p.hEllipsoid.toFixed(2), +q.hEllipsoid.toFixed(2));
    if (Math.abs(p.speed - q.speed) > 0.011) note('speed', i, p.speed, q.speed);
    if (p.turnMode !== q.turnMode) note('turn mode', i, p.turnMode, q.turnMode);
    if (!(Math.abs(p.damping - q.damping) <= 0.011) && !(Number.isNaN(p.damping) && Number.isNaN(q.damping))) note('turn damping', i, p.damping, q.damping);
    if (p.straight !== q.straight) note('useStraightLine', i, p.straight, q.straight);
    const acts = (w: FileWaypoint) => w.groups.map(g => `${g.trigger}[${g.start}-${g.end}]:${g.actions.map(x => x.func + (x.params.recordPointCloudOperate ? '=' + x.params.recordPointCloudOperate : '')).join('+')}`);
    if (JSON.stringify(acts(p)) !== JSON.stringify(acts(q))) note('actions', i, acts(p), acts(q));
  });
  for (const [what, n] of Object.entries(counts)) if (n > 3) console.log(`  … ${what}: ${n} waypoints differ in all`);
  console.log(`  ${a.waypoints.length} waypoints; largest difference: position ${worst.pos.toFixed(3)} m, EGM96 height ${worst.h.toFixed(3)} m, ellipsoid height ${worst.hEll.toFixed(3)} m`);
  if (differences === before) console.log('  identical in everything that is compared');
}
compare('waylines.wpml (what the aircraft executes)', ours.w, theirs.w);
compare('template.kml (what Pilot 2 edits)', ours.t, theirs.t);

if (demPath) {
  const buf = readFileSync(demPath);
  const elev = rasterElev(await readGeoTiff(buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength) as ArrayBuffer));
  console.log(`\n── terrain check of the Pilot 2 version (AGL ${agl} m, corridor ±${corridor} m)`);
  const rep = verifyRouteFiles(theirs.template, theirs.waylines, { elev, aglM: agl, corridorM: corridor, maxClimbMs: 4, maxDescentMs: 3, maxSpeedMs: 20, lidar: true });
  console.log(rep.ok ? `  PASSED: lowest leg clearance ${rep.minClearanceM.toFixed(1)} m` : '  FAILED:\n' + rep.issues.map(i => `    ${i.code}: ${i.message}`).join('\n'));
  if (!rep.ok) differences++;
}
console.log(differences ? `\n${differences} difference(s): Pilot 2 does not keep the route exactly as written. Do not fly until each one is understood.` : '\nNo differences: Pilot 2 keeps the route exactly as written.');
process.exit(differences ? 1 : 0);
