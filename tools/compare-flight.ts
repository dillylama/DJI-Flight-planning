// Compares flight data logged by 3DM Fly (adb logcat -s 3DMFlyFD) with the route file that was flown.
//   node tools/compare-flight.ts <route.kmz> <fd.log> [--from HH:MM:SS] [--to HH:MM:SS] [--takeoff-alt 100]
// For each waypoint the aircraft passed: the height it held there against the file (both readings of the
// file's ellipsoidal heights: as ellipsoidal, and converted to EGM96), the speed, and the straight-line
// distance to the waypoint. Then the worst vertical rate and the slowest leg. Positions are in the log, but
// nothing here prints them.
import { readFileSync } from 'node:fs';
import JSZip from 'jszip';
import { parseRouteFile } from '../packages/core/src/index.ts';

const args = process.argv.slice(2);
const flag = (name: string) => { const i = args.indexOf('--' + name); return i >= 0 ? args.splice(i, 2)[1] : undefined; };
const from = flag('from') ?? '00:00:00', to = flag('to') ?? '23:59:59', takeoffAlt = Number(flag('takeoff-alt') ?? 100);
const [kmzPath, logPath] = args;
if (!kmzPath || !logPath) { console.error('usage: node tools/compare-flight.ts <route.kmz> <fd.log> [--from HH:MM:SS] [--to HH:MM:SS] [--takeoff-alt 100]'); process.exit(2); }

const zip = await JSZip.loadAsync(readFileSync(kmzPath));
const wName = Object.keys(zip.files).find(f => /waylines\.wpml$/.test(f))!;
const W = parseRouteFile(await zip.files[wName].async('string'), 'waylines').waypoints;
const mLat = 110900, mLon = 111320 * Math.cos(W[0].lat * Math.PI / 180);

interface S { t: string; lat: number; lon: number; alt: number; vn: number; ve: number; vd: number; wp: number; st: string; rec: string; mode: string }
const samples: S[] = readFileSync(logPath, 'utf8').split('\n').filter(l => / FD /.test(l)).map(l => {
  const o: Record<string, string | number> = { t: l.slice(0, 8) };
  for (const m of l.matchAll(/(\w+)=([^ ]+)/g)) o[m[1]] = Number.isNaN(+m[2]) ? m[2] : +m[2];
  return o as unknown as S;
}).filter(s => s.t >= from && s.t <= to);
if (!samples.length) { console.error('no samples in that window'); process.exit(1); }
console.log(`${samples.length} samples ${samples[0].t}–${samples.at(-1)!.t}; route ${W.length} waypoints; take-off reference ${takeoffAlt} m`);

// closest approach to each waypoint (horizontal), with the height and speed at that moment
const rows: string[] = [];
let errEll: number[] = [], errEgm: number[] = [];
for (let i = 0; i < W.length; i++) {
  let best: S | null = null, bestD = Infinity;
  for (const s of samples) {
    const d = Math.hypot((s.lon - W[i].lon) * mLon, (s.lat - W[i].lat) * mLat);
    if (d < bestD) { bestD = d; best = s; }
  }
  if (!best || bestD > 25) continue;
  const relEll = W[i].hEllipsoid - takeoffAlt, relEgm = W[i].hEgm96 - takeoffAlt;
  errEll.push(best.alt - relEll); errEgm.push(best.alt - relEgm);
  const v = Math.hypot(best.vn, best.ve);
  rows.push(`wp ${String(i + 1).padStart(3)}  ${best.t}  pass ${bestD.toFixed(1).padStart(4)} m  alt ${best.alt.toFixed(1).padStart(6)}  file ell ${relEll.toFixed(1).padStart(6)} (${(best.alt - relEll).toFixed(1).padStart(5)})  egm ${relEgm.toFixed(1).padStart(6)} (${(best.alt - relEgm).toFixed(1).padStart(5)})  speed ${v.toFixed(1).padStart(4)} / file ${W[i].speed.toFixed(2)}  ${best.rec}`);
}
console.log(rows.join('\n'));
const stats = (a: number[]) => a.length ? `mean ${(a.reduce((x, y) => x + y, 0) / a.length).toFixed(1)} m, spread ${Math.min(...a).toFixed(1)}…${Math.max(...a).toFixed(1)} m` : 'no data';
console.log(`\nheight − file (ellipsoidal as written, above the take-off reference): ${stats(errEll)}`);
console.log(`height − file (converted to EGM96, above the take-off reference):    ${stats(errEgm)}`);

// vertical rate and speed, whole window
let maxUp = 0, maxDown = 0, maxV = 0, minVMoving = Infinity;
for (const s of samples) {
  if (s.st !== 'EXECUTING') continue;
  maxUp = Math.max(maxUp, -s.vd); maxDown = Math.max(maxDown, s.vd);
  const v = Math.hypot(s.vn, s.ve); maxV = Math.max(maxV, v);
  if (v > 0.5) minVMoving = Math.min(minVMoving, v);
}
console.log(`while executing: climb up to ${maxUp.toFixed(1)} m/s, descent up to ${maxDown.toFixed(1)} m/s, ground speed ${minVMoving.toFixed(1)}–${maxV.toFixed(1)} m/s`);
// heights below the straight leg between consecutive waypoints (interpolated along track), worst case
let worstBelow = 0, worstAt = '';
for (const s of samples) {
  if (s.st !== 'EXECUTING' || s.wp < 0 || s.wp + 1 >= W.length) continue;
  const a = W[s.wp], b = W[s.wp + 1];
  const ax = (s.lon - a.lon) * mLon, ay = (s.lat - a.lat) * mLat, bx = (b.lon - a.lon) * mLon, by = (b.lat - a.lat) * mLat;
  const L2 = bx * bx + by * by; if (L2 < 1) continue;
  const t = Math.min(1, Math.max(0, (ax * bx + ay * by) / L2));
  const legEll = a.hEllipsoid + t * (b.hEllipsoid - a.hEllipsoid) - takeoffAlt;
  const below = legEll - s.alt;
  if (below > worstBelow) { worstBelow = below; worstAt = `${s.t} on leg ${s.wp + 1}`; }
}
console.log(`lowest point relative to the straight legs (ellipsoidal reading): ${worstBelow.toFixed(1)} m below, at ${worstAt || 'never'}`);
