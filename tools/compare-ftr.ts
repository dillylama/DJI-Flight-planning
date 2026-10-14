// Compares a Pilot 2 flight-task replay record with the route file that was flown.
//   node tools/compare-ftr.ts <route.kmz> <FTR_Record_….txt> [--mrk photos.MRK]
// The record lives on the RC at /sdcard/DJI/com.dji.industry.pilot/FlightRecord/FlightTaskReplay/FTR_Record_<id>/,
// a gzip-compressed JSON with a 5 Hz track (frameState: time, location lat/lon/ellipsoidal altitude, attitude).
// Prints, per waypoint the aircraft passed: pass distance, height against the file (both ellipsoidal), ground speed;
// then rates, the worst below-leg height and the trigger events. With --mrk the L3's RTK photo positions are checked
// against the route too (a second, independent height source). Positions are never printed.
import { readFileSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import JSZip from 'jszip';
import { parseRouteFile } from '../packages/core/src/index.ts';

const args = process.argv.slice(2);
const flag = (name: string) => { const i = args.indexOf('--' + name); return i >= 0 ? args.splice(i, 2)[1] : undefined; };
const mrkPath = flag('mrk');
const [kmzPath, ftrPath] = args;
if (!kmzPath || !ftrPath) { console.error('usage: node tools/compare-ftr.ts <route.kmz> <FTR_Record.txt> [--mrk photos.MRK]'); process.exit(2); }

const zip = await JSZip.loadAsync(readFileSync(kmzPath));
const wName = Object.keys(zip.files).find(f => /waylines\.wpml$/.test(f))!;
const W = parseRouteFile(await zip.files[wName].async('string'), 'waylines').waypoints;
const mLat = 110900, mLon = 111320 * Math.cos(W[0].lat * Math.PI / 180);

let raw = readFileSync(ftrPath);
if (raw[0] === 0x1f && raw[1] === 0x8b) raw = gunzipSync(raw);
const rec = JSON.parse(raw.toString('utf8'));
const uav = rec.device.uav[0];
const homeAlt: number = uav.home.altitude;
const t0: number = rec.flight.start_time;
interface S { t: number; lat: number; lon: number; alt: number; v: number; vz: number }
const S: S[] = [];
for (const f of rec.frameState) {
  const fc = f.uav?.[0]?.fc?.[0]; if (!fc) continue;
  S.push({ t: (f.time - t0) / 1000, lat: fc.location.latitude, lon: fc.location.longitude, alt: fc.location.altitude, v: 0, vz: 0 });
}
for (let i = 1; i < S.length; i++) {
  const a = S[i - 1], b = S[i], dt = b.t - a.t; if (dt <= 0) continue;
  b.v = Math.hypot((b.lon - a.lon) * mLon, (b.lat - a.lat) * mLat) / dt; b.vz = (b.alt - a.alt) / dt;
}
const hms = (ms: number) => new Date(ms).toLocaleTimeString('en-GB');
const stats = (a: number[]) => a.length ? `mean ${(a.reduce((x, y) => x + y, 0) / a.length).toFixed(1)}, spread ${Math.min(...a).toFixed(1)}…${Math.max(...a).toFixed(1)}` : 'no data';
console.log(`record ${hms(t0)}–${hms(rec.flight.end_time)} local, ${S.length} positions at ${rec.flight.sampling_rate} Hz; aircraft fw ${uav.fw_version}, fc ${uav.fc[0].fw_version}; home ${homeAlt.toFixed(1)} m ellipsoidal`);
console.log(`route ${W.length} waypoints; record: max height ${uav.max_height.toFixed(1)} m ell (${(uav.max_height - homeAlt).toFixed(1)} above home), max horizontal speed ${uav.max_horizontal_speed.toFixed(1)} m/s, max vertical ${uav.max_vertical_speed.toFixed(1)} m/s`);

// closest approach to each waypoint (horizontal), in route order and inside the mission window (trigger 40 = mission
// started, 43 = mission finished, when present) so the return flight over the block is not mistaken for a pass
const trig = (code: number) => rec.trigger?.find((t: { action: number }) => t.action === code);
const tStart = trig(40) ? (trig(40).time - t0) / 1000 : 0, tEnd = trig(43) ? (trig(43).time - t0) / 1000 : Infinity;
console.log(`mission window t+${tStart.toFixed(0)}…${Number.isFinite(tEnd) ? tEnd.toFixed(0) : '∞'} s`);
const rows: string[] = []; const err: number[] = []; const passes: number[] = []; const passIdx: number[] = [];
let from = S.findIndex(s => s.t >= tStart);
for (let i = 0; i < W.length; i++) {
  let best = -1, bestD = Infinity;
  for (let k = Math.max(0, from); k < S.length && S[k].t <= tEnd; k++) {
    const d = Math.hypot((S[k].lon - W[i].lon) * mLon, (S[k].lat - W[i].lat) * mLat);
    if (d < bestD) { bestD = d; best = k; }
  }
  if (best < 0 || bestD > 30) { rows.push(`wp ${String(i + 1).padStart(3)}  not passed within 30 m (closest ${bestD.toFixed(0)} m)`); continue; }
  const s = S[best]; passIdx.push(best); from = best;
  err.push(s.alt - W[i].hEllipsoid); passes.push(bestD);
  const win = S.filter(x => Math.abs(x.t - s.t) <= 1 && x.v > 0); const v = win.reduce((a, x) => a + x.v, 0) / Math.max(1, win.length);
  rows.push(`wp ${String(i + 1).padStart(3)}  t+${s.t.toFixed(0).padStart(4)} s  pass ${bestD.toFixed(1).padStart(4)} m  alt ${s.alt.toFixed(1)} (${(s.alt - homeAlt).toFixed(1).padStart(5)} above home)  file ${W[i].hEllipsoid.toFixed(1)}  diff ${(s.alt - W[i].hEllipsoid).toFixed(1).padStart(5)} m  speed ${v.toFixed(1).padStart(4)} / file ${W[i].speed.toFixed(1)}  ${W[i].turnMode ?? ''}`);
}
console.log(rows.join('\n'));
console.log(`\nheight − file at ${err.length}/${W.length} waypoints: ${stats(err)} m;  pass distance: ${stats(passes)} m`);

// along the route: worst height relative to the straight leg between consecutive waypoints, rates, speeds
if (passIdx.length > 1) {
  let worst = Infinity, worstLeg = -1, maxUp = 0, maxDown = 0, minV = Infinity, maxV = 0;
  for (let i = 0; i < passIdx.length - 1; i++) {
    const a = passIdx[i], b = passIdx[i + 1]; if (b <= a) continue;
    const wa = W[i], wb = W[i + 1];
    const dx = (wb.lon - wa.lon) * mLon, dy = (wb.lat - wa.lat) * mLat, L2 = dx * dx + dy * dy;
    for (let k = a; k <= b; k++) {
      const s = S[k];
      const f = L2 > 0 ? Math.max(0, Math.min(1, (((s.lon - wa.lon) * mLon) * dx + ((s.lat - wa.lat) * mLat) * dy) / L2)) : 0;
      const legH = wa.hEllipsoid + f * (wb.hEllipsoid - wa.hEllipsoid);
      if (s.alt - legH < worst) { worst = s.alt - legH; worstLeg = i + 1; }
      maxUp = Math.max(maxUp, s.vz); maxDown = Math.max(maxDown, -s.vz);
      if (k > a) { maxV = Math.max(maxV, s.v); if (s.v > 0.5) minV = Math.min(minV, s.v); }
    }
  }
  console.log(`between waypoints: lowest ${worst.toFixed(1)} m relative to the straight leg (leg from wp ${worstLeg}); climb up to ${maxUp.toFixed(1)} m/s, descent up to ${maxDown.toFixed(1)} m/s; ground speed ${minV.toFixed(1)}–${maxV.toFixed(1)} m/s`);
}

console.log('\ntriggers (Pilot 2 action codes as recorded):');
for (const tr of rec.trigger ?? []) console.log(`  action ${tr.action}  t+${((tr.time - t0) / 1000).toFixed(0)} s  ${hms(tr.time)}  ${JSON.stringify(tr.value)}`);

if (mrkPath) {
  // L3 .MRK columns: photo, gps seconds, [week], N mm, E mm, V mm, lat, lon, ellipsoidal height, sd n/e/v, Q (50 = RTK fixed)
  const lines = readFileSync(mrkPath, 'utf8').split('\n').filter(l => /Ellh/.test(l));
  const P = lines.map(l => { const c = l.split('\t'); return { id: c[0], lat: parseFloat(c[6]), lon: parseFloat(c[7]), h: parseFloat(c[8]), q: parseInt(c[10]) }; }).filter(p => Number.isFinite(p.lat));
  const fixed = P.filter(p => p.q === 50).length;
  const d: number[] = [];
  for (const p of P) {
    let best = Infinity, hLeg = NaN;
    for (let i = 0; i < W.length - 1; i++) {
      const wa = W[i], wb = W[i + 1];
      const dx = (wb.lon - wa.lon) * mLon, dy = (wb.lat - wa.lat) * mLat, L2 = dx * dx + dy * dy;
      const px = (p.lon - wa.lon) * mLon, py = (p.lat - wa.lat) * mLat;
      const f = L2 > 0 ? Math.max(0, Math.min(1, (px * dx + py * dy) / L2)) : 0;
      const dist = Math.hypot(px - f * dx, py - f * dy);
      if (dist < best) { best = dist; hLeg = wa.hEllipsoid + f * (wb.hEllipsoid - wa.hEllipsoid); }
    }
    if (best <= 15) d.push(p.h - hLeg);
  }
  console.log(`\nL3 photo positions (${mrkPath.split(/[\\/]/).pop()}): ${P.length} photos, ${fixed} RTK fixed; antenna height − route leg at ${d.length} photos within 15 m of a leg: ${stats(d)} m`);
}
