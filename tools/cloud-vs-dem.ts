// Compares a processed L3 point cloud (LAS) with the DEM the route was planned on, and measures the real AGL.
//   node tools/cloud-vs-dem.ts <cloud.las> <dem.tif> <route.kmz | --track FTR_Record.txt [--from s --to s]> [--utm 34S] [--heights ellipsoidal|egm96] [--cell 10]
//                              [--class 2] [--every 1] [--corridor 40] [--agl 200]
// The LAS is streamed (any size). Points are binned into square cells; the lowest point per cell is the ground
// estimate (or, with --class N, the lowest point of that classification, e.g. 2 = ground after Terra's
// classification). LAS coordinates may be geographic (x = longitude, y = latitude) or a UTM zone (--utm 34S).
// Heights in the LAS are ellipsoidal (Terra's default for WGS 84) unless --heights egm96; the DEM is EGM96, so cloud
// heights are converted with the same geoid model the planner uses. Output: cloud ground − DEM per cell (mean, rms,
// spread, percentiles), the worst cells, and the AGL the aircraft really had over the cells inside the route corridor
// against the planned AGL. Nothing here prints coordinates.
import { createReadStream, readFileSync } from 'node:fs';
import JSZip from 'jszip';
import { ellipsoidToEgm96 } from 'egm96-universal';
import { readGeoTiff, rasterElev, parseRouteFile } from '../packages/core/src/index.ts';

const args = process.argv.slice(2);
const flag = (name: string, d?: string) => { const i = args.indexOf('--' + name); return i >= 0 ? args.splice(i, 2)[1] : d; };
const utm = flag('utm'), geo = flag('geo'), heights = flag('heights', 'ellipsoidal'), cellM = +flag('cell', '10')!, cls = flag('class'), every = +flag('every', '1')!, corridorM = +flag('corridor', '40')!, aglPlan = +flag('agl', '0')!, track = flag('track'), tFrom = +flag('from', '0')!, tTo = +flag('to', 'Infinity')!;
const [lasPath, demPath, kmzPath] = args;
if (!lasPath || !demPath || !(kmzPath || track)) { console.error('usage: node tools/cloud-vs-dem.ts <cloud.las> <dem.tif> <route.kmz> [--utm 34S | --geo lidar_geo_desc.json] [--heights ellipsoidal|egm96] [--cell 10] [--class 2] [--every 1] [--corridor 40] [--agl 200]'); process.exit(2); }
// DJI Terra's default output frame is a local ENU system (lidar_geo_desc.json: cs_type LOCAL_ENU_CS, ref_GPS lat/lon/
// ellipsoidal altitude): x = metres east, y = metres north of the reference, z = metres above its altitude.
let enu: { lat: number; lon: number; alt: number } | null = null;
if (geo) {
  const g = JSON.parse(readFileSync(geo, 'utf8'));
  if (g.cs_type !== 'LOCAL_ENU_CS') { console.error(`cs_type ${g.cs_type} not handled; use --utm or geographic output`); process.exit(2); }
  enu = { lat: g.ref_GPS.latitude, lon: g.ref_GPS.longitude, alt: g.ref_GPS.altitude };
}

// ---- UTM → lat/lon (WGS84), only needed when the cloud is projected
function utmToLatLon(e: number, n: number, zone: number, south: boolean): [number, number] {
  const a = 6378137, f = 1 / 298.257223563, k0 = 0.9996, e2 = f * (2 - f), ep2 = e2 / (1 - e2);
  const x = e - 500000, y = south ? n - 10000000 : n;
  const M = y / k0, mu = M / (a * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 ** 3 / 256));
  const e1 = (1 - Math.sqrt(1 - e2)) / (1 + Math.sqrt(1 - e2));
  const phi1 = mu + (3 * e1 / 2 - 27 * e1 ** 3 / 32) * Math.sin(2 * mu) + (21 * e1 * e1 / 16 - 55 * e1 ** 4 / 32) * Math.sin(4 * mu) + (151 * e1 ** 3 / 96) * Math.sin(6 * mu);
  const N1 = a / Math.sqrt(1 - e2 * Math.sin(phi1) ** 2), T1 = Math.tan(phi1) ** 2, C1 = ep2 * Math.cos(phi1) ** 2;
  const R1 = a * (1 - e2) / (1 - e2 * Math.sin(phi1) ** 2) ** 1.5, D = x / (N1 * k0);
  const lat = phi1 - (N1 * Math.tan(phi1) / R1) * (D * D / 2 - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * ep2) * D ** 4 / 24 + (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * ep2 - 3 * C1 * C1) * D ** 6 / 720);
  const lon0 = ((zone - 1) * 6 - 180 + 3) * Math.PI / 180;
  const lon = lon0 + (D - (1 + 2 * T1 + C1) * D ** 3 / 6 + (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * ep2 + 24 * T1 * T1) * D ** 5 / 120) / Math.cos(phi1);
  return [lat * 180 / Math.PI, lon * 180 / Math.PI];
}
const zone = utm ? parseInt(utm) : 0, south = utm ? /S$/i.test(utm) : false;

// ---- route: the file that was flown, or (--track) the flown track itself from a Pilot 2 flight-task replay record
// (FTR_Record_…, 5 Hz, ellipsoidal altitude; see compare-ftr.ts), one position every 2 s inside the mission window.
// With --track the "route" is what the aircraft really did, so the AGL below is the real clearance of that flight.
let W: { lat: number; lon: number; hEgm96: number }[];
if (track) {
  let raw = readFileSync(track);
  if (raw[0] === 0x1f && raw[1] === 0x8b) raw = (await import('node:zlib')).gunzipSync(raw);
  const rec = JSON.parse(raw.toString('utf8'));
  const trig = (code: number) => rec.trigger?.find((t: { action: number }) => t.action === code);
  const t0 = rec.flight.start_time, tStart = trig(40)?.time ?? t0, tEnd = trig(43)?.time ?? Infinity;
  W = [];
  let k = 0;
  for (const f of rec.frameState) {
    const fc = f.uav?.[0]?.fc?.[0]; if (!fc?.location || f.time < tStart || f.time > tEnd || (f.time - t0) / 1000 < tFrom || (f.time - t0) / 1000 > tTo) continue;   // --from/--to: seconds after record start, to keep only the lines (the sloped climb-out is inside the mission window)
    if (k++ % 10) continue;
    const { latitude: lat, longitude: lon, altitude } = fc.location;
    W.push({ lat, lon, hEgm96: ellipsoidToEgm96(lat, lon, altitude) });
  }
  console.log(`track: ${W.length} positions (every 2 s) inside the mission window of ${track.split(/[\/]/).slice(-2)[0]}`);
} else {
  const zip = await JSZip.loadAsync(readFileSync(kmzPath));
  const wName = Object.keys(zip.files).find(f => /waylines\.wpml$/.test(f))!;
  W = parseRouteFile(await zip.files[wName].async('string'), 'waylines').waypoints;
}
const lat0 = W[0].lat, mLat = 110900, mLon = 111320 * Math.cos(lat0 * Math.PI / 180);
const legs = W.slice(0, -1).map((a, i) => ({ ax: 0, ay: 0, bx: (W[i + 1].lon - a.lon) * mLon, by: (W[i + 1].lat - a.lat) * mLat, a, b: W[i + 1] }));
function routeAt(lon: number, lat: number): { dist: number; h: number } {
  let best = Infinity, h = NaN;
  for (const L of legs) {
    const px = (lon - L.a.lon) * mLon, py = (lat - L.a.lat) * mLat, L2 = L.bx * L.bx + L.by * L.by;
    const f = L2 > 0 ? Math.max(0, Math.min(1, (px * L.bx + py * L.by) / L2)) : 0;
    const d = Math.hypot(px - f * L.bx, py - f * L.by);
    if (d < best) { best = d; h = L.a.hEgm96 + f * (L.b.hEgm96 - L.a.hEgm96); }
  }
  return { dist: best, h };
}

// ---- DEM
const db = readFileSync(demPath);
const elev = rasterElev(await readGeoTiff(db.buffer.slice(db.byteOffset, db.byteOffset + db.byteLength) as ArrayBuffer));

// ---- LAS header
const fd = readFileSync(lasPath, { flag: 'r' }).subarray(0, 0);   // placeholder to satisfy types; real read below
const head = Buffer.alloc(375);
{ const fs = await import('node:fs'); const h = fs.openSync(lasPath, 'r'); fs.readSync(h, head, 0, 375, 0); fs.closeSync(h); }
if (head.toString('latin1', 0, 4) !== 'LASF') { console.error('not a LAS file (LAZ must be decompressed first)'); process.exit(1); }
const verMaj = head[24], verMin = head[25];
const offPts = head.readUInt32LE(96), fmt = head[104] & 0x3f, recLen = head.readUInt16LE(105);
let nPts = head.readUInt32LE(107); if (nPts === 0 && verMin >= 4) nPts = Number(head.readBigUInt64LE(247));
const sx = head.readDoubleLE(131), sy = head.readDoubleLE(139), sz = head.readDoubleLE(147), ox = head.readDoubleLE(155), oy = head.readDoubleLE(163), oz = head.readDoubleLE(171);
const clsOff = fmt >= 6 ? 16 : 15;
console.log(`LAS ${verMaj}.${verMin}, point format ${fmt}, ${nPts.toLocaleString()} points, record ${recLen} B; coordinates ${enu ? 'local ENU (Terra)' : utm ? 'UTM ' + utm : 'geographic'}, heights ${heights}; cells ${cellM} m${cls ? ', class ' + cls : ''}${every > 1 ? ', every ' + every + 'th point' : ''}`);

// ---- stream points → per-cell lowest point (cell keyed on metres east/north of waypoint 1)
const cells = new Map<string, { z: number; lat: number; lon: number; n: number }>();
let seen = 0, used = 0, rem = Buffer.alloc(0);
const classWanted = cls != null ? +cls : null;
await new Promise<void>((res, rej) => {
  const s = createReadStream(lasPath, { start: offPts, highWaterMark: 8 << 20 });
  s.on('data', (chunk: Buffer) => {
    const buf = rem.length ? Buffer.concat([rem, chunk]) : chunk;
    const full = Math.floor(buf.length / recLen) * recLen;
    for (let p = 0; p < full; p += recLen) {
      seen++;
      if (every > 1 && seen % every) continue;
      if (classWanted !== null && (buf[p + clsOff] & (fmt >= 6 ? 0xff : 0x1f)) !== classWanted) continue;
      const X = buf.readInt32LE(p) * sx + ox, Y = buf.readInt32LE(p + 4) * sy + oy; let Z = buf.readInt32LE(p + 8) * sz + oz;
      let lat: number, lon: number;
      if (enu) { lat = enu.lat + Y / mLat; lon = enu.lon + X / (111320 * Math.cos(enu.lat * Math.PI / 180)); Z += enu.alt; }
      else if (utm) [lat, lon] = utmToLatLon(X, Y, zone, south); else { lon = X; lat = Y; }
      const ex = Math.floor((lon - W[0].lon) * mLon / cellM), ny = Math.floor((lat - W[0].lat) * mLat / cellM);
      const key = ex + ',' + ny; const c = cells.get(key);
      if (!c) cells.set(key, { z: Z, lat, lon, n: 1 }); else { c.n++; if (Z < c.z) { c.z = Z; c.lat = lat; c.lon = lon; } }
      used++;
    }
    rem = Buffer.from(buf.subarray(full));
  });
  s.on('end', res); s.on('error', rej);
});
console.log(`${seen.toLocaleString()} points read, ${used.toLocaleString()} used, ${cells.size.toLocaleString()} cells`);

// ---- compare
interface Row { dz: number; agl: number | null; dist: number; n: number }
const rows: Row[] = [];
for (const c of cells.values()) {
  if (c.n < 3) continue;
  const zEgm = heights === 'egm96' ? c.z : ellipsoidToEgm96(c.lat, c.lon, c.z);
  const dem = elev(c.lon, c.lat); if (!Number.isFinite(dem)) continue;
  const r = routeAt(c.lon, c.lat);
  rows.push({ dz: zEgm - dem, agl: r.dist <= corridorM ? r.h - zEgm : null, dist: r.dist, n: c.n });
}
const q = (a: number[], p: number) => a.length ? a[Math.min(a.length - 1, Math.floor(p * a.length))] : NaN;
if (!rows.length) { console.error('no cells with points: wrong coordinate option, or no points of that class (Terra writes class 2 only when ground classification was switched on)'); process.exit(1); }
const dzAll = rows.map(r => r.dz).sort((a, b) => a - b);
const stat = (a: number[]) => { const m = a.reduce((x, y) => x + y, 0) / a.length; return `mean ${m.toFixed(2)} m, rms ${Math.sqrt(a.reduce((x, y) => x + y * y, 0) / a.length).toFixed(2)} m, 5–95 % ${q(a, 0.05).toFixed(1)}…${q(a, 0.95).toFixed(1)} m, extremes ${a[0].toFixed(1)}…${a[a.length - 1].toFixed(1)} m`; };
console.log(`\ncloud ground − GLO-30 DEM, ${rows.length} cells: ${stat(dzAll)}`);
console.log(`  (positive = real ground HIGHER than the DEM, the dangerous direction)`);
const inCorr = rows.filter(r => r.agl !== null);
const dzCorr = inCorr.map(r => r.dz).sort((a, b) => a - b);
if (inCorr.length) {
  console.log(`inside the ±${corridorM} m corridor, ${inCorr.length} cells: ${stat(dzCorr)}`);
  const agl = inCorr.map(r => r.agl!).sort((a, b) => a - b);
  console.log(`measured AGL under the route (route height − cloud ground): lowest ${agl[0].toFixed(1)} m, 5 % ${q(agl, 0.05).toFixed(1)} m, median ${q(agl, 0.5).toFixed(1)} m, highest ${agl[agl.length - 1].toFixed(1)} m`);
  if (aglPlan > 0) {
    const bad = agl.filter(a => a < aglPlan - 10).length;
    console.log(`G4 criterion (never below planned ${aglPlan} m − 10): ${bad === 0 ? 'PASS' : 'FAIL, ' + bad + ' cells below ' + (aglPlan - 10) + ' m'}`);
  }
  const worst = inCorr.filter(r => r.dz > 0).sort((a, b) => b.dz - a.dz).slice(0, 5);
  console.log(`worst cells (ground above DEM): ${worst.map(r => `+${r.dz.toFixed(1)} m (${r.n} pts, ${r.dist.toFixed(0)} m off the line)`).join('; ') || 'none'}`);
}
