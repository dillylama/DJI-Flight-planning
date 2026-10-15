// Checks surveyed ground points against a processed point cloud (LAS) and the planning DEM.
//   node tools/gcp-check.ts <points.csv> <cloud.las> [dem.tif] [--utm 34S] [--radius 1.0] [--heights ellipsoidal|egm96] [--every 1]
// points.csv: a header line, then one point per line with columns found by name (case-insensitive): id/name, lat/latitude,
// lon/longitude/lng, h/height/ellh/altitude (ellipsoidal, WGS84) and optionally an orthometric column (orthometric/egm96/msl).
// The Emlid Flow CSV export works as is. Cloud coordinates as in cloud-vs-dem.ts (Terra: UTM, ellipsoidal heights).
// For each point: number of cloud points within the radius, their lowest and median height, cloud − point (both),
// and DEM − point (DEM is EGM96, so the point is converted with the same geoid model the planner uses). Nothing prints
// coordinates; the ids are printed as given.
import { readFileSync, openSync, readSync, closeSync, createReadStream } from 'node:fs';
import { ellipsoidToEgm96 } from 'egm96-universal';
import { readGeoTiff, rasterElev } from '../packages/core/src/index.ts';

const args = process.argv.slice(2);
const flag = (name: string, d?: string) => { const i = args.indexOf('--' + name); return i >= 0 ? args.splice(i, 2)[1] : d; };
const utm = flag('utm'), radius = +flag('radius', '1.0')!, heights = flag('heights', 'ellipsoidal'), every = +flag('every', '1')!;
const [csvPath, lasPath, demPath] = args;
if (!csvPath || !lasPath) { console.error('usage: node tools/gcp-check.ts <points.csv> <cloud.las> [dem.tif] [--utm 34S] [--radius 1.0] [--heights ellipsoidal|egm96] [--every 1]'); process.exit(2); }

// ---- points
const lines = readFileSync(csvPath, 'utf8').split(/\r?\n/).filter(l => l.trim());
const sep = lines[0].includes(';') && !lines[0].includes(',') ? ';' : ',';
const head = lines[0].split(sep).map(s => s.trim().replace(/^"|"$/g, '').toLowerCase());
const col = (...names: string[]) => head.findIndex(h => names.some(n => h === n || h.startsWith(n)));
const iId = col('id', 'name', 'point'), iLat = col('lat'), iLon = col('lon', 'lng', 'long'), iH = col('ellipsoidal height', 'ellh', 'h', 'height', 'altitude', 'elevation'), iOrtho = col('orthometric', 'egm96', 'msl');
if (iLat < 0 || iLon < 0 || iH < 0) { console.error(`cannot find lat/lon/height columns in: ${head.join(' | ')}`); process.exit(2); }
interface Pt { id: string; lat: number; lon: number; h: number; ortho: number | null }
const pts: Pt[] = lines.slice(1).map((l, k) => { const c = l.split(sep).map(s => s.trim().replace(/^"|"$/g, '')); return { id: iId >= 0 ? c[iId] : String(k + 1), lat: +c[iLat], lon: +c[iLon], h: +c[iH], ortho: iOrtho >= 0 && c[iOrtho] !== '' ? +c[iOrtho] : null }; })
  .filter(p => Number.isFinite(p.lat) && Number.isFinite(p.lon) && Number.isFinite(p.h));
console.log(`${pts.length} points from ${csvPath.split(/[\\/]/).pop()} (columns: id=${head[iId] ?? '-'}, lat=${head[iLat]}, lon=${head[iLon]}, height=${head[iH]}${iOrtho >= 0 ? ', orthometric=' + head[iOrtho] : ''})`);
const lat0 = pts[0].lat, mLat = 110900, mLon = 111320 * Math.cos(lat0 * Math.PI / 180);

// ---- UTM → lat/lon (same as cloud-vs-dem.ts)
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

// ---- LAS header
const hb = Buffer.alloc(375); { const fd = openSync(lasPath, 'r'); readSync(fd, hb, 0, 375, 0); closeSync(fd); }
if (hb.toString('latin1', 0, 4) !== 'LASF') { console.error('not a LAS file'); process.exit(1); }
const offPts = hb.readUInt32LE(96), recLen = hb.readUInt16LE(105), verMin = hb[25];
let nPts = hb.readUInt32LE(107); if (nPts === 0 && verMin >= 4) nPts = Number(hb.readBigUInt64LE(247));
const sx = hb.readDoubleLE(131), sy = hb.readDoubleLE(139), sz = hb.readDoubleLE(147), ox = hb.readDoubleLE(155), oy = hb.readDoubleLE(163), oz = hb.readDoubleLE(171);
console.log(`LAS ${hb[24]}.${verMin}, ${nPts.toLocaleString()} points; radius ${radius} m${every > 1 ? ', every ' + every + 'th point' : ''}`);

// ---- stream: collect heights near each point (cell-indexed on 2 m bins for speed)
const near: number[][] = pts.map(() => []);
const bins = new Map<string, number[]>();
pts.forEach((p, i) => { const bx = Math.floor((p.lon - pts[0].lon) * mLon / 2), by = Math.floor((p.lat - pts[0].lat) * mLat / 2); for (let dx = -1; dx <= 1; dx++) for (let dy = -1; dy <= 1; dy++) { const k = (bx + dx) + ',' + (by + dy); (bins.get(k) ?? bins.set(k, []).get(k)!).push(i); } });
let seen = 0, rem = Buffer.alloc(0);
await new Promise<void>((res, rej) => {
  const s = createReadStream(lasPath, { start: offPts, highWaterMark: 8 << 20 });
  s.on('data', (chunk: Buffer) => {
    const buf = rem.length ? Buffer.concat([rem, chunk]) : chunk;
    const full = Math.floor(buf.length / recLen) * recLen;
    for (let p = 0; p < full; p += recLen) {
      seen++; if (every > 1 && seen % every) continue;
      const X = buf.readInt32LE(p) * sx + ox, Y = buf.readInt32LE(p + 4) * sy + oy, Z = buf.readInt32LE(p + 8) * sz + oz;
      let lat: number, lon: number;
      if (utm) [lat, lon] = utmToLatLon(X, Y, zone, south); else { lon = X; lat = Y; }
      const k = Math.floor((lon - pts[0].lon) * mLon / 2) + ',' + Math.floor((lat - pts[0].lat) * mLat / 2);
      const cand = bins.get(k); if (!cand) continue;
      for (const i of cand) { const q = pts[i]; if (Math.hypot((lon - q.lon) * mLon, (lat - q.lat) * mLat) <= radius) near[i].push(Z); }
    }
    rem = Buffer.from(buf.subarray(full));
  });
  s.on('end', res); s.on('error', rej);
});

// ---- DEM
let elev: ((lon: number, lat: number) => number | null) | null = null;
if (demPath) { const db = readFileSync(demPath); elev = rasterElev(await readGeoTiff(db.buffer.slice(db.byteOffset, db.byteOffset + db.byteLength) as ArrayBuffer)); }

const med = (a: number[]) => { const s = [...a].sort((x, y) => x - y); return s[Math.floor(s.length / 2)]; };
console.log(`\n${'id'.padEnd(10)} ${'n'.padStart(5)}  ${'cloud lowest − pt'.padStart(18)}  ${'cloud median − pt'.padStart(18)}  ${'DEM − pt'.padStart(10)}`);
const dLow: number[] = [], dMed: number[] = [], dDem: number[] = [];
for (let i = 0; i < pts.length; i++) {
  const p = pts[i], z = near[i];
  const ptH = heights === 'egm96' ? (p.ortho ?? ellipsoidToEgm96(p.lat, p.lon, p.h)) : p.h;   // compare in the cloud's own datum
  let row = `${p.id.padEnd(10)} ${String(z.length).padStart(5)}`;
  if (z.length) { const lo = Math.min(...z) - ptH, m = med(z) - ptH; dLow.push(lo); dMed.push(m); row += `  ${(lo * 100).toFixed(1).padStart(15)} cm  ${(m * 100).toFixed(1).padStart(15)} cm`; }
  else row += `  ${'no cloud points'.padStart(18)}  ${''.padStart(18)}`;
  if (elev) { const d = elev(p.lon, p.lat); const ptEgm = p.ortho ?? ellipsoidToEgm96(p.lat, p.lon, p.h); if (d != null && Number.isFinite(d)) { dDem.push(d - ptEgm); row += `  ${(d - ptEgm).toFixed(2).padStart(8)} m`; } }
  console.log(row);
}
const stats = (a: number[], unit: 'cm' | 'm') => { if (!a.length) return 'no data'; const k = unit === 'cm' ? 100 : 1; const mean = a.reduce((x, y) => x + y, 0) / a.length; const rms = Math.sqrt(a.reduce((x, y) => x + y * y, 0) / a.length); return `mean ${(mean * k).toFixed(unit === 'cm' ? 1 : 2)} ${unit}, rms ${(rms * k).toFixed(unit === 'cm' ? 1 : 2)} ${unit}, spread ${(Math.min(...a) * k).toFixed(unit === 'cm' ? 1 : 2)}…${(Math.max(...a) * k).toFixed(unit === 'cm' ? 1 : 2)} ${unit}`; };
console.log(`\ncloud lowest − point: ${stats(dLow, 'cm')}\ncloud median − point: ${stats(dMed, 'cm')}${elev ? `\nGLO-30 DEM − point:   ${stats(dDem, 'm')}` : ''}`);
console.log('(positive = cloud/DEM above the surveyed point)');
