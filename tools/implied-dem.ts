// What terrain did a terrain-following flight believe in? implied ground = flown height − planned AGL, compared with
// the planning DEM at the point, binned by how much the DEM rises within ±75 m (hill tops vs flats).
//   node tools/implied-dem.ts <FTR_Record.txt> <dem.tif> <aglM> <fromS> <toS>
import { readFileSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { ellipsoidToEgm96 } from 'egm96-universal';
import { readGeoTiff, rasterElev } from '../packages/core/src/index.ts';
const [ftr, dem, aglS, from, to] = process.argv.slice(2); const agl = +aglS;
let raw = readFileSync(ftr); if (raw[0] === 0x1f) raw = gunzipSync(raw);
const rec = JSON.parse(raw.toString()); const t0 = rec.flight.start_time;
const db = readFileSync(dem); const elev = rasterElev(await readGeoTiff(db.buffer.slice(db.byteOffset, db.byteOffset + db.byteLength) as ArrayBuffer));
type R = { err: number; relief: number; demH: number; vz: number };
const rows: R[] = []; let prev: { t: number; h: number } | null = null;
for (const f of rec.frameState) {
  const fc = f.uav?.[0]?.fc?.[0]; if (!fc?.location) continue;
  const t = (f.time - t0) / 1000; if (t < +from || t > +to) continue;
  const { latitude: lat, longitude: lon, altitude } = fc.location; const h = ellipsoidToEgm96(lat, lon, altitude);
  const d0 = elev(lon, lat); if (!Number.isFinite(d0)) continue;
  const mLat = 110900, mLon = 111320 * Math.cos(lat * Math.PI / 180);
  let mx = d0, mn = d0;
  for (let dx = -75; dx <= 75; dx += 15) for (let dy = -75; dy <= 75; dy += 15) { const e = elev(lon + dx / mLon, lat + dy / mLat); if (Number.isFinite(e)) { if (e > mx) mx = e; if (e < mn) mn = e; } }
  const vz = prev ? (h - prev.h) / (t - prev.t) : 0; prev = { t, h };
  rows.push({ err: h - agl - d0, relief: mx - mn, demH: d0, vz });
}
const q = (a: number[], p: number) => { const s = [...a].sort((x, y) => x - y); return s[Math.floor(p * (s.length - 1))]; };
const st = (a: number[]) => `n ${String(a.length).padStart(5)}  lowest ${q(a, 0).toFixed(1).padStart(6)}  5 % ${q(a, 0.05).toFixed(1).padStart(6)}  median ${q(a, 0.5).toFixed(1).padStart(6)}  95 % ${q(a, 0.95).toFixed(1).padStart(6)}  highest ${q(a, 1).toFixed(1).padStart(6)} m`;
console.log(`implied ground (flown height − ${agl} m) minus GLO-30 at the point; negative = the flight believed the ground lower than GLO-30 says`);
console.log(`  all samples:                 ${st(rows.map(r => r.err))}`);
for (const [lo, hi] of [[0, 5], [5, 15], [15, 30], [30, 1e9]] as const) {
  const a = rows.filter(r => r.relief >= lo && r.relief < hi).map(r => r.err); if (a.length) console.log(`  DEM relief within ±75 m ${String(lo).padStart(2)}–${hi > 1000 ? '∞ ' : String(hi).padStart(2)} m: ${st(a)}`);
}
const hs = rows.map(r => r.demH); const h20 = q(hs, 0.8), h50 = q(hs, 0.5);
console.log(`  over the highest fifth of the ground (DEM ≥ ${h20.toFixed(0)} m): ${st(rows.filter(r => r.demH >= h20).map(r => r.err))}`);
console.log(`  over the lower half (DEM < ${h50.toFixed(0)} m):            ${st(rows.filter(r => r.demH < h50).map(r => r.err))}`);
