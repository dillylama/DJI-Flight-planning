// Flown track (Pilot 2 FTR record) against the planning DEM: clearance at the point and over the highest ground within ±75 m.
//   node tools/track-vs-dem.ts <FTR_Record.txt> <dem.tif> <fromS> <toS>   (seconds after record start; nothing printed is a position)
// track height (EGM96) minus planning DEM along a flown track, at the point and the highest DEM within +-75 m
import { readFileSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { ellipsoidToEgm96 } from 'egm96-universal';
import { readGeoTiff, rasterElev } from '../packages/core/src/index.ts';
const [ftr, dem, from, to] = process.argv.slice(2);
let raw = readFileSync(ftr); if (raw[0] === 0x1f) raw = gunzipSync(raw);
const rec = JSON.parse(raw.toString()); const t0 = rec.flight.start_time;
const db = readFileSync(dem); const elev = rasterElev(await readGeoTiff(db.buffer.slice(db.byteOffset, db.byteOffset + db.byteLength) as ArrayBuffer));
const pt: number[] = [], corr: number[] = [];
for (const f of rec.frameState) {
  const fc = f.uav?.[0]?.fc?.[0]; if (!fc?.location) continue;
  const t = (f.time - t0) / 1000; if (t < +from || t > +to) continue;
  const { latitude: lat, longitude: lon, altitude } = fc.location; const h = ellipsoidToEgm96(lat, lon, altitude);
  const d0 = elev(lon, lat); if (!Number.isFinite(d0)) continue;
  pt.push(h - d0);
  let mx = d0; const mLat = 110900, mLon = 111320 * Math.cos(lat * Math.PI / 180);
  for (let dx = -75; dx <= 75; dx += 15) for (let dy = -75; dy <= 75; dy += 15) { const e = elev(lon + dx / mLon, lat + dy / mLat); if (Number.isFinite(e) && e > mx) mx = e; }
  corr.push(h - mx);
}
const q = (a: number[], p: number) => { const s = [...a].sort((x, y) => x - y); return s[Math.floor(p * (s.length - 1))]; };
const st = (a: number[]) => `lowest ${q(a, 0).toFixed(1)}  5 % ${q(a, 0.05).toFixed(1)}  median ${q(a, 0.5).toFixed(1)}  highest ${q(a, 1).toFixed(1)} m`;
console.log(`${pt.length} samples; track - GLO-30 at the point: ${st(pt)}; track - highest GLO-30 within +-75 m: ${st(corr)}`);
