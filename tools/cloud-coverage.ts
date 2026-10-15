// Point density and coverage of a processed L3 cloud over the block it was flown for, for comparing two missions
// of the same block (e.g. FlightHub 2 vs the planner, 15 Oct 2026).
//   node tools/cloud-coverage.ts <aoi.kmz|kml> [--utm 34S | --geo lidar_geo_desc.json] [--cell 5] [--edge 20]
//                               [--every 1] <cloud.las> [cloud2.las ...]
// The AOI is the first polygon in the KML. Points are streamed (any number of LAS files, one cloud) and binned into
// square cells in a local metre grid; reported per cell inside the AOI: density (points/m²) distribution, empty and
// thin cells (< 25 % of the median), and the same for the outer ring (--edge m inside the AOI boundary), where a
// mission that places the swath edge on the block edge thins out first. Also how far the cloud reaches beyond the
// AOI. Coordinates: geographic (default), a UTM zone, or DJI Terra's local ENU (lidar_geo_desc.json). No coordinates
// are printed.
import { createReadStream, readFileSync, openSync, readSync, closeSync } from 'node:fs';
import JSZip from 'jszip';

const args = process.argv.slice(2);
const flag = (name: string, d?: string) => { const i = args.indexOf('--' + name); return i >= 0 ? args.splice(i, 2)[1] : d; };
const utm = flag('utm'), geo = flag('geo'), cellM = +flag('cell', '5')!, edgeM = +flag('edge', '20')!, every = +flag('every', '1')!;
const [aoiPath, ...lasPaths] = args;
if (!aoiPath || !lasPaths.length) { console.error('usage: node tools/cloud-coverage.ts <aoi.kmz|kml> [--utm 34S | --geo lidar_geo_desc.json] [--cell 5] [--edge 20] [--every 1] <cloud.las> [...]'); process.exit(2); }

// ---- AOI polygon (lon, lat)
let kml: string;
if (/\.kmz$/i.test(aoiPath)) {
  const zip = await JSZip.loadAsync(readFileSync(aoiPath));
  const name = Object.keys(zip.files).find(f => /\.kml$/i.test(f))!;
  kml = await zip.files[name].async('string');
} else kml = readFileSync(aoiPath, 'utf8');
const m = kml.replace(/\s+/g, ' ').match(/<Polygon>.*?<coordinates>\s*([^<]+?)\s*<\/coordinates>/);
if (!m) { console.error('no <Polygon> in the KML'); process.exit(2); }
const poly = m[1].trim().split(/\s+/).map(t => t.split(',').map(Number)).map(([lon, lat]) => [lon, lat] as [number, number]);
if (poly.length > 1 && poly[0][0] === poly[poly.length - 1][0] && poly[0][1] === poly[poly.length - 1][1]) poly.pop();
const lat0 = poly.reduce((s, p) => s + p[1], 0) / poly.length, lon0 = poly.reduce((s, p) => s + p[0], 0) / poly.length;
const mLat = 110900, mLon = 111320 * Math.cos(lat0 * Math.PI / 180);
const P = poly.map(([lon, lat]) => [(lon - lon0) * mLon, (lat - lat0) * mLat] as [number, number]);   // metres about the centroid
const inside = (x: number, y: number) => {
  let c = false;
  for (let i = 0, j = P.length - 1; i < P.length; j = i++) {
    const [xi, yi] = P[i], [xj, yj] = P[j];
    if ((yi > y) !== (yj > y) && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) c = !c;
  }
  return c;
};
const edgeDist = (x: number, y: number) => {   // distance to the polygon boundary
  let best = Infinity;
  for (let i = 0, j = P.length - 1; i < P.length; j = i++) {
    const [ax, ay] = P[j], [bx, by] = P[i], dx = bx - ax, dy = by - ay, L2 = dx * dx + dy * dy;
    const f = L2 ? Math.max(0, Math.min(1, ((x - ax) * dx + (y - ay) * dy) / L2)) : 0;
    best = Math.min(best, Math.hypot(x - ax - f * dx, y - ay - f * dy));
  }
  return best;
};
const areaHa = Math.abs(P.reduce((s, [x, y], i) => { const [x2, y2] = P[(i + 1) % P.length]; return s + x * y2 - x2 * y; }, 0)) / 2 / 1e4;

// ---- coordinate frames
let enu: { lat: number; lon: number; alt: number } | null = null;
if (geo) {
  const g = JSON.parse(readFileSync(geo, 'utf8'));
  if (g.cs_type !== 'LOCAL_ENU_CS') { console.error(`cs_type ${g.cs_type} not handled`); process.exit(2); }
  enu = { lat: g.ref_GPS.latitude, lon: g.ref_GPS.longitude, alt: g.ref_GPS.altitude };
}
function utmToLatLon(e: number, n: number, zone: number, south: boolean): [number, number] {
  const a = 6378137, f = 1 / 298.257223563, k0 = 0.9996, e2 = f * (2 - f), ep2 = e2 / (1 - e2);
  const x = e - 500000, y = south ? n - 10000000 : n;
  const M = y / k0, mu = M / (a * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 ** 3 / 256));
  const e1 = (1 - Math.sqrt(1 - e2)) / (1 + Math.sqrt(1 - e2));
  const phi1 = mu + (3 * e1 / 2 - 27 * e1 ** 3 / 32) * Math.sin(2 * mu) + (21 * e1 * e1 / 16 - 55 * e1 ** 4 / 32) * Math.sin(4 * mu) + (151 * e1 ** 3 / 96) * Math.sin(6 * mu);
  const N1 = a / Math.sqrt(1 - e2 * Math.sin(phi1) ** 2), T1 = Math.tan(phi1) ** 2, C1 = ep2 * Math.cos(phi1) ** 2;
  const R1 = a * (1 - e2) / (1 - e2 * Math.sin(phi1) ** 2) ** 1.5, D = x / (N1 * k0);
  const lat = phi1 - (N1 * Math.tan(phi1) / R1) * (D * D / 2 - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * ep2) * D ** 4 / 24 + (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * ep2 - 3 * C1 * C1) * D ** 6 / 720);
  const lon0z = ((zone - 1) * 6 - 180 + 3) * Math.PI / 180;
  const lon = lon0z + (D - (1 + 2 * T1 + C1) * D ** 3 / 6 + (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * ep2 + 24 * T1 * T1) * D ** 5 / 120) / Math.cos(phi1);
  return [lat * 180 / Math.PI, lon * 180 / Math.PI];
}
const zone = utm ? parseInt(utm) : 0, south = utm ? /S$/i.test(utm) : false;

// ---- stream all LAS files into one cell grid
const counts = new Map<number, number>();   // key = (ix + 1e5) * 2e5 + (iy + 1e5)
const key = (ix: number, iy: number) => (ix + 100000) * 200000 + (iy + 100000);
let total = 0, used = 0, minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
for (const lasPath of lasPaths) {
  const head = Buffer.alloc(375);
  { const h = openSync(lasPath, 'r'); readSync(h, head, 0, 375, 0); closeSync(h); }
  if (head.toString('latin1', 0, 4) !== 'LASF') { console.error(`${lasPath}: not a LAS file`); process.exit(1); }
  const verMin = head[25], offPts = head.readUInt32LE(96), recLen = head.readUInt16LE(105);
  let nPts = head.readUInt32LE(107); if (nPts === 0 && verMin >= 4) nPts = Number(head.readBigUInt64LE(247));
  const sx = head.readDoubleLE(131), sy = head.readDoubleLE(139), ox = head.readDoubleLE(155), oy = head.readDoubleLE(163);
  let seen = 0, rem = Buffer.alloc(0);
  await new Promise<void>((res, rej) => {
    const s = createReadStream(lasPath, { start: offPts, highWaterMark: 8 << 20 });
    s.on('data', (chunk: Buffer) => {
      const buf = rem.length ? Buffer.concat([rem, chunk]) : chunk;
      const full = Math.floor(buf.length / recLen) * recLen;
      for (let p = 0; p < full; p += recLen) {
        seen++;
        if (every > 1 && seen % every) continue;
        const X = buf.readInt32LE(p) * sx + ox, Y = buf.readInt32LE(p + 4) * sy + oy;
        let x: number, y: number;
        if (enu) { x = X + (enu.lon - lon0) * mLon; y = Y + (enu.lat - lat0) * mLat; }
        else if (utm) { const [lat, lon] = utmToLatLon(X, Y, zone, south); x = (lon - lon0) * mLon; y = (lat - lat0) * mLat; }
        else { x = (X - lon0) * mLon; y = (Y - lat0) * mLat; }
        const ix = Math.floor(x / cellM), iy = Math.floor(y / cellM), k = key(ix, iy);
        counts.set(k, (counts.get(k) ?? 0) + 1);
        if (x < minX) minX = x; if (x > maxX) maxX = x; if (y < minY) minY = y; if (y > maxY) maxY = y;
        used++;
      }
      rem = Buffer.from(buf.subarray(full));
    });
    s.on('end', res); s.on('error', rej);
  });
  total += seen;
  console.log(`${lasPath.replace(/\\/g, '/').split('/').pop()}: ${nPts.toLocaleString()} points`);
}
console.log(`${total.toLocaleString()} points read, ${used.toLocaleString()} used (every ${every}), cells ${cellM} m; coordinates ${enu ? 'local ENU (Terra)' : utm ? 'UTM ' + utm : 'geographic'}`);

// ---- per-cell density inside the AOI (every AOI cell, including empty ones)
const cellArea = cellM * cellM, scale = every / cellArea;   // points per m² from a cell count
const xs = P.map(p => p[0]), ys = P.map(p => p[1]);
const interior: number[] = [], ring: number[] = [];
let aoiCells = 0, emptyIn = 0, emptyRing = 0;
for (let ix = Math.floor(Math.min(...xs) / cellM); ix <= Math.floor(Math.max(...xs) / cellM); ix++) {
  for (let iy = Math.floor(Math.min(...ys) / cellM); iy <= Math.floor(Math.max(...ys) / cellM); iy++) {
    const cx = (ix + 0.5) * cellM, cy = (iy + 0.5) * cellM;
    if (!inside(cx, cy)) continue;
    aoiCells++;
    const d = (counts.get(key(ix, iy)) ?? 0) * scale;
    if (edgeDist(cx, cy) <= edgeM) { ring.push(d); if (d === 0) emptyRing++; } else { interior.push(d); if (d === 0) emptyIn++; }
  }
}
const q = (a: number[], p: number) => { const s = [...a].sort((x, y) => x - y); return s.length ? s[Math.min(s.length - 1, Math.floor(p * s.length))] : NaN; };
const mean = (a: number[]) => a.reduce((x, y) => x + y, 0) / (a.length || 1);
const medAll = q([...interior, ...ring], 0.5);
const thin = (a: number[]) => a.filter(d => d < 0.25 * medAll).length;
const line = (name: string, a: number[], empty: number) =>
  console.log(`  ${name.padEnd(22)} ${a.length.toString().padStart(7)} cells  mean ${mean(a).toFixed(1).padStart(6)}  5 % ${q(a, 0.05).toFixed(1).padStart(6)}  median ${q(a, 0.5).toFixed(1).padStart(6)}  95 % ${q(a, 0.95).toFixed(1).padStart(6)} pts/m²   empty ${(100 * empty / (a.length || 1)).toFixed(2)} %   thin (<25 % of median) ${(100 * thin(a) / (a.length || 1)).toFixed(2)} %`);
console.log(`\nAOI ${areaHa.toFixed(1)} ha, ${aoiCells.toLocaleString()} cells of ${cellM} m; points inside AOI ≈ ${(([...interior, ...ring].reduce((s, d) => s + d, 0)) * cellArea / 1e6).toFixed(1)} M`);
line('whole AOI', [...interior, ...ring], emptyIn + emptyRing);
line(`interior (>${edgeM} m in)`, interior, emptyIn);
line(`edge ring (≤${edgeM} m)`, ring, emptyRing);

// ---- reach beyond the AOI: how far outside the boundary points exist (cells with ≥ 1 pt/m²)
let reach = 0, outsideCells = 0;
for (const [k, n] of counts) {
  if (n * scale < 1) continue;
  const ix = Math.floor(k / 200000) - 100000, iy = (k % 200000) - 100000, cx = (ix + 0.5) * cellM, cy = (iy + 0.5) * cellM;
  if (inside(cx, cy)) continue;
  outsideCells++; const d = edgeDist(cx, cy); if (d > reach) reach = d;
}
console.log(`outside the AOI: ${(outsideCells * cellArea / 1e4).toFixed(1)} ha with ≥ 1 pt/m², reaching ${reach.toFixed(0)} m beyond the boundary; cloud extent ${((maxX - minX) / 1000).toFixed(2)} × ${((maxY - minY) / 1000).toFixed(2)} km`);
