import { type Projection, type XY, add, dist, mul } from './geo.ts';

// Orthometric terrain height (metres) at a point; null where there is no data.
//   upper   highest DEM cell around the point: never below the bilinear value. Clearance maths uses this,
//           so a peak that sits between sample points still counts.
//   cell    DEM cell size in metres [east–west, north–south]; sets how densely corridors must be sampled
//           and how far past the corridor edge they are searched.
export interface ElevFn {
  (lon: number, lat: number): number | null;
  upper?: (lon: number, lat: number) => number | null;
  cell?: [number, number];
}

// Thrown when a route, corridor, transit or RTH line touches terrain with no data. Never ignored:
// a gap in the DEM is unknown ground, not low ground.
export class TerrainGapError extends Error {
  readonly lon: number;
  readonly lat: number;
  constructor(lon: number, lat: number) {
    super(`The DEM has no data at ${lat.toFixed(5)}, ${lon.toFixed(5)}`);
    this.name = 'TerrainGapError';
    this.lon = lon; this.lat = lat;
  }
}

// Sample spacing that cannot step over a DEM cell: a lattice this fine puts at least one sample in every
// disc of radius 0.5 × the smaller cell side, and `upper` at a sample returns its cell's highest corner.
export const sampleStep = (elev: ElevFn, maxStep: number) => (elev.cell ? Math.min(maxStep, 0.7 * Math.min(...elev.cell)) : maxStep);

// How far beyond a corridor the sampling reaches, so that EVERY cell touching the corridor holds a sample:
// half the cell diagonal (corridor edge to cell centre) + half the smaller side (room for one lattice point).
// The planner therefore clears the highest DEM cell that touches a corridor, not just the cells inside it.
export const cellPad = (elev: ElevFn) => (elev.cell ? 0.5 * Math.hypot(...elev.cell) + 0.5 * Math.min(...elev.cell) : 0);

export interface Station { s: number; p: XY; max: number }

// Terrain profile along a→b: at each station, the highest terrain across the corridor (±half, plus the
// cell padding). Stations include both ends; across-track offsets include the centre line and both edges.
export function corridorProfile(elev: ElevFn, proj: Projection, a: XY, b: XY, half: number, maxStep: number, s0 = 0, s1 = dist(a, b)): Station[] {
  const up = elev.upper ?? elev;
  const step = sampleStep(elev, maxStep);
  const L = dist(a, b);
  const dir: XY = L > 1e-9 ? mul([b[0] - a[0], b[1] - a[1]], 1 / L) : [1, 0];
  const nrm: XY = [dir[1], -dir[0]];
  half += cellPad(elev);
  const nA = half > 0 ? Math.max(1, Math.ceil(half / step)) : 0;
  const n = Math.max(1, Math.ceil(Math.abs(s1 - s0) / step));
  const out: Station[] = [];
  for (let k = 0; k <= n; k++) {
    const s = s0 + ((s1 - s0) * k) / n;
    const p = add(a, mul(dir, s));
    let m = -Infinity;
    for (let j = -nA; j <= nA; j++) {
      const q = nA ? add(p, mul(nrm, (j * half) / nA)) : p;
      const [lon, lat] = proj.inv(q[0], q[1]);
      const h = up(lon, lat);
      if (h == null || !Number.isFinite(h)) throw new TerrainGapError(lon, lat);
      if (h > m) m = h;
    }
    out.push({ s, p, max: m });
    if (s1 === s0) break;
  }
  return out;
}

// Highest terrain in the rectangle around a→b: ±half across-track, extended `ext` beyond both ends, so the
// outside of a corner and the ground just past a waypoint are covered too.
export function corridorMax(elev: ElevFn, proj: Projection, a: XY, b: XY, half: number, ext: number, maxStep: number): number {
  const L = dist(a, b);
  const e = ext > 0 ? ext + cellPad(elev) : cellPad(elev);
  let m = -Infinity;
  const take = (st: Station[]) => { for (const x of st) if (x.max > m) m = x.max; };
  take(corridorProfile(elev, proj, a, b, half, maxStep, 0, L));
  if (e > 0) {
    take(corridorProfile(elev, proj, a, b, half, maxStep, -e, 0));
    take(corridorProfile(elev, proj, a, b, half, maxStep, L, L + e));
  }
  return m;
}

// Terrain at a point, strict: a gap is an error.
export function terrainAt(elev: ElevFn, proj: Projection, p: XY): number {
  const [lon, lat] = proj.inv(p[0], p[1]);
  const h = elev(lon, lat);
  if (h == null || !Number.isFinite(h)) throw new TerrainGapError(lon, lat);
  return h;
}
