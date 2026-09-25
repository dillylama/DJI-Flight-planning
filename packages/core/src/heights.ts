import { type XY, add, dist, mul } from './geo.ts';
import type { Plan } from './plan.ts';
import type { Route, RouteWp } from './route.ts';

// Orthometric terrain height at a point; null/NaN where the DEM has no data.
export type ElevFn = (lon: number, lat: number) => number | null;

export interface FlightWp extends RouteWp {
  lon: number;
  lat: number;
  terrainMax: number;      // max terrain (+corridor) along both adjacent legs
  h: number;               // orthometric flight height
  hWrite: number;          // h + geoidN — what goes in the file if the header is ellipsoidal
  terrainUnderWp: number;
}

// Absolute heights. Each waypoint gets ≥ max terrain (+ corridor) along BOTH adjacent legs + AGL,
// so every straight leg clears terrain by at least aglM; then climb/descent is gradient-limited
// by raising (never lowering) waypoints.
export function applyHeights(plan: Plan, route: Route, elev: ElevFn): Route<FlightWp> {
  const { o, proj } = plan;
  const { wps } = route;
  const terr = (xy: XY): number => {
    const [lo, la] = proj.inv(xy[0], xy[1]);
    const h = elev(lo, la);
    return h != null && Number.isFinite(h) ? h : -Infinity;
  };
  const segMax = (a: XY, b: XY): number => {
    const L = dist(a, b), n = Math.max(1, Math.ceil(L / o.demSampleM));
    const dir: XY = L > 0 ? mul([b[0] - a[0], b[1] - a[1]], 1 / L) : [1, 0];
    const nrm: XY = [dir[1], -dir[0]];
    let m = -Infinity;
    for (let k = 0; k <= n; k++) {
      const p = add(a, mul([b[0] - a[0], b[1] - a[1]], k / n));
      for (const off of [-o.corridorM, 0, o.corridorM]) m = Math.max(m, terr(add(p, mul(nrm, off))));
    }
    return m;
  };

  const legMax: number[] = [];
  for (let i = 0; i < wps.length - 1; i++) legMax.push(segMax(wps[i].xy, wps[i + 1].xy));

  const tMax = wps.map((_, i) => {
    const t = Math.max(i > 0 ? legMax[i - 1] : -Infinity, i < legMax.length ? legMax[i] : -Infinity);
    if (!Number.isFinite(t)) throw new Error('DEM has no data under waypoint ' + i);
    return t;
  });
  const h = tMax.map(t => t + o.aglM);

  // Gradient limit: backward pass (climb early), forward pass (descend late).
  for (let i = wps.length - 2; i >= 0; i--) h[i] = Math.max(h[i], h[i + 1] - o.maxGradient * dist(wps[i].xy, wps[i + 1].xy));
  for (let i = 1; i < wps.length; i++) h[i] = Math.max(h[i], h[i - 1] - o.maxGradient * dist(wps[i].xy, wps[i - 1].xy));

  const out: FlightWp[] = wps.map((w, i) => {
    const [lon, lat] = proj.inv(w.xy[0], w.xy[1]);
    return { ...w, lon, lat, terrainMax: tMax[i], h: h[i], hWrite: h[i] + o.geoidN, terrainUnderWp: terr(w.xy) };
  });
  return { ...route, wps: out };
}
