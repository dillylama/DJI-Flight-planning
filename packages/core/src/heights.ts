import { type XY, add, dist, mul } from './geo.ts';
import type { Plan } from './plan.ts';
import { legSpeed, type Route, type RouteWp } from './route.ts';

// Orthometric terrain height at a point; null/NaN where the DEM has no data.
export type ElevFn = (lon: number, lat: number) => number | null;

export interface FlightWp extends RouteWp {
  lon: number;
  lat: number;
  terrainMax: number;      // max terrain (+corridor) along both adjacent legs
  h: number;               // orthometric flight height
  hWrite: number;          // h + geoidN — what goes in the file if the header is ellipsoidal
  terrainUnderWp: number;
  slowed?: boolean;        // the leg this waypoint's speed applies to was slowed for the climb/descent limit
}

// Absolute heights. Each waypoint gets ≥ max terrain (+ corridor) along BOTH adjacent legs + AGL, so
// every straight leg clears terrain by at least aglM. Then the vertical rate is handled one of two ways:
//   'slow'  (default, UgCS-style): heights follow the terrain; on any leg where the climb or descent
//           rate at line speed would exceed climbMs / descentMs, the leg's speed is reduced so the
//           vertical rate is exactly at the limit. AGL stays even; the aircraft slows on steep ground.
//   'raise': keep line speed everywhere and limit rise/run to maxGradient by raising waypoints (never
//           lowering), so lines crossing steep terrain climb early and end up high above the low ground.
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
    // Across-track samples every ≤ demSampleM out to ±corridorM, so a narrow ridge inside the corridor is not missed.
    const nA = Math.max(1, Math.ceil(o.corridorM / o.demSampleM));
    const offs = Array.from({ length: 2 * nA + 1 }, (_, j) => ((j - nA) * o.corridorM) / nA);
    let m = -Infinity;
    for (let k = 0; k <= n; k++) {
      const p = add(a, mul([b[0] - a[0], b[1] - a[1]], k / n));
      for (const off of offs) m = Math.max(m, terr(add(p, mul(nrm, off))));
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
  const speed = wps.map(w => w.speed);
  const slowed = wps.map(() => false);

  if (o.verticalMode === 'raise') {
    // Gradient limit: backward pass (climb early), forward pass (descend late).
    for (let i = wps.length - 2; i >= 0; i--) h[i] = Math.max(h[i], h[i + 1] - o.maxGradient * dist(wps[i].xy, wps[i + 1].xy));
    for (let i = 1; i < wps.length; i++) h[i] = Math.max(h[i], h[i - 1] - o.maxGradient * dist(wps[i].xy, wps[i - 1].xy));
  } else {
    // Speed limit per leg: vertical rate = |Δh| / d × v must not exceed the climb / descent limit.
    for (let i = 0; i < wps.length - 1; i++) {
      const d = dist(wps[i].xy, wps[i + 1].xy), dh = h[i + 1] - h[i];
      if (d < 1e-6 || dh === 0) continue;
      const limit = dh > 0 ? o.climbMs : o.descentMs;
      const vLeg = legSpeed(wps, i);
      const vz = Math.abs(dh) / d * vLeg;
      if (vz > limit) {
        const v = Math.max(o.minLegSpeedMs, limit * d / Math.abs(dh));
        // WPML: waypoint i's speed is flown from i to i+1. DJI also ramps to the next waypoint's speed across
        // the leg, so cap both ends: the steep leg then never exceeds the limit under either reading.
        speed[i] = Math.min(speed[i], v);
        speed[i + 1] = Math.min(speed[i + 1], v);
        slowed[i] = true;
      }
    }
  }

  const out: FlightWp[] = wps.map((w, i) => {
    const [lon, lat] = proj.inv(w.xy[0], w.xy[1]);
    return { ...w, speed: speed[i], slowed: slowed[i] || undefined, lon, lat, terrainMax: tMax[i], h: h[i], hWrite: h[i] + o.geoidN, terrainUnderWp: terr(w.xy) };
  });
  return { ...route, wps: out };
}

// Time to fly a route at its per-leg speeds (seconds), horizontal legs only.
export function routeTimeS(wps: RouteWp[]): number {
  let t = 0;
  for (let i = 0; i < wps.length - 1; i++) t += dist(wps[i].xy, wps[i + 1].xy) / legSpeed(wps, i);
  return t;
}
