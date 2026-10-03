import { dist } from './geo.ts';
import type { Plan } from './plan.ts';
import { legSpeed, type Route, type RouteWp } from './route.ts';
import { SAFETY } from './safety.ts';
import { corridorMax, terrainAt, type ElevFn } from './terrain.ts';

export interface FlightWp extends RouteWp {
  lon: number;
  lat: number;
  terrainMax: number;      // highest terrain in the corridors of both adjacent legs
  h: number;               // flight height, orthometric (EGM96, the DEM's datum)
  terrainUnderWp: number;
  slowed?: boolean;        // this waypoint's speed was lowered for the climb/descent limit
  roundingM: number;       // how far the rounded flight path may pass below the straight legs at this waypoint
  lagM: number;            // height the aircraft may lose here while it steepens its climb (pull-up)
  turnM: number;           // how far the rounded turn may swing sideways off the legs at this waypoint
}

// The smallest turn damping we write: DJI needs a value above zero on fly-through waypoints.
export const MIN_DAMPING_M = 1;

// Absolute heights.
//
// 1. Terrain envelope. Every leg's corridor (±corridorM across-track, extended beyond both ends by the
//    corridor width or the stopping distance at route speed, whichever is longer, sampled at least once
//    per DEM cell with the highest cell value) gives the leg's highest terrain. A waypoint is set to the
//    higher of its two legs + AGL, so every straight leg clears the terrain in its whole corridor by at
//    least aglM. A gap in the DEM anywhere in a corridor is an error, never skipped.
// 2. Alignment manoeuvres are flown level: approach + figure-8 + first run-in share one height, and so do
//    the last run-out + figure-8 + exit.
// 3. Vertical rate, one of two ways:
//      'slow'  (default): heights follow the terrain; any leg whose climb or descent at line speed would
//              exceed climbMs / descentMs is flown slower, and so are the legs around it, so the aircraft
//              is never faster than the leg allows whichever way it changes speed between waypoints.
//      'raise': keep line speed and cap rise/run at maxGradient by raising waypoints (never lowering).
// 4. Path rounding. The aircraft rounds each waypoint over the turn damping distance. Damping is shortened
//    wherever the rounding would exceed roundingMaxM vertically, or leave the terrain corridor sideways.
// 5. Pull-up lag. Where the flight path steepens, the height the aircraft could lose if it only starts to
//    change its vertical speed at the waypoint is recorded, and counted in the clearance budget.
export function applyHeights(plan: Plan, route: Route, elev: ElevFn): Route<FlightWp> {
  const { o, proj } = plan;
  const { wps } = route;
  const n = wps.length;
  const D: number[] = [];
  for (let i = 0; i < n - 1; i++) D.push(dist(wps[i].xy, wps[i + 1].xy));

  const vTop = Math.max(...wps.map(w => w.speed));
  const ext = Math.max(o.corridorM, (vTop * vTop) / (2 * SAFETY.brakeMs2));
  const legMax: number[] = [];
  for (let i = 0; i < n - 1; i++) legMax.push(corridorMax(elev, proj, wps[i].xy, wps[i + 1].xy, o.corridorM, ext, o.demSampleM));

  const tMax = wps.map((_, i) => Math.max(i > 0 ? legMax[i - 1] : -Infinity, i < legMax.length ? legMax[i] : -Infinity));
  const h = tMax.map(t => t + o.aglM);

  // Level alignment groups (only ever raising). Returns how much it changed.
  const levelRange = (from: number, to: number): number => {
    if (to <= from) return 0;
    let m = -Infinity, change = 0;
    for (let i = from; i <= to; i++) m = Math.max(m, h[i]);
    for (let i = from; i <= to; i++) { change = Math.max(change, m - h[i]); h[i] = m; }
    return change;
  };
  let a = 0;
  while (a < n && (wps[a].role === 'approach' || wps[a].role === 'fig8')) a++;
  let b = n - 1;
  while (b >= 0 && (wps[b].role === 'exit' || wps[b].role === 'fig8')) b--;
  const level = () =>
    Math.max(a > 0 ? levelRange(0, a < n && wps[a].role === 'runin' ? a : a - 1) : 0,
      b < n - 1 ? levelRange(b >= 0 && wps[b].role === 'runout' ? b : b + 1, n - 1) : 0);
  level();

  const speed = wps.map(w => w.speed);
  const slowed = wps.map(() => false);

  if (o.verticalMode === 'raise') {
    // Gradient limit: backward pass (climb early), forward pass (descend late); then re-level the
    // alignment groups and repeat until nothing moves (heights only rise, so this settles quickly).
    for (let pass = 0; pass < 8; pass++) {
      for (let i = n - 2; i >= 0; i--) h[i] = Math.max(h[i], h[i + 1] - o.maxGradient * D[i]);
      for (let i = 1; i < n; i++) h[i] = Math.max(h[i], h[i - 1] - o.maxGradient * D[i - 1]);
      if (level() < 1e-9) break;
    }
  } else {
    // Fastest each leg may be flown: vertical rate = |Δh| / d × v must stay within the climb / descent limit.
    const lim = D.map((d, i) => {
      const dh = h[i + 1] - h[i];
      if (dh === 0) return Infinity;
      return d < 1e-6 ? 0 : ((dh > 0 ? o.climbMs : o.descentMs) * d) / Math.abs(dh);
    });
    // Fastest the aircraft may be going when it STARTS leg i: within that leg's limit, and slow enough to
    // brake to the next leg's entry speed before it gets there.
    const entry: number[] = new Array(n).fill(Infinity);
    for (let i = n - 2; i >= 0; i--) entry[i] = Math.min(lim[i], Math.sqrt(entry[i + 1] ** 2 + 2 * SAFETY.brakeMs2 * D[i]));
    // WPML: waypoint i's speed is the target from i to i+1. Whether the aircraft changes speed after passing
    // the waypoint or ramps towards the next one, it is never faster than a leg allows if each target is
    // within the limit of its own leg, of the leg before it, and of the entry speed of the leg after it.
    for (let i = 0; i < n; i++) {
      const cap = Math.min(i < n - 1 ? lim[i] : Infinity, i > 0 ? lim[i - 1] : Infinity, i < n - 1 ? entry[i + 1] : Infinity);
      if (cap < speed[i]) { speed[i] = Math.max(o.minLegSpeedMs, cap); slowed[i] = true; }
    }
  }

  // What the aircraft does at each interior waypoint.
  const damping = wps.map(w => w.dampingM);
  const rounding = wps.map(() => 0), turn = wps.map(() => 0), lag = wps.map(() => 0);
  const grad = D.map((d, i) => (d < 1e-6 ? 0 : (h[i + 1] - h[i]) / d));
  // Highest / lowest speed the aircraft can have on leg i: its own target, or one it still carries from a neighbour.
  const vHi = (i: number) => Math.max(speed[i], speed[i + 1], i > 0 ? speed[i - 1] : 0);
  const vLo = (i: number) => Math.min(speed[i], speed[i + 1], i > 0 ? speed[i - 1] : Infinity);
  for (let i = 1; i < n - 1; i++) {
    if (D[i - 1] < 1e-6 || D[i] < 1e-6) continue;
    // Sideways: with turn damping d and a heading change θ the path stays within d of the waypoint, and
    // within d·tan(θ/2) of the legs for gentle turns. Keep that inside the terrain corridor.
    const ax = wps[i].xy[0] - wps[i - 1].xy[0], ay = wps[i].xy[1] - wps[i - 1].xy[1];
    const bx = wps[i + 1].xy[0] - wps[i].xy[0], by = wps[i + 1].xy[1] - wps[i].xy[1];
    const kh = Math.min(1, Math.tan(Math.abs(Math.atan2(ax * by - ay * bx, ax * bx + ay * by)) / 2));
    if (kh > 1e-9) damping[i] = Math.min(damping[i], Math.max(MIN_DAMPING_M, (SAFETY.turnInsideCorridor * o.corridorM) / kh));
    // Vertically: with a change Δγ in flight-path angle a tangent arc passes d·tan(Δγ/4) inside the corner;
    // d·tan(Δγ/2) is a safe upper bound for whichever smoothing the aircraft applies.
    const kv = Math.tan(Math.abs(Math.atan(grad[i]) - Math.atan(grad[i - 1])) / 2);
    if (kv > 1e-9) damping[i] = Math.min(damping[i], Math.max(MIN_DAMPING_M, o.roundingMaxM / kv));   // only ever shortened
    turn[i] = damping[i] * kh;
    rounding[i] = damping[i] * kv;
    // Pull-up: the vertical speed has to rise by Δvz here. If the aircraft only starts at the waypoint and
    // holds vertAccelMs2, it drops Δvz² / 2a below the new leg before it is back on it.
    const vzOut = grad[i] * (grad[i] > 0 ? vHi(i) : vLo(i));
    const vzIn = grad[i - 1] * (grad[i - 1] < 0 ? vHi(i - 1) : vLo(i - 1));
    if (vzOut > vzIn) lag[i] = (vzOut - vzIn) ** 2 / (2 * SAFETY.vertAccelMs2);
  }

  const out: FlightWp[] = wps.map((w, i) => {
    const [lon, lat] = proj.inv(w.xy[0], w.xy[1]);
    return {
      ...w, speed: speed[i], dampingM: damping[i], slowed: slowed[i] || undefined, lon, lat,
      terrainMax: tMax[i], h: h[i], terrainUnderWp: terrainAt(elev, proj, w.xy),
      roundingM: rounding[i], lagM: lag[i], turnM: turn[i],
    };
  });
  return { ...route, wps: out };
}

// Time to fly a route at its per-leg speeds (seconds), horizontal legs only.
export function routeTimeS(wps: RouteWp[]): number {
  let t = 0;
  for (let i = 0; i < wps.length - 1; i++) t += dist(wps[i].xy, wps[i + 1].xy) / legSpeed(wps, i);
  return t;
}
