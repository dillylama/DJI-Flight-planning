import { dist } from './geo.ts';
import type { Plan } from './plan.ts';
import { isStop, legSpeed, type Route, type RouteWp } from './route.ts';
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
  const h = tMax.map((t, i) => t + Math.max(o.aglM, wps[i].aglM ?? 0));

  // Level alignment groups (only ever raising). Returns how much it changed.
  const levelRange = (from: number, to: number): number => {
    if (to <= from) return 0;
    let m = -Infinity, change = 0;
    for (let i = from; i <= to; i++) m = Math.max(m, h[i]);
    for (let i = from; i <= to; i++) { change = Math.max(change, m - h[i]); h[i] = m; }
    return change;
  };
  let s0 = 0;                                   // first and last waypoint that are not transit
  while (s0 < n && wps[s0].role === 'transit') s0++;
  let e0 = n - 1;
  while (e0 >= 0 && wps[e0].role === 'transit') e0--;
  let a = s0;
  while (a < n && (wps[a].role === 'approach' || wps[a].role === 'fig8')) a++;
  let b = e0;
  while (b >= 0 && (wps[b].role === 'exit' || wps[b].role === 'fig8')) b--;
  const level = () =>
    Math.max(a > s0 ? levelRange(s0, a < n && wps[a].role === 'runin' ? a : a - 1) : 0,
      b < e0 ? levelRange(b >= 0 && wps[b].role === 'runout' ? b : b + 1, e0) : 0);
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
    if (D[i - 1] < 1e-6 || D[i] < 1e-6 || isStop(wps[i])) continue;   // the aircraft stops at a stop turn: nothing is rounded
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

// Waypoint thinning.
//
// A waypoint every 50–150 m along a line carries no information over flat ground: the terrain-following
// heights of neighbours are the same, and Pilot 2 slows down (and above ~2,000 waypoints crashes) on big
// files. Thinning keeps, on each line, only the waypoints needed for the straight legs between them to stay
// within `thinToleranceM` ABOVE the terrain-following height at every waypoint that was dropped. It never
// lowers anything: a longer leg is still held above the highest ground in its whole corridor by applyHeights,
// exactly like a short one, so the clearance guarantee is unchanged and the exported file is verified as
// before. The tolerance is only how much extra height (and fewer points on the ground) is accepted.
//
// Kept whatever the tolerance: every waypoint that is not a plain line waypoint (run-ins, run-outs, corners,
// approach, exit, transit, figure-8), every waypoint carrying an action, every stop turn, the first and last.
//
// Method: with the dense route's heights as the reference, split each run of droppable waypoints at the one
// where a straight leg over the run would sit highest above the reference, until every run is within the
// tolerance (an estimate); then build the thinned route for real and put back, one per leg, any dropped
// waypoint the real heights still exceed the tolerance at, until none is left.
export function thinRoute(plan: Plan, route: Route, dense: Route<FlightWp>, elev: ElevFn): Route<FlightWp> {
  const tol = plan.o.thinToleranceM;
  const { wps } = route;
  const n = wps.length;
  const before = n;
  if (!(tol > 0) || n < 3) return { ...dense, thinned: { before, after: n } };
  const r = dense.wps.map(w => w.h);
  const cum = [0];
  for (let i = 1; i < n; i++) cum.push(cum[i - 1] + dist(wps[i - 1].xy, wps[i].xy));
  const keep = wps.map((w, i) => i === 0 || i === n - 1 || w.role !== 'line' || w.actions.length > 0 || isStop(w) || w.aglM != null);

  // Estimate pass: the leg a→b will sit at least at the highest reference in [a, b]; the worst excess is at
  // the lowest reference between them. A peak inside the run that stands more than the tolerance above both
  // ends is kept first, so the legs on either side of it can come back down instead of carrying its height.
  const split = (a: number, b: number): void => {
    if (b - a < 2) return;
    let hi = -Infinity, hiAt = -1, lo = Infinity, loAt = -1;
    for (let i = a + 1; i < b; i++) {
      if (r[i] > hi) { hi = r[i]; hiAt = i; }
      if (r[i] < lo) { lo = r[i]; loAt = i; }
    }
    const top = Math.max(hi, r[a], r[b]);
    if (top - lo <= tol) return;
    const at = hi > Math.max(r[a], r[b]) + tol ? hiAt : loAt;
    keep[at] = true;
    split(a, at); split(at, b);
  };
  let a0 = 0;
  for (let i = 1; i < n; i++) if (keep[i]) { split(a0, i); a0 = i; }

  // Real pass: build, measure against the reference, put back the worst offender per leg, repeat.
  let flight: Route<FlightWp> = dense;
  for (let iter = 0; iter < 30; iter++) {
    const idx = wps.map((_, i) => i).filter(i => keep[i]);
    flight = applyHeights(plan, { ...route, wps: idx.map(i => wps[i]) }, elev);
    let added = 0;
    for (let k = 0; k < idx.length - 1; k++) {
      const a = idx[k], b = idx[k + 1];
      const ha = flight.wps[k].h, hb = flight.wps[k + 1].h, L = cum[b] - cum[a];
      let worst = tol, at = -1;
      for (let i = a + 1; i < b; i++) {
        const f = L > 0 ? (cum[i] - cum[a]) / L : 0;
        const ex = ha + f * (hb - ha) - r[i];
        if (ex > worst) { worst = ex; at = i; }
      }
      if (at >= 0) { keep[at] = true; added++; }
    }
    if (!added) break;
  }
  return { ...flight, thinned: { before, after: flight.wps.length } };
}

// Heights for a route the way the planner exports them: terrain-following heights, then thinning when the
// plan asks for it. `dense` is the un-thinned result (for the profile display); `flight` is what is flown.
export function flightRoute(plan: Plan, route: Route, elev: ElevFn): { flight: Route<FlightWp>; dense: Route<FlightWp> } {
  const dense = applyHeights(plan, route, elev);
  const flight = plan.o.thinToleranceM > 0 ? thinRoute(plan, route, dense, elev) : { ...dense, thinned: { before: dense.wps.length, after: dense.wps.length } };
  return { flight, dense };
}

// Time to fly a route at its per-leg speeds (seconds), horizontal legs only.
export function routeTimeS(wps: RouteWp[]): number {
  let t = 0;
  for (let i = 0; i < wps.length - 1; i++) t += dist(wps[i].xy, wps[i + 1].xy) / legSpeed(wps, i);
  return t;
}
