import { type LonLat, type XY, add, dist, mul } from './geo.ts';
import type { ElevFn, FlightWp } from './heights.ts';
import type { Plan } from './plan.ts';
import type { Route } from './route.ts';

// How the aircraft gets from take-off to the first waypoint (DJI WPML `flyToWaylineMode`).
//  safely       climb vertically to max(take-off security height, first-waypoint height), fly level
//               to the first waypoint, descend if it is lower.
//  pointToPoint climb vertically to the take-off security height, then fly a straight (sloping) line
//               to the first waypoint; if that waypoint is lower, fly level then descend.
// Behaviour as described in DJI's WPML docs; confirm on the M400 in the simulator.
export type FlyToMode = 'safely' | 'pointToPoint';

export interface TakeoffOptions {
  takeoffSecurityM: number;   // DJI takeOffSecurityHeight, above the take-off point (1.2–1500)
  flyToMode: FlyToMode;
  transitSpeedMs: number;     // DJI globalTransitionalSpeed
  rthHeightM: number | null;  // above take-off; null = use the recommended value
  minClearanceM: number;      // required terrain clearance on transit and RTH legs
}

export const TAKEOFF_DEFAULTS: TakeoffOptions = {
  takeoffSecurityM: 60,
  flyToMode: 'safely',
  transitSpeedMs: 15,
  rthHeightM: null,
  minClearanceM: 60,
};

export interface Pt3 { lon: number; lat: number; h: number }

export interface Transit {
  homeElev: number;
  path: Pt3[];                    // home → first waypoint
  distanceM: number;
  timeS: number;
  minClearanceM: number;          // along the transit, excluding the vertical climb at home
  minClearanceAt: Pt3 | null;
  rthRecommendedM: number;        // lowest RTH height (above take-off) that clears terrain from every waypoint
  rthHeightM: number;             // value actually used (user's or recommended)
  rthWorstClearanceM: number;     // with rthHeightM, the worst clearance on any straight line home
  rthWorstWp: number;
  rthFromLast: Pt3[];             // last waypoint → home at RTH altitude
  rthDistanceM: number;
}

export function planTransit(plan: Plan, flight: Route<FlightWp>, elev: ElevFn, home: LonLat, opts: Partial<TakeoffOptions> = {}): Transit {
  const o = { ...TAKEOFF_DEFAULTS, ...opts };
  const { proj } = plan;
  const corridor = plan.o.corridorM, step = plan.o.demSampleM;
  const homeElev = elev(home[0], home[1]);
  if (homeElev == null || !Number.isFinite(homeElev)) throw new Error('The DEM has no data at the home point');
  const homeXY = proj.fwd(home[0], home[1]);
  const terr = (p: XY) => { const [lo, la] = proj.inv(p[0], p[1]); const h = elev(lo, la); return h != null && Number.isFinite(h) ? h : -Infinity; };
  const at = (p: XY, h: number): Pt3 => { const [lon, lat] = proj.inv(p[0], p[1]); return { lon, lat, h }; };

  // Max terrain within ±corridor along a straight leg a→b.
  const legTerrain = (a: XY, b: XY, stepM: number) => {
    const L = dist(a, b), n = Math.max(1, Math.ceil(L / stepM));
    const dir: XY = L > 0 ? mul([b[0] - a[0], b[1] - a[1]], 1 / L) : [1, 0];
    const nrm: XY = [dir[1], -dir[0]];
    const out: { t: number; h: number; p: XY }[] = [];
    for (let k = 0; k <= n; k++) {
      const p = add(a, mul([b[0] - a[0], b[1] - a[1]], k / n));
      let m = -Infinity;
      for (const off of [-corridor, 0, corridor]) m = Math.max(m, terr(add(p, mul(nrm, off))));
      out.push({ t: k / n, h: m, p });
    }
    return out;
  };

  // ── Transit home → WP1
  const wp1 = flight.wps[0];
  const h1 = wp1.h, hSec = homeElev + o.takeoffSecurityM;
  const pts: { xy: XY; h: number }[] = [{ xy: homeXY, h: homeElev }];
  if (o.flyToMode === 'safely') {
    const cruise = Math.max(hSec, h1);
    pts.push({ xy: homeXY, h: cruise }, { xy: wp1.xy, h: cruise });
    if (h1 < cruise) pts.push({ xy: wp1.xy, h: h1 });
  } else {
    pts.push({ xy: homeXY, h: hSec });
    if (h1 >= hSec) pts.push({ xy: wp1.xy, h: h1 });
    else pts.push({ xy: wp1.xy, h: hSec }, { xy: wp1.xy, h: h1 });
  }
  let distanceM = 0, minClr = Infinity, minAt: Pt3 | null = null;
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1], b = pts[i];
    const horiz = dist(a.xy, b.xy);
    distanceM += Math.hypot(horiz, b.h - a.h);
    if (horiz < 1) continue;                              // vertical climb/descent at a point
    for (const s of legTerrain(a.xy, b.xy, step)) {
      const c = a.h + (b.h - a.h) * s.t - s.h;
      if (c < minClr) { minClr = c; minAt = at(s.p, a.h + (b.h - a.h) * s.t); }
    }
  }

  // ── RTH: from every waypoint, straight home at max(current height, home + RTH height)
  const need: number[] = flight.wps.map(w => Math.max(...legTerrain(w.xy, homeXY, 60).map(s => s.h)) + o.minClearanceM);
  let rec = o.takeoffSecurityM;
  flight.wps.forEach((w, i) => { if (w.h < need[i]) rec = Math.max(rec, need[i] - homeElev); });
  const rthRecommendedM = Math.ceil(rec / 10) * 10;
  const rthHeightM = o.rthHeightM ?? rthRecommendedM;
  let worst = Infinity, worstWp = 0;
  flight.wps.forEach((w, i) => {
    const c = Math.max(w.h, homeElev + rthHeightM) - (need[i] - o.minClearanceM);
    if (c < worst) { worst = c; worstWp = i; }
  });
  const last = flight.wps[flight.wps.length - 1];
  const hRet = Math.max(last.h, homeElev + rthHeightM);
  const rthFromLast = [at(last.xy, last.h), at(last.xy, hRet), at(homeXY, hRet), at(homeXY, homeElev)];

  return {
    homeElev, path: pts.map(p => at(p.xy, p.h)), distanceM, timeS: distanceM / o.transitSpeedMs,
    minClearanceM: minClr, minClearanceAt: minAt,
    rthRecommendedM, rthHeightM, rthWorstClearanceM: worst, rthWorstWp: worstWp,
    rthFromLast, rthDistanceM: dist(last.xy, homeXY) + (hRet - last.h) + (hRet - homeElev),
  };
}
