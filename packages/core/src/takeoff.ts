import { type LonLat, type XY, add, dist, mul } from './geo.ts';
import type { FlightWp } from './heights.ts';
import type { Plan } from './plan.ts';
import type { Route } from './route.ts';
import { corridorMax, corridorProfile, type ElevFn, TerrainGapError } from './terrain.ts';

// How the aircraft gets from take-off to the first waypoint (DJI WPML `flyToWaylineMode`).
//  safely       climb vertically to max(take-off security height, first-waypoint height), fly level
//               to the first waypoint, descend if it is lower.
//  pointToPoint climb vertically to the take-off security height, then fly a straight (sloping) line
//               to the first waypoint; if that waypoint is lower, fly level then descend.
// Behaviour as described in DJI's WPML docs; confirm on the M400 in the simulator. The security height
// only applies when the mission is started from the ground: start it from the planned home point.
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
  rthRecommendedM: number;        // lowest RTH height (above take-off) that clears terrain from anywhere on the route
  rthHeightM: number;             // value actually used (user's or recommended)
  rthWorstClearanceM: number;     // with rthHeightM, the worst clearance on any straight line home
  rthWorstWp: number;             // waypoint at the start of the leg where that happens
  rthFromLast: Pt3[];             // last waypoint → home at RTH altitude
  rthDistanceM: number;
  topAboveHomeM: number;          // highest point of the flight (route and take-off transit) above the take-off point
  wp1AboveHomeM: number;          // first waypoint above the take-off point: what the RC must show there
  lowAboveHomeM: number;          // lowest point of the route relative to the take-off point (negative = below it)
}

export function planTransit(plan: Plan, flight: Route<FlightWp>, elev: ElevFn, home: LonLat, opts: Partial<TakeoffOptions> = {}): Transit {
  const o = { ...TAKEOFF_DEFAULTS, ...opts };
  const { proj } = plan;
  const corridor = plan.o.corridorM, step = plan.o.demSampleM;
  const homeElev = elev(home[0], home[1]);
  if (homeElev == null || !Number.isFinite(homeElev)) throw new TerrainGapError(home[0], home[1]);
  const homeXY = proj.fwd(home[0], home[1]);
  const at = (p: XY, h: number): Pt3 => { const [lon, lat] = proj.inv(p[0], p[1]); return { lon, lat, h }; };

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
    const p = pts[i - 1], q = pts[i];
    const horiz = dist(p.xy, q.xy);
    distanceM += Math.hypot(horiz, q.h - p.h);
    if (horiz < 1) continue;                              // vertical climb/descent at a point
    for (const s of corridorProfile(elev, proj, p.xy, q.xy, corridor, step)) {
      const hPath = p.h + (q.h - p.h) * (s.s / horiz);
      const c = hPath - s.max;
      if (c < minClr) { minClr = c; minAt = at(s.p, hPath); }
    }
  }

  // ── RTH: from points all along the route (every waypoint and at most a corridor width apart in between),
  // straight home at max(current height, home + RTH height).
  const wps = flight.wps;
  const origins: { xy: XY; h: number; wp: number }[] = [];
  const gap = Math.max(30, corridor);
  for (let i = 0; i < wps.length; i++) {
    origins.push({ xy: wps[i].xy, h: wps[i].h, wp: i });
    if (i === wps.length - 1) break;
    const L = dist(wps[i].xy, wps[i + 1].xy), k = Math.max(0, Math.ceil(L / gap) - 1);   // origins ≤ gap apart
    for (let j = 1; j <= k; j++) {
      const t = j / (k + 1);
      origins.push({ xy: add(wps[i].xy, mul([wps[i + 1].xy[0] - wps[i].xy[0], wps[i + 1].xy[1] - wps[i].xy[1]], t)), h: wps[i].h + (wps[i + 1].h - wps[i].h) * t, wp: i });
    }
  }
  const lineMax = origins.map(p => corridorMax(elev, proj, p.xy, homeXY, corridor, 0, step));
  // Between two neighbouring origins the aircraft can be anywhere on the leg, at any height between theirs,
  // and its line home lies between their two lines (inside their corridors, since they are at most a
  // corridor width apart). So each pair counts with its higher terrain and its lower height.
  const spans = origins.slice(0, -1).map((p, i) => ({ top: Math.max(lineMax[i], lineMax[i + 1]), h: Math.min(p.h, origins[i + 1].h), wp: p.wp }));
  let rec = o.takeoffSecurityM;
  for (const s of spans) if (s.h < s.top + o.minClearanceM) rec = Math.max(rec, s.top + o.minClearanceM - homeElev);
  const rthRecommendedM = Math.ceil(rec / 10) * 10;
  const rthHeightM = o.rthHeightM ?? rthRecommendedM;
  let worst = Infinity, worstWp = 0;
  for (const s of spans) {
    const c = Math.max(s.h, homeElev + rthHeightM) - s.top;
    if (c < worst) { worst = c; worstWp = s.wp; }
  }
  const last = wps[wps.length - 1];
  const hRet = Math.max(last.h, homeElev + rthHeightM);
  const rthFromLast = [at(last.xy, last.h), at(last.xy, hRet), at(homeXY, hRet), at(homeXY, homeElev)];

  return {
    homeElev, path: pts.map(p => at(p.xy, p.h)), distanceM, timeS: distanceM / o.transitSpeedMs,
    minClearanceM: minClr, minClearanceAt: minAt,
    rthRecommendedM, rthHeightM, rthWorstClearanceM: worst, rthWorstWp: worstWp,
    rthFromLast, rthDistanceM: dist(last.xy, homeXY) + (hRet - last.h) + (hRet - homeElev),
    topAboveHomeM: Math.max(...wps.map(w => w.h), ...pts.map(p => p.h)) - homeElev,
    wp1AboveHomeM: wp1.h - homeElev, lowAboveHomeM: Math.min(...wps.map(w => w.h)) - homeElev,
  };
}
