import { type LonLat, type XY, dist } from './geo.ts';
import { applyHeights, routeTimeS, type ElevFn, type FlightWp } from './heights.ts';
import type { Plan } from './plan.ts';
import { buildRoute, legSpeed, type Route, type RouteWp } from './route.ts';
import { planTransit, TAKEOFF_DEFAULTS, type TakeoffOptions, type Transit } from './takeoff.ts';
import type { Issue } from './validate.ts';

// Split a job into battery-sized sorties at line boundaries. Each sortie is flown as its own route:
// take-off transit → approach (recording on) → figure-8 → lines s..e → RTH. Planned splits fall on
// complete lines, so no line is re-flown unless overlapLines > 0 (resume after a failure is separate).
export interface SortieOptions {
  usableMin: number;              // usable minutes per battery set (reserve already removed)
  firstLine?: number;             // first line of the job (e.g. a resume start); default 0
  speedMs?: number;
  fig8?: boolean;
  fig8End?: boolean;
  overlapLines?: number;          // lines re-flown at the start of each following sortie; default 0
  maxWaypoints?: number | null;   // Pilot 2 / aircraft cap per route, if known
  climbMs?: number;               // vertical rates used for the time estimate (use conservative limits)
  descentMs?: number;
  full?: Route<FlightWp>;         // heights of the whole job from firstLine, if already computed
}

export interface SortieTime { total: number; route: number; transit: number; rth: number; vertical: number }
export interface Sortie {
  index: number;
  fromLine: number;
  toLine: number;
  route: Route<FlightWp>;
  transit: Transit | null;
  time: SortieTime;
  overBudget: boolean;
}
export interface SortiePlan { sorties: Sortie[]; issues: Issue[]; totalMin: number }

const routeLen = (wps: { xy: XY }[]) => { let s = 0; for (let i = 1; i < wps.length; i++) s += dist(wps[i].xy, wps[i - 1].xy); return s; };

// Minutes for one sortie. Horizontal legs at their speeds; the vertical climb at home and the final
// descent at the conservative vertical rates. Without a home point only the route itself counts.
function sortieTime(
  wps: (RouteWp & { h?: number })[], speed: number, homeXY: XY | null, homeElev: number | null,
  hTop: number, tk: TakeoffOptions, climbMs: number, descentMs: number,
): SortieTime {
  const route = routeTimeS(wps) / 60;      // per-leg speeds (slowed climbs count); `speed` only names the line speed
  if (!homeXY || homeElev == null) return { total: route, route, transit: 0, rth: 0, vertical: 0 };
  const transit = dist(homeXY, wps[0].xy) / tk.transitSpeedMs / 60;
  const rth = dist(wps[wps.length - 1].xy, homeXY) / tk.transitSpeedMs / 60;
  const vertical = (Math.max(0, hTop - homeElev) / climbMs + Math.max(0, hTop - homeElev) / descentMs) / 60;
  return { total: route + transit + rth + vertical, route, transit, rth, vertical };
}

export function planSorties(
  plan: Plan, elev: ElevFn, home: LonLat | null, takeoff: Partial<TakeoffOptions>, opts: SortieOptions,
): SortiePlan {
  const tk = { ...TAKEOFF_DEFAULTS, ...takeoff };
  const first = opts.firstLine ?? 0, last = plan.lines.length - 1;
  const speed = opts.speedMs ?? plan.o.speedMs;
  const fig8 = opts.fig8 ?? true, overlap = Math.max(0, opts.overlapLines ?? 0);
  const climbMs = opts.climbMs ?? 4, descentMs = opts.descentMs ?? 3;
  const issues: Issue[] = [];

  // Heights of the whole job once: used to estimate the climb for any candidate range.
  const full = opts.full ?? applyHeights(plan, buildRoute(plan, { startLine: first, speedMs: speed, fig8, fig8End: opts.fig8End }), elev);
  const topOfLine = new Map<number, number>();
  for (const w of full.wps) if (w.line != null) topOfLine.set(w.line, Math.max(topOfLine.get(w.line) ?? -Infinity, w.h));
  const hTopRange = (s: number, e: number) => { let m = -Infinity; for (let i = s; i <= e; i++) m = Math.max(m, topOfLine.get(i) ?? -Infinity); return m; };
  // Slow-down factor per line from the full route (time at leg speeds ÷ time at line speed), so the
  // cheap estimates below account for legs slowed on steep ground without re-sampling the DEM.
  const slowT = new Map<number, number>(), fastT = new Map<number, number>();
  for (let i = 1; i < full.wps.length; i++) {
    const li = full.wps[i].line; if (li == null) continue;
    const d = dist(full.wps[i].xy, full.wps[i - 1].xy);
    slowT.set(li, (slowT.get(li) ?? 0) + d / legSpeed(full.wps, i - 1)); fastT.set(li, (fastT.get(li) ?? 0) + d / speed);
  }
  const factor = (li: number | undefined) => (li != null && fastT.get(li) ? slowT.get(li)! / fastT.get(li)! : 1);

  const homeXY = home ? plan.proj.fwd(home[0], home[1]) : null;
  const homeElev = home ? elev(home[0], home[1]) : null;
  if (home && (homeElev == null || !Number.isFinite(homeElev))) throw new Error('The DEM has no data at the home point');
  if (!home) issues.push({ severity: 'info', code: 'SORTIE_NO_HOME', message: 'Sorties are timed without take-off transit and RTH because home is not set.' });

  const estimate = (s: number, e: number) => {
    const r = buildRoute(plan, { startLine: s, endLine: e, speedMs: speed, fig8, fig8End: opts.fig8End });
    for (let i = 0; i < r.wps.length - 1; i++) r.wps[i].speed = speed / factor(r.wps[i + 1].line);
    return { r, t: sortieTime(r.wps, speed, homeXY, homeElev, hTopRange(s, e), tk, climbMs, descentMs) };
  };
  const fits = (s: number, e: number) => {
    const { r, t } = estimate(s, e);
    return t.total <= opts.usableMin && (opts.maxWaypoints == null || r.wps.length <= opts.maxWaypoints);
  };

  // Largest e for each start by binary search (time grows with every line added).
  const ranges: [number, number][] = [];
  let s = first;
  while (s <= last) {
    let lo = s, hi = last, best = s - 1;
    while (lo <= hi) {
      const mid = (lo + hi) >> 1;
      if (fits(s, mid)) { best = mid; lo = mid + 1; } else hi = mid - 1;
    }
    const e = Math.max(best, s);                  // a single line always makes a sortie (flagged if over)
    ranges.push([s, e]);
    if (e >= last) break;
    s = Math.max(e + 1 - overlap, s + 1);
  }

  // Build each sortie exactly: its own heights, transit/RTH check and time.
  const sorties: Sortie[] = ranges.map(([a, b], index) => {
    const route = applyHeights(plan, buildRoute(plan, { startLine: a, endLine: b, speedMs: speed, fig8, fig8End: opts.fig8End }), elev);
    const transit = home ? planTransit(plan, route, elev, home, tk) : null;
    const hTop = Math.max(...route.wps.map(w => w.h));
    const time = sortieTime(route.wps, speed, homeXY, homeElev, hTop, tk, climbMs, descentMs);
    return { index, fromLine: a, toLine: b, route, transit, time, overBudget: time.total > opts.usableMin + 1e-9 };
  });

  for (const so of sorties) if (so.overBudget) issues.push({
    severity: so.fromLine === so.toLine ? 'error' : 'warn', code: 'SORTIE_TIME',
    message: so.fromLine === so.toLine
      ? `Sortie ${so.index + 1}: line ${so.fromLine + 1} alone needs ${so.time.total.toFixed(1)} min, more than ${opts.usableMin} min. Move home closer, split the block, or fly faster.`
      : `Sortie ${so.index + 1} comes out at ${so.time.total.toFixed(1)} min after exact heights (budget ${opts.usableMin} min). Consider one line fewer.`,
  });
  if (opts.maxWaypoints != null) for (const so of sorties) if (so.route.wps.length > opts.maxWaypoints) issues.push({
    severity: 'error', code: 'SORTIE_WP_CAP', message: `Sortie ${so.index + 1} has ${so.route.wps.length} waypoints, over the cap of ${opts.maxWaypoints}.`,
  });

  return { sorties, issues, totalMin: sorties.reduce((t, so) => t + so.time.total, 0) };
}
