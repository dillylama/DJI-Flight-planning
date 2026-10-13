import { type XY, G, D2R, add, dist, mul } from './geo.ts';
import type { Plan } from './plan.ts';

export type Role = 'transit' | 'approach' | 'fig8' | 'runin' | 'line' | 'runout' | 'exit';
export type Action = 'START_RECORD' | 'STOP_RECORD';
export type TurnMode = 'toPointAndPassWithContinuityCurvature' | 'toPointAndStopWithDiscontinuityCurvature';
export const STOP_TURN: TurnMode = 'toPointAndStopWithDiscontinuityCurvature';
export const isStop = (w: { turnMode: string }) => w.turnMode === STOP_TURN;

export interface RouteWp {
  xy: XY;
  role: Role;
  speed: number;           // DJI waypointSpeed: the speed flown from this waypoint to the next one
  actions: Action[];
  line?: number;
  dampingM: number;
  turnMode: TurnMode;
  aglM?: number;           // clearance this waypoint needs when it is more than the plan's AGL (transit legs)
}

// Speed on leg i (waypoint i → i+1). DJI WPML: `waypointSpeed` is the speed from that waypoint to the
// next, so leg i takes waypoint i's speed. (docs/ugcs-notes.md records the source.)
export const legSpeed = (wps: { speed: number }[], i: number) => wps[i].speed;

export interface Route<W extends RouteWp = RouteWp> {
  wps: W[];
  startIdx: number;        // first line flown
  fig8RadiusM: number;
  speedMs: number;
}

export interface BuildOptions {
  fromLine?: number;       // line where data stopped; the route restarts one line earlier
  speedMs?: number;
  fig8?: boolean;          // IMU-excitation figure-8 before the first line (LiDAR); off for photogrammetry
  fig8End?: boolean;       // figure-8 after the last line too, before recording stops (default: same as fig8)
  startLine?: number;      // explicit first line (sorties); overrides fromLine, no line of overlap added
  endLine?: number;        // last line flown, inclusive (sorties); default the last line
  // Terrain-following transit: the way from home to the approach point and back from the exit point become
  // waypoints of the route (every wpSpacingM), with heights from the DEM like every other leg, instead of the
  // aircraft's own straight "fly to route" leg. The route then starts and ends above home, and the approach
  // and exit waypoints, where recording starts and stops, are stop turns.
  transit?: { home: XY; speedMs: number; clearanceM: number } | null;
}

// Full job, a resume from any line, or one sortie (a line range).
export function buildRoute(plan: Plan, { fromLine = 0, speedMs, fig8 = true, fig8End, startLine, endLine, transit = null }: BuildOptions = {}): Route {
  const { o, d, c, lines } = plan;
  const v = speedMs || o.speedMs;
  const endEight = fig8End ?? fig8;
  const startIdx = startLine ?? Math.max(0, fromLine - (fromLine > 0 ? 1 : 0));
  const lastIdx = Math.min(lines.length - 1, endLine ?? lines.length - 1);
  const toXY = (u: number, vv: number): XY => add(mul(d, u), mul(c, vv));
  const wps: (Omit<RouteWp, 'dampingM' | 'turnMode'> & { stop?: boolean })[] = [];
  const push = (xy: XY, role: Role, extra: Partial<RouteWp> & { stop?: boolean } = {}) =>
    wps.push({ xy, role, speed: v, actions: [], ...extra });
  // Transit waypoints from a towards b, at most wpSpacingM apart: a itself and the points in between, not b.
  const ferry = (a: XY, b: XY, includeA: boolean) => {
    const n = Math.max(1, Math.ceil(dist(a, b) / o.wpSpacingM));
    for (let k = includeA ? 0 : 1; k < (includeA ? n : n + 1); k++) {
      push(add(a, mul([b[0] - a[0], b[1] - a[1]], k / n)), 'transit', { speed: transit!.speedMs, aglM: Math.max(o.aglM, transit!.clearanceM) });
    }
  };

  // Run-in / run-out are never shorter than 20 m: a zero-length one would put two waypoints on the same spot.
  const runIn = Math.max(20, o.runInM), runOut = Math.max(20, o.runOutM);

  // Figure-8 about crossover X, entered along hdg: left lobe then right lobe, back at X each time.
  const r = o.fig8RadiusM || Math.max(o.fig8MinRadiusM, (v * v) / (G * Math.tan(o.fig8BankDeg * D2R)));
  const straight = Math.max(2.5 * r, o.alignStraightS * v);   // straight run before/after the 8 (Phoenix: ≥10 s at ≥5 m/s)
  const eight = (X: XY, hdg: XY) => {
    const right: XY = [hdg[1], -hdg[0]];
    push(X, 'fig8');
    const loop = (centre: XY, startVec: XY) => {
      for (let k = 1; k < o.fig8PtsPerLoop; k++) {
        const t = (2 * Math.PI * k) / o.fig8PtsPerLoop;
        push(add(centre, add(mul(startVec, r * Math.cos(t)), mul(hdg, r * Math.sin(t)))), 'fig8');
      }
      push(X, 'fig8');
    };
    loop(add(X, mul(right, -r)), right);          // left lobe
    loop(add(X, mul(right, r)), mul(right, -1));  // right lobe
  };

  // Start: approach (recording on) → figure-8 one radius behind the run-in start → lines.
  const first = lines[startIdx];
  const hdg0 = mul(d, first.dir);
  const runInStart = toXY(first.dir > 0 ? first.umin - runIn : first.umax + runIn, first.v);
  const X0 = add(runInStart, mul(hdg0, -r));
  const approachXY = add(X0, mul(hdg0, -straight));
  if (transit) ferry(transit.home, approachXY, true);
  push(approachXY, 'approach', { actions: ['START_RECORD'], stop: !!transit });   // recording running before the 8
  if (fig8) eight(X0, hdg0);

  for (let i = startIdx; i <= lastIdx; i++) {
    const L = lines[i];
    const a = L.dir > 0 ? L.umin : L.umax, b = L.dir > 0 ? L.umax : L.umin;
    const s = L.dir;
    push(toXY(a - s * runIn, L.v), 'runin', { line: i });
    // A very short line (the block only clips this strip) gets one waypoint, never two almost on the same
    // spot; the run-out beyond `b` still carries the aircraft across the whole strip.
    const nSeg = Math.abs(b - a) < 20 ? 0 : Math.max(1, Math.ceil(Math.abs(b - a) / o.wpSpacingM));
    for (let k = 0; k <= nSeg; k++) push(toXY(nSeg ? a + ((b - a) * k) / nSeg : a, L.v), 'line', { line: i });
    push(toXY(b + s * runOut, L.v), 'runout', { line: i });
  }

  // End: figure-8 one radius beyond the run-out end, then an exit point where recording stops, so the
  // trajectory is aligned at both ends for forward/backward post-processing.
  if (endEight) {
    const last = lines[lastIdx];
    const hdg1 = mul(d, last.dir);
    const X1 = add(wps[wps.length - 1].xy, mul(hdg1, r));
    eight(X1, hdg1);
    push(add(X1, mul(hdg1, straight)), 'exit');
  }
  const end = wps[wps.length - 1];
  end.actions.push('STOP_RECORD');
  if (transit) { end.stop = true; end.speed = transit.speedMs; ferry(end.xy, transit.home, false); }

  // Damping per waypoint. Where the heading changes, the turn radius at turnBankDeg for the speed flown into
  // the waypoint, so the aircraft can round the corner at speed (it passes r·(sec(θ/2) − 1) inside it, 0.41 r
  // for a right angle); on a straight run just enough to fly through. Never more than dampingFrac of the
  // shorter adjacent leg, which keeps the two dampings on any leg inside it (DJI requires that), or dampingMaxM.
  // applyHeights shortens it again where the vertical rounding or the terrain corridor needs it.
  const out: RouteWp[] = wps.map(({ stop, ...w }, i) => {
    const legs: number[] = [];
    if (i > 0) legs.push(dist(w.xy, wps[i - 1].xy));
    if (i < wps.length - 1) legs.push(dist(w.xy, wps[i + 1].xy));
    let want = 10;
    if (i > 0 && i < wps.length - 1) {
      const ax = w.xy[0] - wps[i - 1].xy[0], ay = w.xy[1] - wps[i - 1].xy[1], bx = wps[i + 1].xy[0] - w.xy[0], by = wps[i + 1].xy[1] - w.xy[1];
      const theta = Math.abs(Math.atan2(ax * by - ay * bx, ax * bx + ay * by));
      if (theta > 5 * D2R) {
        const vIn = wps[i - 1].speed, r = (vIn * vIn) / (G * Math.tan(o.turnBankDeg * D2R));
        want = Math.max(10, r * Math.tan(Math.min(theta, Math.PI / 2) / 2));
      }
    }
    return {
      ...w,
      dampingM: stop ? 0 : Math.min(o.dampingMaxM, o.dampingFrac * Math.min(...legs), want),
      turnMode: stop ? STOP_TURN : 'toPointAndPassWithContinuityCurvature',
    };
  });

  return { wps: out, startIdx, fig8RadiusM: r, speedMs: v };
}
