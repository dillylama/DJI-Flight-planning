import { type XY, G, D2R, add, dist, mul } from './geo.ts';
import type { Plan } from './plan.ts';

export type Role = 'approach' | 'fig8' | 'runin' | 'line' | 'runout' | 'exit';
export type Action = 'START_RECORD' | 'STOP_RECORD';
export type TurnMode = 'toPointAndPassWithContinuityCurvature';

export interface RouteWp {
  xy: XY;
  role: Role;
  speed: number;           // DJI waypointSpeed: the speed flown from this waypoint to the next one
  actions: Action[];
  line?: number;
  dampingM: number;
  turnMode: TurnMode;
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
}

// Full job, a resume from any line, or one sortie (a line range).
export function buildRoute(plan: Plan, { fromLine = 0, speedMs, fig8 = true, fig8End, startLine, endLine }: BuildOptions = {}): Route {
  const { o, d, c, lines } = plan;
  const v = speedMs || o.speedMs;
  const endEight = fig8End ?? fig8;
  const startIdx = startLine ?? Math.max(0, fromLine - (fromLine > 0 ? 1 : 0));
  const lastIdx = Math.min(lines.length - 1, endLine ?? lines.length - 1);
  const toXY = (u: number, vv: number): XY => add(mul(d, u), mul(c, vv));
  const wps: Omit<RouteWp, 'dampingM' | 'turnMode'>[] = [];
  const push = (xy: XY, role: Role, extra: Partial<RouteWp> = {}) =>
    wps.push({ xy, role, speed: v, actions: [], ...extra });

  // Figure-8 about crossover X, entered along hdg: left lobe then right lobe, back at X each time.
  const r = o.fig8RadiusM || (v * v) / (G * Math.tan(o.fig8BankDeg * D2R));
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
  const runInStart = toXY(first.dir > 0 ? first.umin - o.runInM : first.umax + o.runInM, first.v);
  const X0 = add(runInStart, mul(hdg0, -r));
  push(add(X0, mul(hdg0, -straight)), 'approach', { actions: ['START_RECORD'] });   // recording running before the 8
  if (fig8) eight(X0, hdg0);

  for (let i = startIdx; i <= lastIdx; i++) {
    const L = lines[i];
    const a = L.dir > 0 ? L.umin : L.umax, b = L.dir > 0 ? L.umax : L.umin;
    const s = L.dir;
    push(toXY(a - s * o.runInM, L.v), 'runin', { line: i });
    const nSeg = Math.max(1, Math.ceil(Math.abs(b - a) / o.wpSpacingM));
    for (let k = 0; k <= nSeg; k++) push(toXY(a + ((b - a) * k) / nSeg, L.v), 'line', { line: i });
    push(toXY(b + s * o.runOutM, L.v), 'runout', { line: i });
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
  wps[wps.length - 1].actions.push('STOP_RECORD');

  // Damping per waypoint: fraction of the shorter adjacent leg (DJI requires < leg length).
  const out: RouteWp[] = wps.map((w, i) => {
    const legs: number[] = [];
    if (i > 0) legs.push(dist(w.xy, wps[i - 1].xy));
    if (i < wps.length - 1) legs.push(dist(w.xy, wps[i + 1].xy));
    return {
      ...w,
      dampingM: Math.min(o.dampingMaxM, o.dampingFrac * Math.min(...legs)),
      turnMode: 'toPointAndPassWithContinuityCurvature',
    };
  });

  return { wps: out, startIdx, fig8RadiusM: r, speedMs: v };
}
