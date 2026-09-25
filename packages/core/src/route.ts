import { type XY, G, D2R, add, dist, mul } from './geo.ts';
import type { Plan } from './plan.ts';

export type Role = 'approach' | 'fig8' | 'runin' | 'line' | 'runout';
export type Action = 'START_RECORD' | 'STOP_RECORD';
export type TurnMode = 'toPointAndPassWithContinuityCurvature';

export interface RouteWp {
  xy: XY;
  role: Role;
  speed: number;
  actions: Action[];
  line?: number;
  dampingM: number;
  turnMode: TurnMode;
}

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
  startLine?: number;      // explicit first line (sorties); overrides fromLine, no line of overlap added
  endLine?: number;        // last line flown, inclusive (sorties); default the last line
}

// Full job, a resume from any line, or one sortie (a line range).
export function buildRoute(plan: Plan, { fromLine = 0, speedMs, fig8 = true, startLine, endLine }: BuildOptions = {}): Route {
  const { o, d, c, lines } = plan;
  const v = speedMs || o.speedMs;
  const startIdx = startLine ?? Math.max(0, fromLine - (fromLine > 0 ? 1 : 0));
  const lastIdx = Math.min(lines.length - 1, endLine ?? lines.length - 1);
  const toXY = (u: number, vv: number): XY => add(mul(d, u), mul(c, vv));
  const wps: Omit<RouteWp, 'dampingM' | 'turnMode'>[] = [];
  const push = (xy: XY, role: Role, extra: Partial<RouteWp> = {}) =>
    wps.push({ xy, role, speed: v, actions: [], ...extra });

  const first = lines[startIdx];
  const hdg = mul(d, first.dir);
  const u0 = first.dir > 0 ? first.umin - o.runInM : first.umax + o.runInM;
  const runInStart = toXY(u0, first.v);

  // Figure-8: crossover X one radius behind the run-in start, lobes left and right of the line axis.
  const r = o.fig8RadiusM || (v * v) / (G * Math.tan(o.fig8BankDeg * D2R));
  const X = add(runInStart, mul(hdg, -r));
  const right: XY = [hdg[1], -hdg[0]];
  const approach = add(X, mul(hdg, -2.5 * r));
  push(approach, 'approach', { actions: ['START_RECORD'] });   // recording running before the 8
  if (fig8) {
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
  }

  for (let i = startIdx; i <= lastIdx; i++) {
    const L = lines[i];
    const a = L.dir > 0 ? L.umin : L.umax, b = L.dir > 0 ? L.umax : L.umin;
    const s = L.dir;
    push(toXY(a - s * o.runInM, L.v), 'runin', { line: i });
    const nSeg = Math.max(1, Math.ceil(Math.abs(b - a) / o.wpSpacingM));
    for (let k = 0; k <= nSeg; k++) push(toXY(a + ((b - a) * k) / nSeg, L.v), 'line', { line: i });
    push(toXY(b + s * o.runOutM, L.v), 'runout', { line: i });
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
