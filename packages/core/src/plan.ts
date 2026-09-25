import { type LonLat, type Projection, type XY, D2R, bandExtent, dot, makeProj, polygonAreaM2 } from './geo.ts';

export interface PlanOptions {
  aglM: number;            // nominal clearance above terrain (m)
  speedMs: number;         // line speed
  fovDeg: number;          // L3 effective across-track FOV used for swath (set from the L3 scan mode)
  sidelapPct: number;
  courseDeg: number;       // line heading, degrees clockwise from north
  runInM: number;
  runOutM: number;
  wpSpacingM: number;      // max waypoint spacing along lines
  corridorM: number;       // lateral buffer when finding max terrain under the path
  demSampleM: number;      // DEM sampling step (GLO-30 native)
  maxGradient: number;     // max climb/descent between waypoints (rise/run)
  geoidN: number;          // added to DEM (orthometric) height when writing ellipsoidal heights
  fig8BankDeg: number;
  fig8RadiusM: number | null; // null = computed from speed + bank
  fig8PtsPerLoop: number;
  dampingFrac: number;     // damping distance as fraction of the shorter adjacent leg
  dampingMaxM: number;
}

export const DEFAULTS: PlanOptions = {
  aglM: 500,
  speedMs: 17,
  fovDeg: 70,
  sidelapPct: 50,
  courseDeg: 0,
  runInM: 150,
  runOutM: 150,
  wpSpacingM: 150,
  corridorM: 75,
  demSampleM: 30,
  maxGradient: 0.15,
  geoidN: 0,
  fig8BankDeg: 25,
  fig8RadiusM: null,
  fig8PtsPerLoop: 12,
  dampingFrac: 0.4,
  dampingMaxM: 60,
};

export interface Line {
  index: number;
  v: number;               // across-track offset of the line axis
  umin: number;            // along-track extent over the block
  umax: number;
  dir: 1 | -1;             // serpentine direction along d
}

export interface Plan {
  o: PlanOptions;
  proj: Projection;
  d: XY;                   // unit along-line (E,N)
  c: XY;                   // unit across-line, to the right of d
  lines: Line[];
  swath: number;
  spacing: number;
  areaHa: number;
  polyXY: XY[];
}

export function planLines(polyLonLat: LonLat[], opts: Partial<PlanOptions> = {}): Plan {
  const o: PlanOptions = { ...DEFAULTS, ...opts };
  const lat0 = polyLonLat.reduce((s, p) => s + p[1], 0) / polyLonLat.length;
  const lon0 = polyLonLat.reduce((s, p) => s + p[0], 0) / polyLonLat.length;
  const proj = makeProj(lat0, lon0);
  const xy = polyLonLat.map(([lo, la]) => proj.fwd(lo, la));

  const th = o.courseDeg * D2R;
  const d: XY = [Math.sin(th), Math.cos(th)];
  const c: XY = [Math.cos(th), -Math.sin(th)];
  const uv: XY[] = xy.map(p => [dot(p, d), dot(p, c)]);

  const swath = 2 * o.aglM * Math.tan((o.fovDeg / 2) * D2R);
  const spacing = swath * (1 - o.sidelapPct / 100);
  const vs = uv.map(p => p[1]);
  const vmin = Math.min(...vs), vmax = Math.max(...vs);
  const n = Math.ceil((vmax - vmin) / spacing) + 1;
  const vStart = (vmin + vmax) / 2 - ((n - 1) * spacing) / 2;

  const lines: Line[] = [];
  for (let i = 0; i < n; i++) {
    const v = vStart + i * spacing;
    const ext = bandExtent(uv, v - spacing / 2, v + spacing / 2);
    if (!ext) continue;
    const k = lines.length;
    lines.push({ index: k, v, umin: ext[0], umax: ext[1], dir: k % 2 === 0 ? 1 : -1 });
  }

  return { o, proj, d, c, lines, swath, spacing, areaHa: polygonAreaM2(xy) / 1e4, polyXY: xy };
}
