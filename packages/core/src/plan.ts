import { type LonLat, type ProjModel, type Projection, type XY, D2R, bandExtent, dot, makeProj, polygonAreaM2 } from './geo.ts';

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
  verticalMode: 'slow' | 'raise'; // 'slow': follow terrain, slow legs to the climb/descent limits; 'raise': keep speed, raise waypoints
  climbMs: number;         // vertical rate limits used by 'slow' (conservative M400 values; see equipment.ts)
  descentMs: number;
  minLegSpeedMs: number;   // floor for a slowed leg; a leg needing less is flagged as too steep
  maxGradient: number;     // 'raise' mode: max climb/descent between waypoints (rise/run)
  roundingMaxM: number;    // turn damping is shortened wherever the path's vertical rounding at a waypoint would exceed this
  fig8BankDeg: number;
  fig8RadiusM: number | null; // null = computed from speed + bank
  fig8MinRadiusM: number;  // floor for the computed radius, so a slow route still gets a flyable figure-8
  fig8PtsPerLoop: number;
  alignStraightS: number;  // straight level flight before the first figure-8 and after the last one, in seconds (≥ 2.5 radii)
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
  verticalMode: 'slow',
  climbMs: 4,
  descentMs: 3,
  minLegSpeedMs: 1,
  maxGradient: 0.15,
  roundingMaxM: 5,
  fig8BankDeg: 25,
  fig8RadiusM: null,
  fig8MinRadiusM: 20,
  fig8PtsPerLoop: 12,
  alignStraightS: 12,
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

// Inputs the geometry cannot work with at all. (Safety floors such as the minimum AGL are separate: see safety.ts.)
function checkInputs(poly: LonLat[], o: PlanOptions): void {
  const bad = (m: string) => { throw new Error(m); };
  if (poly.length < 3) bad('The block needs at least 3 corners.');
  if (!poly.every(p => Number.isFinite(p[0]) && Number.isFinite(p[1]) && Math.abs(p[0]) <= 180 && Math.abs(p[1]) <= 90)) bad('The block has a corner that is not a valid longitude, latitude.');
  const lons = poly.map(p => p[0]), lats = poly.map(p => p[1]);
  if (Math.max(...lats.map(Math.abs)) > 80) bad('Blocks beyond 80° latitude are not supported.');
  if (Math.max(...lons) - Math.min(...lons) > 2 || Math.max(...lats) - Math.min(...lats) > 2) bad('The block spans more than 2° (or crosses the 180° meridian). Split it into smaller blocks.');
  const pos: [keyof PlanOptions, number][] = [['aglM', 1], ['speedMs', 0.5], ['wpSpacingM', 10], ['demSampleM', 1], ['climbMs', 0.1], ['descentMs', 0.1], ['minLegSpeedMs', 0.1], ['fig8PtsPerLoop', 6]];
  for (const [k, min] of pos) if (!(Number.isFinite(o[k] as number) && (o[k] as number) >= min)) bad(`${k} must be a number of at least ${min}.`);
  const nonNeg: (keyof PlanOptions)[] = ['runInM', 'runOutM', 'corridorM', 'maxGradient', 'roundingMaxM', 'fig8MinRadiusM', 'alignStraightS', 'dampingMaxM', 'courseDeg'];
  for (const k of nonNeg) if (!Number.isFinite(o[k] as number) || (k !== 'courseDeg' && (o[k] as number) < 0)) bad(`${k} must be a number, 0 or more.`);
  if (!(o.fovDeg >= 1 && o.fovDeg <= 150)) bad('fovDeg must be between 1° and 150°.');
  if (!(o.sidelapPct >= 0 && o.sidelapPct <= 90)) bad('sidelapPct must be between 0 and 90 %.');
  if (!(o.fig8BankDeg >= 5 && o.fig8BankDeg <= 45)) bad('fig8BankDeg must be between 5° and 45°.');
  if (!(o.dampingFrac > 0 && o.dampingFrac < 0.5)) bad('dampingFrac must be above 0 and below 0.5 (two dampings must fit in a leg).');
  if (o.fig8RadiusM != null && !(o.fig8RadiusM >= 5)) bad('fig8RadiusM must be at least 5 m.');
  if (!Number.isInteger(o.fig8PtsPerLoop)) bad('fig8PtsPerLoop must be a whole number.');
}

export function planLines(polyLonLat: LonLat[], opts: Partial<PlanOptions> = {}, projModel: ProjModel = 'wgs84'): Plan {
  const o: PlanOptions = { ...DEFAULTS, ...opts };
  checkInputs(polyLonLat, o);
  const lat0 = polyLonLat.reduce((s, p) => s + p[1], 0) / polyLonLat.length;
  const lon0 = polyLonLat.reduce((s, p) => s + p[0], 0) / polyLonLat.length;
  const proj = makeProj(lat0, lon0, projModel);
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
  if (!(spacing > 0) || !(n <= 2000)) throw new Error(`Line spacing ${spacing.toFixed(1)} m gives ${n} lines for this block: check AGL, field of view and sidelap.`);
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
