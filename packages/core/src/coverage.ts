import { type LonLat, D2R, add, dot, mul } from './geo.ts';
import type { FlightWp } from './heights.ts';
import type { Plan } from './plan.ts';
import type { Route } from './route.ts';

export interface Swath { line: number; left: LonLat[]; right: LonLat[]; u: number[]; half: number[] }
export interface Overlap {
  lines: [number, number];
  poly: LonLat[];          // the band both strips cover
  widthMinM: number;       // overlap width along the pair
  widthMaxM: number;
  pctMin: number;          // overlap width as % of the narrower strip at that point
  pctMax: number;
}
export interface Coverage {
  plannedSidelapPct: number;
  achievedMinPct: number;
  achievedMeanPct: number;
  achievedMaxPct: number;
  swathMinM: number;
  swathMaxM: number;
  swaths: Swath[];
  overlaps: Overlap[];     // between consecutive flown lines
}

// LiDAR strip coverage on the data lines. Swath at each line waypoint = 2 · AGL · tan(FOV/2), using
// that waypoint's actual AGL. Sidelap with the neighbouring strip ≈ 1 − spacing / swath.
// Assumes level ground across-track: on a cross-slope the downhill edge reaches further and the uphill
// edge less, so treat these numbers as nominal.
export function coverage(plan: Plan, flight: Route<FlightWp>): Coverage {
  const { o, d, c, spacing, proj } = plan;
  const k = 2 * Math.tan((o.fovDeg / 2) * D2R);
  const byLine = new Map<number, FlightWp[]>();
  for (const w of flight.wps) if (w.role === 'line' && w.line != null) {
    if (!byLine.has(w.line)) byLine.set(w.line, []);
    byLine.get(w.line)!.push(w);
  }
  const laps: number[] = [], widths: number[] = [];
  const swaths: Swath[] = [];
  for (const [line, wps] of byLine) {
    const left: LonLat[] = [], right: LonLat[] = [], u: number[] = [], half: number[] = [];
    for (const w of wps) {
      const width = k * (w.h - w.terrainUnderWp);
      widths.push(width);
      laps.push(100 * (1 - spacing / width));
      const l = add(w.xy, mul(c, -width / 2)), r = add(w.xy, mul(c, width / 2));
      left.push(proj.inv(l[0], l[1])); right.push(proj.inv(r[0], r[1]));
      u.push(dot(w.xy, d)); half.push(width / 2);
    }
    // order along-track so interpolation works whichever way the line was flown
    const idx = u.map((_, i) => i).sort((a, b) => u[a] - u[b]);
    swaths.push({ line, left, right, u: idx.map(i => u[i]), half: idx.map(i => half[i]) });
  }

  // Overlap band between consecutive lines: across-track from (v₂ − half₂) to (v₁ + half₁), sampled at
  // both lines' along-track stations over the stretch they share.
  const interp = (s: Swath, x: number) => {
    const { u, half } = s;
    if (x <= u[0]) return half[0];
    if (x >= u[u.length - 1]) return half[half.length - 1];
    let i = 1; while (u[i] < x) i++;
    const t = (x - u[i - 1]) / (u[i] - u[i - 1]);
    return half[i - 1] + (half[i] - half[i - 1]) * t;
  };
  const overlaps: Overlap[] = [];
  const sorted = [...swaths].sort((a, b) => a.line - b.line);
  for (let n = 1; n < sorted.length; n++) {
    const A = sorted[n - 1], B = sorted[n];
    if (B.line !== A.line + 1) continue;
    const vA = plan.lines[A.line].v, vB = plan.lines[B.line].v;
    const lo = Math.max(A.u[0], B.u[0]), hi = Math.min(A.u[A.u.length - 1], B.u[B.u.length - 1]);
    if (hi <= lo) continue;
    const xs = [...new Set([...A.u, ...B.u].filter(x => x >= lo && x <= hi).concat([lo, hi]))].sort((p, q) => p - q);
    const near: LonLat[] = [], far: LonLat[] = [];
    let wMin = Infinity, wMax = -Infinity, pMin = Infinity, pMax = -Infinity;
    for (const x of xs) {
      const hA = interp(A, x), hB = interp(B, x);
      const farEdgeA = vA + hA, nearEdgeB = vB - hB;
      const w = Math.max(0, farEdgeA - nearEdgeB);
      const pct = 100 * w / Math.min(2 * hA, 2 * hB);
      wMin = Math.min(wMin, w); wMax = Math.max(wMax, w); pMin = Math.min(pMin, pct); pMax = Math.max(pMax, pct);
      const p1 = add(mul(d, x), mul(c, nearEdgeB)), p2 = add(mul(d, x), mul(c, Math.max(nearEdgeB, farEdgeA)));
      near.push(proj.inv(p1[0], p1[1])); far.push(proj.inv(p2[0], p2[1]));
    }
    overlaps.push({ lines: [A.line, B.line], poly: [...near, ...far.reverse()], widthMinM: wMin, widthMaxM: wMax, pctMin: pMin, pctMax: pMax });
  }

  return {
    plannedSidelapPct: o.sidelapPct,
    achievedMinPct: Math.min(...laps), achievedMaxPct: Math.max(...laps),
    achievedMeanPct: laps.reduce((s, v) => s + v, 0) / laps.length,
    swathMinM: Math.min(...widths), swathMaxM: Math.max(...widths),
    swaths, overlaps,
  };
}
