import { type LonLat, D2R, add, mul } from './geo.ts';
import type { FlightWp } from './heights.ts';
import type { Plan } from './plan.ts';
import type { Route } from './route.ts';

export interface Swath { line: number; left: LonLat[]; right: LonLat[] }
export interface Coverage {
  plannedSidelapPct: number;
  achievedMinPct: number;
  achievedMeanPct: number;
  achievedMaxPct: number;
  swathMinM: number;
  swathMaxM: number;
  swaths: Swath[];
}

// LiDAR strip coverage on the data lines. Swath at each line waypoint = 2 · AGL · tan(FOV/2), using
// that waypoint's actual AGL. Sidelap with the neighbouring strip ≈ 1 − spacing / swath.
// Assumes level ground across-track: on a cross-slope the downhill edge reaches further and the uphill
// edge less, so treat these numbers as nominal.
export function coverage(plan: Plan, flight: Route<FlightWp>): Coverage {
  const { o, c, spacing, proj } = plan;
  const k = 2 * Math.tan((o.fovDeg / 2) * D2R);
  const byLine = new Map<number, FlightWp[]>();
  for (const w of flight.wps) if (w.role === 'line' && w.line != null) {
    if (!byLine.has(w.line)) byLine.set(w.line, []);
    byLine.get(w.line)!.push(w);
  }
  const laps: number[] = [], widths: number[] = [];
  const swaths: Swath[] = [];
  for (const [line, wps] of byLine) {
    const left: LonLat[] = [], right: LonLat[] = [];
    for (const w of wps) {
      const width = k * (w.h - w.terrainUnderWp);
      widths.push(width);
      laps.push(100 * (1 - spacing / width));
      const l = add(w.xy, mul(c, -width / 2)), r = add(w.xy, mul(c, width / 2));
      left.push(proj.inv(l[0], l[1])); right.push(proj.inv(r[0], r[1]));
    }
    swaths.push({ line, left, right });
  }
  return {
    plannedSidelapPct: o.sidelapPct,
    achievedMinPct: Math.min(...laps), achievedMaxPct: Math.max(...laps),
    achievedMeanPct: laps.reduce((s, v) => s + v, 0) / laps.length,
    swathMinM: Math.min(...widths), swathMaxM: Math.max(...widths),
    swaths,
  };
}
