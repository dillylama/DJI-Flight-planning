import { dist } from './geo.ts';
import type { FlightWp } from './heights.ts';
import type { Plan } from './plan.ts';
import type { Route } from './route.ts';

export interface RouteStats {
  areaHa: number;
  lines: number;
  swathM: number;
  spacingM: number;
  waypoints: number;
  routeKm: number;
  dataKm: number;
  flightMin: number;
  fig8RadiusM: number;
  hMin: number;
  hMax: number;
  aglOnLineMin: number;
  aglOnLineMax: number;
}

export function stats(plan: Plan, route: Route<FlightWp>): RouteStats {
  const { wps } = route;
  let len = 0, dataLen = 0;
  for (let i = 1; i < wps.length; i++) {
    const dd = dist(wps[i].xy, wps[i - 1].xy);
    len += dd;
    if (wps[i].role === 'line' && wps[i - 1].role === 'line') dataLen += dd;
  }
  const aglUnder = wps.filter(w => w.role === 'line').map(w => w.h - w.terrainUnderWp);
  return {
    areaHa: plan.areaHa, lines: plan.lines.length, swathM: plan.swath, spacingM: plan.spacing,
    waypoints: wps.length, routeKm: len / 1000, dataKm: dataLen / 1000,
    flightMin: len / route.speedMs / 60, fig8RadiusM: route.fig8RadiusM,
    hMin: Math.min(...wps.map(w => w.h)), hMax: Math.max(...wps.map(w => w.h)),
    aglOnLineMin: Math.min(...aglUnder), aglOnLineMax: Math.max(...aglUnder),
  };
}
