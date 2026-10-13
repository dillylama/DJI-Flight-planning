import { dist } from './geo.ts';
import type { FlightWp } from './heights.ts';
import type { Plan } from './plan.ts';
import type { Route } from './route.ts';
import { SAFETY } from './safety.ts';

export type Severity = 'error' | 'warn' | 'info';
export interface Issue { severity: Severity; code: string; message: string; wps?: number[] }

export interface ValidateOptions {
  maxWaypoints?: number | null;   // Pilot 2 / aircraft cap — unknown until the RC import test
  aglHighPct?: number;            // warn when line AGL exceeds nominal by more than this
}

// Turn damping as it is written to the file: the first and last waypoint are stop turns with none.
export const fileDamping = (wps: { dampingM: number }[], i: number) => (i === 0 || i === wps.length - 1 ? 0 : wps[i].dampingM);

// Errors block export; warnings need the pilot's eyes; info is context.
export function validate(plan: Plan, route: Route<FlightWp>, opts: ValidateOptions = {}): Issue[] {
  const { o } = plan;
  const { wps } = route;
  const aglHighPct = opts.aglHighPct ?? 30;
  const issues: Issue[] = [];

  // DJI: on every leg the two turn-damping distances together must be shorter than the leg.
  const badDamp: number[] = [], shortLeg: number[] = [];
  for (let i = 0; i < wps.length - 1; i++) {
    const L = dist(wps[i].xy, wps[i + 1].xy);
    if (L < SAFETY.minLegM) shortLeg.push(i);
    else if (fileDamping(wps, i) + fileDamping(wps, i + 1) >= L) badDamp.push(i);
  }
  if (shortLeg.length) issues.push({ severity: 'error', code: 'SHORT_LEG', message: `${shortLeg.length} leg(s) are shorter than ${SAFETY.minLegM} m (duplicate waypoints).`, wps: shortLeg });
  if (badDamp.length) issues.push({ severity: 'error', code: 'DAMPING', message: `On ${badDamp.length} leg(s) the turn damping of the two waypoints adds up to the leg length or more; DJI will reject the route.`, wps: badDamp });

  const lineWps = wps.map((w, i) => [w, i] as const).filter(([w]) => w.role === 'line');
  const low = lineWps.filter(([w]) => w.h - w.terrainUnderWp < o.aglM - 0.01).map(([, i]) => i);
  if (low.length) issues.push({ severity: 'error', code: 'AGL_LOW', message: `${low.length} line waypoint(s) below nominal AGL ${o.aglM} m.`, wps: low });

  const hiLimit = o.aglM * (1 + aglHighPct / 100);
  const high = lineWps.filter(([w]) => w.h - w.terrainUnderWp > hiLimit);
  if (high.length) {
    const worst = Math.max(...high.map(([w]) => w.h - w.terrainUnderWp));
    issues.push({
      severity: 'warn', code: 'AGL_HIGH',
      message: `${high.length} line waypoint(s) more than ${aglHighPct}% above nominal AGL (worst ${worst.toFixed(0)} m vs ${o.aglM} m). Coverage is safe (the swath widens) but point density and L3 range margin drop. ${o.verticalMode === 'raise' ? "Usual cause: lines crossing steep terrain with the climb gradient limit. Fly lines along the slope (Optimise course with terrain loaded), switch the vertical profile to 'follow terrain, slow on climbs', or tighten waypoint spacing." : 'Usual cause: a ridge or step inside a leg or the terrain corridor, so the waypoint is held up by the highest ground near it. Tighten waypoint spacing or fly lines along the slope.'}`,
      wps: high.map(([, i]) => i),
    });
  }

  if (opts.maxWaypoints != null && wps.length > opts.maxWaypoints) {
    issues.push({ severity: 'error', code: 'WP_CAP', message: `${wps.length} waypoints exceeds the cap of ${opts.maxWaypoints}. Split into sorties.` });
  } else if (opts.maxWaypoints == null) {
    issues.push({ severity: 'info', code: 'WP_CAP_UNKNOWN', message: `${wps.length} waypoints. The Pilot 2 / M400 waypoint cap is not confirmed yet (pending the RC import test).` });
  }

  const fig = wps.filter(w => w.role === 'fig8');
  if (fig.length > 1) {
    const leg = dist(fig[0].xy, fig[1].xy);
    const bank = Math.atan(route.speedMs ** 2 / (9.81 * route.fig8RadiusM)) * 180 / Math.PI;
    issues.push({
      severity: 'info', code: 'FIG8',
      message: `Figure-8 radius ${route.fig8RadiusM.toFixed(0)} m, ~${leg.toFixed(0)} m between points; at ${route.speedMs} m/s that is a ${bank.toFixed(0)}° bank, flown level. In waypoint mode the aircraft may slow through the loops; confirm the achieved speed and bank in the simulator.`,
    });
  }

  // The rounded turn at a waypoint must stay over ground the corridor search has covered.
  const wide = wps.map((w, i) => [w, i] as const).filter(([w]) => w.turnM > o.corridorM).map(([, i]) => i);
  if (wide.length) issues.push({ severity: 'error', code: 'TURN_CORRIDOR', message: `At ${wide.length} waypoint(s) the rounded turn can swing further off the legs than the ±${o.corridorM} m terrain corridor. Widen the corridor.`, wps: wide });

  if (o.verticalMode === 'slow') {
    // Terrain-following: slowed legs are expected. A leg is a problem when even the speed floor is too fast
    // for it. Checked at the fastest speed the aircraft can have on the leg: its own target, or a neighbour's.
    const steep: number[] = [];
    for (let i = 0; i < wps.length - 1; i++) {
      const d = dist(wps[i].xy, wps[i + 1].xy), dh = wps[i + 1].h - wps[i].h;
      if (d < SAFETY.minLegM || dh === 0) continue;                  // duplicate waypoints are reported above
      const v = Math.max(wps[i].speed, wps[i + 1].speed, i > 0 ? wps[i - 1].speed : 0);
      if ((Math.abs(dh) / d) * v > (dh > 0 ? o.climbMs : o.descentMs) * (1 + 1e-9)) steep.push(i);
    }
    if (steep.length) issues.push({ severity: 'error', code: 'TOO_STEEP', message: `${steep.length} leg(s) would need less than ${o.minLegSpeedMs} m/s to stay within the ${o.climbMs} / ${o.descentMs} m/s climb/descent limits: the terrain step is too sharp for this waypoint spacing. Raise AGL there, tighten waypoint spacing, or fly lines along the slope.`, wps: steep });
    const slowed = wps.filter((w, i) => i < wps.length - 1 && w.slowed).length;
    if (slowed) issues.push({ severity: 'info', code: 'SLOWED', message: `${slowed} leg(s) slowed below line speed to hold the ${o.climbMs} m/s climb / ${o.descentMs} m/s descent limits (see "Line speed" in the stats).` });
  } else {
    const slopeLegs: number[] = [];
    for (let i = 1; i < wps.length; i++) {
      const d = dist(wps[i].xy, wps[i - 1].xy);
      if (d > 0 && Math.abs(wps[i].h - wps[i - 1].h) / d > o.maxGradient + 1e-6) slopeLegs.push(i);
    }
    if (slopeLegs.length) issues.push({ severity: 'error', code: 'GRADIENT', message: `${slopeLegs.length} leg(s) exceed the ${(o.maxGradient * 100).toFixed(0)}% gradient limit.`, wps: slopeLegs });
  }

  return issues;
}
