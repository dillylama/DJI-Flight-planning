import type { FlightWp } from './heights.ts';
import type { Plan } from './plan.ts';
import type { Route } from './route.ts';
import type { Issue } from './validate.ts';

// Hard floors and the clearance budget. Heights in the route file are absolute, so every error in the
// terrain model or in the aircraft's own height goes straight into terrain clearance. The planner
// guarantees the modelled clearance (AGL over the highest DEM cell in each leg's corridor); this module
// checks that enough is left after the things the model cannot know.
export const SAFETY = {
  corridorMinM: 30,            // never search less than one GLO-30 cell either side of the track
  aglMinM: 50,                 // lowest AGL the planner will plan with absolute heights on a 30 m DEM
  worstCaseErrorM: 30,         // refuse to export when less than this is left after the allowances
  worstCaseWarnM: 50,
  uncertaintyDefaultM: 20,     // DEM error (GLO-30: ~±4 m, more on slopes and canopy) + height without RTK (~±10 m)
  uncertaintyMinM: 5,          // never plan with less, whatever the DTM and positioning
  clearanceOverUncertaintyM: 20, // transit / RTH clearance must exceed the uncertainty by at least this
  minLegM: 2,                  // waypoints closer than this are treated as duplicates
  maxAboveHomeM: 1500,         // DJI height limit above the take-off point
  // Aircraft behaviour assumed where DJI publishes nothing. Deliberately weak, so the allowances are large;
  // to be replaced by measured values from the simulator and the first flight logs.
  brakeMs2: 2,                 // horizontal deceleration the aircraft can at least hold
  vertAccelMs2: 2,             // vertical acceleration it can at least hold when the flight path steepens
  turnInsideCorridor: 0.9,     // the rounded turn at a waypoint must stay inside this share of the corridor
};

export interface ClearanceBudget {
  aglM: number;                // planned: every straight leg clears the DEM corridor by at least this
  roundingM: number;           // worst geometric rounding of the flight path at a waypoint …
  lagM: number;                // … plus the height the aircraft can lose while it steepens its climb there
  pathM: number;               // the worst sum of the two at any one waypoint
  uncertaintyM: number;        // terrain + position allowance
  worstCaseM: number;          // what is left
}

export function clearanceBudget(plan: Plan, flight: Route<FlightWp>, uncertaintyM: number): ClearanceBudget {
  let pathM = 0, roundingM = 0, lagM = 0;
  for (const w of flight.wps) if (w.roundingM + w.lagM > pathM) { pathM = w.roundingM + w.lagM; roundingM = w.roundingM; lagM = w.lagM; }
  return { aglM: plan.o.aglM, roundingM, lagM, pathM, uncertaintyM, worstCaseM: plan.o.aglM - pathM - uncertaintyM };
}

export interface SafetyOptions { uncertaintyM: number; minClearanceM?: number }

export function safetyChecks(plan: Plan, flight: Route<FlightWp>, opts: SafetyOptions): Issue[] {
  const { o } = plan;
  const S = SAFETY, issues: Issue[] = [];
  const unc = opts.uncertaintyM;
  if (!(unc >= S.uncertaintyMinM)) issues.push({ severity: 'error', code: 'UNCERTAINTY', message: `The terrain + position uncertainty must be at least ${S.uncertaintyMinM} m (${S.uncertaintyDefaultM} m is the default for GLO-30).` });
  if (!(o.corridorM >= S.corridorMinM)) issues.push({ severity: 'error', code: 'CORRIDOR_MIN', message: `Terrain corridor ±${o.corridorM} m is below the ${S.corridorMinM} m minimum (one DEM cell either side of the track).` });
  if (!(o.aglM >= S.aglMinM)) issues.push({ severity: 'error', code: 'AGL_MIN', message: `AGL ${o.aglM} m is below the ${S.aglMinM} m minimum for absolute-height routes planned on a 30 m DEM. Use DJI Real-Time Follow for lower flights.` });
  const b = clearanceBudget(plan, flight, Number.isFinite(unc) ? unc : S.uncertaintyDefaultM);
  const detail = `${b.aglM} m AGL − ${b.pathM.toFixed(0)} m path rounding and lag at waypoints − ${b.uncertaintyM} m terrain and position uncertainty`;
  if (!(b.worstCaseM >= S.worstCaseErrorM)) issues.push({ severity: 'error', code: 'CLEARANCE_BUDGET', message: `Worst-case terrain clearance is only ${b.worstCaseM.toFixed(0)} m (${detail}); at least ${S.worstCaseErrorM} m is required. Fly higher, or lower the uncertainty only if you have a surveyed DTM and RTK fixed for the whole flight.` });
  else if (b.worstCaseM < S.worstCaseWarnM) issues.push({ severity: 'warn', code: 'CLEARANCE_BUDGET', message: `Worst-case terrain clearance is ${b.worstCaseM.toFixed(0)} m (${detail}). Obstacles that are not in the DEM (masts, cables, cranes) are not covered: check the area.` });
  if (opts.minClearanceM != null && !(opts.minClearanceM >= unc + S.clearanceOverUncertaintyM)) {
    issues.push({ severity: 'error', code: 'MIN_CLEARANCE', message: `Min terrain clearance for transit and RTH (${opts.minClearanceM} m) must be at least the uncertainty + ${S.clearanceOverUncertaintyM} m = ${unc + S.clearanceOverUncertaintyM} m.` });
  }
  return issues;
}
