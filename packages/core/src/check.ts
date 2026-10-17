// Mission checker: any DJI route file (Pilot 2, FlightHub 2 or this planner) measured against the DEM.
//
// The FlightHub 2 comparison of 15 Oct 2026 showed why this exists: set to 320 m AGL on its "global elevation
// data", FH2 flew 271 m above the ground directly under the line at the koppies and 248 m above ground 75 m beside
// it, while this planner's file held 320 m. A planned AGL in DJI's own terrain following is not a clearance. So:
//   readMissionFiles   waylines.wpml (+ template.kml) → waypoints with heights above mean sea level (EGM96), whatever
//                      height mode the file uses (WGS84 ellipsoidal, EGM96, or relative to the take-off point).
//   checkClearance     every leg sampled against the DEM: clearance over the ground under the line and over the
//                      highest ground in the ±corridor, against the AGL the mission was planned for.
//   liftRoute          the same waypoints with heights raised, waypoints inserted where a straight leg would still
//                      be short, and speeds capped with the measured climb/descent/braking model, ready for writeWpml.
// Nothing here prints coordinates.
import { ellipsoidToEgm96 } from 'egm96-universal';
import { makeProj, dist, type XY } from './geo.ts';
import { type ElevFn, corridorProfile, corridorMax, type Station } from './terrain.ts';
import { parseRouteFile, type ParsedRoute, type FileGroup } from './verify.ts';
import { STOP_TURN, type Route, type TurnMode } from './route.ts';
import type { FlightWp } from './heights.ts';
import { DEFAULTS } from './plan.ts';
import { SAFETY } from './safety.ts';
import type { WpmlOptions } from './wpml.ts';
import { L3_WPML } from './wpml.ts';

export interface CheckWp {
  src: number;              // index in the file this waypoint came from (inserted waypoints: the leg's start)
  lon: number; lat: number;
  h: number;                // height above mean sea level (EGM96)
  speed: number;            // DJI waypointSpeed: speed from here to the next waypoint
  turnMode: string; damping: number;
  inserted?: boolean;
  slowed?: boolean;
  speed0?: number;          // the file's speed before the lift's speed rule (so repeated passes start from it)
}

export interface MissionFile {
  wps: CheckWp[];
  heightMode: string;              // the waylines' executeHeightMode
  waylines: ParsedRoute;
  template: ParsedRoute | null;
  templateType: string | null;     // waypoint | mapping2d | mapping3d | mappingStrip | …
  recordStart: number | null;      // waypoint index where point-cloud recording starts (recordPointCloud startRecord)
  recordStop: number | null;
  imuCalibration: boolean;         // aircraftCalibration actions present (Pilot 2 "IMU calibration")
  rgbSpacingM: number | null;      // takePhoto on a multipleDistance trigger: its spacing
  lidar: { samplingRate: number; returnMode: string; scanningMode: string } | null;
  flightHeightM: number | null;    // the height the file was planned with, when the template says (prefill for the AGL)
  notes: string[];
}

const groupsOf = (r: ParsedRoute): FileGroup[] => r.waypoints.flatMap(w => w.groups);
const tag = (xml: string, name: string): string | null => new RegExp(`<wpml:${name}>\\s*([^<]*?)\\s*</wpml:${name}>`).exec(xml)?.[1] ?? null;

// takeoffGroundEgm96: ground height (EGM96) at the take-off point, needed when the file's heights are relative to the
// take-off point and the template does not carry the take-off reference point.
export function readMissionFiles(waylinesWpml: string, templateKml: string | null, takeoffGroundEgm96: number | null = null): MissionFile {
  const w = parseRouteFile(waylinesWpml, 'waylines');
  let t: ParsedRoute | null = null;
  try { t = templateKml ? parseRouteFile(templateKml, 'template') : null; } catch { t = null; }   // mapping templates have no waypoints; fine
  if (w.waypoints.length < 2) throw new Error('The route has fewer than 2 waypoints.');
  const notes: string[] = [];
  const mode = w.heightMode;
  let ref: number | null = null;   // EGM96 height of the take-off point when heights are relative to it
  if (mode === 'relativeToStartPoint') {
    const trp = templateKml ? tag(templateKml, 'takeOffRefPoint') : null;
    if (trp) {
      const [lat, lon, alt] = trp.split(',').map(Number);
      if ([lat, lon, alt].every(Number.isFinite)) { ref = ellipsoidToEgm96(lat, lon, alt); notes.push('Heights are relative to the take-off point in the file (takeOffRefPoint).'); }
    }
    if (ref == null && takeoffGroundEgm96 != null) { ref = takeoffGroundEgm96; notes.push('Heights are relative to the take-off point; the DEM ground at the given take-off position is used as the reference (its error adds to every height).'); }
    if (ref == null) throw new Error('The file\'s heights are relative to the take-off point (executeHeightMode relativeToStartPoint) and it carries no take-off reference point. Set the take-off position so the ground height there can be used.');
  } else if (mode !== 'WGS84' && mode !== 'EGM96') {
    throw new Error(`executeHeightMode "${mode}" is not understood (WGS84, EGM96 or relativeToStartPoint).`);
  }
  const wps: CheckWp[] = w.waypoints.map((p, i) => ({
    src: i, lon: p.lon, lat: p.lat,
    h: mode === 'WGS84' ? p.hEgm96 : mode === 'EGM96' ? p.hEllipsoid : ref! + p.hEllipsoid,   // hEllipsoid = executeHeight as written
    speed: p.speed, turnMode: p.turnMode, damping: Number.isFinite(p.damping) ? p.damping : 0,
  }));
  for (const [i, p] of wps.entries()) if (![p.lon, p.lat, p.h, p.speed].every(Number.isFinite) || !(p.speed > 0)) throw new Error(`Waypoint ${i + 1} has a missing or invalid position, height or speed.`);

  const groups = groupsOf(w);
  const find = (func: string, param?: [string, string]) => groups.find(g => g.actions.some(a => a.func === func && (!param || a.params[param[0]] === param[1])));
  const start = find('recordPointCloud', ['recordPointCloudOperate', 'startRecord']), stop = find('recordPointCloud', ['recordPointCloudOperate', 'stopRecord']);
  const photo = groups.find(g => g.trigger === 'multipleDistance' && g.actions.some(a => a.func === 'takePhoto'));
  const payload = templateKml ?? '';
  const sr = tag(payload, 'samplingRate'), rm = tag(payload, 'returnMode'), sm = tag(payload, 'scanningMode');
  const lidar = sr && rm && sm ? { samplingRate: +sr, returnMode: rm, scanningMode: sm } : null;
  const templateType = templateKml ? tag(templateKml, 'templateType') : null;
  const fh = templateKml ? tag(templateKml, 'height') ?? (t && Number.isFinite(t.globalHeight) ? String(t.globalHeight) : null) : null;
  if (templateType && templateType !== 'waypoint') notes.push(`The template is a "${templateType}" mission: the checked waypoints are the ones Pilot 2 / FlightHub 2 generated into waylines.wpml.`);
  return {
    wps, heightMode: mode, waylines: w, template: t, templateType,
    recordStart: start ? start.start : null, recordStop: stop ? stop.start : null,
    imuCalibration: !!find('aircraftCalibration') || (t?.caliFlight === 1),
    rgbSpacingM: photo && Number.isFinite(photo.triggerParam) && photo.triggerParam > 0 ? photo.triggerParam : null,
    lidar, flightHeightM: fh != null && Number.isFinite(+fh) ? +fh : null, notes,
  };
}

export interface CheckOptions {
  aglM: number;            // the AGL the mission was planned for
  corridorM: number;       // lateral half-width searched for the highest ground (planner default 75 m)
  marginM?: number;        // shortfall tolerated before a leg is reported (default 10 m, the G4 criterion)
  stepM?: number;          // along-track sample spacing (default 10 m; never coarser than the DEM cell)
}
export interface CheckStation { s: number; leg: number; lon: number; lat: number; h: number; under: number; corridor: number }
export interface LegReport { leg: number; lengthM: number; minClearM: number; minClearAtM: number; minUnderM: number; shortM: number }
export interface CheckReport {
  aglM: number; corridorM: number; marginM: number;
  lengthM: number;
  stations: CheckStation[];     // the whole route, s = distance along it
  legs: LegReport[];
  minClearM: number;            // over the corridor
  minUnderM: number;            // over the ground directly under the line
  nShort: number;               // legs below aglM − marginM over the corridor
  shortLengthM: number;         // route length flown short
  worstLeg: number | null;
  // Too high: height above (corridor max + AGL) along the route — what a DSM's dilation, a margin and DJI's own
  // smoothing cost in density. densityFactor ≈ aglM / (aglM + median excess): LiDAR density falls about 1/height.
  aboveM: { p50: number; p95: number; max: number };
  densityFactor: number;
}

function legStations(elev: ElevFn, proj: ReturnType<typeof makeProj>, a: CheckWp, b: CheckWp, corridorM: number, stepM: number): { corr: Station[]; under: Station[]; A: XY; B: XY } {
  const A = proj.fwd(a.lon, a.lat), B = proj.fwd(b.lon, b.lat);
  return { corr: corridorProfile(elev, proj, A, B, corridorM, stepM), under: corridorProfile(elev, proj, A, B, 0, stepM), A, B };
}

export function checkClearance(wps: CheckWp[], elev: ElevFn, o: CheckOptions): CheckReport {
  if (wps.length < 2) throw new Error('A route needs at least 2 waypoints.');
  const marginM = o.marginM ?? 10, stepM = o.stepM ?? 10;
  const proj = makeProj(wps[0].lat, wps[0].lon);
  const stations: CheckStation[] = []; const legs: LegReport[] = [];
  let s0 = 0, minClear = Infinity, minUnder = Infinity, shortLen = 0, worst: number | null = null;
  for (let i = 0; i < wps.length - 1; i++) {
    const a = wps[i], b = wps[i + 1];
    const { corr, under, A, B } = legStations(elev, proj, a, b, o.corridorM, stepM);
    const L = dist(A, B);
    let lc = Infinity, lcAt = 0, lu = Infinity;
    for (let k = 0; k < corr.length; k++) {
      const f = L > 0 ? corr[k].s / L : 0, h = a.h + f * (b.h - a.h);
      const [lon, lat] = proj.inv(corr[k].p[0], corr[k].p[1]);
      const c = h - corr[k].max, u = h - (under[k]?.max ?? corr[k].max);
      stations.push({ s: s0 + corr[k].s, leg: i, lon, lat, h, under: under[k]?.max ?? corr[k].max, corridor: corr[k].max });
      if (c < lc) { lc = c; lcAt = corr[k].s; }
      if (u < lu) lu = u;
    }
    const shortM = Math.max(0, o.aglM - marginM - lc);
    legs.push({ leg: i, lengthM: L, minClearM: lc, minClearAtM: lcAt, minUnderM: lu, shortM });
    if (lc < minClear) { minClear = lc; worst = i; }
    if (lu < minUnder) minUnder = lu;
    if (shortM > 0) shortLen += L;
    s0 += L;
  }
  const ex = stations.map(st => st.h - st.corridor - o.aglM).sort((a, b) => a - b);
  const q = (p: number) => (ex.length ? ex[Math.floor(p * (ex.length - 1))] : 0);
  const aboveM = { p50: q(0.5), p95: q(0.95), max: q(1) };
  return { aglM: o.aglM, corridorM: o.corridorM, marginM, lengthM: s0, stations, legs, minClearM: minClear, minUnderM: minUnder, nShort: legs.filter(l => l.shortM > 0).length, shortLengthM: shortLen, worstLeg: worst,
    aboveM, densityFactor: o.aglM / Math.max(1, o.aglM + Math.max(0, aboveM.p50)) };
}

// Heights rebuilt to the planned AGL over the DEM, the planner's own way (heights.ts): dense vertices every
// `spacingM` along each leg plus the file's waypoints; each vertex AGL above the highest ground in the corridors of
// its two dense legs, every leg extended `ext` beyond both ends (the ground under a turn, and the stopping
// distance), so every leg clears by construction; then thinned: a vertex is dropped when the straight line between
// its neighbours stays between the dense heights and dense + excessM (the planner's thinning tolerance), so the
// route hugs terrain + AGL without a waypoint at every DEM cell. Speeds: the measured aircraft model (climb 3.5 /
// descent 2.5 m/s, braking 2 m/s² into a slower leg). The file's own heights are replaced; with keepHigher a file
// waypoint above the requirement keeps its height (a deliberate climb over an obstacle).
export interface LiftOptions extends CheckOptions { spacingM?: number; excessM?: number; keepHigher?: boolean; climbMs?: number; descentMs?: number; brakeMs2?: number }
export function liftRoute(wps: CheckWp[], elev: ElevFn, o: LiftOptions): CheckWp[] {
  if (wps.length < 2) throw new Error('A route needs at least 2 waypoints.');
  const spacing = o.spacingM ?? 50, excess = o.excessM ?? DEFAULTS.thinToleranceM, stepM = o.stepM ?? 10;
  const climb = o.climbMs ?? DEFAULTS.climbMs, desc = o.descentMs ?? DEFAULTS.descentMs, brake = o.brakeMs2 ?? SAFETY.brakeMs2;
  const proj = makeProj(wps[0].lat, wps[0].lon);
  const vTop = Math.max(...wps.map(w => w.speed));
  const ext = Math.max(o.corridorM, (vTop * vTop) / (2 * brake));
  // 1. dense vertices
  type V = CheckWp & { xy: XY; orig: boolean };
  const dense: V[] = [];
  for (let i = 0; i < wps.length - 1; i++) {
    const a = wps[i], b = wps[i + 1], A = proj.fwd(a.lon, a.lat), B = proj.fwd(b.lon, b.lat), L = dist(A, B);
    const n = Math.max(1, Math.ceil(L / spacing));
    for (let k = 0; k < n; k++) {
      if (k === 0) { dense.push({ ...a, xy: A, orig: true, speed: a.speed0 ?? a.speed, speed0: a.speed0 ?? a.speed, slowed: false }); continue; }
      const xy: XY = [A[0] + ((B[0] - A[0]) * k) / n, A[1] + ((B[1] - A[1]) * k) / n];
      const [lon, lat] = proj.inv(xy[0], xy[1]);
      dense.push({ src: a.src, lon, lat, h: NaN, speed: a.speed0 ?? a.speed, speed0: a.speed0 ?? a.speed, turnMode: 'toPointAndPassWithContinuityCurvature', damping: a.damping, inserted: true, xy, orig: false });
    }
  }
  const last = wps[wps.length - 1];
  dense.push({ ...last, xy: proj.fwd(last.lon, last.lat), orig: true, speed: last.speed0 ?? last.speed, speed0: last.speed0 ?? last.speed, slowed: false });
  // 2. heights: AGL over the corridor max of both adjacent dense legs (each extended by ext)
  const legMax = dense.slice(0, -1).map((v, i) => corridorMax(elev, proj, v.xy, dense[i + 1].xy, o.corridorM, ext, stepM));
  const req = dense.map((_, i) => Math.max(i > 0 ? legMax[i - 1] : -Infinity, i < legMax.length ? legMax[i] : -Infinity) + o.aglM);
  dense.forEach((v, i) => { v.h = o.keepHigher && v.orig && Number.isFinite(wps.find(w => w.src === v.src)?.h ?? NaN) ? Math.max(req[i], wps.find(w => w.src === v.src)!.h) : req[i]; });
  // 3. thinning: keep the file's waypoints; drop dense vertices while the straight line stays within [dense, dense + excess]
  const s = dense.map((v, i) => (i === 0 ? 0 : dist(dense[i - 1].xy, v.xy)));
  for (let i = 1; i < s.length; i++) s[i] += s[i - 1];
  const keep: number[] = [0];
  let k0 = 0;
  while (k0 < dense.length - 1) {
    let k1 = k0 + 1;
    for (let k = k0 + 2; k < dense.length; k++) {
      let ok = true;
      for (let j = k0 + 1; j < k && ok; j++) {
        const f = (s[j] - s[k0]) / Math.max(1e-9, s[k] - s[k0]), h = dense[k0].h + f * (dense[k].h - dense[k0].h);
        if (h < dense[j].h - 0.02 || h > dense[j].h + excess) ok = false;
      }
      if (!ok) break;
      k1 = k;
      if (dense[k].orig) break;   // the file's own waypoints always stay
    }
    keep.push(k1); k0 = k1;
  }
  const out: CheckWp[] = keep.map(i => { const { xy: _xy, orig: _o, ...w } = dense[i]; return w; });
  // 4. dampings inside the legs, speeds by the measured model
  const xy = keep.map(i => dense[i].xy);
  const D = xy.slice(0, -1).map((p, i) => dist(p, xy[i + 1]));
  for (let i = 0; i < out.length; i++) {
    const minLeg = Math.min(i > 0 ? D[i - 1] : Infinity, i < D.length ? D[i] : Infinity);
    if (out[i].turnMode !== STOP_TURN && Number.isFinite(minLeg)) out[i].damping = Math.max(1, Math.min(out[i].damping || 1, 0.45 * minLeg));
  }
  // 1 % under the rate limits: the independent check measures leg lengths with its own distance model
  const lim = D.map((d, i) => {
    const dh = out[i + 1].h - out[i].h;
    const vRate = dh > 0 ? (0.99 * climb * d) / dh : dh < 0 ? (0.99 * desc * d) / -dh : Infinity;
    return Math.min(out[i].speed, vRate);
  });
  const floor1 = (v: number) => Math.max(1, Math.floor(v * 10 + 1e-9) / 10);   // written to 0.1 m/s, never rounded up
  const cap = new Array<number>(D.length);
  for (let i = D.length - 1; i >= 0; i--) cap[i] = floor1(Math.min(lim[i], i < D.length - 1 ? Math.sqrt(cap[i + 1] ** 2 + 2 * brake * D[i]) : Infinity));
  for (let i = 0; i < D.length; i++) {
    if (cap[i] < out[i].speed - 0.05) { out[i].slowed = true; out[i].speed = cap[i]; }
  }
  return out;
}

// The lifted waypoints as a route for writeWpml (this planner's own, proven file layout): one data line, recording
// from the file's start-record waypoint to its stop-record waypoint (or first to last), the file's speeds and turn
// modes. FlightHub 2 / Pilot 2 pause–resume actions between lines are not carried over: recording runs through.
export function liftedRoute(mission: MissionFile, wps: CheckWp[], elev: ElevFn, corridorM: number): Route<FlightWp> {
  const proj = makeProj(wps[0].lat, wps[0].lon);
  const up = elev.upper ?? elev;
  const startSrc = mission.recordStart ?? 0, stopSrc = mission.recordStop ?? mission.wps.length - 1;
  const firstOf = (src: number) => wps.findIndex(w => w.src === src && !w.inserted);
  let iStart = firstOf(startSrc), iStop = firstOf(stopSrc);
  if (iStart < 0) iStart = 0; if (iStop < 0 || iStop <= iStart) iStop = wps.length - 1;
  const turn = (m: string): TurnMode => /Stop/.test(m) ? STOP_TURN : m === 'coordinateTurn' ? 'coordinateTurn' : 'toPointAndPassWithContinuityCurvature';
  const out: FlightWp[] = wps.map((w, i) => {
    const xy = proj.fwd(w.lon, w.lat);
    const ground = up(w.lon, w.lat) ?? w.h;
    const A = i > 0 ? proj.fwd(wps[i - 1].lon, wps[i - 1].lat) : xy, B = i < wps.length - 1 ? proj.fwd(wps[i + 1].lon, wps[i + 1].lat) : xy;
    const tm = Math.max(...corridorProfile(elev, proj, A, xy, corridorM, 10).map(s => s.max), ...corridorProfile(elev, proj, xy, B, corridorM, 10).map(s => s.max));
    return {
      xy, lon: w.lon, lat: w.lat, h: w.h, speed: w.speed, role: 'line', line: 0, actions: [...(i === iStart ? ['START_RECORD' as const] : []), ...(i === iStop ? ['STOP_RECORD' as const] : [])],
      dampingM: i === 0 || i === wps.length - 1 ? 0 : Math.max(1, w.damping), turnMode: i === 0 || i === wps.length - 1 ? STOP_TURN : turn(w.turnMode),
      terrainMax: tm, terrainUnderWp: ground, slowed: w.slowed, roundingM: 0, lagM: 0, turnM: 0,
    };
  });
  return { wps: out, startIdx: 0, fig8RadiusM: 0, speedMs: Math.max(...out.map(w => w.speed)) };
}

// writeWpml options that keep the mission's own settings (take-off, signal loss, L3 parameters, calibration, photos).
export function wpmlOptionsFor(mission: MissionFile): WpmlOptions {
  const w = mission.waylines;
  const lidar = mission.lidar && (L3_WPML.returnModes as readonly string[]).includes(mission.lidar.returnMode) && (L3_WPML.scanningModes as readonly string[]).includes(mission.lidar.scanningMode)
    && (L3_WPML.samplingRates as readonly number[]).includes(mission.lidar.samplingRate)
    ? { samplingRate: mission.lidar.samplingRate, returnMode: mission.lidar.returnMode as typeof L3_WPML.returnModes[number], scanningMode: mission.lidar.scanningMode as typeof L3_WPML.scanningModes[number], modelColoring: false }
    : null;
  const takeoff: WpmlOptions['takeoff'] = { flyToMode: w.flyToWaylineMode === 'safely' ? 'safely' : 'pointToPoint', rcLost: w.exitOnRCLost === 'goContinue' ? 'continue' : 'goBack' };
  if (Number.isFinite(w.takeOffSecurityHeight) && w.takeOffSecurityHeight >= 1.2) takeoff.takeoffSecurityM = w.takeOffSecurityHeight;
  if (Number.isFinite(w.transitSpeed) && w.transitSpeed > 0) takeoff.transitSpeedMs = w.transitSpeed;
  return {
    takeoff,
    payloadPositionIndex: Number.isFinite(w.payloadPosition) ? w.payloadPosition : 0,
    lidar,
    djiImuCalibration: mission.imuCalibration,
    rgbPhotoSpacingM: mission.rgbSpacingM,
  };
}
