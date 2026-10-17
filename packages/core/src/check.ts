// Mission checker: any DJI route file (Pilot 2, FlightHub 2 or this planner) measured against the DEM.
//
// The FlightHub 2 comparison of 15 Oct 2026 showed why this exists: set to 320 m AGL on its "global elevation
// data", FH2 flew 271 m above the ground directly under the line at the koppies and 248 m above ground 75 m beside
// it, while this planner's file held 320 m. A planned AGL in DJI's own terrain following is not a clearance. So:
//   readMissionFiles   waylines.wpml (+ template.kml) → waypoints with heights above mean sea level (EGM96), whatever
//                      height mode the file uses (WGS84 ellipsoidal, EGM96, or relative to the take-off point).
//   checkClearance     every leg sampled against the DEM: clearance over the ground under the line and over the
//                      highest ground in the ±corridor, against the AGL the mission was planned for.
//   previewDsm         what a different terrain model (see geotiff-write.ts dsmFromRaster) would do to the same route.
// This module never writes or edits a route: the flight plan is Pilot 2's / FlightHub 2's own (Luke, 17 Oct 2026);
// the only thing the checker hands back is a better surface for them to plan on. Nothing here prints coordinates.
import { ellipsoidToEgm96 } from 'egm96-universal';
import { makeProj, dist, type XY } from './geo.ts';
import { type ElevFn, corridorProfile, type Station } from './terrain.ts';
import { parseRouteFile, type ParsedRoute, type FileGroup } from './verify.ts';
import { readGeoTiff, rasterElev } from './dem.ts';

export interface CheckWp {
  src: number;              // index in the file this waypoint came from (inserted waypoints: the leg's start)
  lon: number; lat: number;
  h: number;                // height above mean sea level (EGM96)
  speed: number;            // DJI waypointSpeed: speed from here to the next waypoint
  turnMode: string; damping: number;
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

// ── What would a different DSM do to this mission? (a preview, never an edit of the route) ─────────────────────
// Pilot 2 builds the route from the terrain model it is given. First-order model: where the surface under the line
// changes by Δ, the route changes by Δ there, and keeps whatever it already does on top of the surface (its straight
// legs between waypoints, its smoothing). So a new surface `next` moves the stations of the check by
// next − current, and the clearance is measured again against the same corridor rule. `current` is the surface the
// mission was planned on (the DSM carried in the KMZ when there is one, else the DEM itself). It is a preview: the
// mission Pilot 2 makes from the new surface must be exported and checked again before it is flown.
export interface DsmPreview {
  shortShare: number;       // share of the route more than marginM short of the corridor + AGL
  lowestClearM: number;     // lowest height over the corridor ground
  aboveP50M: number; aboveP95M: number;   // height above corridor + AGL
  densityFactor: number;
}
export function previewDsm(report: CheckReport, current: ElevFn, next: ElevFn): DsmPreview | null {
  const ex: number[] = []; let short = 0, low = Infinity, n = 0;
  for (const s of report.stations) {
    const a = current(s.lon, s.lat), b = next(s.lon, s.lat);
    if (a == null || b == null) continue;
    const h = s.h + (b - a), clear = h - s.corridor;
    n++; if (clear < low) low = clear;
    if (clear < report.aglM - report.marginM) short++;
    ex.push(clear - report.aglM);
  }
  if (!n) return null;
  ex.sort((x, y) => x - y);
  const q = (p: number) => ex[Math.floor(p * (ex.length - 1))];
  return { shortShare: short / n, lowestClearM: low, aboveP50M: q(0.5), aboveP95M: q(0.95), densityFactor: report.aglM / Math.max(1, report.aglM + Math.max(0, q(0.5))) };
}

// The first surface in the list that has data at the point (a mission can carry several DSM tiles).
export function compositeElev(list: ElevFn[]): ElevFn {
  return (lon, lat) => { for (const e of list) { const v = e(lon, lat); if (v != null && Number.isFinite(v)) return v; } return null; };
}

// The terrain model(s) carried inside a mission KMZ (wpmz/res/dsm/*.tif: the ASTER tiles Pilot 2 planned on, or a DSM
// the user imported). DJI stores them as WGS84 ellipsoidal heights ("wgs84_" in the file name), so they are read as
// ellipsoidal and come back as heights above mean sea level, like every other surface here. Unreadable tiles are skipped.
export async function embeddedDsm(tiles: ArrayBuffer[]): Promise<ElevFn | null> {
  const list: ElevFn[] = [];
  for (const b of tiles) { try { list.push(rasterElev(await readGeoTiff(b, { verticalDatum: 'ellipsoidal' }))); } catch { /* skip */ } }
  return list.length ? compositeElev(list) : null;
}
