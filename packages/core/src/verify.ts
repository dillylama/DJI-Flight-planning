import { egm96ToEllipsoid, ellipsoidToEgm96 } from 'egm96-universal';
import type { ElevFn } from './terrain.ts';
import type { Issue } from './validate.ts';

// Independent check of an exported route file against the terrain. It deliberately shares NO code with the
// planner: it parses the XML that would be flown, converts the ellipsoidal heights back to EGM96, lays its
// own stations along every leg with its own geodesy, and looks up the terrain itself. If the planner, the
// sortie splitter or the writer ever put a waypoint in the wrong place, at the wrong height or in the
// wrong datum, this is where it shows. Export is refused unless it passes.
//
// What it cannot check: the terrain model itself (it reads the same DEM), and what the aircraft does with
// the file. Those are covered by the uncertainty allowance and by the import / simulator / flight gates.

// ── a minimal XML reader (elements and text only; enough for KML/WPML) ─────────────────────────────
interface Node { tag: string; text: string; kids: Node[] }
function parseXml(xml: string): Node {
  const root: Node = { tag: '#root', text: '', kids: [] };
  const stack: Node[] = [root];
  const re = /<!--[\s\S]*?-->|<\?[\s\S]*?\?>|<(\/?)([A-Za-z_][\w:.-]*)([^<>]*?)(\/?)>|([^<]+)/g;
  let m: RegExpExecArray | null;
  while ((m = re.exec(xml))) {
    if (m[2] === undefined) { if (m[5] !== undefined) stack[stack.length - 1].text += m[5]; continue; }
    const tag = m[2];
    if (m[1]) {
      if (stack.length < 2 || stack[stack.length - 1].tag !== tag) throw new Error(`Malformed XML near </${tag}>`);
      stack.pop();
    } else {
      const node: Node = { tag, text: '', kids: [] };
      stack[stack.length - 1].kids.push(node);
      if (!m[4]) stack.push(node);
    }
  }
  if (stack.length !== 1) throw new Error(`Malformed XML: <${stack[stack.length - 1].tag}> is not closed`);
  return root;
}
const local = (tag: string) => tag.replace(/^[\w.-]+:/, '');
const kids = (n: Node | undefined, name: string) => (n ? n.kids.filter(k => local(k.tag) === name) : []);
const kid = (n: Node | undefined, name: string) => kids(n, name)[0];
const txt = (n: Node | undefined, name: string) => kid(n, name)?.text.trim();
const num = (n: Node | undefined, name: string) => { const t = txt(n, name); return t === undefined || t === '' ? NaN : Number(t); };
function descend(n: Node | undefined, ...path: string[]): Node | undefined { for (const p of path) n = kid(n, p); return n; }

// ── WGS84 geodesy, local to this file ─────────────────────────────────────────────────────────────
const A = 6378137, F = 1 / 298.257223563, E2 = F * (2 - F), RAD = Math.PI / 180;
function metresPerDegree(latDeg: number): { lat: number; lon: number } {
  const s = Math.sin(latDeg * RAD), w = Math.sqrt(1 - E2 * s * s);
  return { lat: (A * (1 - E2)) / (w * w * w) * RAD, lon: (A / w) * Math.cos(latDeg * RAD) * RAD };
}

export interface FileAction { func: string; params: Record<string, string> }
export interface FileGroup { id: number; start: number; end: number; trigger: string; triggerParam: number; actions: FileAction[] }
export interface FileWaypoint {
  index: number; lon: number; lat: number;
  hEllipsoid: number;        // as written (waylines: executeHeight, template: ellipsoidHeight)
  hEgm96: number;            // waylines: converted back; template: `height` as written
  speed: number; turnMode: string; damping: number; straight: number;
  groups: FileGroup[];
}
export interface ParsedRoute {
  namespace: string;
  drone: number; payload: number; payloadPosition: number;
  flyToWaylineMode: string; finishAction: string; exitOnRCLost: string; rcLostAction: string;
  takeOffSecurityHeight: number; transitSpeed: number;
  heightMode: string;        // waylines: executeHeightMode; template: heightMode
  autoFlightSpeed: number;
  globalHeight: number;      // template only
  startActions: string[];
  waypoints: FileWaypoint[];
}

function parseGroups(pm: Node): FileGroup[] {
  return kids(pm, 'actionGroup').map(g => ({
    id: num(g, 'actionGroupId'), start: num(g, 'actionGroupStartIndex'), end: num(g, 'actionGroupEndIndex'),
    trigger: txt(kid(g, 'actionTrigger'), 'actionTriggerType') ?? '', triggerParam: num(kid(g, 'actionTrigger'), 'actionTriggerParam'),
    actions: kids(g, 'action').map(a => ({
      func: txt(a, 'actionActuatorFunc') ?? '',
      params: Object.fromEntries((kid(a, 'actionActuatorFuncParam')?.kids ?? []).map(p => [local(p.tag), p.text.trim()])),
    })),
  }));
}

export function parseRouteFile(xml: string, kind: 'waylines' | 'template'): ParsedRoute {
  const doc = descend(parseXml(xml), 'kml', 'Document');
  if (!doc) throw new Error('Not a KML/WPML document');
  const mc = kid(doc, 'missionConfig'), folder = kid(doc, 'Folder');
  if (!mc || !folder) throw new Error('missionConfig or Folder is missing');
  if (kids(doc, 'Folder').length !== 1) throw new Error('More than one wayline folder');
  const globalTurn = txt(folder, 'globalWaypointTurnMode') ?? '', globalStraight = num(folder, 'globalUseStraightLine');
  const waypoints: FileWaypoint[] = kids(folder, 'Placemark').map(pm => {
    const c = (txt(kid(pm, 'Point'), 'coordinates') ?? '').split(',').map(s => Number(s.trim()));
    const turn = kid(pm, 'waypointTurnParam');
    if (kind === 'waylines') {
      const hEllipsoid = num(pm, 'executeHeight');
      return {
        index: num(pm, 'index'), lon: c[0], lat: c[1], hEllipsoid, hEgm96: ellipsoidToEgm96(c[1], c[0], hEllipsoid),
        speed: num(pm, 'waypointSpeed'), turnMode: txt(turn, 'waypointTurnMode') ?? '', damping: num(turn, 'waypointTurnDampingDist'),
        straight: num(pm, 'useStraightLine'), groups: parseGroups(pm),
      };
    }
    const useGlobalH = num(pm, 'useGlobalHeight') === 1, useGlobalV = num(pm, 'useGlobalSpeed') === 1, useGlobalT = num(pm, 'useGlobalTurnParam') === 1;
    const sl = num(pm, 'useStraightLine');
    return {
      index: num(pm, 'index'), lon: c[0], lat: c[1], hEllipsoid: num(pm, 'ellipsoidHeight'),
      hEgm96: useGlobalH ? num(folder, 'globalHeight') : num(pm, 'height'),
      speed: useGlobalV ? num(folder, 'autoFlightSpeed') : num(pm, 'waypointSpeed'),
      turnMode: useGlobalT ? globalTurn : (txt(turn, 'waypointTurnMode') ?? ''), damping: useGlobalT ? NaN : num(turn, 'waypointTurnDampingDist'),
      straight: Number.isNaN(sl) ? globalStraight : sl, groups: parseGroups(pm),
    };
  });
  const csys = kid(folder, 'waylineCoordinateSysParam');
  return {
    namespace: /xmlns:wpml="([^"]+)"/.exec(xml)?.[1] ?? '',
    drone: num(kid(mc, 'droneInfo'), 'droneEnumValue'), payload: num(kid(mc, 'payloadInfo'), 'payloadEnumValue'),
    payloadPosition: num(kid(mc, 'payloadInfo'), 'payloadPositionIndex'),
    flyToWaylineMode: txt(mc, 'flyToWaylineMode') ?? '', finishAction: txt(mc, 'finishAction') ?? '',
    exitOnRCLost: txt(mc, 'exitOnRCLost') ?? '', rcLostAction: txt(mc, 'executeRCLostAction') ?? '',
    takeOffSecurityHeight: num(mc, 'takeOffSecurityHeight'), transitSpeed: num(mc, 'globalTransitionalSpeed'),
    heightMode: kind === 'waylines' ? (txt(folder, 'executeHeightMode') ?? '') : (txt(csys, 'heightMode') ?? ''),
    autoFlightSpeed: num(folder, 'autoFlightSpeed'), globalHeight: num(folder, 'globalHeight'),
    startActions: kids(kid(folder, 'startActionGroup'), 'action').map(a => txt(a, 'actionActuatorFunc') ?? ''),
    waypoints,
  };
}

export interface VerifyOptions {
  elev: ElevFn;
  aglM: number;            // the AGL the route was planned for: every leg must clear the terrain by this
  corridorM: number;       // the planner's corridor; checked here over 95 % of it
  maxClimbMs: number;      // no leg may demand more than these at the fastest speed written on or next to it
  maxDescentMs: number;
  maxSpeedMs: number;
  lidar: boolean;          // expect one startRecord on the first waypoint and one stopRecord on the last
  tolM?: number;           // numerical tolerance on heights (default 0.05 m)
  vertAccelMs2?: number;   // vertical acceleration assumed for the pull-up allowance (default 2 m/s²)
}

export interface VerifyReport {
  ok: boolean;
  issues: Issue[];
  waypoints: number;
  lengthM: number;
  minClearanceM: number;                 // straight legs over the highest DEM cell within the corridor
  minClearanceAt: { leg: number; lon: number; lat: number } | null;
  maxRoundingM: number;                  // most the rounded path can sag below the legs, from the written damping
  maxLagM: number;                       // most the aircraft can drop below a leg while it steepens its climb
  maxPathErrorM: number;                 // worst rounding + lag at any one waypoint
  maxTurnM: number;                      // most the rounded turn can swing sideways off the legs
  maxClimbMs: number;
  maxDescentMs: number;
  minSpeedMs: number;
  maxSpeedMs: number;
  topEgm96M: number;
  lowEgm96M: number;
  finishAction: string;
  rcLostAction: string;                  // what the file tells the aircraft to do when the RC link is lost
  straightLegs: boolean;
  samples: number;
}

const STOP_MODES = ['toPointAndStopWithDiscontinuityCurvature', 'toPointAndStopWithContinuityCurvature'];
const TURN_MODES = [...STOP_MODES, 'toPointAndPassWithContinuityCurvature', 'coordinateTurn'];
const TRIGGERS = ['reachPoint', 'betweenAdjacentPoints', 'multipleDistance', 'multipleTiming'];
const FINISH_ACTIONS = ['goHome', 'noAction', 'autoLand', 'gotoFirstWaypoint'];
const RC_LOST = ['goBack', 'landing', 'hover'];

export function verifyRouteFiles(templateKml: string, waylinesWpml: string, opts: VerifyOptions): VerifyReport {
  const tol = opts.tolM ?? 0.05;
  const issues: Issue[] = [];
  const err = (code: string, message: string, wps?: number[]) => { issues.push({ severity: 'error', code, message, wps }); };
  const report: VerifyReport = {
    ok: false, issues, waypoints: 0, lengthM: 0, minClearanceM: Infinity, minClearanceAt: null,
    maxRoundingM: 0, maxLagM: 0, maxPathErrorM: 0, maxTurnM: 0, maxClimbMs: 0, maxDescentMs: 0,
    minSpeedMs: Infinity, maxSpeedMs: 0, topEgm96M: -Infinity, lowEgm96M: Infinity,
    finishAction: '', rcLostAction: '', straightLegs: false, samples: 0,
  };
  if (!(opts.aglM > 0) || !(opts.corridorM >= 0) || !(opts.maxClimbMs > 0) || !(opts.maxDescentMs > 0) || !(opts.maxSpeedMs > 0)) {
    err('V_FORMAT', 'The verifier was given unusable limits.'); return report;
  }
  let w: ParsedRoute, t: ParsedRoute;
  try { w = parseRouteFile(waylinesWpml, 'waylines'); t = parseRouteFile(templateKml, 'template'); }
  catch (e) { err('V_FORMAT', 'The route file could not be read back: ' + (e as Error).message); return report; }
  const P = w.waypoints, n = P.length;
  report.waypoints = n;
  report.finishAction = w.finishAction;
  report.rcLostAction = w.exitOnRCLost === 'goContinue' ? 'goContinue' : w.rcLostAction;

  // ── header
  for (const [name, f] of [['waylines.wpml', w], ['template.kml', t]] as const) {
    if (f.namespace !== 'http://www.dji.com/wpmz/1.0.6') err('V_HEADER', `${name}: unexpected WPML namespace "${f.namespace}".`);
    if (f.drone !== 103) err('V_HEADER', `${name}: droneEnumValue is ${f.drone}, expected 103 (M400).`);
    if (f.payload !== 117) err('V_HEADER', `${name}: payloadEnumValue is ${f.payload}, expected 117 (Zenmuse L3).`);
    if (!['safely', 'pointToPoint'].includes(f.flyToWaylineMode)) err('V_HEADER', `${name}: flyToWaylineMode "${f.flyToWaylineMode}".`);
    if (!(f.takeOffSecurityHeight >= 1.2 && f.takeOffSecurityHeight <= 1500)) err('V_HEADER', `${name}: takeOffSecurityHeight ${f.takeOffSecurityHeight} is outside 1.2–1500 m.`);
    if (!(f.transitSpeed > 0 && f.transitSpeed <= opts.maxSpeedMs)) err('V_HEADER', `${name}: globalTransitionalSpeed ${f.transitSpeed}.`);
    if (!(f.autoFlightSpeed > 0 && f.autoFlightSpeed <= opts.maxSpeedMs)) err('V_HEADER', `${name}: autoFlightSpeed ${f.autoFlightSpeed}.`);
    if (!FINISH_ACTIONS.includes(f.finishAction)) err('V_HEADER', `${name}: finishAction "${f.finishAction}".`);
    if (!['goContinue', 'executeLostAction'].includes(f.exitOnRCLost)) err('V_HEADER', `${name}: exitOnRCLost "${f.exitOnRCLost}".`);
    if (f.exitOnRCLost === 'executeLostAction' && !RC_LOST.includes(f.rcLostAction)) err('V_HEADER', `${name}: executeRCLostAction "${f.rcLostAction}".`);
  }
  if (w.finishAction !== t.finishAction || w.exitOnRCLost !== t.exitOnRCLost || w.rcLostAction !== t.rcLostAction || w.flyToWaylineMode !== t.flyToWaylineMode
    || w.takeOffSecurityHeight !== t.takeOffSecurityHeight || w.transitSpeed !== t.transitSpeed || w.payloadPosition !== t.payloadPosition) {
    err('V_TEMPLATE', 'template.kml and waylines.wpml disagree on the mission settings (take-off, finish or signal-loss behaviour).');
  }
  if (w.heightMode !== 'WGS84') err('V_HEIGHT_MODE', `waylines.wpml executeHeightMode is "${w.heightMode}"; heights are written as WGS84 ellipsoidal.`);
  if (t.heightMode !== 'EGM96') err('V_HEIGHT_MODE', `template.kml heightMode is "${t.heightMode}"; heights are written as EGM96.`);
  if (n < 2) { err('V_FORMAT', 'Fewer than 2 waypoints in waylines.wpml.'); return report; }

  // ── waypoints: indices, coordinates, speeds, turns
  P.forEach((p, i) => {
    if (p.index !== i) err('V_INDEX', `Waypoint ${i + 1} carries index ${p.index}.`, [i]);
    if (!(Math.abs(p.lat) <= 90) || !(Math.abs(p.lon) <= 180)) err('V_COORD', `Waypoint ${i + 1} has coordinates ${p.lon}, ${p.lat}.`, [i]);
    if (!Number.isFinite(p.hEllipsoid) || !Number.isFinite(p.hEgm96)) err('V_HEIGHT', `Waypoint ${i + 1} has no usable height.`, [i]);
    if (!(p.speed > 0) || p.speed > opts.maxSpeedMs + 1e-9) err('V_SPEED', `Waypoint ${i + 1} speed is ${p.speed} m/s (allowed: above 0, up to ${opts.maxSpeedMs}).`, [i]);
    if (!TURN_MODES.includes(p.turnMode)) err('V_TURN', `Waypoint ${i + 1} has turn mode "${p.turnMode}".`, [i]);
    if (!(p.damping >= 0)) err('V_DAMPING', `Waypoint ${i + 1} has turn damping ${p.damping}.`, [i]);
    if (p.straight !== 0 && p.straight !== 1) err('V_TURN', `Waypoint ${i + 1} has useStraightLine ${p.straight}.`, [i]);
    if (Number.isFinite(p.hEgm96)) { report.topEgm96M = Math.max(report.topEgm96M, p.hEgm96); report.lowEgm96M = Math.min(report.lowEgm96M, p.hEgm96); }
    if (p.speed > 0) { report.minSpeedMs = Math.min(report.minSpeedMs, p.speed); report.maxSpeedMs = Math.max(report.maxSpeedMs, p.speed); }
  });
  if (!STOP_MODES.includes(P[0].turnMode) || !STOP_MODES.includes(P[n - 1].turnMode)) err('V_TURN', 'The first and last waypoint must be stop turns, as Pilot 2 writes them.', [0, n - 1]);
  // Everything below treats the legs as straight lines. A fly-through waypoint with useStraightLine 0 makes
  // the whole path a free curve whose distance from the legs is not bounded by anything in the file.
  report.straightLegs = P.every((p, i) => p.straight === 1 || (STOP_MODES.includes(p.turnMode) && (i === 0 || i === n - 1)));
  if (!report.straightLegs) err('V_TURN', 'The file does not ask for straight legs (useStraightLine 1) on every waypoint, so its terrain clearance cannot be checked.');
  if (issues.length) return report;   // geometry below needs sane numbers

  // ── legs: length, damping sum, vertical rate, terrain clearance
  const up = opts.elev.upper ?? opts.elev;
  const cell = opts.elev.cell;
  const step = cell ? Math.min(10, 0.7 * Math.min(cell[0], cell[1])) : 10;   // never coarser than the DEM
  const half = 0.95 * opts.corridorM;
  const nAcross = half > 0 ? Math.max(1, Math.ceil(half / step)) : 0;
  const legLen: number[] = [], legDh: number[] = [], legDx: number[] = [], legDy: number[] = [];
  for (let i = 0; i < n - 1; i++) {
    const a = P[i], b = P[i + 1];
    const mpd = metresPerDegree((a.lat + b.lat) / 2);
    const dx = (b.lon - a.lon) * mpd.lon, dy = (b.lat - a.lat) * mpd.lat, L = Math.hypot(dx, dy), dh = b.hEgm96 - a.hEgm96;
    legLen.push(L); legDh.push(dh); legDx.push(dx); legDy.push(dy);
    report.lengthM += Math.hypot(L, dh);
    if (L < 2) { err('V_LEG', `Leg ${i + 1} is only ${L.toFixed(2)} m long (duplicate waypoints).`, [i]); continue; }
    const da = STOP_MODES.includes(a.turnMode) ? 0 : a.damping, db = STOP_MODES.includes(b.turnMode) ? 0 : b.damping;
    if (da + db >= Math.hypot(L, dh)) err('V_DAMPING', `Leg ${i + 1}: turn damping ${da} + ${db} m is not shorter than the ${Math.hypot(L, dh).toFixed(1)} m leg.`, [i]);
    // Vertical rate at the fastest speed the aircraft can have on this leg: its own, the one it arrives
    // with from the leg before, or the one it ramps towards for the leg after.
    const v = Math.max(a.speed, b.speed, i > 0 ? P[i - 1].speed : 0), rate = (dh / L) * v;
    if (rate > report.maxClimbMs) report.maxClimbMs = rate;
    if (-rate > report.maxDescentMs) report.maxDescentMs = -rate;
    if (rate > opts.maxClimbMs + 0.01) err('V_CLIMB', `Leg ${i + 1} demands a ${rate.toFixed(2)} m/s climb (limit ${opts.maxClimbMs}).`, [i]);
    if (-rate > opts.maxDescentMs + 0.01) err('V_DESCENT', `Leg ${i + 1} demands a ${(-rate).toFixed(2)} m/s descent (limit ${opts.maxDescentMs}).`, [i]);

    // Terrain under the leg and around both of its ends: stations along the leg and `half` beyond each
    // waypoint (where the turn is flown), each with offsets across the corridor.
    const ux = dx / L, uy = dy / L;
    const nAlong = Math.max(1, Math.ceil(L / step));
    for (let k = -nAcross; k <= nAlong + nAcross; k++) {
      const sM = k < 0 ? (k * half) / nAcross : k > nAlong ? L + ((k - nAlong) * half) / nAcross : (k * L) / nAlong;
      const s = sM / L, lon0 = a.lon + (b.lon - a.lon) * s, lat0 = a.lat + (b.lat - a.lat) * s;
      const hPath = a.hEgm96 + dh * Math.min(1, Math.max(0, s));
      for (let j = -nAcross; j <= nAcross; j++) {
        const off = nAcross ? (j * half) / nAcross : 0;
        const lon = lon0 + (off * uy) / mpd.lon, lat = lat0 + (off * -ux) / mpd.lat;
        const g = up(lon, lat);
        report.samples++;
        if (g == null || !Number.isFinite(g)) { err('V_NODATA', `The DEM has no data near leg ${i + 1} (${lat.toFixed(5)}, ${lon.toFixed(5)}).`, [i]); return report; }
        const c = hPath - g;
        if (c < report.minClearanceM) { report.minClearanceM = c; report.minClearanceAt = { leg: i, lon: lon0, lat: lat0 }; }
      }
    }
  }
  if (!(report.minClearanceM >= opts.aglM - tol)) {
    const at = report.minClearanceAt;
    err('V_CLEARANCE', at
      ? `The exported file clears the terrain by only ${report.minClearanceM.toFixed(1)} m on leg ${at.leg + 1} (${at.lat.toFixed(5)}, ${at.lon.toFixed(5)}); it was planned for ${opts.aglM} m. Do not fly this file.`
      : 'The terrain clearance of the exported file could not be established. Do not fly this file.', at ? [at.leg] : undefined);
  }

  // ── what the aircraft does at each fly-through waypoint, from the written damping, heights and speeds
  const accel = opts.vertAccelMs2 ?? 2;
  const vHi = (j: number) => Math.max(P[j].speed, P[j + 1].speed, j > 0 ? P[j - 1].speed : 0);
  const vLo = (j: number) => Math.min(P[j].speed, P[j + 1].speed, j > 0 ? P[j - 1].speed : Infinity);
  for (let i = 1; i < n - 1; i++) {
    if (STOP_MODES.includes(P[i].turnMode) || legLen[i - 1] < 2 || legLen[i] < 2) continue;
    if (P[i].turnMode === 'toPointAndPassWithContinuityCurvature' && !(P[i].damping > 0)) err('V_DAMPING', `Waypoint ${i + 1} is a fly-through turn with no damping distance.`, [i]);
    const g0 = legDh[i - 1] / legLen[i - 1], g1 = legDh[i] / legLen[i];
    const rounding = P[i].damping * Math.tan(Math.abs(Math.atan(g1) - Math.atan(g0)) / 2);
    const heading = Math.abs(Math.atan2(legDx[i - 1] * legDy[i] - legDy[i - 1] * legDx[i], legDx[i - 1] * legDx[i] + legDy[i - 1] * legDy[i]));
    const turn = P[i].damping * Math.min(1, Math.tan(heading / 2));
    const vzIn = g0 * (g0 < 0 ? vHi(i - 1) : vLo(i - 1)), vzOut = g1 * (g1 > 0 ? vHi(i) : vLo(i));
    const lag = vzOut > vzIn ? (vzOut - vzIn) ** 2 / (2 * accel) : 0;
    report.maxRoundingM = Math.max(report.maxRoundingM, rounding);
    report.maxLagM = Math.max(report.maxLagM, lag);
    report.maxPathErrorM = Math.max(report.maxPathErrorM, rounding + lag);
    report.maxTurnM = Math.max(report.maxTurnM, turn);
    if (turn > half + tol) err('V_TURN', `Waypoint ${i + 1}: the rounded turn can swing ${turn.toFixed(1)} m off the legs, outside the ±${half.toFixed(0)} m of terrain that was checked.`, [i]);
  }

  // ── actions
  const all = P.flatMap(p => p.groups);
  all.forEach((g, k) => {
    if (g.id !== k) err('V_ACTIONS', `Action group ids are not 0…n in order (found ${g.id} at position ${k}).`);
    if (!(g.start >= 0 && g.end >= g.start && g.end < n)) err('V_ACTIONS', `Action group ${g.id} spans waypoints ${g.start}–${g.end}, outside the route.`);
    if (!TRIGGERS.includes(g.trigger)) err('V_ACTIONS', `Action group ${g.id} has trigger "${g.trigger}".`);
    if (g.trigger.startsWith('multiple') && !(g.triggerParam > 0)) err('V_ACTIONS', `Action group ${g.id} has no trigger interval.`);
    if (!g.actions.length) err('V_ACTIONS', `Action group ${g.id} is empty.`);
  });
  P.forEach((p, i) => p.groups.forEach(g => { if (g.start !== i) err('V_ACTIONS', `Action group ${g.id} sits on waypoint ${i + 1} but starts at index ${g.start}.`, [i]); }));
  if (opts.lidar) {
    const rec = P.flatMap((p, i) => p.groups.flatMap(g => g.actions.filter(a => a.func === 'recordPointCloud').map(a => ({ i, op: a.params.recordPointCloudOperate }))));
    if (rec.length !== 2 || rec[0].op !== 'startRecord' || rec[0].i !== 0 || rec[1].op !== 'stopRecord' || rec[1].i !== n - 1) {
      err('V_ACTIONS', `Point-cloud recording must start on the first waypoint and stop on the last; the file has ${rec.map(r => `${r.op}@${r.i + 1}`).join(', ') || 'no record actions'}.`);
    }
  }

  // ── template.kml must describe the same route (Pilot 2 rebuilds waylines.wpml from it)
  const T = t.waypoints;
  if (T.length !== n) err('V_TEMPLATE', `template.kml has ${T.length} waypoints, waylines.wpml has ${n}.`);
  else {
    const bad = (i: number, what: string) => err('V_TEMPLATE', `template.kml and waylines.wpml disagree on waypoint ${i + 1}: ${what}.`, [i]);
    for (let i = 0; i < n; i++) {
      const a = T[i], b = P[i];
      if (a.index !== b.index) { bad(i, 'index'); break; }
      if (a.lon !== b.lon || a.lat !== b.lat) { bad(i, 'position'); break; }
      if (!(Math.abs(a.hEllipsoid - b.hEllipsoid) <= 1e-4)) { bad(i, `ellipsoid height ${a.hEllipsoid} vs ${b.hEllipsoid}`); break; }
      if (!(Math.abs(egm96ToEllipsoid(a.lat, a.lon, a.hEgm96) - a.hEllipsoid) <= 0.01)) { bad(i, `EGM96 height ${a.hEgm96} does not match its ellipsoid height ${a.hEllipsoid}`); break; }
      if (a.speed !== b.speed) { bad(i, `speed ${a.speed} vs ${b.speed}`); break; }
      if (a.turnMode !== b.turnMode || a.damping !== b.damping || a.straight !== b.straight) { bad(i, 'turn settings'); break; }
      if (JSON.stringify(a.groups) !== JSON.stringify(b.groups)) { bad(i, 'actions'); break; }
    }
  }
  // Fail-safe fallbacks: a reader that ignored the per-waypoint values would fly everything at the global
  // height and speed, so the height must clear every leg and the speed must suit the steepest one.
  if (!(t.globalHeight >= report.topEgm96M - tol)) err('V_TEMPLATE', `template.kml globalHeight ${t.globalHeight} is below the highest waypoint (${report.topEgm96M.toFixed(1)} m).`);
  if (!(t.autoFlightSpeed <= report.minSpeedMs + 1e-9) || t.autoFlightSpeed !== w.autoFlightSpeed) err('V_TEMPLATE', `The global speed (${t.autoFlightSpeed} / ${w.autoFlightSpeed} m/s) must be the slowest waypoint speed (${report.minSpeedMs} m/s) in both files.`);

  report.ok = issues.length === 0;
  return report;
}
