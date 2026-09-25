import { egm96ToEllipsoid } from 'egm96-universal';
import JSZip from 'jszip';
import { dist } from './geo.ts';
import type { FlightWp } from './heights.ts';
import { legSpeed, type Route } from './route.ts';
import { TAKEOFF_DEFAULTS, type TakeoffOptions } from './takeoff.ts';

// DJI WPML writer for the M400 + Zenmuse L3. Every element and value below is copied from real Pilot 2
// exports (samples/m400-l3-waypoint-reference.kmz and samples/m400-l3-area-reference.kmz, WPML 1.0.6).
// Heights: template.kml carries EGM96 orthometric `height` + `ellipsoidHeight`; waylines.wpml executes in
// WGS84 ellipsoidal heights. Pilot 2 uses the EGM96 grid that egm96-universal implements (matches to 0.1 mm).
// Our DEM (GLO-30) is EGM2008: treating it as EGM96 costs < ~1–2 m, negligible against AGL and clearance.

export const WPML_NS = 'http://www.dji.com/wpmz/1.0.6';
export const M400 = { droneEnumValue: 103, droneSubEnumValue: 0 } as const;
export const ZENMUSE_L3 = { payloadEnumValue: 117, payloadSubEnumValue: 0 } as const;

// L3 payload values seen in the samples. Others are rejected until a sample shows their exact spelling.
export const L3_WPML = {
  samplingRates: [100000, 350000, 1000000, 2000000],   // Hz; the sample wrote 350000 for 350 kHz
  returnModes: ['sedecupleReturn'] as const,             // 16 returns (sample)
  scanningModes: ['repetitive'] as const,                // linear/repetitive (sample)
};

export interface WpmlOptions {
  takeoff?: Partial<TakeoffOptions>;
  payloadPositionIndex?: number;       // 0 = the default port (samples)
  lidar?: { samplingRate: number; returnMode: typeof L3_WPML.returnModes[number]; scanningMode: typeof L3_WPML.scanningModes[number]; modelColoring: boolean } | null;
  djiImuCalibration?: boolean;         // DJI aircraftCalibration before startRecord and after stopRecord (as Pilot 2 does)
  rgbPhotoSpacingM?: number | null;    // L3 RGB continuous shooting on data lines at this distance; null = off
  gimbalStartGroup?: boolean;          // DJI start group: gimbal −90°, focus calibration, focus ∞ (as Pilot 2 does for L3)
  createTime?: number;
}

export interface WpmlFiles { templateKml: string; waylinesWpml: string; distanceM: number; durationS: number }

const f = (v: number, d = 6) => String(+v.toFixed(d));
const coord = (w: FlightWp) => `${w.lon},${w.lat}`;   // full double precision, as Pilot 2 writes

interface Action { func: string; params: [string, string | number][] }
interface Group { start: number; end: number; trigger: 'reachPoint' | 'betweenAdjacentPoints' | 'multipleDistance'; triggerParam?: number; actions: Action[] }

const pp = (i: number): [string, number] => ['payloadPositionIndex', i];
const gimbalRotate = (i: number): Action => ({ func: 'gimbalRotate', params: [
  ['gimbalHeadingYawBase', 'aircraft'], ['gimbalRotateMode', 'absoluteAngle'], ['gimbalPitchRotateEnable', 1], ['gimbalPitchRotateAngle', -90],
  ['gimbalRollRotateEnable', 0], ['gimbalRollRotateAngle', 0], ['gimbalYawRotateEnable', 1], ['gimbalYawRotateAngle', 0],
  ['gimbalRotateTimeEnable', 0], ['gimbalRotateTime', 10], pp(i)] });
const hover = (s: number): Action => ({ func: 'hover', params: [['hoverTime', s]] });
const calibration = (heading: 0 | 1): Action => ({ func: 'aircraftCalibration', params: [['calibrationHeading', heading], ['calibrationTimes', 3], ['calibrationDistance', 30]] });
const record = (op: 'startRecord' | 'stopRecord', i: number): Action => ({ func: 'recordPointCloud', params: [['recordPointCloudOperate', op], pp(i)] });

function actionXml(a: Action, id: number, ind: string): string {
  const params = a.params.length
    ? `${ind}  <wpml:actionActuatorFuncParam>\n${a.params.map(([k, v]) => `${ind}    <wpml:${k}>${v}</wpml:${k}>`).join('\n')}\n${ind}  </wpml:actionActuatorFuncParam>\n`
    : '';
  return `${ind}<wpml:action>\n${ind}  <wpml:actionId>${id}</wpml:actionId>\n${ind}  <wpml:actionActuatorFunc>${a.func}</wpml:actionActuatorFunc>\n${params}${ind}</wpml:action>`;
}

// Action groups per waypoint index, with globally increasing actionGroupId (as Pilot 2 writes them).
function buildGroups(route: Route<FlightWp>, o: Required<Pick<WpmlOptions, 'payloadPositionIndex' | 'djiImuCalibration'>> & { rgb: number | null }): Map<number, Group[]> {
  const wps = route.wps, pos = o.payloadPositionIndex;
  const at = new Map<number, Group[]>();
  const add = (i: number, g: Group) => { if (!at.has(i)) at.set(i, []); at.get(i)!.push(g); };
  wps.forEach((w, i) => {
    if (w.actions.includes('START_RECORD')) add(i, { start: i, end: i, trigger: 'reachPoint',
      actions: [...(o.djiImuCalibration ? [calibration(0)] : []), record('startRecord', pos)] });
  });
  if (o.rgb) {
    // Each data line: lock gimbal and shoot every `rgb` metres from its first to its last line waypoint.
    const lines = new Map<number, number[]>();
    wps.forEach((w, i) => { if (w.role === 'line' && w.line != null) { if (!lines.has(w.line)) lines.set(w.line, []); lines.get(w.line)!.push(i); } });
    for (const idx of lines.values()) {
      const a = idx[0], b = idx[idx.length - 1];
      add(a, { start: a, end: b, trigger: 'betweenAdjacentPoints', actions: [{ func: 'gimbalAngleLock', params: [pp(pos)] }] });
      add(a, { start: a, end: b, trigger: 'multipleDistance', triggerParam: o.rgb,
        actions: [gimbalRotate(pos), { func: 'startContinuousShooting', params: [pp(pos), ['useGlobalPayloadLensIndex', 0]] }] });
      add(b, { start: b, end: b, trigger: 'reachPoint',
        actions: [{ func: 'stopContinuousShooting', params: [pp(pos)] }, { func: 'gimbalAngleUnlock', params: [] }] });
    }
  }
  wps.forEach((w, i) => {
    if (w.actions.includes('STOP_RECORD')) add(i, { start: i, end: i, trigger: 'reachPoint',
      actions: [record('stopRecord', pos), ...(o.djiImuCalibration ? [calibration(1)] : [])] });
  });
  return at;
}

function groupsXml(groups: Group[] | undefined, nextId: { v: number }, ind: string): string {
  if (!groups) return '';
  return groups.map(g => {
    const trig = `${ind}  <wpml:actionTrigger>\n${ind}    <wpml:actionTriggerType>${g.trigger}</wpml:actionTriggerType>\n` +
      (g.triggerParam != null ? `${ind}    <wpml:actionTriggerParam>${f(g.triggerParam)}</wpml:actionTriggerParam>\n` : '') + `${ind}  </wpml:actionTrigger>\n`;
    return `${ind}<wpml:actionGroup>\n${ind}  <wpml:actionGroupId>${nextId.v++}</wpml:actionGroupId>\n` +
      `${ind}  <wpml:actionGroupStartIndex>${g.start}</wpml:actionGroupStartIndex>\n${ind}  <wpml:actionGroupEndIndex>${g.end}</wpml:actionGroupEndIndex>\n` +
      `${ind}  <wpml:actionGroupMode>sequence</wpml:actionGroupMode>\n${trig}` +
      g.actions.map((a, k) => actionXml(a, k, ind + '  ')).join('\n') + `\n${ind}</wpml:actionGroup>\n`;
  }).join('');
}

const missionConfig = (tk: TakeoffOptions, pos: number) => `    <wpml:missionConfig>
      <wpml:flyToWaylineMode>${tk.flyToMode}</wpml:flyToWaylineMode>
      <wpml:finishAction>goHome</wpml:finishAction>
      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>
      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
      <wpml:takeOffSecurityHeight>${f(tk.takeoffSecurityM, 2)}</wpml:takeOffSecurityHeight>
      <wpml:globalTransitionalSpeed>${f(tk.transitSpeedMs, 2)}</wpml:globalTransitionalSpeed>
      <wpml:droneInfo>
        <wpml:droneEnumValue>${M400.droneEnumValue}</wpml:droneEnumValue>
        <wpml:droneSubEnumValue>${M400.droneSubEnumValue}</wpml:droneSubEnumValue>
      </wpml:droneInfo>
      <wpml:waylineAvoidLimitAreaMode>0</wpml:waylineAvoidLimitAreaMode>
      <wpml:payloadInfo>
        <wpml:payloadEnumValue>${ZENMUSE_L3.payloadEnumValue}</wpml:payloadEnumValue>
        <wpml:payloadSubEnumValue>${ZENMUSE_L3.payloadSubEnumValue}</wpml:payloadSubEnumValue>
        <wpml:payloadPositionIndex>${pos}</wpml:payloadPositionIndex>
      </wpml:payloadInfo>
    </wpml:missionConfig>
`;

const headingXml = (ind: string, angleEnable: boolean) => `${ind}<wpml:waypointHeadingParam>
${ind}  <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>
${ind}  <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>
${ind}  <wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>
${angleEnable ? `${ind}  <wpml:waypointHeadingAngleEnable>0</wpml:waypointHeadingAngleEnable>\n` : ''}${ind}  <wpml:waypointHeadingPoiIndex>0</wpml:waypointHeadingPoiIndex>
${ind}</wpml:waypointHeadingParam>
`;

// Pilot 2 writes the first and last waypoint as "stop" turns with zero damping.
const turnOf = (wps: FlightWp[], i: number) => (i === 0 || i === wps.length - 1)
  ? { mode: 'toPointAndStopWithDiscontinuityCurvature', damp: 0 }
  : { mode: wps[i].turnMode, damp: wps[i].dampingM };

export function writeWpml(route: Route<FlightWp>, opts: WpmlOptions = {}): WpmlFiles {
  const tk = { ...TAKEOFF_DEFAULTS, ...opts.takeoff };
  const pos = opts.payloadPositionIndex ?? 0;
  const lidar = opts.lidar ?? null;
  if (lidar) {
    if (!L3_WPML.samplingRates.includes(lidar.samplingRate)) throw new Error(`L3 sampling rate ${lidar.samplingRate} not verified`);
    if (!L3_WPML.returnModes.includes(lidar.returnMode)) throw new Error(`L3 return mode ${lidar.returnMode} not verified`);
    if (!L3_WPML.scanningModes.includes(lidar.scanningMode)) throw new Error(`L3 scanning mode ${lidar.scanningMode} not verified`);
  }
  const wps = route.wps;
  if (wps.length < 2) throw new Error('A route needs at least 2 waypoints');
  const ell = wps.map(w => egm96ToEllipsoid(w.lat, w.lon, w.h));
  let distanceM = 0, durationS = 0;
  for (let i = 1; i < wps.length; i++) {
    const d = Math.hypot(dist(wps[i].xy, wps[i - 1].xy), wps[i].h - wps[i - 1].h);
    distanceM += d; durationS += d / legSpeed(wps, i - 1);
  }
  const groups = buildGroups(route, { payloadPositionIndex: pos, djiImuCalibration: opts.djiImuCalibration ?? false, rgb: opts.rgbPhotoSpacingM ?? null });
  const now = opts.createTime ?? Date.now();

  // ── template.kml
  const tId = { v: 0 };
  const tPlacemarks = wps.map((w, i) => {
    const t = turnOf(wps, i);
    return `      <Placemark>
        <Point>
          <coordinates>
            ${coord(w)}
          </coordinates>
        </Point>
        <wpml:index>${i}</wpml:index>
        <wpml:ellipsoidHeight>${f(ell[i])}</wpml:ellipsoidHeight>
        <wpml:height>${f(w.h)}</wpml:height>
        <wpml:waypointSpeed>${f(w.speed, 2)}</wpml:waypointSpeed>
        <wpml:waypointTurnParam>
          <wpml:waypointTurnMode>${t.mode}</wpml:waypointTurnMode>
          <wpml:waypointTurnDampingDist>${f(t.damp, 2)}</wpml:waypointTurnDampingDist>
        </wpml:waypointTurnParam>
        <wpml:useGlobalHeight>0</wpml:useGlobalHeight>
        <wpml:useGlobalSpeed>0</wpml:useGlobalSpeed>
        <wpml:useGlobalHeadingParam>1</wpml:useGlobalHeadingParam>
        <wpml:useGlobalTurnParam>0</wpml:useGlobalTurnParam>
        <wpml:useStraightLine>0</wpml:useStraightLine>
${groupsXml(groups.get(i), tId, '        ')}        <wpml:isRisky>0</wpml:isRisky>
      </Placemark>`;
  }).join('\n');
  const payloadParam = `      <wpml:payloadParam>
        <wpml:payloadPositionIndex>${pos}</wpml:payloadPositionIndex>
${lidar ? `        <wpml:returnMode>${lidar.returnMode}</wpml:returnMode>
        <wpml:samplingRate>${lidar.samplingRate}</wpml:samplingRate>
        <wpml:scanningMode>${lidar.scanningMode}</wpml:scanningMode>
        <wpml:modelColoringEnable>${lidar.modelColoring ? 1 : 0}</wpml:modelColoringEnable>
` : ''}        <wpml:photoSize/>
      </wpml:payloadParam>`;
  const templateKml = `<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="${WPML_NS}">
  <Document>
    <wpml:createTime>${now}</wpml:createTime>
    <wpml:updateTime>${now}</wpml:updateTime>
${missionConfig(tk, pos)}    <Folder>
      <wpml:templateType>waypoint</wpml:templateType>
      <wpml:templateId>0</wpml:templateId>
      <wpml:waylineCoordinateSysParam>
        <wpml:coordinateMode>WGS84</wpml:coordinateMode>
        <wpml:heightMode>EGM96</wpml:heightMode>
        <wpml:positioningType>GPS</wpml:positioningType>
      </wpml:waylineCoordinateSysParam>
      <wpml:autoFlightSpeed>${f(route.speedMs, 2)}</wpml:autoFlightSpeed>
      <wpml:globalHeight>${f(wps[0].h)}</wpml:globalHeight>
      <wpml:caliFlightEnable>0</wpml:caliFlightEnable>
      <wpml:gimbalPitchMode>manual</wpml:gimbalPitchMode>
      <wpml:globalWaypointHeadingParam>
        <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>
        <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>
        <wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>
        <wpml:waypointHeadingPoiIndex>0</wpml:waypointHeadingPoiIndex>
      </wpml:globalWaypointHeadingParam>
      <wpml:globalWaypointTurnMode>toPointAndPassWithContinuityCurvature</wpml:globalWaypointTurnMode>
      <wpml:globalUseStraightLine>0</wpml:globalUseStraightLine>
${tPlacemarks}
${payloadParam}
    </Folder>
  </Document>
</kml>
`;

  // ── waylines.wpml
  const wId = { v: 0 };
  const startGroup = opts.gimbalStartGroup ? `      <wpml:startActionGroup>
${[gimbalRotate(pos), hover(1),
    { func: 'setFocusType', params: [['cameraFocusType', 'auto'], pp(pos)] }, hover(0.5),
    { func: 'focus', params: [['focusX', 0.5], ['focusY', 0.5], ['isPointFocus', 1], ['isInfiniteFocus', 0], pp(pos), ['isCalibrationFocus', 1]] },
    { func: 'setFocusType', params: [['cameraFocusType', 'manual'], pp(pos)] },
    { func: 'focus', params: [['focusX', 0], ['focusY', 0], ['focusRegionWidth', 0], ['focusRegionHeight', 0], ['isPointFocus', 0], ['isInfiniteFocus', 1], pp(pos), ['isCalibrationFocus', 0]] },
    hover(1),
  ].map((a, k) => actionXml(a as Action, k, '        ')).join('\n')}
      </wpml:startActionGroup>
` : '';
  const wPlacemarks = wps.map((w, i) => {
    const t = turnOf(wps, i);
    return `      <Placemark>
        <Point>
          <coordinates>
            ${coord(w)}
          </coordinates>
        </Point>
        <wpml:index>${i}</wpml:index>
        <wpml:executeHeight>${f(ell[i])}</wpml:executeHeight>
        <wpml:waypointSpeed>${f(w.speed, 2)}</wpml:waypointSpeed>
${headingXml('        ', true)}        <wpml:waypointTurnParam>
          <wpml:waypointTurnMode>${t.mode}</wpml:waypointTurnMode>
          <wpml:waypointTurnDampingDist>${f(t.damp, 2)}</wpml:waypointTurnDampingDist>
        </wpml:waypointTurnParam>
        <wpml:useStraightLine>0</wpml:useStraightLine>
${groupsXml(groups.get(i), wId, '        ')}        <wpml:waypointGimbalHeadingParam>
          <wpml:waypointGimbalPitchAngle>0</wpml:waypointGimbalPitchAngle>
          <wpml:waypointGimbalYawAngle>0</wpml:waypointGimbalYawAngle>
        </wpml:waypointGimbalHeadingParam>
        <wpml:isRisky>0</wpml:isRisky>
        <wpml:waypointWorkType>0</wpml:waypointWorkType>
      </Placemark>`;
  }).join('\n');
  const waylinesWpml = `<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="${WPML_NS}">
  <Document>
${missionConfig(tk, pos)}    <Folder>
      <wpml:templateId>0</wpml:templateId>
      <wpml:executeHeightMode>WGS84</wpml:executeHeightMode>
      <wpml:waylineId>0</wpml:waylineId>
      <wpml:distance>${f(distanceM)}</wpml:distance>
      <wpml:duration>${f(durationS)}</wpml:duration>
      <wpml:autoFlightSpeed>${f(route.speedMs, 2)}</wpml:autoFlightSpeed>
${startGroup}      <wpml:realTimeFollowSurfaceByFov>0</wpml:realTimeFollowSurfaceByFov>
${wPlacemarks}
    </Folder>
  </Document>
</kml>
`;
  return { templateKml, waylinesWpml, distanceM, durationS };
}

export async function writeKmz(files: WpmlFiles): Promise<Uint8Array> {
  const zip = new JSZip();
  zip.file('wpmz/template.kml', files.templateKml);
  zip.file('wpmz/waylines.wpml', files.waylinesWpml);
  return zip.generateAsync({ type: 'uint8array', compression: 'DEFLATE' });
}
