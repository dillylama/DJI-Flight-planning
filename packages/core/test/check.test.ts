import { test } from 'node:test';
import assert from 'node:assert/strict';
import { egm96ToEllipsoid, ellipsoidToEgm96 } from 'egm96-universal';
import {
  rasterElev, readGeoTiff, readMissionFiles, checkClearance, liftRoute, liftedRoute, wpmlOptionsFor, writeWpml, dsmFromRaster, writeGeoTiff,
  parseRouteFile, type Raster,
} from '../src/index.ts';

// A 2 km × 1 km raster of 30 m cells (0.00027°): flat at 700 m with a 60 m koppie in the middle of the route.
const W = 70, H = 40, d = 0.00027, west = 22.0, north = -33.0;
const data = new Float32Array(W * H);
for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
  const dx = (x - 35) * 30, dy = (y - 20) * 30, r = Math.hypot(dx, dy);
  data[y * W + x] = 700 + (r < 150 ? 60 * (1 - r / 150) : 0);
}
const r: Raster = { width: W, height: H, west, north, dLon: d, dLat: d, data, nodata: -9999, pixelIsPoint: false };
const elev = rasterElev(r);
const lat = north - 20.5 * d;   // the line runs east–west through the koppie's centre

const pm = (i: number, lon: number, hEll: number, speed = 15) => `
  <Placemark><Point><coordinates>${lon},${lat}</coordinates></Point><wpml:index>${i}</wpml:index>
    <wpml:executeHeight>${hEll}</wpml:executeHeight><wpml:waypointSpeed>${speed}</wpml:waypointSpeed>
    <wpml:waypointTurnParam><wpml:waypointTurnMode>toPointAndPassWithContinuityCurvature</wpml:waypointTurnMode><wpml:waypointTurnDampingDist>5</wpml:waypointTurnDampingDist></wpml:waypointTurnParam>
    <wpml:useStraightLine>1</wpml:useStraightLine>${i === 0 ? `<wpml:actionGroup><wpml:actionGroupId>0</wpml:actionGroupId><wpml:actionGroupStartIndex>0</wpml:actionGroupStartIndex><wpml:actionGroupEndIndex>0</wpml:actionGroupEndIndex><wpml:actionGroupMode>sequence</wpml:actionGroupMode><wpml:actionTrigger><wpml:actionTriggerType>reachPoint</wpml:actionTriggerType></wpml:actionTrigger><wpml:action><wpml:actionId>0</wpml:actionId><wpml:actionActuatorFunc>recordPointCloud</wpml:actionActuatorFunc><wpml:actionActuatorFuncParam><wpml:recordPointCloudOperate>startRecord</wpml:recordPointCloudOperate><wpml:payloadPositionIndex>0</wpml:payloadPositionIndex></wpml:actionActuatorFuncParam></wpml:action></wpml:actionGroup>` : ''}
  </Placemark>`;
const waylines = (mode: string, pms: string) => `<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.6"><Document>
  <wpml:missionConfig><wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode><wpml:finishAction>goHome</wpml:finishAction><wpml:exitOnRCLost>goContinue</wpml:exitOnRCLost><wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
    <wpml:takeOffSecurityHeight>60</wpml:takeOffSecurityHeight><wpml:globalTransitionalSpeed>15</wpml:globalTransitionalSpeed>
    <wpml:droneInfo><wpml:droneEnumValue>103</wpml:droneEnumValue><wpml:droneSubEnumValue>0</wpml:droneSubEnumValue></wpml:droneInfo>
    <wpml:payloadInfo><wpml:payloadEnumValue>117</wpml:payloadEnumValue><wpml:payloadSubEnumValue>0</wpml:payloadSubEnumValue><wpml:payloadPositionIndex>0</wpml:payloadPositionIndex></wpml:payloadInfo></wpml:missionConfig>
  <Folder><wpml:templateId>0</wpml:templateId><wpml:executeHeightMode>${mode}</wpml:executeHeightMode><wpml:waylineId>0</wpml:waylineId><wpml:distance>0</wpml:distance><wpml:duration>0</wpml:duration><wpml:autoFlightSpeed>15</wpml:autoFlightSpeed>
  ${pms}</Folder></Document></kml>`;
const template = (type: string, extra = '') => `<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.6"><Document><wpml:createTime>1</wpml:createTime>
  <wpml:missionConfig><wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode><wpml:finishAction>goHome</wpml:finishAction><wpml:exitOnRCLost>goContinue</wpml:exitOnRCLost><wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
    <wpml:takeOffSecurityHeight>60</wpml:takeOffSecurityHeight>${extra}<wpml:globalTransitionalSpeed>15</wpml:globalTransitionalSpeed>
    <wpml:droneInfo><wpml:droneEnumValue>103</wpml:droneEnumValue><wpml:droneSubEnumValue>0</wpml:droneSubEnumValue></wpml:droneInfo>
    <wpml:payloadInfo><wpml:payloadEnumValue>117</wpml:payloadEnumValue><wpml:payloadSubEnumValue>0</wpml:payloadSubEnumValue><wpml:payloadPositionIndex>0</wpml:payloadPositionIndex></wpml:payloadInfo></wpml:missionConfig>
  <Folder><wpml:templateType>${type}</wpml:templateType><wpml:templateId>0</wpml:templateId><wpml:autoFlightSpeed>15</wpml:autoFlightSpeed><wpml:height>120</wpml:height>
    <wpml:payloadParam><wpml:payloadPositionIndex>0</wpml:payloadPositionIndex><wpml:samplingRate>100000</wpml:samplingRate><wpml:scanningMode>repetitive</wpml:scanningMode><wpml:returnMode>sedecupleReturn</wpml:returnMode><wpml:modelColoringEnable>1</wpml:modelColoringEnable></wpml:payloadParam>
  </Folder></Document></kml>`;

// A level line at 120 m above the 700 m plain: DJI-style "AGL" that ignores the koppie (like FH2's follow on coarse data).
const lonA = west + 5 * d, lonB = west + 65 * d;
const flatFile = waylines('WGS84', pm(0, lonA, egm96ToEllipsoid(lat, lonA, 820)) + pm(1, lonB, egm96ToEllipsoid(lat, lonB, 820)));

test('readMissionFiles: WGS84 heights come back as EGM96, mission settings are picked up', () => {
  const m = readMissionFiles(flatFile, template('mapping2d'));
  assert.equal(m.wps.length, 2);
  assert.ok(Math.abs(m.wps[0].h - 820) < 0.01);
  assert.equal(m.recordStart, 0);
  assert.equal(m.templateType, 'mapping2d');
  assert.equal(m.flightHeightM, 120);
  assert.deepEqual(m.lidar, { samplingRate: 100000, returnMode: 'sedecupleReturn', scanningMode: 'repetitive' });
  assert.ok(m.notes.some(n => /mapping2d/.test(n)));
});

test('relativeToStartPoint: heights need the take-off reference (file or caller)', () => {
  const rel = waylines('relativeToStartPoint', pm(0, lonA, 120) + pm(1, lonB, 120));
  assert.throws(() => readMissionFiles(rel, template('waypoint')), /relative to the take-off point/);
  const m = readMissionFiles(rel, template('waypoint'), 700);
  assert.ok(Math.abs(m.wps[0].h - 820) < 1e-9);
  const trp = template('waypoint', `<wpml:takeOffRefPoint>${lat},${lonA},${egm96ToEllipsoid(lat, lonA, 700)}</wpml:takeOffRefPoint>`);
  const m2 = readMissionFiles(rel, trp);
  assert.ok(Math.abs(m2.wps[1].h - 820) < 0.01);
});

test('checkClearance finds the koppie under a level leg; the planner-style corridor sees the slopes too', () => {
  const m = readMissionFiles(flatFile, null);
  const rep = checkClearance(m.wps, elev, { aglM: 120, corridorM: 75 });
  assert.equal(rep.legs.length, 1);
  assert.ok(rep.minUnderM < 65 && rep.minUnderM > 55, `under ${rep.minUnderM}`);     // 820 − 760 = 60 at the top
  assert.ok(rep.minClearM <= rep.minUnderM);
  assert.equal(rep.nShort, 1);
  assert.ok(rep.legs[0].shortM > 45, `short ${rep.legs[0].shortM}`);
  assert.ok(rep.stations.length > 100);
  assert.ok(rep.aboveM.max < 1 && rep.densityFactor > 0.99, 'a level leg at the plan over the plain is not too high');
  assert.ok(Math.abs(rep.lengthM - 60 * d * 111320 * Math.cos(lat * Math.PI / 180)) < 10);
});

test('liftRoute clears the koppie with inserted waypoints and slows the climb', () => {
  const m = readMissionFiles(flatFile, null);
  const up = liftRoute(m.wps, elev, { aglM: 120, corridorM: 75 });
  assert.ok(up.length > 3, `inserted ${up.length - 2}`);
  assert.ok(up.some(w => w.inserted));
  const rep = checkClearance(up, elev, { aglM: 120, corridorM: 75, marginM: 0 });
  assert.equal(rep.nShort, 0, `lifted route still short: ${rep.minClearM}`);
  assert.ok(rep.minClearM >= 120 - 1.5, `min clearance ${rep.minClearM}`);
  assert.ok(up.some(w => w.slowed), 'the climb onto the koppie must be slowed');
  // and it hugs terrain + AGL: never more than the thinning tolerance above the requirement (corridor max, looking one
  // corridor width ahead and behind along the track, as the planner and the verifier do) — no long shallow ramp
  const need = (k: number) => Math.max(...rep.stations.filter(t => Math.abs(t.s - rep.stations[k].s) <= 75 + 2 * 50 /* ext (corridor) + a dense leg each side */).map(t => t.corridor)) + 120;
  const excess = Math.max(...rep.stations.map((s, k) => s.h - need(k)));
  assert.ok(excess <= 10 + 1, `excess ${excess}`);
  // (a level leg well inside the raster, 12 cells north of the koppie: the corridor and its extension must stay on data)
  const flat = liftRoute(readMissionFiles(waylines('WGS84', pm(0, west + 10 * d, egm96ToEllipsoid(lat, west + 10 * d, 830)) + pm(1, west + 24 * d, egm96ToEllipsoid(lat, west + 24 * d, 830))), null).wps, elev, { aglM: 120, corridorM: 75 });
  assert.equal(flat.length, 2, 'a level leg over level ground gets no waypoints');
  // ends untouched, heights monotone up then down over the hill
  assert.equal(up[0].src, 0); assert.equal(up[up.length - 1].src, 1);
  const top = Math.max(...up.map(w => w.h));
  assert.ok(top >= 880 - 1 && top <= 882, `top ${top}`);
});

test('liftedRoute + wpmlOptionsFor write a Pilot 2 file that re-reads with the lifted heights', () => {
  const m = readMissionFiles(flatFile, template('mapping2d'));
  const up = liftRoute(m.wps, elev, { aglM: 120, corridorM: 75 });
  const route = liftedRoute(m, up, elev, 75);
  const files = writeWpml(route, wpmlOptionsFor(m));
  const back = parseRouteFile(files.waylinesWpml, 'waylines');
  assert.equal(back.waypoints.length, up.length);
  assert.ok(Math.abs(back.waypoints[0].hEgm96 - up[0].h) < 0.05);
  assert.equal(back.exitOnRCLost, 'goContinue');
  assert.ok(/samplingRate>100000</.test(files.templateKml));
  assert.ok(/modelColoringEnable>0</.test(files.templateKml));
  assert.ok(back.waypoints[0].groups.some(g => g.actions.some(a => a.params.recordPointCloudOperate === 'startRecord')));
});

test('dsmFromRaster: dilation puts the planner corridor into the surface, heights become ellipsoidal, round trip', async () => {
  const spec = dsmFromRaster(r, { dilateM: 75, marginM: 0, datum: 'ellipsoidal' });
  const buf = writeGeoTiff(spec);
  const back = await readGeoTiff(buf, { verticalDatum: 'ellipsoidal' });   // read converts back to orthometric
  assert.equal(back.width, W); assert.equal(back.height, H);
  const e2 = rasterElev(back);
  // on the plain the DSM equals the DEM; near the koppie it is the max within 75 m, so higher than the DEM
  const plain = e2(west + 5.5 * d, lat)!, slope = e2(west + 40.5 * d, lat)!, dem = elev(west + 40.5 * d, lat)!;
  assert.ok(Math.abs(plain - 700) < 0.05, `plain ${plain}`);
  assert.ok(slope > dem + 5, `dilated ${slope} vs dem ${dem}`);
  assert.ok(Math.abs(ellipsoidToEgm96(lat, west + 5.5 * d, spec.values[Math.round(20.5) * W + 5]) - 700) < 0.05);
  const bare = dsmFromRaster(r, { dilateM: 0, datum: 'orthometric', marginM: 10 });
  assert.ok(Math.abs(bare.values[20 * W + 5] - 710) < 1e-3);
});
