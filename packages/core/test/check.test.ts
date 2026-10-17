import { test } from 'node:test';
import assert from 'node:assert/strict';
import { egm96ToEllipsoid, ellipsoidToEgm96 } from 'egm96-universal';
import {
  rasterElev, readGeoTiff, readMissionFiles, checkClearance, previewDsm, compositeElev, embeddedDsm, dsmFromRaster, writeGeoTiff, type Raster,
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
const asRaster = (s: ReturnType<typeof dsmFromRaster>): Raster => ({ width: s.width, height: s.height, west: s.tie[0], north: s.tie[1], dLon: s.pixel[0], dLat: s.pixel[1], data: s.values, nodata: s.nodata ?? null, pixelIsPoint: false });
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

test('previewDsm: a dilated surface with a margin lifts the route over the koppie; the current surface changes nothing', () => {
  const m = readMissionFiles(flatFile, null);
  const rep = checkClearance(m.wps, elev, { aglM: 120, corridorM: 75 });
  const same = previewDsm(rep, elev, elev)!;
  assert.ok(Math.abs(same.shortShare - rep.nShort / rep.legs.length) < 1 || same.shortShare > 0, 'the koppie is short as flown');
  assert.ok(Math.abs(same.lowestClearM - rep.minClearM) < 0.01);
  const surf = (dilateM: number, marginM: number) => rasterElev(asRaster(dsmFromRaster(r, { dilateM, marginM, datum: 'orthometric' })));
  const bare = previewDsm(rep, elev, surf(0, 0))!, dil = previewDsm(rep, elev, surf(75, 0))!, withM = previewDsm(rep, elev, surf(75, 20))!;
  assert.ok(Math.abs(bare.lowestClearM - same.lowestClearM) < 0.5, 'bare DEM is the current surface');
  assert.ok(dil.shortShare < same.shortShare - 0.02, `dilation lifts the flanks: ${same.shortShare} → ${dil.shortShare} of the route short`);   // the peak itself is already at the ground's top, so the low point stays
  assert.ok(withM.lowestClearM > dil.lowestClearM + 15 && withM.aboveP50M > dil.aboveP50M, 'margin costs height');
  assert.ok(withM.densityFactor <= dil.densityFactor);
});

test('compositeElev takes the first surface with data; embeddedDsm reads DJI-style ellipsoidal tiles back as sea-level heights', async () => {
  const a = rasterElev({ width: 3, height: 3, west: 10, north: 5, dLon: 0.001, dLat: 0.001, data: [1, 1, 1, 1, 1, 1, 1, 1, 1], nodata: -9999, pixelIsPoint: false });
  const b = rasterElev({ width: 3, height: 3, west: 20, north: 5, dLon: 0.001, dLat: 0.001, data: [2, 2, 2, 2, 2, 2, 2, 2, 2], nodata: -9999, pixelIsPoint: false });
  const c = compositeElev([a, b]);
  assert.equal(c(10.0015, 4.9985), 1); assert.equal(c(20.0015, 4.9985), 2); assert.equal(c(30, 30), null);
  const spec = dsmFromRaster(r, { dilateM: 0, marginM: 0, datum: 'ellipsoidal' });
  const dsm = (await embeddedDsm([writeGeoTiff(spec), new ArrayBuffer(8)]))!;   // the second "tile" is garbage and is skipped
  assert.ok(Math.abs(dsm(west + 5.5 * d, lat)! - 700) < 0.1);
  assert.equal(await embeddedDsm([]), null);
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
