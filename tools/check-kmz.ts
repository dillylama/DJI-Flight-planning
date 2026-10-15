// Mission checker (CLI): any DJI route KMZ (Pilot 2, FlightHub 2, this planner) against a DEM.
//   node tools/check-kmz.ts <mission.kmz> <dem.tif> --agl 320 [--corridor 75] [--margin 10] [--step 10]
//                           [--takeoff lat,lon] [--lift out.kmz] [--dsm out.tif [--dilate 75] [--dsm-margin 0] [--dsm-datum ellipsoidal|orthometric]]
// Reports, per leg, the lowest clearance over the ground directly under the line and over the highest ground within
// ±corridor, against the AGL the mission was planned for; --lift writes a corrected KMZ (heights raised, waypoints
// inserted, speeds by the measured aircraft model, this planner's file layout); --dsm writes the DEM as a terrain
// model for Pilot 2's own terrain following (EPSG:4326 GeoTIFF, corridor-dilated). Prints no coordinates.
import { readFileSync, writeFileSync } from 'node:fs';
import JSZip from 'jszip';
import { readGeoTiff, rasterElev, readMissionFiles, checkClearance, liftRoute, liftedRoute, wpmlOptionsFor, writeWpml, writeKmz, dsmFromRaster, writeGeoTiff, verifyRouteFiles, bufferedBbox } from '../packages/core/src/index.ts';

const args = process.argv.slice(2);
const flag = (name: string, d?: string) => { const i = args.indexOf('--' + name); return i >= 0 ? args.splice(i, 2)[1] : d; };
const agl = +flag('agl', 'NaN')!, corridor = +flag('corridor', '75')!, margin = +flag('margin', '10')!, step = +flag('step', '10')!;
const takeoff = flag('takeoff'), lift = flag('lift'), dsm = flag('dsm'), dilate = +flag('dilate', '75')!, dsmMargin = +flag('dsm-margin', '0')!, dsmDatum = flag('dsm-datum', 'ellipsoidal') as 'ellipsoidal' | 'orthometric';
const [kmzPath, demPath] = args;
if (!kmzPath || !demPath || !Number.isFinite(agl)) { console.error('usage: node tools/check-kmz.ts <mission.kmz> <dem.tif> --agl 320 [--corridor 75] [--margin 10] [--takeoff lat,lon] [--lift out.kmz] [--dsm out.tif]'); process.exit(2); }

const zip = await JSZip.loadAsync(readFileSync(kmzPath));
const file = async (re: RegExp) => { const n = Object.keys(zip.files).find(f => re.test(f)); return n ? zip.files[n].async('string') : null; };
const waylines = await file(/waylines\.wpml$/), templateKml = await file(/template\.kml$/);
if (!waylines) { console.error('no waylines.wpml in the KMZ'); process.exit(1); }
const db = readFileSync(demPath);
const raster = await readGeoTiff(db.buffer.slice(db.byteOffset, db.byteOffset + db.byteLength) as ArrayBuffer);
const elev = rasterElev(raster);

let takeoffGround: number | null = null;
if (takeoff) { const [la, lo] = takeoff.split(',').map(Number); takeoffGround = elev(lo, la); if (takeoffGround == null) { console.error('take-off point is outside the DEM'); process.exit(1); } }
const m = readMissionFiles(waylines, templateKml, takeoffGround);
console.log(`${kmzPath.split(/[\\/]/).pop()}: ${m.wps.length} waypoints, heights ${m.heightMode}${m.templateType ? `, template ${m.templateType}` : ''}${m.flightHeightM != null ? `, planned height in the file ${m.flightHeightM} m` : ''}`);
for (const n of m.notes) console.log('  note: ' + n);
console.log(`  recording ${m.recordStart != null ? `WP ${m.recordStart + 1}` : 'start'} → ${m.recordStop != null ? `WP ${m.recordStop + 1}` : 'end'}; IMU calibration ${m.imuCalibration ? 'on' : 'off'}; RGB photos ${m.rgbSpacingM ? `every ${m.rgbSpacingM} m` : 'off'}; L3 ${m.lidar ? `${m.lidar.samplingRate / 1000} kHz ${m.lidar.returnMode} ${m.lidar.scanningMode}` : 'parameters not in the file'}`);

const rep = checkClearance(m.wps, elev, { aglM: agl, corridorM: corridor, marginM: margin, stepM: step });
const f1 = (x: number) => x.toFixed(1);
console.log(`\nclearance against the DEM, planned AGL ${agl} m, corridor ±${corridor} m, margin ${margin} m, route ${(rep.lengthM / 1000).toFixed(2)} km:`);
console.log(`  lowest over the ground under the line ${f1(rep.minUnderM)} m; lowest over the corridor ${f1(rep.minClearM)} m (leg ${rep.worstLeg != null ? rep.worstLeg + 1 : '-'})`);
console.log(`  legs short of ${agl - margin} m: ${rep.nShort} of ${rep.legs.length}, ${(rep.shortLengthM / 1000).toFixed(2)} km of route`);
const shortLegs = rep.legs.filter(l => l.shortM > 0).sort((a, b) => b.shortM - a.shortM);
for (const l of shortLegs.slice(0, 15)) console.log(`    leg ${String(l.leg + 1).padStart(3)} (WP ${l.leg + 1}→${l.leg + 2}, ${l.lengthM.toFixed(0)} m): corridor clearance ${f1(l.minClearM)} m at ${l.minClearAtM.toFixed(0)} m along, under the line ${f1(l.minUnderM)} m → ${f1(l.shortM)} m short`);
if (shortLegs.length > 15) console.log(`    … ${shortLegs.length - 15} more`);
console.log(rep.nShort ? '\nRESULT: the file does NOT hold its planned clearance over this DEM.' : '\nRESULT: the file holds its planned clearance over this DEM.');

if (lift) {
  const up = liftRoute(m.wps, elev, { aglM: agl, corridorM: corridor, stepM: step });
  const rep2 = checkClearance(up, elev, { aglM: agl, corridorM: corridor, marginM: 0, stepM: step });
  const route = liftedRoute(m, up, elev, corridor);
  const files = writeWpml(route, wpmlOptionsFor(m));
  const v = verifyRouteFiles(files.templateKml, files.waylinesWpml, { elev, aglM: agl, corridorM: corridor, maxClimbMs: 3.5, maxDescentMs: 2.5, maxSpeedMs: Math.max(...up.map(w => w.speed)), lidar: true });
  writeFileSync(lift, await writeKmz(files));
  console.log(`\nlifted: ${up.length} waypoints (${up.filter(w => w.inserted).length} inserted, ${up.filter(w => w.slowed).length} slowed), lowest corridor clearance now ${f1(rep2.minClearM)} m, ${(files.durationS / 60).toFixed(1)} min at the file speeds → ${lift}${v.ok ? ' (independent check passed)' : ` (independent check: ${v.issues.filter(i => i.severity === 'error').map(i => i.code).join(', ') || 'warnings only'})`}`);
}
if (dsm) {
  const b = bufferedBbox(m.wps.map(w => [w.lon, w.lat] as [number, number]), Math.max(500, corridor + 300));
  const spec = dsmFromRaster(raster, { dilateM: dilate, marginM: dsmMargin, datum: dsmDatum, bbox: b });
  writeFileSync(dsm, Buffer.from(writeGeoTiff(spec)));
  console.log(`DSM for Pilot 2: ${spec.width} × ${spec.height} cells, highest ground within ${dilate} m + ${dsmMargin} m, ${dsmDatum} heights, EPSG:4326 → ${dsm}`);
}
