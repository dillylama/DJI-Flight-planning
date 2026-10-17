// Mission checker (CLI): any DJI route KMZ (Pilot 2, FlightHub 2, this planner) against a DEM. Never edits the route.
//   node tools/check-kmz.ts <mission.kmz> <dem.tif> --agl 320 [--corridor 75] [--margin 10] [--step 10] [--takeoff lat,lon]
//                           [--dsm out.tif [--dilate 75] [--dsm-margin 40]] [--table]
// Reports, per leg, the lowest clearance over the ground under the line and over the highest ground within ±corridor,
// against the AGL the mission was planned for, and how far above plan it flies (point density). --table previews the
// dilation × margin options of a DSM for Pilot 2's terrain following against this route (first-order: Pilot 2 passes
// the surface change straight through); --dsm writes one (EPSG:4326 GeoTIFF, WGS84 ellipsoidal heights). Prints no
// coordinates.
import { readFileSync, writeFileSync } from 'node:fs';
import JSZip from 'jszip';
import { readGeoTiff, rasterElev, readMissionFiles, checkClearance, previewDsm, embeddedDsm, dsmFromRaster, writeGeoTiff, bufferedBbox, type Raster } from '../packages/core/src/index.ts';

const args = process.argv.slice(2);
const flag = (name: string, d?: string) => { const i = args.indexOf('--' + name); return i >= 0 ? args.splice(i, 2)[1] : d; };
const agl = +flag('agl', 'NaN')!, corridor = +flag('corridor', '75')!, margin = +flag('margin', '10')!, step = +flag('step', '10')!;
const takeoff = flag('takeoff'), dsm = flag('dsm'), table = args.includes('--table'), dilate = +flag('dilate', '75')!, dsmMargin = +flag('dsm-margin', '40')!, dsmDatum = flag('dsm-datum', 'ellipsoidal') as 'ellipsoidal' | 'orthometric';
const [kmzPath, demPath] = args.filter(x => x !== '--table');
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
console.log(`  too high: above corridor + ${agl} m by ${f1(rep.aboveM.p50)} m (median), ${f1(rep.aboveM.p95)} m (95 %), ${f1(rep.aboveM.max)} m (max) → about ${Math.round(100 * rep.densityFactor)} % of the planned point density`);
const shortLegs = rep.legs.filter(l => l.shortM > 0).sort((a, b) => b.shortM - a.shortM);
for (const l of shortLegs.slice(0, 15)) console.log(`    leg ${String(l.leg + 1).padStart(3)} (WP ${l.leg + 1}→${l.leg + 2}, ${l.lengthM.toFixed(0)} m): corridor clearance ${f1(l.minClearM)} m at ${l.minClearAtM.toFixed(0)} m along, under the line ${f1(l.minUnderM)} m → ${f1(l.shortM)} m short`);
if (shortLegs.length > 15) console.log(`    … ${shortLegs.length - 15} more`);
console.log(rep.nShort ? '\nRESULT: the file does NOT hold its planned clearance over this DEM.' : '\nRESULT: the file holds its planned clearance over this DEM.');

const dsmTiles = Object.keys(zip.files).filter(n => /res\/dsm\/.*\.tiff?$/i.test(n));
const current = (await embeddedDsm(await Promise.all(dsmTiles.map(n => zip.files[n].async('arraybuffer'))))) ?? elev;
console.log(`  the mission was planned on ${dsmTiles.length ? `the ${dsmTiles.length} DSM tile(s) inside the file (${dsmTiles.map(n => n.split('/').pop()).join(', ')})` : 'no DSM carried in the file: the preview assumes the DEM given'}`);
const asRaster = (s: ReturnType<typeof dsmFromRaster>): Raster => ({ width: s.width, height: s.height, west: s.tie[0], north: s.tie[1], dLon: s.pixel[0], dLat: s.pixel[1], data: s.values, nodata: s.nodata ?? null, pixelIsPoint: false });
const row = (label: string, p: NonNullable<ReturnType<typeof previewDsm>>) => console.log(`  ${label.padEnd(34)} short ${(100 * p.shortShare).toFixed(0).padStart(3)} % of the route, lowest ${p.lowestClearM.toFixed(0).padStart(4)} m, above plan ${p.aboveP50M.toFixed(0).padStart(3)} m median / ${p.aboveP95M.toFixed(0)} m 95 %, density ${(100 * p.densityFactor).toFixed(0)} %`);
if (table) {
  console.log('\nDSM options (preview, first-order):');
  row('as planned now', previewDsm(rep, current, current)!);
  for (const D of [0, 30, 50, 75]) for (const M of [0, 20, 40]) row(`DEM max ${D} m + ${M} m`, previewDsm(rep, current, rasterElev(asRaster(dsmFromRaster(raster, { dilateM: D, marginM: M, datum: 'orthometric' }))))!);
}
if (dsm) {
  const b = bufferedBbox(m.wps.map(w => [w.lon, w.lat] as [number, number]), Math.max(500, corridor + 300));
  const spec = dsmFromRaster(raster, { dilateM: dilate, marginM: dsmMargin, datum: dsmDatum, bbox: b });
  writeFileSync(dsm, Buffer.from(writeGeoTiff(spec)));
  console.log(`DSM for Pilot 2: ${spec.width} × ${spec.height} cells, highest ground within ${dilate} m + ${dsmMargin} m, ${dsmDatum} heights, EPSG:4326 → ${dsm}`);
}
