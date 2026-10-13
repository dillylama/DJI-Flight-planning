// A short SIMULATOR-ONLY route for watching how the aircraft takes the 90° turns between lines: three lines,
// no figure-8, no calibration passes, so the first corner comes within a minute of take-off.
//   node tools/make-turn-test.ts [--speed 10] [--bank 20]   → samples/generated/SIM-ONLY-turn-<speed>.kmz
import { readFileSync, writeFileSync } from 'node:fs';
import { planLines, buildRoute, applyHeights, readGeoTiff, rasterElev, writeWpml, writeKmz, verifyRouteFiles, type LonLat } from '../packages/core/src/index.ts';

const args = process.argv.slice(2);
const flag = (n: string, d: number) => { const i = args.indexOf('--' + n); return i >= 0 ? Number(args.splice(i, 2)[1]) : d; };
const speed = flag('speed', 10), bank = flag('bank', 20);
const root = new URL('..', import.meta.url);
const sampleWpml = readFileSync(new URL('samples/extracted/waypoint/wpmz/waylines.wpml', root), 'utf8');
const [lon0, lat0] = sampleWpml.match(/<coordinates>\s*([\d.-]+),([\d.-]+)/)!.slice(1).map(Number);
const m = 1 / 110900, k = Math.cos(lat0 * Math.PI / 180);
const P = (x: number, y: number): LonLat => [lon0 + x * m / k, lat0 + y * m];
const b = readFileSync(new URL('samples/generated/dem.tif', root));
const elev = rasterElev(await readGeoTiff(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) as ArrayBuffer));

const plan = planLines([P(-150, -100), P(150, -100), P(150, 100), P(-150, 100)], { aglM: 100, speedMs: speed, courseDeg: 0, wpSpacingM: 100, runInM: 60, runOutM: 60, turnBankDeg: bank });
const flight = applyHeights(plan, buildRoute(plan, { fig8: false }), elev);
const files = writeWpml(flight, { lidar: { samplingRate: 350000, returnMode: 'sedecupleReturn', scanningMode: 'repetitive', modelColoring: true } });
const rep = verifyRouteFiles(files.templateKml, files.waylinesWpml, { elev, aglM: 100, corridorM: 75, maxClimbMs: 4, maxDescentMs: 3, maxSpeedMs: 20, lidar: true });
if (!rep.ok) throw new Error(rep.issues.map(i => i.message).join(' | '));
const out = new URL(`samples/generated/turn-${speed}.kmz`, root);
writeFileSync(out, await writeKmz(files));
const corners = flight.wps.filter((w, i) => w.role === 'runout' && flight.wps[i + 1]?.role === 'runin').map(w => w.dampingM.toFixed(1));
console.log(`${flight.wps.length} waypoints, ${plan.lines.length} lines, spacing ${plan.spacing.toFixed(0)} m, corner damping ${corners.join(' / ')} m at ${speed} m/s and ${bank}° bank. Now: node tools/make-sim-variant.ts ${out.pathname.slice(1)} samples/generated/SIM-ONLY-turn-${speed}.kmz`);
