// Makes a SIMULATOR-ONLY copy of a route KMZ with every height lowered by the same amount, so that the route
// fits above the simulator's flat "ground" (DJI's simulator starts the aircraft at about 100 m above sea level
// wherever it is placed) and below the aircraft's Max Altitude. Positions, speeds, turns and actions are untouched.
//   node tools/make-sim-variant.ts <in.kmz> <out.kmz> [--sim-ground 100] [--lowest-above 60]
// NEVER fly the output: its heights have nothing to do with the real ground.
import { readFileSync, writeFileSync } from 'node:fs';
import JSZip from 'jszip';
import { parseRouteFile } from '../packages/core/src/index.ts';

const args = process.argv.slice(2);
const flag = (name: string, d: number) => { const i = args.indexOf('--' + name); return i >= 0 ? Number(args.splice(i, 2)[1]) : d; };
const simGround = flag('sim-ground', 100), lowestAbove = flag('lowest-above', 60);
const [inPath, outPath] = args;
if (!inPath || !outPath) { console.error('usage: node tools/make-sim-variant.ts <in.kmz> <out.kmz> [--sim-ground 100] [--lowest-above 60]'); process.exit(2); }
if (!/SIM-ONLY/.test(outPath)) { console.error('The output name must contain SIM-ONLY.'); process.exit(2); }

const zip = await JSZip.loadAsync(readFileSync(inPath));
const tName = Object.keys(zip.files).find(f => /template\.kml$/.test(f))!, wName = Object.keys(zip.files).find(f => /waylines\.wpml$/.test(f))!;
let template = await zip.files[tName].async('string'), waylines = await zip.files[wName].async('string');
const before = parseRouteFile(template, 'template').waypoints.map(p => p.hEgm96);
const shift = Math.min(...before) - (simGround + lowestAbove);
const lower = (xml: string, tag: string) => xml.replace(new RegExp(`<wpml:${tag}>([^<]*)</wpml:${tag}>`, 'g'), (_a, v) => `<wpml:${tag}>${+(Number(v) - shift).toFixed(6)}</wpml:${tag}>`);
for (const tag of ['height', 'ellipsoidHeight', 'globalHeight']) template = lower(template, tag);
waylines = lower(waylines, 'executeHeight');

const t = parseRouteFile(template, 'template'), w = parseRouteFile(waylines, 'waylines');
const h = t.waypoints.map(p => p.hEgm96);
if (w.waypoints.some((p, i) => Math.abs(p.hEgm96 - h[i]) > 0.01)) throw new Error('template and waylines disagree after the shift');
const out = new JSZip();
out.file(tName, template); out.file(wName, waylines);
writeFileSync(outPath, await out.generateAsync({ type: 'uint8array', compression: 'DEFLATE' }));
console.log(`SIMULATOR ONLY: every height lowered by ${shift.toFixed(2)} m.`);
console.log(`Heights above sea level ${Math.min(...h).toFixed(1)}–${Math.max(...h).toFixed(1)} m; above a ${simGround} m simulator ground: waypoint 1 ${(h[0] - simGround).toFixed(1)} m, lowest ${(Math.min(...h) - simGround).toFixed(1)} m, highest ${(Math.max(...h) - simGround).toFixed(1)} m.`);
console.log(`Waypoint 1 ellipsoidal ${w.waypoints[0].hEllipsoid.toFixed(1)} m (above a ${simGround} m ground: ${(w.waypoints[0].hEllipsoid - simGround).toFixed(1)} m if the aircraft does not apply the geoid).`);
