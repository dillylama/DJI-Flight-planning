// Translates a LAS point cloud by a constant east / north / up shift, exactly and without touching the points:
// LAS coordinates are raw × scale + offset, so only the header's offsets and bounds change (the file is copied).
//   node tools/las-shift.ts <in.las> <out.las> --east 0.327 --north -0.143 --up -0.847
// Use: a cloud processed with a wrong base-station position (DJI Terra takes the RINEX header position unless told
// otherwise) is off by the base error as a rigid translation; this applies the correction. Shifts are metres in the
// cloud's projected frame (UTM easting/northing/height), which at these sizes equals east/north/up.
import { openSync, readSync, writeSync, closeSync, fstatSync } from 'node:fs';
import { copyFileSync } from 'node:fs';

const args = process.argv.slice(2);
const flag = (name: string) => { const i = args.indexOf('--' + name); return i >= 0 ? +args.splice(i, 2)[1] : 0; };
const east = flag('east'), north = flag('north'), up = flag('up');
const [inPath, outPath] = args;
if (!inPath || !outPath) { console.error('usage: node tools/las-shift.ts <in.las> <out.las> --east m --north m --up m'); process.exit(2); }
copyFileSync(inPath, outPath);
const fd = openSync(outPath, 'r+');
const h = Buffer.alloc(375); readSync(fd, h, 0, 375, 0);
if (h.toString('latin1', 0, 4) !== 'LASF') { console.error('not a LAS file'); process.exit(1); }
const get = (o: number) => h.readDoubleLE(o), put = (o: number, v: number) => h.writeDoubleLE(v, o);
// offsets X Y Z at 155/163/171; max/min X at 179/187, Y at 195/203, Z at 211/219
for (const [o, d] of [[155, east], [163, north], [171, up], [179, east], [187, east], [195, north], [203, north], [211, up], [219, up]] as const) put(o, get(o) + d);
writeSync(fd, h, 0, 375, 0);
closeSync(fd);
console.log(`${outPath}: shifted east ${east} m, north ${north} m, up ${up} m (${(fstatSync(openSync(outPath, 'r')).size / 1e6).toFixed(0)} MB)`);
