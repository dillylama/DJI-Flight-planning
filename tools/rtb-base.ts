// Reads the base-station position the aircraft used in flight from the L3's .RTB file (the RTCM3 corrections it
// received): message 1006 = antenna reference point in ECEF (0.1 mm) plus a broadcast antenna height. Prints the ECEF
// position for rnx2rtkp's base position (-r x y z). Default: the reference point as broadcast, WITHOUT adding the
// antenna height, because that is what the aircraft uses (G3 flight, 14 Oct 2026: PPK − RTK heights −1.7 cm this way,
// +7.8 cm with the 0.095 m height added). --with-antenna-height adds it for comparison.
//   node tools/rtb-base.ts <file.RTB> [--with-antenna-height]
import { readFileSync } from 'node:fs';

const args = process.argv.slice(2);
const noAh = !args.includes('--with-antenna-height');
const path = args.find(a => !a.startsWith('--'));
if (!path) { console.error('usage: node tools/rtb-base.ts <file.RTB> [--with-antenna-height]'); process.exit(2); }
const b = readFileSync(path);

const bits = (p: Buffer, pos: number, n: number) => { let v = 0n; for (let i = 0; i < n; i++) v = (v << 1n) | BigInt((p[(pos + i) >> 3] >> (7 - ((pos + i) & 7))) & 1); return v; };
const sbits = (p: Buffer, pos: number, n: number) => { const v = bits(p, pos, n); return v & (1n << BigInt(n - 1)) ? Number(v - (1n << BigInt(n))) : Number(v); };

const seen: { x: number; y: number; z: number; h: number }[] = []; let antenna = '';
for (let i = 0; i < b.length - 6;) {
  if (b[i] !== 0xd3 || (b[i + 1] >> 2) !== 0) { i++; continue; }
  const L = ((b[i + 1] & 3) << 8) | b[i + 2]; const p = b.subarray(i + 3, i + 3 + L);
  const mt = Number(bits(p, 0, 12));
  if (mt === 1006 || mt === 1005) {
    seen.push({ x: sbits(p, 34, 38) * 1e-4, y: sbits(p, 74, 38) * 1e-4, z: sbits(p, 114, 38) * 1e-4, h: mt === 1006 ? Number(bits(p, 152, 16)) * 1e-4 : 0 });
  } else if (mt === 1033 && !antenna) { const n = Number(bits(p, 24, 8)); antenna = p.subarray(4, 4 + n).toString('latin1'); }
  i += 6 + L;
}
if (!seen.length) { console.error('no 1005/1006 message found'); process.exit(1); }
const s0 = seen[0];
const moved = seen.some(s => Math.hypot(s.x - s0.x, s.y - s0.y, s.z - s0.z) > 0.001);
// unit up vector at the geodetic latitude
const a = 6378137, f = 1 / 298.257223563, e2 = f * (2 - f);
const lon = Math.atan2(s0.y, s0.x), pr = Math.hypot(s0.x, s0.y); let lat = Math.atan2(s0.z, pr);
for (let k = 0; k < 10; k++) { const N = a / Math.sqrt(1 - e2 * Math.sin(lat) ** 2); const hh = pr / Math.cos(lat) - N; lat = Math.atan2(s0.z, pr * (1 - e2 * N / (N + hh))); }
const h = noAh ? 0 : s0.h;
const x = s0.x + h * Math.cos(lat) * Math.cos(lon), y = s0.y + h * Math.cos(lat) * Math.sin(lon), z = s0.z + h * Math.sin(lat);
console.log(`${seen.length} station messages, antenna "${antenna || '?'}", broadcast antenna height ${s0.h.toFixed(4)} m, position ${moved ? 'MOVED during the flight' : 'constant'}`);
console.log(`base ECEF for rnx2rtkp -r: ${x.toFixed(4)} ${y.toFixed(4)} ${z.toFixed(4)}`);
