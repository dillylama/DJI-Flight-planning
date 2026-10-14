// PPK camera positions for the L3's RGB photos, and a check of the PPK trajectory against the MRK.
//   node tools/ppk-cameras.ts <rtklib.pos> <photos.MRK> [--photos <dir of JPGs>] [--out cameras.csv] [--offsets apply|none]
// .pos: RTKLIB rnx2rtkp output, GPST, lat/lon/height (deg, ellipsoidal), Q (1 = fixed, 2 = float), ns, sd n/e/u.
// .MRK (L3): photo id, GPS seconds of week, [week], antenna→camera offset N/E/V in mm, lat, lon, ellipsoidal height,
//   sd n/e/v (m), Q (50 = RTK fixed). On the G3 flight (14 Oct 2026) a third of the lines had no exposure time
//   (-259200) and zero offsets, although every JPG carries drone-dji:UTCAtExposure to the microsecond (GPST, despite
//   the name: it equals the MRK time exactly) and the flight attitude. With --photos those photos are recovered: the
//   exposure time comes from the JPG, and the antenna→camera offset from a body-frame lever arm fitted per camera
//   (LV / RV) on the lines that do have offsets, rotated by the photo's yaw/pitch/roll (G3: fits to 7 mm).
// For each photo the PPK antenna position is interpolated at the exposure time, the offsets are applied (default),
// and, where the MRK has a position, compared with it. On an RTK-fixed flight the two should agree to a few cm; that
// agreement is the acceptance test for the pipeline. Positions are written to the CSV only, never printed.
import { readFileSync, writeFileSync, readdirSync } from 'node:fs';
import { join } from 'node:path';

const args = process.argv.slice(2);
const flag = (name: string) => { const i = args.indexOf('--' + name); return i >= 0 ? args.splice(i, 2)[1] : undefined; };
const outPath = flag('out'), offsets = flag('offsets') ?? 'apply', photoDir = flag('photos');
const [posPath, mrkPath] = args;
if (!posPath || !mrkPath) { console.error('usage: node tools/ppk-cameras.ts <rtklib.pos> <photos.MRK> [--photos dir] [--out cameras.csv] [--offsets apply|none]'); process.exit(2); }

const GPS0 = Date.UTC(1980, 0, 6);
const D2R = Math.PI / 180;
interface P { t: number; lat: number; lon: number; h: number; q: number; ns: number; sdn: number; sde: number; sdu: number }
const pos: P[] = [];
for (const line of readFileSync(posPath, 'utf8').split('\n')) {
  if (!line || line.startsWith('%')) continue;
  const c = line.trim().split(/\s+/);
  let t: number;
  const m = c[0].match(/(\d{4})\/(\d\d)\/(\d\d)/), tm = c[1].match(/(\d\d):(\d\d):(\d\d(?:\.\d+)?)/);
  if (m && tm) t = (Date.UTC(+m[1], +m[2] - 1, +m[3], +tm[1], +tm[2], 0) - GPS0) / 1000 + parseFloat(tm[3]);
  else if (/^\d{4}$/.test(c[0]) && /^\d+(\.\d+)?$/.test(c[1])) t = +c[0] * 604800 + +c[1];
  else continue;
  pos.push({ t, lat: +c[2], lon: +c[3], h: +c[4], q: +c[5], ns: +c[6], sdn: +c[7], sde: +c[8], sdu: +c[9] });
}
if (!pos.length) { console.error('no epochs in the .pos file'); process.exit(1); }
const fixed = pos.filter(p => p.q === 1).length;
console.log(`.pos: ${pos.length} epochs, ${fixed} fixed (${(100 * fixed / pos.length).toFixed(1)} %), ${pos.filter(p => p.q === 2).length} float`);

// MRK: every line (with or without a time); the id is like 0003_LV
interface M { id: string; cam: string; t: number | null; n: number; e: number; v: number; lat: number; lon: number; h: number; q: number; yaw?: number; pitch?: number; roll?: number }
const mrk = new Map<string, M>();
for (const line of readFileSync(mrkPath, 'utf8').split('\n')) {
  if (!/Ellh/.test(line)) continue;
  const c = line.split('\t').map(s => s.trim());
  const sow = parseFloat(c[1]), week = parseInt(c[2].replace(/[\[\]]/g, ''));
  const mm = (s: string) => parseInt(s.split(',')[0]) / 1000;
  const hasT = Number.isFinite(sow) && sow >= 0 && week > 0;
  mrk.set(c[0], { id: c[0], cam: c[0].slice(-2), t: hasT ? week * 604800 + sow : null, n: mm(c[3]), e: mm(c[4]), v: mm(c[5]), lat: parseFloat(c[6]), lon: parseFloat(c[7]), h: parseFloat(c[8]), q: parseInt(c[10]) });
}
const withT = [...mrk.values()].filter(m => m.t !== null);
console.log(`.MRK: ${mrk.size} photos, ${withT.length} with an exposure time (${withT.filter(m => m.q === 50).length} RTK fixed), ${mrk.size - withT.length} without`);

// Photos: exposure time (GPST) and attitude from the XMP block of each JPG
let recovered = 0;
if (photoDir) {
  const week0 = withT[0] ? Math.floor(withT[0].t! / 604800) : null;
  for (const f of readdirSync(photoDir)) {
    const idm = f.match(/_(\d{4}_[LR]V)\.JPG$/i); if (!idm) continue;
    const m = mrk.get(idm[1]); if (!m) continue;
    const buf = readFileSync(join(photoDir, f)); const s = buf.subarray(0, Math.min(buf.length, 6_000_000)).toString('latin1');
    const i = s.indexOf('<x:xmpmeta'), j = s.indexOf('</x:xmpmeta>'); if (i < 0 || j < 0) continue;
    const x = s.slice(i, j);
    const g = (k: string) => { const mm = x.match(new RegExp('drone-dji:' + k + '="([^"]*)"')); return mm ? parseFloat(mm[1]) : NaN; };
    m.yaw = g('FlightYawDegree'); m.pitch = g('FlightPitchDegree'); m.roll = g('FlightRollDegree');
    const tm = x.match(/UTCAtExposure="(\d{4})-(\d\d)-(\d\d)T(\d\d):(\d\d):(\d\d(?:\.\d+)?)"/);
    if (tm && m.t === null) {
      // the value is GPST (verified against the MRK and the flight record on G3), not UTC
      const t = (Date.UTC(+tm[1], +tm[2] - 1, +tm[3], +tm[4], +tm[5], 0) - GPS0) / 1000 + parseFloat(tm[6]);
      if (week0 === null || Math.floor(t / 604800) === week0) { m.t = t; recovered++; }
    }
  }
  console.log(`photos: ${recovered} exposure times recovered from JPG metadata`);
}

// Body-frame lever arm per camera from the MRK lines that have offsets and attitude: off_NED = R(yaw,pitch,roll) · l_body
const R = (yaw: number, pitch: number, roll: number) => {
  const cy = Math.cos(yaw * D2R), sy = Math.sin(yaw * D2R), cp = Math.cos(pitch * D2R), sp = Math.sin(pitch * D2R), cr = Math.cos(roll * D2R), sr = Math.sin(roll * D2R);
  return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]];
};
const lever = new Map<string, number[]>();
for (const cam of ['LV', 'RV']) {
  const rows = [...mrk.values()].filter(m => m.cam === cam && m.t !== null && (m.n || m.e || m.v) && Number.isFinite(m.yaw ?? NaN));
  if (rows.length < 4) continue;
  const A: number[][] = [], B: number[] = [];
  for (const m of rows) { const Rm = R(m.yaw!, m.pitch!, m.roll!); for (let k = 0; k < 3; k++) { A.push(Rm[k]); B.push([m.n, m.e, m.v][k]); } }
  const N = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
  for (let r = 0; r < A.length; r++) for (let i = 0; i < 3; i++) { for (let j = 0; j < 3; j++) N[i][j] += A[r][i] * A[r][j]; N[i][3] += A[r][i] * B[r]; }
  for (let i = 0; i < 3; i++) { const p = N[i][i]; for (let j = 0; j < 4; j++) N[i][j] /= p; for (let k = 0; k < 3; k++) if (k !== i) { const f = N[k][i]; for (let j = 0; j < 4; j++) N[k][j] -= f * N[i][j]; } }
  const l = [N[0][3], N[1][3], N[2][3]];
  let worst = 0;
  for (const m of rows) { const Rm = R(m.yaw!, m.pitch!, m.roll!); for (let k = 0; k < 3; k++) worst = Math.max(worst, Math.abs(Rm[k][0] * l[0] + Rm[k][1] * l[1] + Rm[k][2] * l[2] - [m.n, m.e, m.v][k])); }
  lever.set(cam, l);
  console.log(`lever arm ${cam} (body fwd/right/down): ${l.map(v => (v * 1000).toFixed(0)).join(' / ')} mm from ${rows.length} photos, worst residual ${(worst * 1000).toFixed(0)} mm`);
}

const first = withT[0] ?? [...mrk.values()][0];
const mLat = 110900, mLon = 111320 * Math.cos((first?.lat ?? pos[0].lat) * D2R);
const rows = ['photo,gpst_s,lat,lon,ellipsoidal_height,q,sd_n,sd_e,sd_u,offset_source,mrk_dn,mrk_de,mrk_dh'];
const dN: number[] = [], dE: number[] = [], dH: number[] = []; let outside = 0, notFixed = 0, written = 0, noOffset = 0;
for (const m of [...mrk.values()].sort((a, b) => (a.t ?? 0) - (b.t ?? 0))) {
  if (m.t === null) continue;
  let k = pos.findIndex(p => p.t >= m.t!);
  if (k <= 0 || k >= pos.length) { outside++; continue; }
  const a = pos[k - 1], b = pos[k], f = (m.t - a.t) / (b.t - a.t);
  if (b.t - a.t > 1.1) { outside++; continue; }
  const lerp = (x: number, y: number) => x + f * (y - x);
  let lat = lerp(a.lat, b.lat), lon = lerp(a.lon, b.lon), h = lerp(a.h, b.h);
  let n = m.n, e = m.e, v = m.v, src = 'mrk';
  if (!(n || e || v)) {
    const l = lever.get(m.cam);
    if (l && Number.isFinite(m.yaw ?? NaN)) { const Rm = R(m.yaw!, m.pitch!, m.roll!); [n, e, v] = [0, 1, 2].map(i => Rm[i][0] * l[0] + Rm[i][1] * l[1] + Rm[i][2] * l[2]); src = 'lever-arm'; }
    else { src = 'none'; noOffset++; }
  }
  if (offsets === 'apply') { lat += n / mLat; lon += e / mLon; h -= v; }   // V positive = camera below antenna
  const q = Math.max(a.q, b.q); if (q !== 1) notFixed++;
  let dn = NaN, de = NaN, dh = NaN;
  if (src === 'mrk') { dn = (lat - m.lat) * mLat; de = (lon - m.lon) * mLon; dh = h - m.h; dN.push(dn); dE.push(de); dH.push(dh); }
  rows.push(`${m.id},${m.t.toFixed(6)},${lat.toFixed(9)},${lon.toFixed(9)},${h.toFixed(3)},${q},${Math.max(a.sdn, b.sdn).toFixed(3)},${Math.max(a.sde, b.sde).toFixed(3)},${Math.max(a.sdu, b.sdu).toFixed(3)},${src},${Number.isFinite(dn) ? dn.toFixed(3) : ''},${Number.isFinite(de) ? de.toFixed(3) : ''},${Number.isFinite(dh) ? dh.toFixed(3) : ''}`);
  written++;
}
const stats = (a: number[]) => { if (!a.length) return 'no data'; const mean = a.reduce((x, y) => x + y, 0) / a.length; const rms = Math.sqrt(a.reduce((x, y) => x + y * y, 0) / a.length); return `mean ${(mean * 100).toFixed(1)} cm, rms ${(rms * 100).toFixed(1)} cm, spread ${(Math.min(...a) * 100).toFixed(1)}…${(Math.max(...a) * 100).toFixed(1)} cm`; };
console.log(`PPK (offsets ${offsets}) − MRK over ${dN.length} photos with MRK offsets (${outside} outside the solution, ${notFixed} of ${written} not fixed, ${noOffset} without any offset):`);
console.log(`  north ${stats(dN)}\n  east  ${stats(dE)}\n  up    ${stats(dH)}`);
if (outPath) { writeFileSync(outPath, rows.join('\n') + '\n'); console.log(`wrote ${outPath} (${written} cameras of ${mrk.size} photos)`); }
