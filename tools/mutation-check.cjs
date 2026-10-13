// Mutation check: break the planner on purpose, one fault at a time, and confirm the test suite notices.
//   node tools/mutation-check.cjs            (all)      node tools/mutation-check.cjs M6,M7   (some)
// Run it after changing heights, terrain, take-off, the writer or their tests. Every file is restored
// afterwards (also when a run throws). Do not edit packages/core/src while it runs.
const fs = require('fs');
const { spawnSync } = require('child_process');
const core = require('path').join(__dirname, '..', 'packages', 'core') + '/';

const mutations = [
  ['M1 bilinear instead of highest cell', 'src/terrain.ts', 'const up = elev.upper ?? elev;\n  const step = sampleStep', 'const up = elev;\n  const step = sampleStep'],
  ['M2 no cell padding', 'src/terrain.ts', 'half += cellPad(elev);', 'half += 0;'],
  ['M3 coarse sampling (120 m)', 'src/terrain.ts', '(elev.cell ? Math.min(maxStep, 0.7 * Math.min(...elev.cell)) : maxStep)', '120'],
  ['M4 heights 3 m low', 'src/heights.ts', 'const h = tMax.map(t => t + o.aglM);', 'const h = tMax.map(t => t + o.aglM - 3);'],
  ['M5 no geoid conversion in the file', 'src/wpml.ts', 'const ell = wps.map(w => egm96ToEllipsoid(w.lat, w.lon, w.h));', 'const ell = wps.map(w => w.h);'],
  ['M6 RTH checked from waypoints only', 'src/takeoff.ts', 'k = Math.max(0, Math.ceil(L / gap) - 1);', 'k = 0;'],
  ['M7 RTH origins 4 corridors apart', 'src/takeoff.ts', 'const gap = Math.max(30, corridor);', 'const gap = 4 * Math.max(30, corridor) + 200;'],
  ['M8 speed cap ignores the leg before', 'src/heights.ts', 'i > 0 ? lim[i - 1] : Infinity, i < n - 1 ? entry[i + 1] : Infinity', 'Infinity, i < n - 1 ? entry[i + 1] : Infinity'],
  ['M9 speed cap ignores the braking envelope', 'src/heights.ts', 'i > 0 ? lim[i - 1] : Infinity, i < n - 1 ? entry[i + 1] : Infinity', 'i > 0 ? lim[i - 1] : Infinity, Infinity'],
  ['M10 PixelIsPoint read as a corner', 'src/dem.ts', 'const west = pixelIsPoint ? ox - dLon / 2 : ox;', 'const west = ox;'],
  ['M11 leg ends not extended', 'src/terrain.ts', 'const e = ext > 0 ? ext + cellPad(elev) : cellPad(elev);', 'const e = 0;'],
  ['M12 gaps skipped', 'src/terrain.ts', 'if (h == null || !Number.isFinite(h)) throw new TerrainGapError(lon, lat);\n      if (h > m) m = h;', 'if (h == null || !Number.isFinite(h)) continue;\n      if (h > m) m = h;'],
  ['M13 lon/lat swapped in the file', 'src/wpml.ts', 'const coord = (w: FlightWp) => `${w.lon},${w.lat}`;', 'const coord = (w: FlightWp) => `${w.lat},${w.lon}`;'],
  ['M14 speeds rounded up', 'src/wpml.ts', 'Math.floor(v * 10 ** d + 1e-9)', 'Math.ceil(v * 10 ** d + 0.5)'],
  ['M15 level groups not levelled', 'src/heights.ts', '  level();\n\n  const speed', '\n  const speed'],
  ['M16 turn damping not kept inside the corridor', 'src/heights.ts', 'if (kh > 1e-9) damping[i] = Math.min(', 'if (false) damping[i] = Math.min('],
  ['M17 global height = lowest waypoint', 'src/wpml.ts', 'const globalHeight = Math.max(...wps.map(w => w.h));', 'const globalHeight = Math.min(...wps.map(w => w.h));'],
  ['M18 heights 1 m low in waylines only', 'src/wpml.ts', '<wpml:executeHeight>${f(ell[i])}</wpml:executeHeight>', '<wpml:executeHeight>${f(ell[i] - 1)}</wpml:executeHeight>'],
];

const run = () => spawnSync(process.execPath, ['--test', 'test/**/*.test.ts'], { cwd: core, encoding: 'utf8', timeout: 240000 });
const base = run(); const baseOut = (base.stdout || '') + (base.stderr || '');
const basePass = /^ℹ pass (\d+)/m.exec(baseOut), baseFail = /^ℹ fail (\d+)/m.exec(baseOut);
console.log('baseline (no mutation): pass', basePass && basePass[1], 'fail', baseFail && baseFail[1]);
if (!baseFail || baseFail[1] !== '0') { console.log(baseOut.slice(-1500)); process.exit(1); }
const results = [];
const only = process.argv[2]; const list = only ? mutations.filter(m => only.split(',').some(k => m[0].startsWith(k + ' '))) : mutations;
for (const [name, file, from, to] of list) {
  const p = core + file, original = fs.readFileSync(p, 'utf8');
  if (!original.includes(from)) { results.push([name, 'PATTERN NOT FOUND']); continue; }
  try {
    fs.writeFileSync(p, original.replace(from, () => to));
    const r = run();
    const out = (r.stdout || '') + (r.stderr || '');
    const failed = [...out.matchAll(/^✖ (.+?) \(/gm)].map(m => m[1]);
    const uniq = [...new Set(failed)];
    results.push([name, uniq.length ? `CAUGHT by ${uniq.length}: ` + uniq.map(s => s.slice(0, 70)).join(' | ') : 'NOT CAUGHT']);
  } finally {
    fs.writeFileSync(p, original);
  }
}
for (const [n, r] of results) console.log(`${r.startsWith('CAUGHT') ? 'ok   ' : 'FAIL '}${n}\n       ${r}`);
const missed = results.filter(([, r]) => !r.startsWith('CAUGHT'));
console.log(missed.length ? `\n${missed.length} fault(s) were NOT caught: the tests have a hole.` : `\nAll ${results.length} faults were caught.`);
process.exit(missed.length ? 1 : 0);
