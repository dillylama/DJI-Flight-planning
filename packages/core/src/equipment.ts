import { D2R, dist } from './geo.ts';
import type { FlightWp } from './heights.ts';
import type { Plan } from './plan.ts';
import { legSpeed, type Route } from './route.ts';
import type { Camera } from './sensor.ts';
import { lensFovDeg } from './sensor.ts';
import type { Issue } from './validate.ts';

// Official figures: docs/equipment.md (enterprise.dji.com spec pages, checked 2026-09-25).

// ── DJI Matrice 400 ─────────────────────────────────────────────
export const M400_SPEC = {
  maxHorizontalMs: 25, maxAscentMs: 10, maxDescentMs: 8, maxWindMs: 12, maxPitchDeg: 35,
  maxFlightMinH30T: 59, maxHoverMinH30T: 53, maxTakeoffKg: 15.8, ceilingM: 7000,
};

// Conservative planning limits (ours, not DJI's): about 40–60 % of the published maximums, since a
// loaded M400 at altitude, in wind, carrying a 1.75 kg L3 has less margin than the spec-sheet aircraft.
export const M400_LIMITS = {
  lineSpeedWarnMs: 15, lineSpeedMaxMs: 20,        // 60 % / 80 % of 25 m/s
  transitSpeedWarnMs: 15,
  climbWarnMs: 4, climbMaxMs: 6,                   // 40 % / 60 % of 10 m/s
  descentWarnMs: 3, descentMaxMs: 5,               // ~40 % / ~60 % of 8 m/s
  bankWarnDeg: 25, bankMaxDeg: 30,                 // max pitch 35°
  windNoteMs: 12,
  sortieMinDefault: 30,                            // usable minutes per battery set with L3 (no official figure)
};

// ── Zenmuse L3 ──────────────────────────────────────────────────
export interface PulseMode { id: string; khz: number; maxAglM: number; maxRangeM: number; returns: string; note?: string }
export const L3_PULSE: PulseMode[] = [
  { id: '100', khz: 100, maxAglM: 500, maxRangeM: 1500, returns: '4/8/16', note: 'Range 950 m @10 % reflectivity' },
  { id: '350', khz: 350, maxAglM: 300, maxRangeM: 430, returns: '4/8/16', note: 'Range 700 m @10 % reflectivity' },
  { id: '1000', khz: 1000, maxAglM: 100, maxRangeM: 150, returns: '4/8', note: 'DJI: needs Real-Time Follow' },
  { id: '2000', khz: 2000, maxAglM: 50, maxRangeM: 75, returns: '4', note: 'DJI: needs Real-Time Follow' },
];
export interface ScanMode { id: string; name: string; fovH: number; fovV: number; use: string }
export const L3_SCAN: ScanMode[] = [
  { id: 'linear', name: 'Linear', fovH: 80, fovV: 3, use: 'terrain mapping (DJI accuracy spec uses linear)' },
  { id: 'star', name: 'Star-shaped', fovH: 80, fovV: 80, use: 'forest, dense urban' },
  { id: 'nonrep', name: 'Non-repetitive', fovH: 80, fovV: 80, use: 'power lines, forestry' },
];
export const L3_RANGE_CAP_M = 900;                 // default firmware cap unless unlocked by DJI

// ── Cameras ─────────────────────────────────────────────────────
const P1 = (mm: number): Camera => ({
  id: `p1-${mm}`, name: `Zenmuse P1 · ${mm} mm`, imgW: 8192, imgH: 5460,
  hfovDeg: lensFovDeg(35.9, mm), vfovDeg: lensFovDeg(24, mm), minIntervalS: 0.7,
  note: `45 MP full frame, 4.4 µm pixels; DJI GSD = H/${mm === 24 ? 55 : mm === 35 ? 80 : 114} cm/px`,
});
export const CAMERAS: Camera[] = [
  P1(24), P1(35), P1(50),
  { id: 'l3-100', name: 'Zenmuse L3 RGB · 100 MP', imgW: 12288, imgH: 8192, hfovDeg: 62, vfovDeg: 41.2, coverHfovDeg: 107, minIntervalS: 1,
    note: 'Dual 4/3 cameras, 62° × 41.2° each, 107° combined across-track; 1 s min interval' },
  { id: 'l3-25', name: 'Zenmuse L3 RGB · 25 MP', imgW: 6144, imgH: 4096, hfovDeg: 62, vfovDeg: 41.2, coverHfovDeg: 107, minIntervalS: 0.5,
    note: 'Dual 4/3 cameras, 25 MP mode; 0.5 s min interval' },
];

// ── Checks against the limits ──────────────────────────────────
export interface LimitContext {
  sensor: 'lidar' | 'photo';
  pulse?: PulseMode;
  scanFovH?: number;
  sortieMin: number;
  totalMin: number;                 // incl. transit + RTH
}

export function checkLimits(plan: Plan, flight: Route<FlightWp>, ctx: LimitContext): Issue[] {
  const L = M400_LIMITS, issues: Issue[] = [];
  const v = flight.speedMs;
  if (v > L.lineSpeedMaxMs) issues.push({ severity: 'error', code: 'SPEED', message: `Line speed ${v} m/s is above the conservative limit of ${L.lineSpeedMaxMs} m/s (M400 max ${M400_SPEC.maxHorizontalMs} m/s).` });
  else if (v > L.lineSpeedWarnMs) issues.push({ severity: 'warn', code: 'SPEED', message: `Line speed ${v} m/s is above ${L.lineSpeedWarnMs} m/s. Fine in calm air, but it leaves little margin in wind (M400 max ${M400_SPEC.maxHorizontalMs} m/s, wind limit ${M400_SPEC.maxWindMs} m/s).` });

  // Vertical speed on each leg = height change × speed / leg length
  let worstUp = 0, worstDown = 0, upAt = 0, downAt = 0;
  const wps = flight.wps;
  for (let i = 1; i < wps.length; i++) {
    const d = dist(wps[i].xy, wps[i - 1].xy);
    if (d < 1) continue;
    const vz = (wps[i].h - wps[i - 1].h) / d * legSpeed(wps, i - 1);
    if (vz > worstUp) { worstUp = vz; upAt = i; }
    if (-vz > worstDown) { worstDown = -vz; downAt = i; }
  }
  const vert = (what: string, val: number, warn: number, max: number, spec: number, at: number) => {
    val = +val.toFixed(6);                        // slow mode puts legs exactly on the limit; don't flag rounding noise
    if (val > max) issues.push({ severity: 'error', code: what.toUpperCase(), message: `${what} rate ${val.toFixed(1)} m/s at WP ${at + 1} exceeds the conservative limit ${max} m/s (M400 max ${spec} m/s). Lower the max climb/descent gradient or the speed.`, wps: [at] });
    else if (val > warn) issues.push({ severity: 'warn', code: what.toUpperCase(), message: `${what} rate reaches ${val.toFixed(1)} m/s at WP ${at + 1} (conservative limit ${warn} m/s, M400 max ${spec} m/s).`, wps: [at] });
  };
  vert('Climb', worstUp, L.climbWarnMs, L.climbMaxMs, M400_SPEC.maxAscentMs, upAt);
  vert('Descent', worstDown, L.descentWarnMs, L.descentMaxMs, M400_SPEC.maxDescentMs, downAt);

  if (plan.o.fig8BankDeg > L.bankMaxDeg) issues.push({ severity: 'error', code: 'BANK', message: `Figure-8 bank ${plan.o.fig8BankDeg}° is above ${L.bankMaxDeg}° (M400 max pitch ${M400_SPEC.maxPitchDeg}°).` });
  else if (plan.o.fig8BankDeg > L.bankWarnDeg) issues.push({ severity: 'warn', code: 'BANK', message: `Figure-8 bank ${plan.o.fig8BankDeg}° is steep; ${L.bankWarnDeg}° or less is recommended.` });

  if (ctx.sensor === 'lidar' && ctx.pulse) {
    const p = ctx.pulse;
    const lineAgl = wps.filter(w => w.role === 'line').map(w => w.h - w.terrainUnderWp);
    const maxAgl = Math.max(...lineAgl);
    if (plan.o.aglM > p.maxAglM) issues.push({ severity: 'error', code: 'L3_AGL', message: `Nominal AGL ${plan.o.aglM} m is above DJI's ${p.maxAglM} m maximum for ${p.khz} kHz. Use a lower pulse rate or fly lower.` });
    else if (maxAgl > p.maxAglM) {
      const lower = L3_PULSE.filter(q => q.khz < p.khz && q.maxAglM >= maxAgl).at(-1);
      issues.push({ severity: 'warn', code: 'L3_AGL', message: `Line AGL reaches ${maxAgl.toFixed(0)} m over low ground, above DJI's ${p.maxAglM} m for ${p.khz} kHz. Expect weaker returns there. ${lower ? `Consider ${lower.khz} kHz, or` : 'Lower the nominal AGL,'} tighten waypoint spacing, or fly lines along the slope.` });
    }
    const fov = ctx.scanFovH ?? plan.o.fovDeg;
    const slant = maxAgl / Math.cos((fov / 2) * D2R);
    const cap = Math.min(p.maxRangeM, L3_RANGE_CAP_M);
    if (slant > cap) issues.push({ severity: 'warn', code: 'L3_RANGE', message: `Slant range at the swath edge reaches ${slant.toFixed(0)} m (AGL ${maxAgl.toFixed(0)} m, ±${(fov / 2).toFixed(0)}°), beyond ${cap} m (${p.khz} kHz max distance / firmware cap). Edge points will drop out.` });
    if (p.khz >= 1000) issues.push({ severity: 'warn', code: 'L3_RTF', message: `${p.khz} kHz: DJI requires Real-Time Follow at this pulse rate. Explicit waypoint routes don't use it, so fly this mode only after a test.` });
  }

  if (ctx.totalMin > ctx.sortieMin) {
    const n = Math.ceil(ctx.totalMin / ctx.sortieMin);
    issues.push({ severity: 'warn', code: 'SORTIES', message: `Estimated ${ctx.totalMin.toFixed(0)} min is more than one battery set (${ctx.sortieMin} min usable): about ${n} sorties. See the sortie split.` });
  }
  return issues;
}
