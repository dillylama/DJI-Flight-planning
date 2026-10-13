import 'maplibre-gl/dist/maplibre-gl.css';
import './style.css';
import maplibregl, { type GeoJSONSource, type LngLatBoundsLike } from 'maplibre-gl';
import { MapboxOverlay } from '@deck.gl/mapbox';
import { PathLayer, LineLayer } from '@deck.gl/layers';
import {
  DEFAULTS, TAKEOFF_DEFAULTS, M400_LIMITS, M400_SPEC, L3_PULSE, L3_SCAN, CAMERAS, SAFETY,
  planLines, buildRoute, applyHeights, stats, validate, readGeoTiff, rasterElev, rasterBbox, rasterGaps,
  bufferedBbox, openTopoUrl, planTransit, coverage, checkLimits, planSorties, writeWpml, writeKmz,
  cameraFovDeg, gsdCm, aglForGsd, photoPlan, frontlapAtInterval, lidarDensity,
  safetyChecks, clearanceBudget, verifyRouteFiles,
  type PlanOptions, type Plan, type Route, type FlightWp, type RouteStats, type Issue, type ElevFn, type Bbox,
  type LonLat, type TakeoffOptions, type Transit, type Coverage, type Pt3, type Camera, type PhotoPlan, type SortiePlan,
  type ClearanceBudget, type VerifyReport, type WpmlFiles, type VerticalDatum,
} from '@3dm/core';
import JSZip from 'jszip';
import { aoiFromFile, type Aoi } from './aoi.ts';
import { demoPoly, demoElev } from './demo.ts';
import { renderProfile, type ProfilePt } from './profile.ts';
import { TIPS } from './tips.ts';
import { getBlob, putBlob } from './store.ts';
import { publish, newPairingCode, listDevices, type ApiCfg } from './sync.ts';

// ── State ─────────────────────────────────────────────────────────
// `synthetic` terrain is the demo's made-up ridge: fine for a preview, never for a route that is exported.
interface Dem { name: string; elev: ElevFn; bbox?: Bbox; synthetic: boolean; source: 'demo' | 'glo30' | 'file'; datum: VerticalDatum; gaps: number }
// An issue that belongs to one sortie's route (its waypoint numbers refer to that route).
type PlanIssue = Issue & { sortie?: number };
// One route file as it will be flown: built once, checked by the independent verifier, exported byte for byte.
interface RouteFile { name: string; sortie: number | null; fromLine: number; toLine: number; transit: Transit | null; files: WpmlFiles; report: VerifyReport }
interface Verified { ok: boolean; routes: RouteFile[]; ms: number }
interface Result {
  plan: Plan; route: Route; flight: Route<FlightWp> | null; st: RouteStats | null; issues: PlanIssue[];
  demError: string | null; transit: Transit | null; cov: Coverage | null; photo: PhotoPlan | null; totalMin: number | null;
  sp: SortiePlan | null; budget: ClearanceBudget | null; verify: Verified | null;
}
// What the map/profile show: the whole job, or one sortie.
interface View { plan: Plan; route: Route; flight: Route<FlightWp> | null; transit: Transit | null; cov: Coverage | null; issues: PlanIssue[]; lastLine: number }
type SensorKind = 'lidar' | 'photo';
interface SurveyCfg { sensor: SensorKind; pulse: string; scan: string; fig8: boolean; camera: string; frontlap: number; shutterInv: number; sortieMin: number; sortieOverlap: number; maxWp: number | null; djiCal: boolean; rgbPhotos: boolean; fig8End: boolean; uncertaintyM: number; demDatum: VerticalDatum }
const SURVEY_DEFAULTS: SurveyCfg = { sensor: 'lidar', pulse: '100', scan: 'linear', fig8: true, camera: 'p1-35', frontlap: 80, shutterInv: 1000, sortieMin: M400_LIMITS.sortieMinDefault, sortieOverlap: 0, maxWp: null, djiCal: true, rgbPhotos: true, fig8End: true, uncertaintyM: SAFETY.uncertaintyDefaultM, demDatum: 'orthometric' };
const DEMO_DEM: Dem = { name: 'Synthetic ridge (demo)', elev: demoElev, synthetic: true, source: 'demo', datum: 'orthometric', gaps: 0 };
const LAYERS = [
  ['aoi', 'Block outline'], ['lines', 'Data lines'], ['turns', 'Run-in/out & turns'], ['fig8', 'Figure-8 & approach'],
  ['wps', 'Waypoints'], ['rec', 'Record start/stop'], ['swath', 'Swath / photo footprint'], ['overlaps', 'Strip overlaps'], ['drops', 'Drop lines to terrain (3D)'],
  ['transit', 'Take-off transit'], ['rth', 'RTH path'],
] as const;
type LayerId = typeof LAYERS[number][0];

const STORE = '3dm.planner.v1';
type Saved = {
  aoi: Aoi | null; params: Partial<PlanOptions>; demo: boolean; home: LonLat | null; takeoff: Partial<TakeoffOptions>;
  view3d: boolean; survey: SurveyCfg; layers: Record<LayerId, boolean>;
};
const allOn = Object.fromEntries(LAYERS.map(([id]) => [id, true])) as Record<LayerId, boolean>;
const blank: Saved = { aoi: null, params: {}, demo: false, home: null, takeoff: {}, view3d: false, survey: SURVEY_DEFAULTS, layers: allOn };
const load = (): Saved => {
  try {
    const s = JSON.parse(localStorage.getItem(STORE) || '{}');
    return { ...blank, ...s, survey: { ...SURVEY_DEFAULTS, ...s.survey }, layers: { ...allOn, ...s.layers } };
  } catch { return blank; }
};
const saved = load();
const state = {
  ...saved,
  dem: (saved.demo ? DEMO_DEM : null) as Dem | null,
  resume: { on: false, fromLine: 1, speed: 14 },
  selLine: 0 as number | 'transit',
  selSortie: 'all' as 'all' | number,
  demBuf: null as ArrayBuffer | null,
  view: null as View | null,
  pickingHome: false,
  result: null as Result | null,
  planError: null as string | null,
};
const persist = () => {
  const { aoi, params, demo, home, takeoff, view3d, survey, layers } = state;
  try { localStorage.setItem(STORE, JSON.stringify({ aoi, params, demo, home, takeoff, view3d, survey, layers })); } catch { /* private mode */ }
};
const P = <K extends keyof PlanOptions>(k: K): PlanOptions[K] => (state.params[k] ?? DEFAULTS[k]) as PlanOptions[K];
const T = <K extends keyof TakeoffOptions>(k: K): TakeoffOptions[K] => (state.takeoff[k] ?? TAKEOFF_DEFAULTS[k]) as TakeoffOptions[K];
const camera = (): Camera => CAMERAS.find(c => c.id === state.survey.camera) ?? CAMERAS[1];
const pulse = () => L3_PULSE.find(p => p.id === state.survey.pulse) ?? L3_PULSE[0];
const scan = () => L3_SCAN.find(s => s.id === state.survey.scan) ?? L3_SCAN[0];
const isPhoto = () => state.survey.sensor === 'photo';

// ── Layout ────────────────────────────────────────────────────────
type Field<K> = { key: K; label: string; unit: string; step: number; min: number; max: number; only?: SensorKind; vmode?: 'slow' | 'raise' };
const FIELDS: Field<keyof PlanOptions>[] = [
  { key: 'aglM', label: 'AGL', unit: 'm', step: 10, min: SAFETY.aglMinM, max: 1000 },
  { key: 'speedMs', label: 'Line speed', unit: 'm/s', step: 0.5, min: 1, max: M400_LIMITS.lineSpeedMaxMs },
  { key: 'fovDeg', label: 'FOV used', unit: '°', step: 1, min: 10, max: 80, only: 'lidar' },
  { key: 'sidelapPct', label: 'Sidelap', unit: '%', step: 5, min: 0, max: 90 },
  { key: 'courseDeg', label: 'Line course', unit: '°', step: 1, min: 0, max: 359 },
  { key: 'runInM', label: 'Run-in', unit: 'm', step: 10, min: 20, max: 1000 },
  { key: 'runOutM', label: 'Run-out', unit: 'm', step: 10, min: 20, max: 1000 },
  { key: 'wpSpacingM', label: 'Max WP spacing', unit: 'm', step: 10, min: 20, max: 1000 },
  { key: 'corridorM', label: 'Terrain corridor ±', unit: 'm', step: 5, min: SAFETY.corridorMinM, max: 500 },
  { key: 'climbMs', label: 'Max climb rate', unit: 'm/s', step: 0.5, min: 0.5, max: M400_LIMITS.climbMaxMs, vmode: 'slow' },
  { key: 'descentMs', label: 'Max descent rate', unit: 'm/s', step: 0.5, min: 0.5, max: M400_LIMITS.descentMaxMs, vmode: 'slow' },
  { key: 'maxGradient', label: 'Max gradient', unit: 'rise/run', step: 0.01, min: 0.01, max: 0.5, vmode: 'raise' },
  { key: 'fig8BankDeg', label: 'Figure-8 bank', unit: '°', step: 1, min: 5, max: M400_LIMITS.bankMaxDeg, only: 'lidar' },
];
const TK_FIELDS: Field<'takeoffSecurityM' | 'transitSpeedMs' | 'minClearanceM'>[] = [
  { key: 'takeoffSecurityM', label: 'Take-off security height', unit: 'm', step: 5, min: 2, max: 1500 },
  { key: 'transitSpeedMs', label: 'Transit speed', unit: 'm/s', step: 0.5, min: 1, max: M400_LIMITS.lineSpeedMaxMs },
  { key: 'minClearanceM', label: 'Min terrain clearance', unit: 'm', step: 5, min: SAFETY.uncertaintyMinM + SAFETY.clearanceOverUncertaintyM, max: 500 },
];
const UNC_MAX = 200, RTH_MIN = 20, RTH_MAX = SAFETY.maxAboveHomeM;
// Settings restored from an earlier session (or written by an older version) are held to the same ranges
// as the input boxes: anything outside falls back to the default instead of being planned with.
(function sanitiseSaved() {
  const inRange = (v: unknown, min: number, max: number): v is number => typeof v === 'number' && Number.isFinite(v) && v >= min && v <= max;
  for (const f of FIELDS) if (state.params[f.key] != null && !inRange(state.params[f.key], f.min, f.max)) delete state.params[f.key];
  for (const f of TK_FIELDS) if (state.takeoff[f.key] != null && !inRange(state.takeoff[f.key], f.min, f.max)) delete state.takeoff[f.key];
  if (state.params.verticalMode != null && state.params.verticalMode !== 'slow' && state.params.verticalMode !== 'raise') delete state.params.verticalMode;
  for (const k of Object.keys(state.params) as (keyof PlanOptions)[]) if (!FIELDS.some(f => f.key === k) && k !== 'verticalMode') delete state.params[k];
  if (state.takeoff.rthHeightM != null && !inRange(state.takeoff.rthHeightM, RTH_MIN, RTH_MAX)) state.takeoff.rthHeightM = null;
  if (state.takeoff.flyToMode != null && state.takeoff.flyToMode !== 'safely' && state.takeoff.flyToMode !== 'pointToPoint') delete state.takeoff.flyToMode;
  const s = state.survey;
  if (!inRange(s.uncertaintyM, SAFETY.uncertaintyMinM, UNC_MAX)) s.uncertaintyM = SAFETY.uncertaintyDefaultM;
  if (s.demDatum !== 'orthometric' && s.demDatum !== 'ellipsoidal') s.demDatum = 'orthometric';
  if (!inRange(s.sortieMin, 5, 50)) s.sortieMin = M400_LIMITS.sortieMinDefault;
  if (!inRange(s.sortieOverlap, 0, 2)) s.sortieOverlap = 0;
  if (s.maxWp != null && !inRange(s.maxWp, 50, 65535)) s.maxWp = null;
  if (!inRange(s.frontlap, 50, 95)) s.frontlap = SURVEY_DEFAULTS.frontlap;
  if (!inRange(s.shutterInv, 100, 8000)) s.shutterInv = SURVEY_DEFAULTS.shutterInv;
  if (state.home && !(inRange(state.home[0], -180, 180) && inRange(state.home[1], -90, 90))) state.home = null;
})();
const esc = (s: string) => s.replace(/&/g, '&amp;').replace(/"/g, '&quot;').replace(/</g, '&lt;');
const ti = (k: string) => TIPS[k] ? `<i class="ti" tabindex="0" data-tip="${esc(TIPS[k].t)}"${TIPS[k].rec ? ` data-rec="${esc(TIPS[k].rec!)}"` : ''}>i</i>` : '';
const btnTip = (k: string) => TIPS[k] ? ` data-tip="${esc(TIPS[k].t)}"` : '';
const fieldHtml = (f: Field<string>, group: string) =>
  `<label${f.only ? ` data-only="${f.only}"` : ''}${f.vmode ? ` data-vmode="${f.vmode}"` : ''}><span class="lt">${f.label} <em>${f.unit}</em>${ti(f.key)}</span><input type="number" data-group="${group}" data-key="${f.key}" step="${f.step}" min="${f.min}" max="${f.max}" /></label>`;
const opt = (v: string, label: string) => `<option value="${v}">${label}</option>`;

const app = document.getElementById('app')!;
app.innerHTML = `
<header>
  <div class="brand"><span class="mark">3DM</span> Planner <span class="sub">M400 · Zenmuse L3 / P1</span></div>
  <div class="blockname" id="blockName">No block loaded</div>
  <div class="head-actions">
    <button id="exportJson" class="ghost" disabled${btnTip('exportJson')}>Export mission.json</button>
    <button id="exportKmz" class="ghost" disabled>Export KMZ</button>
  </div>
</header>
<aside>
  <section>
    <h2>Block</h2>
    <label class="drop"><input type="file" id="fileBlock" accept=".kmz,.kml" hidden />
      <strong>Drop KMZ / KML</strong><span>or click to choose. Largest polygon is used.</span></label>
    <button id="loadDemo" class="link">Load demo block (synthetic Nimba ridge)</button>
    <div class="note" id="blockInfo"></div>
  </section>
  <section>
    <h2>Terrain</h2>
    <label class="lbl" for="otKey">OpenTopography API key ${ti('otKey')}</label>
    <div class="row"><input id="otKey" type="password" placeholder="Paste your key" autocomplete="off" />
      <button id="fetchDem" disabled${btnTip('fetchDem')}>Fetch GLO-30</button></div>
    <a class="small-link" href="https://portal.opentopography.org/requestService?service=api" target="_blank" rel="noopener">Get a free key ↗</a>
    <label class="drop small"><input type="file" id="fileDem" accept=".tif,.tiff" hidden />
      <span>…or drop a GeoTIFF DEM (EPSG:4326, metres)</span></label>
    <label class="lbl" for="demDatum" style="margin-top:8px">Heights in a dropped GeoTIFF are ${ti('demDatum')}</label>
    <select id="demDatum"><option value="orthometric">above mean sea level (EGM96 / EGM2008 / local)</option><option value="ellipsoidal">ellipsoidal (WGS84), e.g. from PPK or LiDAR</option></select>
    <div class="note" id="demInfo"></div>
  </section>
  <section>
    <h2>Sensor ${ti('sensor')}</h2>
    <div class="seg" id="sensorSeg"><button data-sensor="lidar">LiDAR · Zenmuse L3</button><button data-sensor="photo">Photogrammetry</button></div>
    <div class="fields" data-only="lidar">
      <label><span class="lt">Pulse rate ${ti('lidarMode')}</span><select id="pulseSel">${L3_PULSE.map(p => opt(p.id, `${p.khz} kHz · AGL < ${p.maxAglM} m`)).join('')}</select></label>
      <label><span class="lt">Scan mode</span><select id="scanSel">${L3_SCAN.map(s => opt(s.id, `${s.name} ${s.fovH}°×${s.fovV}°`)).join('')}</select></label>
      <label class="check span2"><input type="checkbox" id="fig8On" /> IMU figure-8 before lines and resumes</label>
      <label class="check span2"><input type="checkbox" id="fig8EndOn" /> IMU figure-8 at the end too ${ti('fig8End')}</label>
      <label class="check span2"><input type="checkbox" id="djiCalOn" /> DJI IMU calibration at start and end ${ti('djiCal')}</label>
      <label class="check span2"><input type="checkbox" id="rgbOn" /> L3 RGB photos on data lines ${ti('rgbPhotos')}</label>
    </div>
    <div class="fields" data-only="photo">
      <label class="span2"><span class="lt">Camera ${ti('camera')}</span><select id="camSel">${CAMERAS.map(c => opt(c.id, c.name)).join('')}</select></label>
      <label><span class="lt">Target GSD <em>cm/px</em>${ti('gsdCm')}</span><input type="number" id="gsdIn" step="0.1" min="0.3" max="20" /></label>
      <label><span class="lt">Frontlap <em>%</em>${ti('frontlapPct')}</span><input type="number" id="frontlapIn" step="5" min="50" max="95" /></label>
      <label><span class="lt">Exposure <em>1/x s</em>${ti('shutterS')}</span><input type="number" id="shutterIn" step="100" min="100" max="8000" /></label>
    </div>
    <div class="sensor-note" id="sensorNote"></div>
  </section>
  <section>
    <h2>Flight</h2>
    <div class="fields" id="fields">${FIELDS.map(f => fieldHtml(f, 'plan')).join('')}
      <label><span class="lt">Terrain + position uncertainty <em>m</em>${ti('uncertaintyM')}</span><input type="number" id="uncIn" step="5" min="${SAFETY.uncertaintyMinM}" max="${UNC_MAX}" /></label>
      <label class="span2"><span class="lt">Vertical profile ${ti('verticalMode')}</span>
        <select id="vmodeSel"><option value="slow">Follow terrain; slow down to the climb/descent limits</option><option value="raise">Keep line speed; raise waypoints (gradient limit)</option></select></label>
    </div>
    <div class="row"><button id="optCourse" class="ghost" disabled${btnTip('optCourse')}>Optimise course</button><button id="resetParams" class="link">Reset defaults</button></div>
  </section>
  <section>
    <h2>Home, take-off &amp; aircraft</h2>
    <div class="row"><button id="setHome" class="ghost"${btnTip('setHome')}>Set home on map</button><span class="note inline" id="homeInfo">Not set</span></div>
    <div class="fields" style="margin-top:10px">
      ${TK_FIELDS.map(f => fieldHtml(f, 'takeoff')).join('')}
      <label><span class="lt">RTH height <em>m</em>${ti('rthHeightM')}</span><input type="number" id="rthHeight" step="10" min="${RTH_MIN}" max="${RTH_MAX}" placeholder="auto" /></label>
      <label class="span2"><span class="lt">Fly to route ${ti('flyToMode')}</span>
        <select id="flyToMode"><option value="safely">Safely: climb, then fly level</option><option value="pointToPoint">Point to point: climb, then slope</option></select></label>
      <label class="span2"><span class="lt">Usable time per battery set <em>min</em><i class="ti" tabindex="0" data-tip="Planning endurance per sortie, landing reserve already removed. DJI quotes 59 min flight / 53 min hover for the M400 with the light H30T only; there is no official figure with the 1.75 kg L3." data-rec="${M400_LIMITS.sortieMinDefault} min with L3 until you have your own logs.">i</i></span><input type="number" id="sortieIn" step="1" min="5" max="50" /></label>
      <label><span class="lt">Lines re-flown <em>per split</em>${ti('sortieOverlap')}</span><input type="number" id="overlapIn" step="1" min="0" max="2" /></label>
      <label><span class="lt">Max WPs per route${ti('maxWp')}</span><input type="number" id="maxWpIn" step="50" min="50" placeholder="unknown" /></label>
    </div>
    <div class="sensor-note">M400 limits used (conservative): line speed ≤ ${M400_LIMITS.lineSpeedWarnMs} m/s (max ${M400_SPEC.maxHorizontalMs}), climb ≤ ${M400_LIMITS.climbWarnMs} m/s (max ${M400_SPEC.maxAscentMs}), descent ≤ ${M400_LIMITS.descentWarnMs} m/s (max ${M400_SPEC.maxDescentMs}), wind limit ${M400_SPEC.maxWindMs} m/s.</div>
  </section>
  <section>
    <h2>Resume</h2>
    <label class="check"><input type="checkbox" id="resOn" /> Build a resume route ${ti('resOn')}</label>
    <div class="fields">
      <label><span class="lt">Data stopped on line ${ti('resLine')}</span><input type="number" id="resLine" min="1" value="1" /></label>
      <label><span class="lt">New speed <em>m/s</em>${ti('resSpeed')}</span><input type="number" id="resSpeed" step="0.5" min="1" max="20" value="14" /></label>
    </div>
  </section>
  <section>
    <h2>Sync to RC ${ti('sync')}</h2>
    <label class="lbl" for="apiUrl">Sync server URL</label>
    <input id="apiUrl" type="url" placeholder="https://…" autocomplete="off" />
    <label class="lbl" for="apiToken" style="margin-top:8px">Office token ${ti('apiToken')}</label>
    <input id="apiToken" type="password" placeholder="Admin token" autocomplete="off" />
    <div class="row"><button id="publishBtn" disabled>Publish to RC</button><button id="pairBtn" class="ghost">Pair an RC</button></div>
    <div class="note" id="syncInfo"></div>
    <div class="pair-code" id="pairCode" hidden></div>
  </section>
</aside>
<main>
  <div id="map"></div>
  <div class="legend">
    <span><i style="background:var(--c-line)"></i>Data line</span><span><i style="background:var(--c-run)"></i>Run-in/out, turn</span>
    <span><i style="background:var(--c-fig8)"></i>Figure-8</span><span><i style="background:var(--c-appr)"></i>Approach / exit</span>
    <span><i style="background:var(--c-transit)"></i>Transit</span><span><i style="background:var(--c-rth)"></i>RTH</span>
    <span><i style="background:var(--c-swath)"></i>Swath</span><span><i style="background:var(--c-overlap)"></i>Strip overlap</span>
    <span><i class="dot wp"></i>Waypoint (zoom in)</span><span><i class="dot" style="background:var(--c-line)"></i>Record start</span><span><i class="dot" style="background:var(--c-appr)"></i>Record stop</span>
    <span><i class="dot flag"></i>Flagged WP (see Checks)</span>
  </div>
  <div class="map-toggles">
    <button id="view3d" class="ghost"${btnTip('view3d')}>3D</button>
    <select id="sortieSel" title="Show the whole job or one sortie"></select>
    <button id="layersBtn" class="ghost"${btnTip('layers')}>Layers ▾</button>
    <div class="layers-panel" id="layersPanel" hidden>${LAYERS.map(([id, label]) => `<label class="check"><input type="checkbox" data-layer="${id}" /> ${label}</label>`).join('')}</div>
  </div>
  <div class="pick-hint" id="pickHint" hidden>Click the map to set home · Esc to cancel</div>
  <section class="bottom">
    <div class="stats" id="stats"></div>
    <div class="profile-wrap">
      <div class="profile-head"><h2>Profile</h2><select id="lineSel"></select><span class="prof-info" id="profInfo"></span>
        <span class="key"><i class="k-flight"></i>Flight <i class="k-floor"></i><span id="floorLabel">Terrain + AGL</span> <i class="k-terr"></i>Terrain</span></div>
      <svg id="profile"></svg>
    </div>
    <div class="issues" id="issues"></div>
  </section>
</main>
<dialog id="preflight"><form method="dialog">
  <h2 id="pfTitle">Before this route is flown</h2>
  <div id="pfBody"></div>
  <label class="check"><input type="checkbox" id="pfAck" /> I have read this and will set these values on the RC before take-off.</label>
  <div class="row"><button type="button" class="ghost" id="pfCancel">Cancel</button><button type="button" id="pfOk" disabled>Continue</button></div>
</form></dialog>
<div id="tooltip" role="tooltip" hidden></div>`;

const $ = <T extends HTMLElement = HTMLElement>(id: string) => document.getElementById(id) as T;

// ── Tooltips: only on the small ⓘ icons and buttons, after a short delay ──
const tipEl = $('tooltip');
let tipTimer = 0;
function showTip(t: HTMLElement) {
  tipEl.innerHTML = '';
  tipEl.append(document.createTextNode(t.dataset.tip!));
  if (t.dataset.rec) { const r = document.createElement('div'); r.className = 'rec'; r.textContent = 'Recommended: ' + t.dataset.rec; tipEl.append(r); }
  tipEl.hidden = false;
  const r = t.getBoundingClientRect(), w = tipEl.offsetWidth, h = tipEl.offsetHeight;
  let x = r.right + 8, y = r.top - 4;
  if (x + w > innerWidth - 8) x = Math.max(8, r.left - w - 8);
  if (y + h > innerHeight - 8) y = innerHeight - h - 8;
  tipEl.style.left = x + 'px'; tipEl.style.top = y + 'px';
}
document.addEventListener('mouseover', e => {
  const t = (e.target as HTMLElement).closest<HTMLElement>('.ti, button[data-tip], .stats [data-tip]');
  clearTimeout(tipTimer);
  if (!t) { tipEl.hidden = true; return; }
  tipTimer = window.setTimeout(() => showTip(t), 350);
});
document.addEventListener('focusin', e => { const t = e.target as HTMLElement; if (t.classList?.contains('ti')) showTip(t); });
document.addEventListener('focusout', () => { tipEl.hidden = true; });

// ── Inputs ────────────────────────────────────────────────────────
function syncFields() {
  document.querySelectorAll<HTMLInputElement>('input[data-group]').forEach(i => {
    i.value = String(i.dataset.group === 'plan' ? P(i.dataset.key as keyof PlanOptions) : T(i.dataset.key as keyof TakeoffOptions));
  });
  $<HTMLInputElement>('rthHeight').value = state.takeoff.rthHeightM != null ? String(state.takeoff.rthHeightM) : '';
  $<HTMLSelectElement>('flyToMode').value = T('flyToMode');
  const s = state.survey;
  $<HTMLSelectElement>('pulseSel').value = s.pulse;
  $<HTMLSelectElement>('scanSel').value = s.scan;
  $<HTMLInputElement>('fig8On').checked = s.fig8;
  $<HTMLInputElement>('fig8EndOn').checked = s.fig8End;
  $<HTMLInputElement>('fig8EndOn').disabled = !s.fig8;
  $<HTMLSelectElement>('vmodeSel').value = P('verticalMode');
  document.querySelectorAll<HTMLElement>('aside [data-vmode]').forEach(el => { el.hidden = el.dataset.vmode !== P('verticalMode'); });
  $<HTMLInputElement>('djiCalOn').checked = s.djiCal;
  $<HTMLInputElement>('rgbOn').checked = s.rgbPhotos;
  $<HTMLSelectElement>('camSel').value = s.camera;
  $<HTMLInputElement>('frontlapIn').value = String(s.frontlap);
  $<HTMLInputElement>('shutterIn').value = String(s.shutterInv);
  $<HTMLInputElement>('sortieIn').value = String(s.sortieMin);
  $<HTMLInputElement>('overlapIn').value = String(s.sortieOverlap);
  $<HTMLInputElement>('maxWpIn').value = s.maxWp != null ? String(s.maxWp) : '';
  $<HTMLInputElement>('uncIn').value = String(s.uncertaintyM);
  $<HTMLSelectElement>('demDatum').value = s.demDatum;
  document.querySelectorAll<HTMLInputElement>('aside input.bad').forEach(i => { i.classList.remove('bad'); i.title = ''; });
  $<HTMLInputElement>('gsdIn').value = gsdCm(camera(), P('aglM')).toFixed(2);
  document.querySelectorAll<HTMLButtonElement>('#sensorSeg button').forEach(b => b.classList.toggle('active', b.dataset.sensor === s.sensor));
  document.querySelectorAll<HTMLElement>('aside [data-only]').forEach(el => { el.hidden = el.dataset.only !== s.sensor; });
  document.querySelectorAll<HTMLInputElement>('[data-layer]').forEach(i => { i.checked = state.layers[i.dataset.layer as LayerId]; });
  renderSensorNote();
}
function renderSensorNote() {
  const el = $('sensorNote');
  if (isPhoto()) {
    const c = camera();
    el.textContent = `${c.note ?? ''}. Across-track FOV ${cameraFovDeg(c).toFixed(1)}°, min interval ${c.minIntervalS} s.`;
  } else {
    const p = pulse(), s = scan();
    el.textContent = `${p.khz} kHz: DJI max AGL ${p.maxAglM} m, max distance ${p.maxRangeM} m, returns ${p.returns}${p.note ? `, ${p.note}` : ''}. ${s.name} ${s.fovH}°×${s.fovV}°: ${s.use}. Swath uses "FOV used" (edge-trimmed from ${s.fovH}°).`;
  }
}
const change = () => { persist(); syncFields(); schedule(); };
// A number box only changes the plan while its value is inside its own min…max. Outside that it is marked
// and the last valid value stays in force, so a half-typed or mistyped figure is never planned with.
function numberIn(i: HTMLInputElement): number | null {
  const v = Number(i.value);
  const ok = i.value.trim() !== '' && Number.isFinite(v) && (i.min === '' || v >= Number(i.min)) && (i.max === '' || v <= Number(i.max));
  i.classList.toggle('bad', !ok);
  i.title = ok ? '' : `Allowed: ${i.min || '…'} to ${i.max || '…'}. The last valid value is still in use.`;
  return ok ? v : null;
}
document.querySelector('aside')!.addEventListener('input', e => {
  const i = e.target as HTMLInputElement;
  if (!i.dataset.group) return;
  const v = numberIn(i);
  if (v == null) return;
  if (i.dataset.group === 'plan') state.params[i.dataset.key as keyof PlanOptions] = v as never;
  else state.takeoff[i.dataset.key as keyof TakeoffOptions] = v as never;
  persist(); schedule();
  if (i.dataset.key === 'aglM') $<HTMLInputElement>('gsdIn').value = gsdCm(camera(), v).toFixed(2);
});
$('rthHeight').addEventListener('input', e => {
  const i = e.target as HTMLInputElement;
  if (i.value.trim() === '') { i.classList.remove('bad'); i.title = ''; state.takeoff.rthHeightM = null; }
  else { const v = numberIn(i); if (v == null) return; state.takeoff.rthHeightM = v; }
  persist(); schedule();
});
$('uncIn').addEventListener('input', e => { const v = numberIn(e.target as HTMLInputElement); if (v != null) { state.survey.uncertaintyM = v; persist(); schedule(); } });
$('flyToMode').addEventListener('change', e => { state.takeoff.flyToMode = (e.target as HTMLSelectElement).value as TakeoffOptions['flyToMode']; persist(); schedule(); });
$('resetParams').onclick = () => { state.params = {}; change(); };
$('sensorSeg').addEventListener('click', e => {
  const b = (e.target as HTMLElement).closest<HTMLButtonElement>('button[data-sensor]');
  if (!b || b.dataset.sensor === state.survey.sensor) return;
  state.survey.sensor = b.dataset.sensor as SensorKind;
  state.params.sidelapPct = isPhoto() ? 70 : 50;             // sensible starting overlap per sensor
  if (isPhoto()) state.params.aglM = Math.min(1000, Math.max(SAFETY.aglMinM, Math.round(aglForGsd(camera(), 2.5))));
  change();
});
$('pulseSel').addEventListener('change', e => { state.survey.pulse = (e.target as HTMLSelectElement).value; change(); });
$('scanSel').addEventListener('change', e => { state.survey.scan = (e.target as HTMLSelectElement).value; change(); });
$('fig8On').addEventListener('change', e => { state.survey.fig8 = (e.target as HTMLInputElement).checked; change(); });
$('fig8EndOn').addEventListener('change', e => { state.survey.fig8End = (e.target as HTMLInputElement).checked; change(); });
$('vmodeSel').addEventListener('change', e => { state.params.verticalMode = (e.target as HTMLSelectElement).value as PlanOptions['verticalMode']; change(); });
$('djiCalOn').addEventListener('change', e => { state.survey.djiCal = (e.target as HTMLInputElement).checked; change(); });
$('rgbOn').addEventListener('change', e => { state.survey.rgbPhotos = (e.target as HTMLInputElement).checked; change(); });
$('camSel').addEventListener('change', e => { state.survey.camera = (e.target as HTMLSelectElement).value; change(); });
$('gsdIn').addEventListener('input', e => {
  const i = e.target as HTMLInputElement, g = numberIn(i);
  if (g == null) return;
  const agl = Math.round(aglForGsd(camera(), g)), f = FIELDS.find(x => x.key === 'aglM')!;
  if (agl < f.min || agl > f.max) { i.classList.add('bad'); i.title = `This GSD needs ${agl} m AGL; allowed is ${f.min} to ${f.max} m.`; return; }
  state.params.aglM = agl;
  persist(); schedule();
  const a = document.querySelector<HTMLInputElement>('input[data-key=aglM]'); if (a) a.value = String(state.params.aglM);
});
$('frontlapIn').addEventListener('input', e => { const v = numberIn(e.target as HTMLInputElement); if (v != null) { state.survey.frontlap = v; persist(); schedule(); } });
$('shutterIn').addEventListener('input', e => { const v = numberIn(e.target as HTMLInputElement); if (v != null) { state.survey.shutterInv = v; persist(); schedule(); } });
$('sortieIn').addEventListener('input', e => { const v = numberIn(e.target as HTMLInputElement); if (v != null) { state.survey.sortieMin = v; persist(); schedule(); } });
$('overlapIn').addEventListener('input', e => { const v = numberIn(e.target as HTMLInputElement); if (v != null && Number.isInteger(v)) { state.survey.sortieOverlap = v; persist(); schedule(); } });
$('maxWpIn').addEventListener('input', e => {
  const i = e.target as HTMLInputElement;
  if (i.value.trim() === '') { i.classList.remove('bad'); i.title = ''; state.survey.maxWp = null; }
  else { const v = numberIn(i); if (v == null || !Number.isInteger(v)) return; state.survey.maxWp = v; }
  persist(); schedule();
});
$('sortieSel').addEventListener('change', e => { const v = (e.target as HTMLSelectElement).value; selectSortie(v === 'all' ? 'all' : Number(v)); });

// ── Map ───────────────────────────────────────────────────────────
const css = (v: string) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const rgba = (v: string, a = 255): [number, number, number, number] => {
  const h = css(v).replace('#', '');
  return [parseInt(h.slice(0, 2), 16), parseInt(h.slice(2, 4), 16), parseInt(h.slice(4, 6), 16), a];
};
const map = new maplibregl.Map({
  container: 'map',
  style: {
    version: 8,
    sources: {
      sat: {
        type: 'raster', tileSize: 256, maxzoom: 19,
        tiles: ['https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'],
        attribution: 'Imagery © Esri, Maxar, Earthstar Geographics',
      },
      terrain3d: {
        type: 'raster-dem', tileSize: 256, maxzoom: 15, encoding: 'terrarium',
        tiles: ['https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png'],
        attribution: '3D ground: AWS Terrain Tiles (Mapzen)',
      },
    },
    layers: [{ id: 'sat', type: 'raster', source: 'sat', paint: { 'raster-saturation': -0.4, 'raster-brightness-max': 0.78 } }],
  },
  // Open framed on the saved block (no waiting for tiles), else the Western Cape.
  ...(state.aoi ? {
    bounds: [
      [Math.min(...state.aoi.poly.map(p => p[0])), Math.min(...state.aoi.poly.map(p => p[1]))],
      [Math.max(...state.aoi.poly.map(p => p[0])), Math.max(...state.aoi.poly.map(p => p[1]))],
    ] as LngLatBoundsLike,
    fitBoundsOptions: { padding: 60 },
  } : { center: [22.03, -33.21] as [number, number], zoom: 5 }),
  maxPitch: 80, attributionControl: { compact: true },
});
map.addControl(new maplibregl.NavigationControl({ visualizePitch: true }), 'top-right');
map.addControl(new maplibregl.ScaleControl({ unit: 'metric' }), 'bottom-right');
if (import.meta.env.DEV) Object.assign(window, { __map: map, __state: state });   // dev-only debugging handles
const deck = new MapboxOverlay({ interleaved: true, layers: [] });
map.addControl(deck);

const empty: GeoJSON.FeatureCollection = { type: 'FeatureCollection', features: [] };
const roleColor = () => ['match', ['get', 'role'],
  'line', css('--c-line'), 'fig8', css('--c-fig8'), 'approach', css('--c-appr'), 'exit', css('--c-appr'), css('--c-run')] as unknown as string;
const visibleRoles = () => [
  ...(state.layers.lines ? ['line'] : []), ...(state.layers.turns ? ['runin', 'runout'] : []), ...(state.layers.fig8 ? ['fig8', 'approach', 'exit'] : []),
];

let mapReady = false;
function onStyleReady() {   // style ready: add our layers without waiting for every tile
  for (const id of ['aoi', 'route', 'wps', 'rec', 'swath', 'overlap', 'transit', 'rth']) map.addSource(id, { type: 'geojson', data: empty });
  map.addLayer({ id: 'swath-fill', type: 'fill', source: 'swath', paint: { 'fill-color': css('--c-swath'), 'fill-opacity': 0.14 } });
  map.addLayer({ id: 'overlap-fill', type: 'fill', source: 'overlap', paint: { 'fill-color': css('--c-overlap'), 'fill-opacity': 0.38 } });
  map.addLayer({ id: 'overlap-line', type: 'line', source: 'overlap', paint: { 'line-color': css('--c-overlap'), 'line-width': 1, 'line-opacity': 0.8 } });
  map.addLayer({ id: 'aoi-fill', type: 'fill', source: 'aoi', paint: { 'fill-color': css('--c-aoi'), 'fill-opacity': 0.06 } });
  map.addLayer({ id: 'aoi-line', type: 'line', source: 'aoi', paint: { 'line-color': css('--c-aoi'), 'line-width': 2 } });
  map.addLayer({ id: 'route-line', type: 'line', source: 'route', layout: { 'line-join': 'round', 'line-cap': 'round' },
    paint: { 'line-color': roleColor(), 'line-width': ['case', ['get', 'sel'], 4.5, 2], 'line-opacity': ['case', ['get', 'sel'], 1, 0.9] } });
  map.addLayer({ id: 'transit-line', type: 'line', source: 'transit', paint: { 'line-color': css('--c-transit'), 'line-width': 2.5, 'line-dasharray': [2, 1.5] } });
  map.addLayer({ id: 'rth-line', type: 'line', source: 'rth', paint: { 'line-color': css('--c-rth'), 'line-width': 2, 'line-dasharray': [1, 1.5] } });
  map.addLayer({ id: 'route-hit', type: 'line', source: 'route', paint: { 'line-color': '#000', 'line-opacity': 0, 'line-width': 14 } });
  map.addLayer({ id: 'wps', type: 'circle', source: 'wps', minzoom: 12,
    paint: { 'circle-radius': ['case', ['get', 'flag'], 6, 3], 'circle-color': roleColor(),
      'circle-stroke-color': ['case', ['get', 'flag'], css('--c-err'), '#1c1c1c'], 'circle-stroke-width': ['case', ['get', 'flag'], 2.5, 1] } });
  map.addLayer({ id: 'rec', type: 'circle', source: 'rec',
    paint: { 'circle-radius': 7, 'circle-color': ['case', ['get', 'start'], css('--c-line'), css('--c-appr')], 'circle-stroke-color': '#fff', 'circle-stroke-width': 2 } });
  map.on('click', 'route-hit', e => {
    if (state.pickingHome) return;
    const line = e.features?.[0]?.properties?.line;
    if (line != null && line !== '') selectLine(Number(line));
  });
  map.on('mouseenter', 'route-hit', () => { if (!state.pickingHome) map.getCanvas().style.cursor = 'pointer'; });
  map.on('mouseleave', 'route-hit', () => { if (!state.pickingHome) map.getCanvas().style.cursor = ''; });
  const recPopup = new maplibregl.Popup({ closeButton: false, offset: 10 });
  map.on('mouseenter', 'rec', e => { const f = e.features?.[0]; if (f) recPopup.setLngLat((f.geometry as GeoJSON.Point).coordinates as [number, number]).setText(f.properties.label).addTo(map); });
  map.on('mouseleave', 'rec', () => recPopup.remove());
  const ovPopup = new maplibregl.Popup({ closeButton: false, offset: 6 });
  map.on('mousemove', 'overlap-fill', e => { const f = e.features?.[0]; if (f) ovPopup.setLngLat(e.lngLat).setText(f.properties.label).addTo(map); });
  map.on('mouseleave', 'overlap-fill', () => ovPopup.remove());
  mapReady = true;
  applyView(false);
  if (state.aoi) fitAoi();
  schedule();
}
// An inline style can finish loading before this line runs, so check first, then fall back to the event.
if (map.isStyleLoaded()) onStyleReady(); else map.once('style.load', onStyleReady);
const src = (id: string) => map.getSource(id) as GeoJSONSource | undefined;

function fitAoi() {
  if (!state.aoi || !mapReady) return;
  const pts = [...state.aoi.poly, ...(state.home ? [state.home] : [])];
  const lons = pts.map(p => p[0]), lats = pts.map(p => p[1]);
  map.fitBounds([[Math.min(...lons), Math.min(...lats)], [Math.max(...lons), Math.max(...lats)]] as LngLatBoundsLike, { padding: 60, duration: 600, pitch: state.view3d ? 60 : 0 });
}

// ── Home marker + picking ────────────────────────────────────────
const homeEl = document.createElement('div');
homeEl.className = 'home-marker'; homeEl.textContent = 'H'; homeEl.title = 'Home / take-off (drag to move)';
const homeMarker = new maplibregl.Marker({ element: homeEl, draggable: true });
homeMarker.on('dragend', () => { const p = homeMarker.getLngLat(); setHome([p.lng, p.lat]); });
function setHome(p: LonLat | null) {
  state.home = p; persist();
  if (p) homeMarker.setLngLat(p).addTo(map); else homeMarker.remove();
  schedule();
}
if (state.home) homeMarker.setLngLat(state.home).addTo(map);
function setPicking(on: boolean) {
  state.pickingHome = on;
  $('pickHint').hidden = !on;
  $('setHome').classList.toggle('active', on);
  map.getCanvas().style.cursor = on ? 'crosshair' : '';
}
$('setHome').onclick = () => setPicking(!state.pickingHome);
map.on('click', e => { if (state.pickingHome) { setPicking(false); setHome([e.lngLat.lng, e.lngLat.lat]); } });
window.addEventListener('keydown', e => { if (e.key === 'Escape') { setPicking(false); $('layersPanel').hidden = true; } });

// ── View: 2D/3D + layers ─────────────────────────────────────────
function applyView(animate = true) {
  $('view3d').classList.toggle('active', state.view3d);
  if (!mapReady) return;
  map.setTerrain(state.view3d ? { source: 'terrain3d', exaggeration: 1 } : null);
  const vis = (on: boolean) => (on ? 'visible' : 'none');
  const L = state.layers, flat = !state.view3d;
  map.setLayoutProperty('aoi-fill', 'visibility', vis(L.aoi));
  map.setLayoutProperty('aoi-line', 'visibility', vis(L.aoi));
  map.setLayoutProperty('swath-fill', 'visibility', vis(L.swath));
  map.setLayoutProperty('overlap-fill', 'visibility', vis(L.overlaps));
  map.setLayoutProperty('overlap-line', 'visibility', vis(L.overlaps));
  map.setLayoutProperty('route-line', 'visibility', vis(flat));
  map.setFilter('route-line', ['in', ['get', 'role'], ['literal', visibleRoles()]]);
  map.setFilter('route-hit', ['in', ['get', 'role'], ['literal', visibleRoles()]]);
  map.setLayoutProperty('wps', 'visibility', vis(flat && L.wps));
  map.setFilter('wps', ['in', ['get', 'role'], ['literal', visibleRoles()]]);
  map.setLayoutProperty('rec', 'visibility', vis(L.rec));
  map.setLayoutProperty('transit-line', 'visibility', vis(flat && L.transit));
  map.setLayoutProperty('rth-line', 'visibility', vis(flat && L.rth));
  if (animate) map.easeTo({ pitch: state.view3d ? 60 : 0, bearing: state.view3d ? map.getBearing() || -20 : 0, duration: 800 });
  renderDeck(state.view);
}
$('view3d').onclick = () => { state.view3d = !state.view3d; persist(); applyView(); };
$('layersBtn').onclick = () => { $('layersPanel').hidden = !$('layersPanel').hidden; };
$('layersPanel').addEventListener('change', e => {
  const i = e.target as HTMLInputElement;
  state.layers[i.dataset.layer as LayerId] = i.checked; persist(); applyView(false);
});
document.addEventListener('click', e => {
  if (!(e.target as HTMLElement).closest('.map-toggles')) $('layersPanel').hidden = true;
});

// ── Compute ──────────────────────────────────────────────────────
let timer = 0;
function schedule() { clearTimeout(timer); timer = window.setTimeout(run, 60); }
const planOpts = (): Partial<PlanOptions> => (isPhoto() ? { ...state.params, fovDeg: cameraFovDeg(camera()) } : state.params);
const routeOpts = (fromLine?: number) => ({
  fig8: !isPhoto() && state.survey.fig8,
  fig8End: !isPhoto() && state.survey.fig8 && state.survey.fig8End,
  ...(state.resume.on && fromLine != null ? { fromLine, speedMs: state.resume.speed } : {}),
});

// Checks whose answer depends on the individual route, repeated for every sortie.
const PER_ROUTE = new Set(['SHORT_LEG', 'DAMPING', 'AGL_LOW', 'TURN_CORRIDOR', 'TOO_STEEP', 'GRADIENT', 'CLIMB', 'DESCENT', 'CLEARANCE_BUDGET', 'MAX_ALTITUDE']);

function compute(): Result | null {
  if (!state.aoi) return null;
  const plan = planLines(state.aoi.poly, planOpts());
  if (!plan.lines.length) return null;
  const fromLine = state.resume.on ? Math.min(plan.lines.length, Math.max(1, state.resume.fromLine)) - 1 : undefined;
  const route = buildRoute(plan, routeOpts(fromLine));
  const r: Result = { plan, route, flight: null, st: null, issues: [], demError: null, transit: null, cov: null, photo: null, totalMin: null, sp: null, budget: null, verify: null };
  if (isPhoto()) r.photo = photoPlan(camera(), plan.o.aglM, route.speedMs, state.survey.frontlap, 1 / state.survey.shutterInv);
  if (!state.dem) return r;
  // Demo terrain belongs to the demo block only. Belt and braces: setAoi already drops it for any other block.
  if (state.dem.synthetic && !state.demo) { state.dem = null; state.demBuf = null; updateDemInfo(); return r; }
  try {
    r.flight = applyHeights(plan, route, state.dem.elev);
  } catch (e) {
    r.demError = (e as Error).message + '. The DEM must cover the whole route with no gaps: block, run-ins, figure-8s and the terrain corridor around them. Fetch GLO-30 again for this route, or load a larger DEM.';
    return r;
  }
  r.st = stats(plan, r.flight);
  r.issues = validate(plan, r.flight).filter(i => !(isPhoto() && i.code === 'FIG8'));
  r.cov = coverage(plan, r.flight);
  if (state.dem.synthetic) r.issues.push({ severity: 'error', code: 'DEMO_TERRAIN', message: 'Demo terrain is made up: this plan is a preview only and cannot be exported or published. Load your own block and fetch GLO-30 (or drop a DEM) to plan a real flight.' });
  const clr = T('minClearanceM'), unc = state.survey.uncertaintyM;
  r.budget = clearanceBudget(plan, r.flight, unc);
  r.issues.push(...safetyChecks(plan, r.flight, { uncertaintyM: unc, minClearanceM: clr }));
  // Transit and RTH checks for one route; `tag` names the sortie when there are several.
  const transitIssues = (t: Transit, tag: string, sortie?: number): PlanIssue[] => {
    const out: PlanIssue[] = [];
    if (t.minClearanceM < clr) out.push({
      severity: 'error', code: 'TRANSIT_CLEARANCE', sortie,
      message: `${tag}Take-off transit clears terrain by only ${t.minClearanceM.toFixed(0)} m (need ${clr} m). ${T('flyToMode') === 'pointToPoint' ? 'The sloping point-to-point leg cuts into terrain: switch to "Safely" or raise the take-off security height.' : 'Raise the take-off security height or move home.'}`,
    });
    if (t.rthWorstClearanceM < clr) out.push({
      severity: 'error', code: 'RTH_CLEARANCE', sortie,
      message: `${tag}RTH at ${t.rthHeightM} m above home would clear terrain by only ${t.rthWorstClearanceM.toFixed(0)} m on the straight line home from WP ${t.rthWorstWp + 1}. Recommended RTH height ≥ ${t.rthRecommendedM} m.`, wps: [t.rthWorstWp],
    });
    if (t.rthHeightM > RTH_MAX) out.push({ severity: 'error', code: 'RTH_HIGH', sortie, message: `${tag}The RTH height needed is ${t.rthHeightM} m above home, more than the ${RTH_MAX} m the aircraft can be set to. Move home closer to the block's high ground.` });
    return out;
  };
  if (state.home) {
    try {
      const t = (r.transit = planTransit(plan, r.flight, state.dem.elev, state.home, state.takeoff));
      r.issues.push(...transitIssues(t, ''));
      if (T('transitSpeedMs') > M400_LIMITS.transitSpeedWarnMs) r.issues.push({ severity: 'warn', code: 'TRANSIT_SPEED', message: `Transit speed ${T('transitSpeedMs')} m/s is above the conservative ${M400_LIMITS.transitSpeedWarnMs} m/s.` });
    } catch (e) {
      r.issues.push({ severity: 'error', code: 'HOME', message: (e as Error).message + '. The DEM must cover the home point and every straight line from the route back to it.' });
    }
  } else {
    r.issues.push({ severity: 'error', code: 'NO_HOME', message: 'Home is not set, so the take-off transit, the RTH height and the height above take-off cannot be checked. Use "Set home on map". Export stays blocked until it is set.' });
  }
  const t = r.transit;
  r.totalMin = r.st.flightMin + (t ? t.timeS / 60 + t.rthDistanceM / T('transitSpeedMs') / 60 : 0);
  const limitCtx = { sensor: state.survey.sensor, pulse: isPhoto() ? undefined : pulse(), scanFovH: isPhoto() ? undefined : plan.o.fovDeg, sortieMin: state.survey.sortieMin };
  r.issues.push(...checkLimits(plan, r.flight, { ...limitCtx, totalMin: r.totalMin, topAboveHomeM: t?.topAboveHomeM }).filter(i => i.code !== 'SORTIES'));
  try {
    r.sp = planSorties(plan, state.dem.elev, state.home, state.takeoff, {
      usableMin: state.survey.sortieMin, firstLine: route.startIdx, speedMs: route.speedMs, fig8: !isPhoto() && state.survey.fig8,
      fig8End: !isPhoto() && state.survey.fig8 && state.survey.fig8End,
      overlapLines: state.survey.sortieOverlap, maxWaypoints: state.survey.maxWp,
      climbMs: P('climbMs'), descentMs: P('descentMs'), full: r.flight,
    });
    r.issues.push(...r.sp.issues);
    // Each sortie is flown as its own route, with its own heights, figure-8s, transit and RTH: check each one.
    // (With a single sortie the route is the whole job, already checked above.)
    if (r.sp.sorties.length > 1) for (const so of r.sp.sorties) {
      const tag = `Sortie ${so.index + 1}: `;
      const own = [
        ...validate(plan, so.route, { maxWaypoints: state.survey.maxWp }),
        ...checkLimits(plan, so.route, { ...limitCtx, totalMin: so.time.total, topAboveHomeM: so.transit?.topAboveHomeM }),
        ...safetyChecks(plan, so.route, { uncertaintyM: unc, minClearanceM: clr }),
      ].filter(i => i.severity !== 'info' && PER_ROUTE.has(i.code));
      for (const i of own) r.issues.push({ ...i, message: tag + i.message, sortie: so.index });
      if (so.transit) r.issues.push(...transitIssues(so.transit, tag, so.index));
    }
  } catch (e) {
    r.issues.push({ severity: 'error', code: 'SORTIES', message: 'Sortie split failed: ' + (e as Error).message });
  }
  if (r.photo) {
    const p = r.photo, c = camera();
    if (p.intervalS < c.minIntervalS) r.issues.push({ severity: 'error', code: 'PHOTO_INTERVAL', message: `Photos needed every ${p.intervalS.toFixed(2)} s, faster than the ${c.name} minimum ${c.minIntervalS} s. Fly ≤ ${p.maxSpeedMs.toFixed(1)} m/s, lower the frontlap, or fly higher.` });
    if (p.blurPx > 1) r.issues.push({ severity: 'error', code: 'BLUR', message: `Motion blur ${p.blurPx.toFixed(2)} px at 1/${state.survey.shutterInv} s. Use a faster exposure or slow down (≤ 0.5 px recommended).` });
    else if (p.blurPx > 0.5) r.issues.push({ severity: 'warn', code: 'BLUR', message: `Motion blur ${p.blurPx.toFixed(2)} px; ≤ 0.5 px recommended.` });
    const gMax = gsdCm(c, r.st.aglOnLineMax);
    if (gMax > p.gsdCm * 1.3) r.issues.push({ severity: 'warn', code: 'GSD_RANGE', message: `GSD degrades to ${gMax.toFixed(1)} cm/px over low ground (target ${p.gsdCm.toFixed(1)}). Tighter waypoint spacing or lines along the slope keep it even.` });
  }
  return r;
}

function viewOf(r: Result | null): View | null {
  if (!r) return null;
  const so = state.selSortie !== 'all' ? r.sp?.sorties[state.selSortie] : undefined;
  // Waypoint numbers in an issue refer to the route it was raised on: the whole job, or one sortie.
  if (!so) return { plan: r.plan, route: r.route, flight: r.flight, transit: r.transit, cov: r.cov, issues: r.issues.filter(i => i.sortie == null), lastLine: r.plan.lines.length - 1 };
  const cov = r.cov && { ...r.cov, swaths: r.cov.swaths.filter(s => s.line >= so.fromLine && s.line <= so.toLine) };
  return { plan: r.plan, route: so.route, flight: so.route, transit: so.transit, cov, issues: r.issues.filter(i => i.sortie === so.index), lastLine: so.toLine };
}
function renderSortieSel(r: Result | null) {
  const sel = $<HTMLSelectElement>('sortieSel');
  const n = r?.sp?.sorties.length ?? 0;
  if (state.selSortie !== 'all' && state.selSortie >= n) state.selSortie = 'all';
  sel.hidden = n < 2;
  sel.innerHTML = '<option value="all">Whole job</option>' + (r?.sp?.sorties ?? []).map(so =>
    `<option value="${so.index}">Sortie ${so.index + 1} · L${so.fromLine + 1}–${so.toLine + 1}</option>`).join('');
  sel.value = String(state.selSortie);
}
function selectSortie(i: 'all' | number) {
  state.selSortie = i;
  const so = i !== 'all' ? state.result?.sp?.sorties[i] : undefined;
  if (so) state.selLine = so.fromLine;
  renderView();
  renderStats(state.result);
}
function renderView() {
  const v = (state.view = viewOf(state.result));
  renderSortieSel(state.result);
  renderMap(v);
  renderDeck(v);
  renderLineSel(v);
  renderProf();
}

// ── The exported files and their independent check ───────────────
// The routes that would be flown: every sortie (a job that fits one battery is a single sortie).
function buildRouteFiles(r: Result): Verified {
  const t0 = performance.now(), o = r.plan.o, slow = o.verticalMode === 'slow';
  const base = safeName(state.aoi!.name) + (state.resume.on ? `_resume-L${state.resume.fromLine}` : '');
  const sorties = r.sp?.sorties ?? [];
  const routes: RouteFile[] = sorties.map(so => {
    const files = kmzFor(so.route, o.aglM);
    const report = verifyRouteFiles(files.templateKml, files.waylinesWpml, {
      elev: state.dem!.elev, aglM: o.aglM, corridorM: o.corridorM, lidar: true,
      maxClimbMs: slow ? o.climbMs : M400_LIMITS.climbMaxMs, maxDescentMs: slow ? o.descentMs : M400_LIMITS.descentMaxMs,
      maxSpeedMs: M400_LIMITS.lineSpeedMaxMs, vertAccelMs2: SAFETY.vertAccelMs2,
    });
    const t = so.transit;
    const tags = t ? `_RTH${t.rthHeightM.toFixed(0)}_ALT${maxAltSetting(t)}` : '';
    const part = sorties.length > 1 ? `_S${String(so.index + 1).padStart(2, '0')}_L${so.fromLine + 1}-${so.toLine + 1}` : '';
    return { name: `${base}${part}${tags}.kmz`, sortie: sorties.length > 1 ? so.index : null, fromLine: so.fromLine, toLine: so.toLine, transit: t, files, report };
  });
  return { ok: routes.length > 0 && routes.every(x => x.report.ok), routes, ms: performance.now() - t0 };
}
// The aircraft's Max Altitude setting that covers the route, the take-off transit and the RTH height
// (20 m spare, rounded up to 10 m).
const maxAltSetting = (t: Transit) => Math.ceil((Math.max(t.topAboveHomeM, t.rthHeightM) + 20) / 10) * 10;
const hasErrors = (r: Result | null) => !r?.flight || !!r.demError || r.issues.some(i => i.severity === 'error');

let verifyTimer = 0;
function verifyNow(r: Result) {
  if (r.verify || hasErrors(r) || !state.dem || state.dem.synthetic || kmzBlocker()) return;
  try {
    r.verify = buildRouteFiles(r);
  } catch (e) {
    r.issues.push({ severity: 'error', code: 'V_FAILED', message: 'The route files could not be built and checked: ' + (e as Error).message });
    return;
  }
  for (const f of r.verify.routes) for (const i of f.report.issues) {
    r.issues.push({ ...i, sortie: f.sortie ?? undefined, message: `${f.sortie != null ? `Sortie ${f.sortie + 1}: ` : ''}File check: ${i.message}` });
  }
}
function scheduleVerify() {
  clearTimeout(verifyTimer);
  const r = state.result;
  if (!r) return;
  verifyTimer = window.setTimeout(() => {
    if (state.result !== r) return;
    verifyNow(r);
    updateButtons(); renderStats(r); renderIssues(r); renderView();
  }, 350);
}

function updateButtons() {
  const r = state.result;
  const errors = hasErrors(r), kmzBlock = kmzBlocker();
  const checking = !errors && !kmzBlock && !r!.verify;
  const verified = r?.verify?.ok === true;
  // A plan that has route files may only leave the planner once those files have passed the independent check.
  const blocked = errors || (!kmzBlock && !verified);
  const why = !r?.flight ? 'Load a block and terrain first.' : errors ? 'Blocked: see the errors under Checks.' : checking ? 'Checking the route files…' : null;
  const ej = $<HTMLButtonElement>('exportJson'), pb = $<HTMLButtonElement>('publishBtn'), kb = $<HTMLButtonElement>('exportKmz');
  ej.disabled = pb.disabled = blocked;
  kb.disabled = blocked || !!kmzBlock;
  kb.textContent = checking ? 'Checking files…' : 'Export KMZ';
  kb.dataset.tip = kmzBlock ?? why ?? 'Download the DJI KMZ of every sortie (template.kml + waylines.wpml), exactly the files that passed the independent check. Import into Pilot 2 or push from 3DM Fly.';
  ej.dataset.tip = why ?? TIPS.exportJson.t;
}

function run() {
  let r: Result | null = null;
  state.planError = null;
  // If anything in the computation fails, there is NO result: nothing stale may stay exportable.
  try { r = compute(); } catch (e) { state.planError = (e as Error).message || String(e); }
  state.result = r;
  updateButtons();
  $<HTMLButtonElement>('optCourse').disabled = !state.aoi;
  $<HTMLButtonElement>('fetchDem').disabled = !state.aoi;
  $('blockName').textContent = state.aoi ? state.aoi.name : 'No block loaded';
  $('homeInfo').textContent = !state.home ? 'Not set'
    : `${state.home[1].toFixed(5)}, ${state.home[0].toFixed(5)}${r?.transit ? ` · ground ${r.transit.homeElev.toFixed(0)} m` : ''}`;
  renderView();
  renderStats(r);
  renderIssues(r);
  scheduleVerify();
}

// ── Render: 2D map ───────────────────────────────────────────────
const lonlat = (plan: Plan, w: { xy: [number, number] }) => plan.proj.inv(w.xy[0], w.xy[1]);
interface Seg { role: string; line: number | null; coords: number[][] }
function segments(r: View, withZ: boolean): Seg[] {
  const src3 = r.flight?.wps ?? r.route.wps;
  const pos = (i: number) => {
    const w = src3[i]; const [lon, lat] = lonlat(r.plan, w);
    return withZ && 'h' in w ? [lon, lat, (w as FlightWp).h] : [lon, lat];
  };
  const out: Seg[] = [];
  let cur: Seg | null = null;
  for (let i = 1; i < src3.length; i++) {
    const w = src3[i], line = w.line ?? null;
    if (!cur || cur.role !== w.role || cur.line !== line) {
      if (cur) out.push(cur);
      cur = { role: w.role, line, coords: [pos(i - 1)] };
    }
    cur.coords.push(pos(i));
  }
  if (cur) out.push(cur);
  return out;
}
const lineFc = (pts: Pt3[] | undefined): GeoJSON.FeatureCollection | GeoJSON.Feature =>
  pts ? { type: 'Feature', properties: {}, geometry: { type: 'LineString', coordinates: pts.map(p => [p.lon, p.lat]) } } : empty;

function renderMap(r: View | null) {
  if (!mapReady) return;
  const aoi = state.aoi;
  src('aoi')!.setData(aoi ? { type: 'Feature', properties: {}, geometry: { type: 'Polygon', coordinates: [[...aoi.poly, aoi.poly[0]]] } } : empty);
  if (!r) { for (const id of ['route', 'wps', 'rec', 'swath', 'overlap', 'transit', 'rth']) src(id)!.setData(empty); return; }
  const flagged = new Set(r.issues.filter(i => i.severity !== 'info').flatMap(i => i.wps ?? []));
  src('route')!.setData({ type: 'FeatureCollection', features: segments(r, false).map(c => ({
    type: 'Feature', properties: { role: c.role, line: c.line ?? '', sel: c.line === state.selLine }, geometry: { type: 'LineString', coordinates: c.coords } })) });
  src('wps')!.setData({ type: 'FeatureCollection', features: r.route.wps.map((w, i) => ({
    type: 'Feature', properties: { role: w.role, flag: flagged.has(i) }, geometry: { type: 'Point', coordinates: lonlat(r.plan, w) } })) });
  const verb = isPhoto() ? 'photo capture' : 'point-cloud recording';
  src('rec')!.setData({ type: 'FeatureCollection', features: r.route.wps.flatMap((w, i) => w.actions.map(a => ({
    type: 'Feature' as const, properties: { start: a === 'START_RECORD', label: `WP ${i + 1}: ${a === 'START_RECORD' ? 'start' : 'stop'} ${verb}` },
    geometry: { type: 'Point' as const, coordinates: lonlat(r.plan, w) } }))) });
  src('swath')!.setData({ type: 'FeatureCollection', features: (r.cov?.swaths ?? []).map(s => ({
    type: 'Feature', properties: { line: s.line },
    geometry: { type: 'Polygon', coordinates: [[...s.left, ...[...s.right].reverse(), s.left[0]]] } })) });
  src('overlap')!.setData({ type: 'FeatureCollection', features: (r.cov?.overlaps ?? []).map(o => ({
    type: 'Feature',
    properties: { label: `Lines ${o.lines[0] + 1}–${o.lines[1] + 1} overlap: ${fmt(o.widthMinM)}–${fmt(o.widthMaxM)} m (${fmt(o.pctMin)}–${fmt(o.pctMax)} % of the narrower strip)` },
    geometry: { type: 'Polygon', coordinates: [[...o.poly, o.poly[0]]] } })) });
  src('transit')!.setData(lineFc(r.transit?.path));
  src('rth')!.setData(lineFc(r.transit?.rthFromLast));
}

// ── Render: 3D (deck.gl, true heights) ───────────────────────────
function renderDeck(r: View | null) {
  if (!state.view3d || !r?.flight) { deck.setProps({ layers: [] }); return; }
  const col: Record<string, [number, number, number, number]> = {
    line: rgba('--c-line'), fig8: rgba('--c-fig8'), approach: rgba('--c-appr'), exit: rgba('--c-appr'), runin: rgba('--c-run'), runout: rgba('--c-run'),
  };
  const roles = new Set(visibleRoles());
  const segs = segments(r, true).filter(s => roles.has(s.role));
  const L = state.layers, t = r.transit;
  const drops = L.drops ? r.flight.wps.filter(w => w.role === 'line').map(w => ({ from: [w.lon, w.lat, w.h], to: [w.lon, w.lat, w.terrainUnderWp] })) : [];
  deck.setProps({
    layers: [
      new LineLayer({ id: 'drops', data: drops, getSourcePosition: d => d.from as [number, number, number], getTargetPosition: d => d.to as [number, number, number],
        getColor: [230, 230, 228, 70], getWidth: 1, widthUnits: 'pixels' }),
      new PathLayer<Seg>({ id: 'route3d', data: segs, getPath: d => d.coords as [number, number, number][],
        getColor: d => (d.line === state.selLine ? [255, 255, 255, 255] : col[d.role] ?? col.runin),
        getWidth: d => (d.line === state.selLine ? 5 : 3), widthUnits: 'pixels', jointRounded: true, capRounded: true,
        updateTriggers: { getColor: state.selLine, getWidth: state.selLine },
        pickable: true, onClick: ({ object }) => { if (object?.line != null) selectLine(object.line); } }),
      ...(t && L.transit ? [new PathLayer<Pt3[]>({ id: 'transit3d', data: [t.path], getPath: d => d.map(p => [p.lon, p.lat, p.h] as [number, number, number]),
        getColor: rgba('--c-transit'), getWidth: 3, widthUnits: 'pixels' })] : []),
      ...(t && L.rth ? [new PathLayer<Pt3[]>({ id: 'rth3d', data: [t.rthFromLast], getPath: d => d.map(p => [p.lon, p.lat, p.h] as [number, number, number]),
        getColor: rgba('--c-rth'), getWidth: 2, widthUnits: 'pixels' })] : []),
    ],
  });
}

// ── Render: stats, checks, profile ───────────────────────────────
const fmt = (v: number, d = 0) => v.toLocaleString('en-ZA', { minimumFractionDigits: d, maximumFractionDigits: d });
function renderStats(r: Result | null) {
  const el = $('stats');
  if (!r) { el.innerHTML = `<div class="empty">${state.planError ? 'No plan: see Checks.' : 'Load a block to plan.'}</div>`; return; }
  const { plan, route, st, cov, photo } = r;
  type Row = [string, string, string?] | string;
  const rows: Row[] = ['Block',
    ['Area', `${fmt(plan.areaHa)} ha`], ['Lines', `${plan.lines.length}`],
    [photo ? 'Footprint / spacing' : 'Swath / spacing', `${fmt(plan.swath)} / ${fmt(plan.spacing)} m`, 'At nominal AGL. Spacing = swath × (1 − sidelap).'],
    ['Waypoints', `${fmt(route.wps.length)}`],
  ];
  if (photo) {
    const c = camera();
    rows.push('Photogrammetry',
      ['GSD (nominal)', `${fmt(photo.gsdCm, 2)} cm/px`],
      ...(st ? [['GSD on lines', `${fmt(gsdCm(c, st.aglOnLineMin), 2)}–${fmt(gsdCm(c, st.aglOnLineMax), 2)} cm/px`, 'From the real AGL range on the data lines.'] as Row] : []),
      ['Photo spacing', `${fmt(photo.photoSpacingM, 1)} m · every ${fmt(photo.intervalS, 2)} s`, `Footprint along ${fmt(photo.footprintAlongM)} m × (1 − frontlap).`],
      ['Max speed (interval)', `${fmt(photo.maxSpeedMs, 1)} m/s`, `Fastest speed the ${c.minIntervalS} s minimum interval allows at this frontlap.`],
      ['Motion blur', `${fmt(photo.blurPx, 2)} px`],
      ...(st ? [['Photos (approx.)', fmt(st.dataKm * 1000 / photo.photoSpacingM)] as Row] : []),
    );
  } else {
    const p = pulse(), d = lidarDensity(p.khz * 1000, route.speedMs, plan.swath, plan.spacing);
    rows.push('LiDAR · L3',
      ['Pulse rate', `${p.khz} kHz`],
      ['Density per strip', `${fmt(d.perStrip)} pts/m²`, 'PRR ÷ (speed × swath) at nominal AGL, first return only, uniform spread assumed.'],
      ['Density with sidelap', `${fmt(d.total)} pts/m²`, 'PRR ÷ (speed × line spacing): all overlapping strips combined.'],
      ...(cov ? [['Min strip density', `${fmt(p.khz * 1000 / (route.speedMs * cov.swathMaxM))} pts/m²`, 'At the widest swath (highest AGL) on the lines.'] as Row] : []),
      ...(cov ? [['Sidelap achieved', `${fmt(cov.achievedMinPct)}–${fmt(cov.achievedMaxPct)} %`, 'Using each line waypoint\'s real AGL. Assumes level ground across-track.'] as Row] : []),
      ...(cov ? [['Swath on lines', `${fmt(cov.swathMinM)}–${fmt(cov.swathMaxM)} m`] as Row] : []),
    );
    const rgb = CAMERAS.find(c => c.id === 'l3-100')!;
    rows.push(['L3 RGB GSD / frontlap', `${fmt(gsdCm(rgb, plan.o.aglM), 1)} cm · ${fmt(frontlapAtInterval(rgb, plan.o.aglM, route.speedMs, rgb.minIntervalS))} %`, 'L3 100 MP mapping cameras at nominal AGL, shooting every 1 s (their minimum).']);
  }
  if (st) rows.push('Flight',
    ['Route', `${fmt(st.routeKm, 1)} km (${fmt(st.dataKm, 1)} on lines)`],
    ['Height (orthometric)', `${fmt(st.hMin)}–${fmt(st.hMax)} m`],
    ['AGL on lines', `${fmt(st.aglOnLineMin)}–${fmt(st.aglOnLineMax)} m`, 'Height above the terrain directly under each line waypoint.'],
    ['Line speed', st.lineSpeedMin === st.lineSpeedMax ? `${fmt(st.lineSpeedMax, 1)} m/s` : `${fmt(st.lineSpeedMin, 1)}–${fmt(st.lineSpeedMax, 1)} m/s`, 'Speed range on the data lines. Legs are slowed where the terrain climb or descent would exceed the rate limits.'],
    ...(st.slowedLegs ? [['Slowed legs', `${st.slowedLegs} · ${fmt(st.slowedKm, 1)} km`, 'Legs (whole route) flown below line speed to hold the climb/descent limits. Point density rises on them (∝ 1/speed).'] as Row] : []),
  );
  const sp = r.sp;
  const selSo = state.selSortie !== 'all' ? sp?.sorties[state.selSortie] : undefined;
  // Safety figures are for the routes that are flown: the selected sortie, or the worst of all of them.
  const flown = selSo ? [selSo.route] : (sp?.sorties.map(so => so.route) ?? (r.flight ? [r.flight] : []));
  if (flown.length) {
    const b = flown.map(f => clearanceBudget(plan, f, state.survey.uncertaintyM)).reduce((a, x) => (x.worstCaseM < a.worstCaseM ? x : a));
    const files = r.verify?.routes.filter(f => !selSo || f.sortie === selSo.index || f.sortie == null) ?? [];
    rows.push('Terrain clearance',
      ['Planned over the DEM', `≥ ${fmt(b.aglM)} m`, `Every straight leg clears the highest DEM cell within ±${plan.o.corridorM} m of it, and beyond its ends, by at least the AGL.`],
      ['Path rounding + lag', `− ${fmt(b.pathM, 1)} m`, `Worst case at a waypoint: the rounded turn can pass ${b.roundingM.toFixed(1)} m below the legs, and the aircraft can lose ${b.lagM.toFixed(1)} m while it steepens its climb (assuming it holds only ${SAFETY.vertAccelMs2} m/s² vertically).`],
      ['Uncertainty allowance', `− ${fmt(b.uncertaintyM)} m`, 'Your allowance for DEM error, vegetation and buildings missing from the DEM, and the aircraft\'s own height error.'],
      ['Worst case left', `${fmt(b.worstCaseM)} m`, `Export is refused below ${SAFETY.worstCaseErrorM} m and warned below ${SAFETY.worstCaseWarnM} m. Obstacles that are not in the DEM (masts, cables, cranes, turbines) are NOT covered by any of this.`],
      ...(files.length ? [[
        'Checked from the files', files.every(f => f.report.ok) ? `${fmt(Math.min(...files.map(f => f.report.minClearanceM)), 1)} m min` : 'FAILED',
        `${files.length} route file(s) read back and checked against the DEM by separate code: ${fmt(files.reduce((s, f) => s + f.report.samples, 0))} terrain samples, lowest clearance of a straight leg shown here. Export is blocked unless every file passes.`,
      ] as Row] : (r.flight && !kmzBlocker() && !hasErrors(r) ? [['Checked from the files', 'checking…'] as Row] : [])),
    );
  }
  const t = selSo?.transit ?? r.transit;
  if (t) rows.push('Take-off & return',
    ['Take-off transit', `${fmt(t.distanceM / 1000, 1)} km · ${fmt(t.timeS / 60, 1)} min`],
    ['Transit clearance', `${fmt(t.minClearanceM)} m min`],
    ['RTH height to set', `${fmt(t.rthHeightM)} m${state.takeoff.rthHeightM == null ? ' auto' : ''} (rec. ≥ ${fmt(t.rthRecommendedM)})`, 'Above the take-off point. The route file cannot carry this: it must be set on the RC before take-off. With several sorties each has its own value; select a sortie to see it.'],
    ['Max Altitude to set', `≥ ${fmt(maxAltSetting(t))} m`, `The flight reaches ${fmt(t.topAboveHomeM)} m above the take-off point and the RTH height is ${fmt(t.rthHeightM)} m. The aircraft\'s Max Altitude setting must be above both, or it will refuse the route, stop climbing, or return home too low.`],
    ['Height at WP 1', `${fmt(t.wp1AboveHomeM)} m above take-off`, 'What the RC should show as height (H) when the aircraft reaches the first waypoint, if it took off from the planned home. More than about 10 m off means the heights are not what was planned: stop the mission.'],
  );
  if (r.totalMin != null) rows.push(['Flight time*', `${fmt(r.sp ? r.sp.totalMin : r.totalMin)} min`, 'Sum over all sorties: each has its own transit, climb, figure-8, lines, RTH and descent, plus 2 min for checks and calibration passes. Still air.']);
  const sortieHtml = sp ? `<h3>Sorties · ${sp.sorties.length} × ≤ ${state.survey.sortieMin} min</h3>` + sp.sorties.map(so =>
    `<div class="sortie${state.selSortie === so.index ? ' sel' : ''}${so.overBudget ? ' over' : ''}" data-sortie="${so.index}" data-tip="Transit ${fmt(so.time.transit, 1)} + climb/descent ${fmt(so.time.vertical, 1)} + route ${fmt(so.time.route, 1)} + RTH ${fmt(so.time.rth, 1)} + fixed ${fmt(so.time.fixed, 1)} min.${so.transit ? ` RTH height ${fmt(so.transit.rthHeightM)} m, Max Altitude ≥ ${fmt(maxAltSetting(so.transit))} m.` : ''} Click to show it on the map."><span>S${so.index + 1} · L${so.fromLine + 1}–${so.toLine + 1}</span><b>${so.route.wps.length} WP · ${fmt(so.time.total, 1)} min</b></div>`).join('') : '';
  el.innerHTML = sortieHtml + rows.map(row => typeof row === 'string' ? `<h3>${row}</h3>`
    : `<div${row[2] ? ` data-tip="${esc(row[2])}"` : ''}><span>${row[0]}</span><b>${row[1]}</b></div>`).join('') +
    (st ? `<p class="foot">*route ÷ speed${t ? ' + transit + RTH from last WP' : ''}; excludes acceleration and figure-8 slow-down.</p>` : '');
}

function renderIssues(r: Result | null) {
  const el = $('issues');
  if (!r) {
    el.innerHTML = state.planError ? `<h2>Checks</h2><div class="issue error"><b>error</b><span>${esc(state.planError)} Nothing is planned and nothing can be exported until this is fixed.</span></div>` : '';
    return;
  }
  const list: Issue[] = r.demError ? [{ severity: 'error', code: 'DEM', message: r.demError }]
    : !state.dem ? [{ severity: 'warn', code: 'NO_DEM', message: 'No terrain loaded: showing plan geometry only. Heights, AGL, profile, overlap and checks need a DEM.' }]
    : r.issues;
  const order = { error: 0, warn: 1, info: 2 };
  const v = r.verify;
  const fileCheck = v?.ok
    ? `<div class="issue ok"><b>ok</b><span>${v.routes.length} route file(s) built and read back by the independent check: lowest leg clearance ${fmt(Math.min(...v.routes.map(f => f.report.minClearanceM)), 1)} m over the DEM (planned ≥ ${r.plan.o.aglM} m), all climb, descent, turn and datum checks passed.</span></div>` : '';
  el.innerHTML = '<h2>Checks</h2>' + fileCheck + (list.length ? [...list].sort((a, b) => order[a.severity] - order[b.severity])
    .map(i => `<div class="issue ${i.severity}"><b>${i.severity}</b><span>${esc(i.message)}</span></div>`).join('')
    : (fileCheck ? '' : '<div class="issue ok"><b>ok</b><span>No problems found.</span></div>'));
}

function renderLineSel(r: View | null) {
  const sel = $<HTMLSelectElement>('lineSel');
  const n = r ? r.lastLine + 1 : 0;
  const start = r?.route.startIdx ?? 0;
  if (state.selLine === 'transit' && !r?.transit) state.selLine = start;
  if (typeof state.selLine === 'number' && (state.selLine >= n || state.selLine < start)) state.selLine = start;
  sel.innerHTML = (r?.transit ? '<option value="transit">Take-off transit</option>' : '') +
    Array.from({ length: n }, (_, i) => `<option value="${i}" ${i < start ? 'disabled' : ''}>Line ${i + 1}</option>`).join('');
  sel.value = String(state.selLine);
}
$<HTMLSelectElement>('lineSel').onchange = e => {
  const v = (e.target as HTMLSelectElement).value;
  selectLine(v === 'transit' ? 'transit' : Number(v));
};
function selectLine(i: number | 'transit') { state.selLine = i; renderMap(state.view); renderDeck(state.view); renderLineSel(state.view); renderProf(); }

function renderProf() {
  const svg = $('profile') as unknown as SVGSVGElement;
  const info = $('profInfo');
  const r = state.view;
  if (!r?.flight || !state.dem) {
    svg.replaceChildren();
    info.textContent = !state.dem ? 'Load terrain (Fetch GLO-30 or drop a GeoTIFF) to see the profile.' : '';
    return;
  }
  let pts: ProfilePt[], floor: number;
  if (state.selLine === 'transit' && r.transit) {
    pts = [...r.transit.path.map(p => ({ xy: r.plan.proj.fwd(p.lon, p.lat), h: p.h, role: 'transit' })),
      ...r.flight.wps.filter(w => w.role === 'approach' || w.role === 'fig8')];
    floor = T('minClearanceM');
    $('floorLabel').textContent = 'Terrain + min clearance';
    info.textContent = `min clearance ${fmt(r.transit.minClearanceM)} m (need ${floor} m)`;
  } else {
    const wps = r.flight.wps.filter(w => w.line === state.selLine);
    pts = wps;
    floor = r.plan.o.aglM;
    $('floorLabel').textContent = 'Terrain + AGL';
    const agl = wps.filter(w => w.role === 'line').map(w => w.h - w.terrainUnderWp);
    info.textContent = agl.length ? `AGL ${fmt(Math.min(...agl))}–${fmt(Math.max(...agl))} m (nominal ${floor}) · height ${fmt(Math.min(...wps.map(w => w.h)))}–${fmt(Math.max(...wps.map(w => w.h)))} m` : '';
  }
  renderProfile(svg, r.plan, pts, state.dem.elev, floor);
}
new ResizeObserver(() => renderProf()).observe($('profile'));
$('stats').addEventListener('click', e => {
  const row = (e.target as HTMLElement).closest<HTMLElement>('[data-sortie]');
  if (!row) return;
  const i = Number(row.dataset.sortie);
  selectSortie(state.selSortie === i ? 'all' : i);
});

// ── Block input ──────────────────────────────────────────────────
function setAoi(aoi: Aoi, demo = false) {
  const wasDemo = state.demo || !!state.dem?.synthetic;
  state.aoi = aoi; state.demo = demo; state.selLine = 0; state.selSortie = 'all';
  // A new block needs its own home point: the old one belongs to another site.
  state.home = null; homeMarker.remove();
  if (demo) { state.dem = DEMO_DEM; state.demBuf = null; state.params = { ...state.params, courseDeg: 20 }; }
  else if (wasDemo) {
    // Never carry the demo's made-up terrain over to a real block: it answers with a height anywhere on Earth.
    state.dem = null; state.demBuf = null;
    restoreCachedDem();
  }
  persist(); syncFields(); fitAoi(); updateDemInfo(); schedule();
  $('blockInfo').textContent = `${aoi.poly.length} vertices${aoi.polygonsFound > 1 ? `, largest of ${aoi.polygonsFound} polygons` : ''}. Set the home point for this block.`;
}
async function onBlockFile(f: File) {
  try { setAoi(await aoiFromFile(f)); } catch (e) { $('blockInfo').textContent = (e as Error).message; }
}
$('loadDemo').onclick = () => setAoi({ name: 'Demo: synthetic Nimba block', poly: demoPoly, polygonsFound: 1 }, true);
$<HTMLInputElement>('fileBlock').onchange = e => { const f = (e.target as HTMLInputElement).files?.[0]; if (f) onBlockFile(f); };

// ── Terrain input (kept in IndexedDB so it survives reloads) ─────
const otKey = $<HTMLInputElement>('otKey');
try { otKey.value = localStorage.getItem('3dm.otKey') ?? ''; } catch { /* ignore */ }
if (!otKey.value && import.meta.env.DEV && import.meta.env.VITE_OPENTOPO_KEY) otKey.value = import.meta.env.VITE_OPENTOPO_KEY;
otKey.onchange = () => { try { localStorage.setItem('3dm.otKey', otKey.value.trim()); } catch { /* ignore */ } };

function updateDemInfo(msg?: string, err = false) {
  const d = state.dem, el = $('demInfo');
  el.classList.toggle('err', err);
  if (msg != null) { el.textContent = msg; return; }
  if (!d) { el.textContent = 'No DEM. Heights, profile and checks need terrain.'; return; }
  if (d.synthetic) { el.textContent = `Loaded: ${d.name}. Made-up terrain for the demo block: preview only, nothing can be exported from it, and the 3D ground (real) will not match.`; return; }
  const cell = d.elev.cell ? `, ${d.elev.cell[0].toFixed(0)} × ${d.elev.cell[1].toFixed(0)} m cells` : '';
  const datum = d.datum === 'ellipsoidal' ? 'ellipsoidal heights, converted to mean sea level with EGM96' : d.source === 'glo30' ? 'heights above mean sea level (EGM2008, used as EGM96)' : 'heights taken as metres above mean sea level';
  el.textContent = `Loaded: ${d.name}${cell}; ${datum}.${d.gaps ? ` ${d.gaps.toLocaleString('en-ZA')} cells have no data: a route that touches one is refused.` : ''}`;
}
type DemSource = 'glo30' | 'file';
async function useDemBuffer(buf: ArrayBuffer, name: string, source: DemSource, datum: VerticalDatum, save = true) {
  const raster = await readGeoTiff(buf, { verticalDatum: datum });
  state.dem = { name: `${name} (${raster.width}×${raster.height})`, elev: rasterElev(raster), bbox: rasterBbox(raster), synthetic: false, source, datum, gaps: rasterGaps(raster) };
  state.demBuf = buf;
  state.demo = false; persist(); updateDemInfo(); schedule();
  if (save) await putBlob('dem', { name, buf, source, datum });
}
// The DEM kept from the last session (not for the demo block, which has its own made-up terrain).
function restoreCachedDem() {
  getBlob<{ name: string; buf: ArrayBuffer; source?: DemSource; datum?: VerticalDatum }>('dem').then(d => {
    if (!d || state.dem || state.demo) return;
    const source = d.source ?? (/GLO-30/.test(d.name) ? 'glo30' : 'file');
    if (source === 'file') { demFile = { buf: d.buf, name: d.name }; if (d.datum) { state.survey.demDatum = d.datum; $<HTMLSelectElement>('demDatum').value = d.datum; } }
    useDemBuffer(d.buf, d.name + ' · cached', source, d.datum ?? 'orthometric', false).catch(e => updateDemInfo('Cached DEM not used: ' + (e as Error).message, true));
  });
}
// How far the route can reach beyond the block: run-in/out, the figure-8 with its straight, the terrain
// corridor and the stopping distance at line speed. The DEM request must cover all of it, for every sortie.
function routeReachM(): number {
  const v = P('speedMs'), bank = P('fig8BankDeg') * Math.PI / 180;
  const r = Math.max(DEFAULTS.fig8MinRadiusM, (v * v) / (9.81 * Math.tan(bank)));
  const fig8 = !isPhoto() && state.survey.fig8 ? 3 * r + Math.max(2.5 * r, DEFAULTS.alignStraightS * v) : 0;
  return Math.max(P('runInM'), P('runOutM')) + fig8 + Math.max(P('corridorM'), (v * v) / (2 * SAFETY.brakeMs2)) + 300;
}
$('fetchDem').onclick = async () => {
  if (!state.aoi) return;
  const key = otKey.value.trim();
  if (!key) { updateDemInfo('Enter your OpenTopography API key first.', true); return; }
  const pts = [...state.aoi.poly, ...(state.home ? [state.home] : [])];
  const b = bufferedBbox(pts, Math.max(2000, routeReachM()));
  updateDemInfo('Requesting Copernicus GLO-30 from OpenTopography…');
  try {
    const res = await fetch(openTopoUrl(b, key, 'COP30'));
    const buf = await res.arrayBuffer();
    const head = new TextDecoder().decode(buf.slice(0, 200));
    if (!res.ok || !/^(II\*|MM\0\*)/.test(head.slice(0, 4))) throw new Error(`OpenTopography: ${head.replace(/<[^>]+>/g, ' ').trim().slice(0, 160) || res.status}`);
    await useDemBuffer(buf, 'Copernicus GLO-30', 'glo30', 'orthometric');
  } catch (e) { updateDemInfo((e as Error).message, true); }
};
let demFile: { buf: ArrayBuffer; name: string } | null = null;   // the last GeoTIFF the user gave us
async function onDemFile(f: File) {
  updateDemInfo('Reading ' + f.name + '…');
  try {
    const buf = await f.arrayBuffer();
    await useDemBuffer(buf, f.name, 'file', state.survey.demDatum);
    demFile = { buf, name: f.name };
  } catch (e) { updateDemInfo(`${f.name} was not loaded: ${(e as Error).message}`, true); }   // the DEM in use, if any, stays
}
$<HTMLInputElement>('fileDem').onchange = e => { const f = (e.target as HTMLInputElement).files?.[0]; if (f) onDemFile(f); };
$('demDatum').addEventListener('change', async e => {
  state.survey.demDatum = (e.target as HTMLSelectElement).value as VerticalDatum;
  persist();
  // A user file that is (or was) in use is read again with the new datum. GLO-30 is always mean sea level.
  if (!demFile || (state.dem && state.dem.source !== 'file')) return;
  state.dem = null; state.demBuf = null;               // nothing is planned on the old reading
  schedule();
  updateDemInfo('Reading ' + demFile.name + '…');
  try { await useDemBuffer(demFile.buf, demFile.name, 'file', state.survey.demDatum); }
  catch (err) { updateDemInfo(`${demFile.name} was not loaded: ${(err as Error).message}`, true); }
});

for (const t of ['dragenter', 'dragover'] as const) window.addEventListener(t, e => { e.preventDefault(); document.body.classList.add('dragging'); });
for (const t of ['dragleave', 'drop'] as const) window.addEventListener(t, e => { e.preventDefault(); if (t === 'drop' || !(e as DragEvent).relatedTarget) document.body.classList.remove('dragging'); });
window.addEventListener('drop', e => {
  for (const f of Array.from(e.dataTransfer?.files ?? [])) {
    if (/\.(kmz|kml)$/i.test(f.name)) onBlockFile(f);
    else if (/\.tiff?$/i.test(f.name)) onDemFile(f);
  }
});

// ── Resume ───────────────────────────────────────────────────────
const resSync = () => {
  state.resume = { on: $<HTMLInputElement>('resOn').checked, fromLine: Number($<HTMLInputElement>('resLine').value) || 1, speed: Number($<HTMLInputElement>('resSpeed').value) || 14 };
  schedule();
};
for (const id of ['resOn', 'resLine', 'resSpeed']) $(id).addEventListener('input', resSync);

// ── Course optimiser ─────────────────────────────────────────────
// Without terrain: fewest lines, then shortest on-line distance.
// With terrain: minimise route length × (mean line AGL / nominal AGL): density/GSD degrade ~linearly
// with AGL, so this favours lines along slopes over lines across ridges. 5° sweep, then ±4° at 1°.
function courseCost(c: number): number {
  const p = planLines(state.aoi!.poly, { ...planOpts(), courseDeg: c });
  if (!state.dem) return p.lines.length * 1e9 + p.lines.reduce((s, L) => s + (L.umax - L.umin), 0);
  try {
    const f = applyHeights(p, buildRoute(p, routeOpts()), state.dem.elev);
    const st = stats(p, f);
    const lineAgl = f.wps.filter(w => w.role === 'line').map(w => w.h - w.terrainUnderWp);
    return st.routeKm * (lineAgl.reduce((s, v) => s + v, 0) / lineAgl.length / p.o.aglM);
  } catch { return Infinity; }
}
$('optCourse').onclick = async () => {
  if (!state.aoi) return;
  const btn = $<HTMLButtonElement>('optCourse');
  btn.disabled = true; btn.textContent = 'Optimising…';
  await new Promise(r => setTimeout(r, 20));
  const norm = (c: number) => ((c % 180) + 180) % 180;
  const sweep = (cs: number[]) => cs.map(c => ({ c: norm(c), cost: courseCost(norm(c)) })).reduce((a, b) => (b.cost < a.cost ? b : a));
  const coarse = sweep(Array.from({ length: 36 }, (_, i) => i * 5));
  const fine = sweep(Array.from({ length: 9 }, (_, i) => coarse.c - 4 + i));
  btn.textContent = 'Optimise course';
  state.params.courseDeg = fine.c; change();
};

// ── Export ───────────────────────────────────────────────────────
const safeName = (s: string) => s.replace(/[^\w.-]+/g, '_');
function download(name: string, data: Uint8Array | Blob, type = 'application/octet-stream') {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(data instanceof Blob ? data : new Blob([data as BlobPart], { type }));
  a.download = name; a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}
const sha256Hex = async (b: Uint8Array) => [...new Uint8Array(await crypto.subtle.digest('SHA-256', b as BufferSource))].map(x => x.toString(16).padStart(2, '0')).join('');
function base64(b: Uint8Array): string {
  let t = '';
  for (let i = 0; i < b.length; i += 0x8000) t += String.fromCharCode(...b.subarray(i, i + 0x8000));
  return btoa(t);
}

// ── Route files (DJI WPML, from the Pilot 2 samples) ─────────────
function kmzBlocker(): string | null {
  if (isPhoto()) return 'KMZ export for photogrammetry needs a P1 (or L3 RGB-only) Pilot 2 sample first.';
  if (state.survey.scan !== 'linear') return 'Only the Linear (repetitive) scan mode is verified from your samples so far.';
  return null;
}
function rgbSpacingM(aglM: number): number {
  const c = CAMERAS.find(c => c.id === 'l3-100')!;
  return 2 * aglM * Math.tan((c.vfovDeg / 2) * Math.PI / 180) * (1 - 0.75);   // 75 % frontlap, as Pilot 2's L3 default
}
function kmzFor(route: Route<FlightWp>, aglM: number): WpmlFiles {
  return writeWpml(route, {
    takeoff: { ...TAKEOFF_DEFAULTS, ...state.takeoff },
    lidar: { samplingRate: pulse().khz * 1000, returnMode: 'sedecupleReturn', scanningMode: 'repetitive', modelColoring: state.survey.rgbPhotos },
    djiImuCalibration: state.survey.djiCal,
    rgbPhotoSpacingM: state.survey.rgbPhotos ? +rgbSpacingM(aglM).toFixed(2) : null,
    gimbalStartGroup: true,
  });
}

// A plan may only leave the planner when it is the current result, has no errors, is not on demo terrain,
// and (where route files exist) those exact files have passed the independent check. Asked again at the
// moment of every export, whatever the buttons showed.
function releasable(): { r: Result; v: Verified | null } | null {
  const r = state.result;
  if (!r || !r.flight || !state.aoi || !state.dem || state.dem.synthetic || hasErrors(r)) return null;
  if (kmzBlocker()) return { r, v: null };
  verifyNow(r);
  if (hasErrors(r) || !r.verify?.ok) { updateButtons(); renderIssues(r); return null; }
  return { r, v: r.verify };
}

// What the route file cannot carry and the pilot has to set or check on the RC.
function preflightRows(files: RouteFile[]) {
  return files.map(f => ({
    name: `${f.sortie != null ? `Sortie ${f.sortie + 1}` : 'Route'} · lines ${f.fromLine + 1}–${f.toLine + 1}`,
    rth: f.transit ? fmt(f.transit.rthHeightM) : '?', alt: f.transit ? fmt(maxAltSetting(f.transit)) : '?',
    wp1: f.transit ? fmt(f.transit.wp1AboveHomeM) : '?', file: f.name,
  }));
}
function preflightNotes(r: Result, files: RouteFile[]): [string, string][] {
  const lost = files[0].report.rcLostAction;
  const worst = Math.min(...files.map(f => r.plan.o.aglM - f.report.maxPathErrorM - state.survey.uncertaintyM));
  return [
    ['RTH height', 'is measured above the take-off point. If the aircraft offers an automatic or "optimal" return route, use the preset-height mode instead, so this value is what it flies.'],
    ['Max Altitude', 'on the aircraft must be at least the value shown, or it will refuse the route or stop climbing.'],
    ['Take off from the planned home point', '(the H on the map). The transit, the RTH height and the height at WP 1 were all checked from there, and the mission must be started from the ground.'],
    ['RTK fixed', `before take-off and through the flight. Heights in the file are absolute; without RTK the aircraft's own height can be off by more than the ${state.survey.uncertaintyM} m allowance.`],
    ['Height at WP 1', 'is what the RC should show as height above take-off when the aircraft reaches the first waypoint. More than about 10 m off: stop the mission.'],
    ['Obstacles that are not in the DEM', `(masts, power lines, cranes, turbines, new buildings, tall trees) are not checked by anything here. Worst-case clearance over the DEM is ${fmt(worst)} m.`],
    ['If the RC link is lost', `the file tells the aircraft to ${lost === 'goBack' ? 'return to home at the RTH height' : lost === 'goContinue' ? 'carry on with the route' : lost}; it also returns home when the route ends.`],
  ];
}
function confirmPreflight(r: Result, files: RouteFile[], action: string): Promise<boolean> {
  const dlg = $<HTMLDialogElement>('preflight'), ack = $<HTMLInputElement>('pfAck'), ok = $<HTMLButtonElement>('pfOk');
  $('pfBody').innerHTML = `<p>The route file cannot carry these. Set them on the RC${files.length > 1 ? ' for each sortie' : ''}:</p>
    <table><thead><tr><th></th><th>RTH height ≥</th><th>Max Altitude ≥</th><th>Height at WP 1</th></tr></thead><tbody>${
      preflightRows(files).map(x => `<tr><td>${esc(x.name)}</td><td>${x.rth} m</td><td>${x.alt} m</td><td>${x.wp1} m</td></tr>`).join('')}</tbody></table>
    <ul>${preflightNotes(r, files).map(([b, t]) => `<li><b>${esc(b)}</b> ${esc(t)}</li>`).join('')}</ul>`;
  ok.textContent = action; ack.checked = false; ok.disabled = true;
  ack.onchange = () => { ok.disabled = !ack.checked; };
  // Answered by the button that was pressed. (The dialog's own close event can arrive late or not at all in
  // an embedded or background view, so nothing waits for it; Esc and any other way of closing mean "no".)
  return new Promise(res => {
    const done = (yes: boolean) => { if (dlg.open) dlg.close(); res(yes); };
    ok.onclick = e => { e.preventDefault(); done(ack.checked); };
    $<HTMLButtonElement>('pfCancel').onclick = e => { e.preventDefault(); done(false); };
    dlg.oncancel = () => done(false);
    dlg.onclose = () => res(false);
    dlg.showModal();
  });
}
function checklistText(r: Result, files: RouteFile[]): string {
  return ['BEFORE FLYING THESE ROUTES', '', `Block: ${state.aoi!.name}`, `Terrain: ${state.dem!.name}`, `Planned: ${new Date().toISOString()}`, '',
    'Set on the RC for each route (the route file cannot carry these):', '',
    ...preflightRows(files).map(x => `  ${x.name}\n    file: ${x.file}\n    RTH height >= ${x.rth} m    Max Altitude >= ${x.alt} m    height at WP 1: ${x.wp1} m above take-off`), '',
    ...preflightNotes(r, files).map(([b, t]) => `- ${b} ${t}`), '',
    `Every file here was read back and checked against the DEM by separate code before it was exported: lowest leg clearance ${fmt(Math.min(...files.map(f => f.report.minClearanceM)), 1)} m (planned >= ${r.plan.o.aglM} m).`, ''].join('\r\n');
}

const wpOut = (w: FlightWp, i: number) => ({
  i, role: w.role, line: w.line ?? null, lat: +w.lat.toFixed(8), lon: +w.lon.toFixed(8),
  h: +w.h.toFixed(2), speed: +w.speed.toFixed(2), dampingM: +w.dampingM.toFixed(2), turnMode: w.turnMode, actions: w.actions,
});
const checkOut = (rep: VerifyReport) => ({
  passed: rep.ok, waypoints: rep.waypoints, minClearanceM: +rep.minClearanceM.toFixed(2), maxPathErrorM: +rep.maxPathErrorM.toFixed(2),
  maxClimbMs: +rep.maxClimbMs.toFixed(2), maxDescentMs: +rep.maxDescentMs.toFixed(2), topEgm96M: +rep.topEgm96M.toFixed(2),
  rcLostAction: rep.rcLostAction, finishAction: rep.finishAction, samples: rep.samples,
});
// The mission package: the plan, and for each sortie the exact KMZ that passed the check (with its SHA-256),
// so the RC flies those bytes and never has to rebuild a route.
async function buildMission(r: Result, v: Verified | null) {
  const kmz = v ? await Promise.all(v.routes.map(async f => {
    const bytes = await writeKmz(f.files);
    return { name: f.name, size: bytes.length, sha256: await sha256Hex(bytes), base64: base64(bytes) };
  })) : null;
  const d = state.dem!;
  return {
    format: '3dm-mission', version: 2, created: new Date().toISOString(),
    block: state.aoi!, params: { ...DEFAULTS, ...planOpts() },
    dem: { name: d.name, source: d.source, datum: d.datum, cellsWithoutData: d.gaps },
    heights: 'h = metres above mean sea level (EGM96, the DEM datum). The KMZ files carry the same heights as WGS84 ellipsoidal executeHeight.',
    sensor: isPhoto() ? { kind: 'photo', camera: camera(), frontlapPct: state.survey.frontlap, exposureS: 1 / state.survey.shutterInv, plan: r.photo }
      : { kind: 'lidar', payload: 'Zenmuse L3', pulse: pulse(), scanMode: scan(), fig8: state.survey.fig8 },
    home: state.home ? { lon: state.home[0], lat: state.home[1], groundH: r.transit?.homeElev ?? null } : null,
    takeoff: { ...TAKEOFF_DEFAULTS, ...state.takeoff, rthHeightM: r.transit?.rthHeightM ?? state.takeoff.rthHeightM ?? null },
    resume: state.resume.on ? state.resume : null, stats: r.st, coverage: r.cov && { ...r.cov, swaths: undefined }, issues: r.issues,
    safety: {
      uncertaintyM: state.survey.uncertaintyM, budget: r.budget, filesChecked: v ? v.ok : false,
      setOnTheRc: 'RTH height and Max Altitude are not carried by the route files: set each sortie\'s values on the RC before take-off, take off from the planned home, with RTK fixed.',
    },
    sortieBudgetMin: state.survey.sortieMin,
    sorties: (r.sp?.sorties ?? []).map((so, k) => ({
      index: so.index, fromLine: so.fromLine, toLine: so.toLine, minutes: +so.time.total.toFixed(2), time: so.time,
      transit: so.transit && { distanceM: so.transit.distanceM, minClearanceM: so.transit.minClearanceM, rthHeightM: so.transit.rthHeightM, path: so.transit.path },
      rthHeightM: so.transit?.rthHeightM ?? null,
      maxAltitudeM: so.transit ? maxAltSetting(so.transit) : null,
      wp1AboveHomeM: so.transit ? +so.transit.wp1AboveHomeM.toFixed(1) : null,
      kmz: kmz ? kmz[k] : null,
      fileCheck: v ? checkOut(v.routes[k].report) : null,
      waypoints: so.route.wps.map(wpOut),
    })),
    waypoints: r.flight!.wps.map(wpOut),
  };
}
const missionBase = () => `${safeName(state.aoi!.name)}${state.resume.on ? `_resume_L${state.resume.fromLine}` : ''}`;

$('exportJson').onclick = async () => {
  const rel = releasable();
  if (!rel) return;
  const mission = await buildMission(rel.r, rel.v);
  if (state.result !== rel.r) return;
  download(`${missionBase()}.mission.json`, new Blob([JSON.stringify(mission, null, 1)], { type: 'application/json' }));
};

$('exportKmz').onclick = async () => {
  const rel = releasable();
  if (!rel?.v) return;
  const { r, v } = rel;
  const sel = state.selSortie !== 'all' ? v.routes.find(f => f.sortie === state.selSortie) : undefined;
  const files = sel ? [sel] : v.routes;
  if (!(await confirmPreflight(r, files, files.length > 1 ? `Download ${files.length} KMZ` : 'Download KMZ'))) return;
  if (state.result !== r || r.verify !== v) return;      // the plan changed while the dialog was open
  if (files.length === 1) { download(files[0].name, await writeKmz(files[0].files)); return; }
  const zip = new JSZip();
  for (const f of files) zip.file(f.name, await writeKmz(f.files));
  zip.file('READ-ME-before-flying.txt', checklistText(r, files));
  download(`${missionBase()}_${files.length}-sorties.zip`, await zip.generateAsync({ type: 'blob' }));
};

// ── Sync to RC ───────────────────────────────────────────────────
const apiUrl = $<HTMLInputElement>('apiUrl'), apiToken = $<HTMLInputElement>('apiToken');
try {
  apiUrl.value = localStorage.getItem('3dm.apiUrl') ?? '';
  apiToken.value = localStorage.getItem('3dm.apiToken') ?? '';
} catch { /* ignore */ }
if (import.meta.env.DEV) {
  if (!apiUrl.value && import.meta.env.VITE_API_URL) apiUrl.value = import.meta.env.VITE_API_URL;
  if (!apiToken.value && import.meta.env.VITE_API_ADMIN_TOKEN) apiToken.value = import.meta.env.VITE_API_ADMIN_TOKEN;
} else if (!apiUrl.value) {
  apiUrl.value = location.origin;   // production: planner and sync API are served from the same origin
}
for (const [el, key] of [[apiUrl, '3dm.apiUrl'], [apiToken, '3dm.apiToken']] as const) {
  el.addEventListener('change', () => { try { localStorage.setItem(key, el.value.trim()); } catch { /* ignore */ } });
}
const apiCfg = (): ApiCfg | null => (apiUrl.value.trim() && apiToken.value.trim() ? { url: apiUrl.value.trim(), token: apiToken.value.trim() } : null);
const syncInfo = (msg: string, err = false) => { const el = $('syncInfo'); el.textContent = msg; el.classList.toggle('err', err); };

$('publishBtn').onclick = async () => {
  const cfg = apiCfg();
  if (!cfg) { syncInfo('Enter the sync server URL and office token.', true); return; }
  const rel = releasable();
  if (!rel) { syncInfo('Nothing to publish: the plan has errors or its route files have not passed the check.', true); return; }
  if (rel.v && !(await confirmPreflight(rel.r, rel.v.routes, 'Publish to RC'))) return;
  if (state.result !== rel.r) { syncInfo('The plan changed; publish again.', true); return; }
  const btn = $<HTMLButtonElement>('publishBtn');
  btn.disabled = true; syncInfo('Publishing…');
  try {
    const mission = await buildMission(rel.r, rel.v);
    const note = `${isPhoto() ? 'Photo' : 'LiDAR'} · ${rel.r.sp?.sorties.length ?? 1} sortie(s) · ${new Date().toLocaleString('en-ZA')}`;
    const { project, version } = await publish(cfg, state.aoi!.name, mission, state.demBuf, note);
    syncInfo(`Published "${project.name}" v${version.n}: ${version.manifest?.sortieCount ?? 0} sortie(s), mission ${(version.mission_size / 1024).toFixed(0)} kB${version.dem_size ? `, DEM ${(version.dem_size / 1048576).toFixed(1)} MB` : ''}. Paired RCs pick it up on their next sync.`);
  } catch (e) {
    syncInfo('Publish failed: ' + (e as Error).message, true);
  } finally { updateButtons(); }
};
$('pairBtn').onclick = async () => {
  const cfg = apiCfg();
  if (!cfg) { syncInfo('Enter the sync server URL and office token.', true); return; }
  try {
    const { code, expiresAt } = await newPairingCode(cfg);
    const el = $('pairCode');
    el.hidden = false;
    el.innerHTML = `<b>${code.replace(/(.{4})/, '$1 ')}</b><span>Enter this code in 3DM Fly on the RC. Valid until ${new Date(expiresAt).toLocaleTimeString('en-ZA', { hour: '2-digit', minute: '2-digit' })}, single use.</span>`;
    const devs = await listDevices(cfg);
    syncInfo(`${devs.filter(d => !d.revoked).length} RC(s) paired.`);
  } catch (e) { syncInfo('Pairing failed: ' + (e as Error).message, true); }
};

// ── Start ────────────────────────────────────────────────────────
syncFields();
updateDemInfo();
if (state.aoi) $('blockInfo').textContent = `${state.aoi.poly.length} vertices.`;
if (!state.demo) restoreCachedDem();
schedule();
