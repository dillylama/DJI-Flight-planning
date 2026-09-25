import 'maplibre-gl/dist/maplibre-gl.css';
import './style.css';
import maplibregl, { type GeoJSONSource, type LngLatBoundsLike } from 'maplibre-gl';
import { MapboxOverlay } from '@deck.gl/mapbox';
import { PathLayer, LineLayer } from '@deck.gl/layers';
import {
  DEFAULTS, TAKEOFF_DEFAULTS, M400_LIMITS, M400_SPEC, L3_PULSE, L3_SCAN, CAMERAS,
  planLines, buildRoute, applyHeights, stats, validate, readGeoTiff, rasterElev, rasterBbox,
  bufferedBbox, openTopoUrl, planTransit, coverage, checkLimits, planSorties,
  cameraFovDeg, gsdCm, aglForGsd, photoPlan, frontlapAtInterval, lidarDensity,
  type PlanOptions, type Plan, type Route, type FlightWp, type RouteStats, type Issue, type ElevFn, type Bbox,
  type LonLat, type TakeoffOptions, type Transit, type Coverage, type Pt3, type Camera, type PhotoPlan, type SortiePlan,
} from '@3dm/core';
import { aoiFromFile, type Aoi } from './aoi.ts';
import { demoPoly, demoElev } from './demo.ts';
import { renderProfile, type ProfilePt } from './profile.ts';
import { TIPS } from './tips.ts';
import { getBlob, putBlob } from './store.ts';
import { publish, newPairingCode, listDevices, type ApiCfg } from './sync.ts';

// ── State ─────────────────────────────────────────────────────────
interface Dem { name: string; elev: ElevFn; bbox?: Bbox }
interface Result {
  plan: Plan; route: Route; flight: Route<FlightWp> | null; st: RouteStats | null; issues: Issue[];
  demError: string | null; transit: Transit | null; cov: Coverage | null; photo: PhotoPlan | null; totalMin: number | null;
  sp: SortiePlan | null;
}
// What the map/profile show: the whole job, or one sortie.
interface View { plan: Plan; route: Route; flight: Route<FlightWp> | null; transit: Transit | null; cov: Coverage | null; issues: Issue[]; lastLine: number }
type SensorKind = 'lidar' | 'photo';
interface SurveyCfg { sensor: SensorKind; pulse: string; scan: string; fig8: boolean; camera: string; frontlap: number; shutterInv: number; sortieMin: number; sortieOverlap: number; maxWp: number | null }
const SURVEY_DEFAULTS: SurveyCfg = { sensor: 'lidar', pulse: '100', scan: 'linear', fig8: true, camera: 'p1-35', frontlap: 80, shutterInv: 1000, sortieMin: M400_LIMITS.sortieMinDefault, sortieOverlap: 0, maxWp: null };
const LAYERS = [
  ['aoi', 'Block outline'], ['lines', 'Data lines'], ['turns', 'Run-in/out & turns'], ['fig8', 'Figure-8 & approach'],
  ['wps', 'Waypoints'], ['rec', 'Record start/stop'], ['swath', 'Swath / photo footprint'], ['drops', 'Drop lines to terrain (3D)'],
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
  dem: (saved.demo ? { name: 'Synthetic ridge (demo)', elev: demoElev } : null) as Dem | null,
  resume: { on: false, fromLine: 1, speed: 14 },
  selLine: 0 as number | 'transit',
  selSortie: 'all' as 'all' | number,
  demBuf: null as ArrayBuffer | null,
  view: null as View | null,
  pickingHome: false,
  result: null as Result | null,
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
type Field<K> = { key: K; label: string; unit: string; step: number; min: number; max: number; only?: SensorKind };
const FIELDS: Field<keyof PlanOptions>[] = [
  { key: 'aglM', label: 'AGL', unit: 'm', step: 10, min: 20, max: 1000 },
  { key: 'speedMs', label: 'Line speed', unit: 'm/s', step: 0.5, min: 1, max: M400_LIMITS.lineSpeedMaxMs },
  { key: 'fovDeg', label: 'FOV used', unit: '°', step: 1, min: 10, max: 80, only: 'lidar' },
  { key: 'sidelapPct', label: 'Sidelap', unit: '%', step: 5, min: 0, max: 90 },
  { key: 'courseDeg', label: 'Line course', unit: '°', step: 1, min: 0, max: 359 },
  { key: 'runInM', label: 'Run-in', unit: 'm', step: 10, min: 0, max: 1000 },
  { key: 'runOutM', label: 'Run-out', unit: 'm', step: 10, min: 0, max: 1000 },
  { key: 'wpSpacingM', label: 'Max WP spacing', unit: 'm', step: 10, min: 20, max: 1000 },
  { key: 'corridorM', label: 'Terrain corridor ±', unit: 'm', step: 5, min: 0, max: 500 },
  { key: 'maxGradient', label: 'Max climb/descent', unit: 'rise/run', step: 0.01, min: 0.01, max: 0.5 },
  { key: 'fig8BankDeg', label: 'Figure-8 bank', unit: '°', step: 1, min: 5, max: M400_LIMITS.bankMaxDeg, only: 'lidar' },
];
const TK_FIELDS: Field<'takeoffSecurityM' | 'transitSpeedMs' | 'minClearanceM'>[] = [
  { key: 'takeoffSecurityM', label: 'Take-off security height', unit: 'm', step: 5, min: 2, max: 1500 },
  { key: 'transitSpeedMs', label: 'Transit speed', unit: 'm/s', step: 0.5, min: 1, max: M400_LIMITS.lineSpeedMaxMs },
  { key: 'minClearanceM', label: 'Min terrain clearance', unit: 'm', step: 5, min: 10, max: 500 },
];
const esc = (s: string) => s.replace(/&/g, '&amp;').replace(/"/g, '&quot;').replace(/</g, '&lt;');
const ti = (k: string) => TIPS[k] ? `<i class="ti" tabindex="0" data-tip="${esc(TIPS[k].t)}"${TIPS[k].rec ? ` data-rec="${esc(TIPS[k].rec!)}"` : ''}>i</i>` : '';
const btnTip = (k: string) => TIPS[k] ? ` data-tip="${esc(TIPS[k].t)}"` : '';
const fieldHtml = (f: Field<string>, group: string) =>
  `<label${f.only ? ` data-only="${f.only}"` : ''}><span class="lt">${f.label} <em>${f.unit}</em>${ti(f.key)}</span><input type="number" data-group="${group}" data-key="${f.key}" step="${f.step}" min="${f.min}" max="${f.max}" /></label>`;
const opt = (v: string, label: string) => `<option value="${v}">${label}</option>`;

const app = document.getElementById('app')!;
app.innerHTML = `
<header>
  <div class="brand"><span class="mark">3DM</span> Planner <span class="sub">M400 · Zenmuse L3 / P1</span></div>
  <div class="blockname" id="blockName">No block loaded</div>
  <div class="head-actions">
    <button id="exportJson" class="ghost" disabled${btnTip('exportJson')}>Export mission.json</button>
    <button id="exportKmz" class="ghost" disabled data-tip="Waits for the RC Waypoint Route sample export (samples/README.md). We don't guess the WPML header.">Export KMZ</button>
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
      <span>…or drop a GeoTIFF DEM (EPSG:4326)</span></label>
    <div class="note" id="demInfo"></div>
  </section>
  <section>
    <h2>Sensor ${ti('sensor')}</h2>
    <div class="seg" id="sensorSeg"><button data-sensor="lidar">LiDAR · Zenmuse L3</button><button data-sensor="photo">Photogrammetry</button></div>
    <div class="fields" data-only="lidar">
      <label><span class="lt">Pulse rate ${ti('lidarMode')}</span><select id="pulseSel">${L3_PULSE.map(p => opt(p.id, `${p.khz} kHz · AGL < ${p.maxAglM} m`)).join('')}</select></label>
      <label><span class="lt">Scan mode</span><select id="scanSel">${L3_SCAN.map(s => opt(s.id, `${s.name} ${s.fovH}°×${s.fovV}°`)).join('')}</select></label>
      <label class="check span2"><input type="checkbox" id="fig8On" /> IMU figure-8 before lines and resumes</label>
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
    <div class="fields" id="fields">${FIELDS.map(f => fieldHtml(f, 'plan')).join('')}</div>
    <div class="row"><button id="optCourse" class="ghost" disabled${btnTip('optCourse')}>Optimise course</button><button id="resetParams" class="link">Reset defaults</button></div>
  </section>
  <section>
    <h2>Home, take-off &amp; aircraft</h2>
    <div class="row"><button id="setHome" class="ghost"${btnTip('setHome')}>Set home on map</button><span class="note inline" id="homeInfo">Not set</span></div>
    <div class="fields" style="margin-top:10px">
      ${TK_FIELDS.map(f => fieldHtml(f, 'takeoff')).join('')}
      <label><span class="lt">RTH height <em>m</em>${ti('rthHeightM')}</span><input type="number" id="rthHeight" step="10" min="20" max="1500" placeholder="auto" /></label>
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
    <span><i style="background:var(--c-fig8)"></i>Figure-8</span><span><i style="background:var(--c-appr)"></i>Approach</span>
    <span><i style="background:var(--c-transit)"></i>Transit</span><span><i style="background:var(--c-rth)"></i>RTH</span>
    <span><i style="background:var(--c-swath)"></i>Swath</span>
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
  $<HTMLSelectElement>('camSel').value = s.camera;
  $<HTMLInputElement>('frontlapIn').value = String(s.frontlap);
  $<HTMLInputElement>('shutterIn').value = String(s.shutterInv);
  $<HTMLInputElement>('sortieIn').value = String(s.sortieMin);
  $<HTMLInputElement>('overlapIn').value = String(s.sortieOverlap);
  $<HTMLInputElement>('maxWpIn').value = s.maxWp != null ? String(s.maxWp) : '';
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
document.querySelector('aside')!.addEventListener('input', e => {
  const i = e.target as HTMLInputElement;
  if (!i.dataset.group) return;
  const v = Number(i.value);
  if (i.value === '' || !Number.isFinite(v)) return;
  if (i.dataset.group === 'plan') state.params[i.dataset.key as keyof PlanOptions] = v as never;
  else state.takeoff[i.dataset.key as keyof TakeoffOptions] = v as never;
  persist(); schedule();
  if (i.dataset.key === 'aglM') $<HTMLInputElement>('gsdIn').value = gsdCm(camera(), v).toFixed(2);
});
$('rthHeight').addEventListener('input', e => { const v = (e.target as HTMLInputElement).value; state.takeoff.rthHeightM = v === '' ? null : Number(v); persist(); schedule(); });
$('flyToMode').addEventListener('change', e => { state.takeoff.flyToMode = (e.target as HTMLSelectElement).value as TakeoffOptions['flyToMode']; persist(); schedule(); });
$('resetParams').onclick = () => { state.params = {}; change(); };
$('sensorSeg').addEventListener('click', e => {
  const b = (e.target as HTMLElement).closest<HTMLButtonElement>('button[data-sensor]');
  if (!b || b.dataset.sensor === state.survey.sensor) return;
  state.survey.sensor = b.dataset.sensor as SensorKind;
  state.params.sidelapPct = isPhoto() ? 70 : 50;             // sensible starting overlap per sensor
  if (isPhoto()) state.params.aglM = Math.round(aglForGsd(camera(), 2.5));
  change();
});
$('pulseSel').addEventListener('change', e => { state.survey.pulse = (e.target as HTMLSelectElement).value; change(); });
$('scanSel').addEventListener('change', e => { state.survey.scan = (e.target as HTMLSelectElement).value; change(); });
$('fig8On').addEventListener('change', e => { state.survey.fig8 = (e.target as HTMLInputElement).checked; change(); });
$('camSel').addEventListener('change', e => { state.survey.camera = (e.target as HTMLSelectElement).value; change(); });
$('gsdIn').addEventListener('input', e => {
  const g = Number((e.target as HTMLInputElement).value);
  if (!(g > 0)) return;
  state.params.aglM = Math.round(aglForGsd(camera(), g));
  persist(); schedule();
  const a = document.querySelector<HTMLInputElement>('input[data-key=aglM]'); if (a) a.value = String(state.params.aglM);
});
$('frontlapIn').addEventListener('input', e => { const v = Number((e.target as HTMLInputElement).value); if (v > 0) { state.survey.frontlap = v; persist(); schedule(); } });
$('shutterIn').addEventListener('input', e => { const v = Number((e.target as HTMLInputElement).value); if (v > 0) { state.survey.shutterInv = v; persist(); schedule(); } });
$('sortieIn').addEventListener('input', e => { const v = Number((e.target as HTMLInputElement).value); if (v > 0) { state.survey.sortieMin = v; persist(); schedule(); } });
$('overlapIn').addEventListener('input', e => { const v = Number((e.target as HTMLInputElement).value); if (v >= 0) { state.survey.sortieOverlap = v; persist(); schedule(); } });
$('maxWpIn').addEventListener('input', e => { const t = (e.target as HTMLInputElement).value; state.survey.maxWp = t === '' ? null : Number(t); persist(); schedule(); });
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
if (import.meta.env.DEV) (window as unknown as { __map: maplibregl.Map }).__map = map;   // dev-only debugging handle
const deck = new MapboxOverlay({ interleaved: true, layers: [] });
map.addControl(deck);

const empty: GeoJSON.FeatureCollection = { type: 'FeatureCollection', features: [] };
const roleColor = () => ['match', ['get', 'role'],
  'line', css('--c-line'), 'fig8', css('--c-fig8'), 'approach', css('--c-appr'), css('--c-run')] as unknown as string;
const visibleRoles = () => [
  ...(state.layers.lines ? ['line'] : []), ...(state.layers.turns ? ['runin', 'runout'] : []), ...(state.layers.fig8 ? ['fig8', 'approach'] : []),
];

let mapReady = false;
function onStyleReady() {   // style ready: add our layers without waiting for every tile
  for (const id of ['aoi', 'route', 'wps', 'rec', 'swath', 'transit', 'rth']) map.addSource(id, { type: 'geojson', data: empty });
  map.addLayer({ id: 'swath-fill', type: 'fill', source: 'swath', paint: { 'fill-color': css('--c-swath'), 'fill-opacity': 0.14 } });
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
  ...(state.resume.on && fromLine != null ? { fromLine, speedMs: state.resume.speed } : {}),
});

function compute(): Result | null {
  if (!state.aoi) return null;
  const plan = planLines(state.aoi.poly, planOpts());
  if (!plan.lines.length) return null;
  const fromLine = state.resume.on ? Math.min(plan.lines.length, Math.max(1, state.resume.fromLine)) - 1 : undefined;
  const route = buildRoute(plan, routeOpts(fromLine));
  const r: Result = { plan, route, flight: null, st: null, issues: [], demError: null, transit: null, cov: null, photo: null, totalMin: null, sp: null };
  if (isPhoto()) r.photo = photoPlan(camera(), plan.o.aglM, route.speedMs, state.survey.frontlap, 1 / state.survey.shutterInv);
  if (!state.dem) return r;
  try {
    r.flight = applyHeights(plan, route, state.dem.elev);
  } catch (e) {
    r.demError = (e as Error).message + '. The DEM does not cover the whole route (block + run-ins + figure-8).';
    return r;
  }
  r.st = stats(plan, r.flight);
  r.issues = validate(plan, r.flight).filter(i => !(isPhoto() && i.code === 'FIG8'));
  r.cov = coverage(plan, r.flight);
  const clr = T('minClearanceM');
  if (state.home) {
    try {
      const t = (r.transit = planTransit(plan, r.flight, state.dem.elev, state.home, state.takeoff));
      if (t.minClearanceM < clr) r.issues.push({
        severity: 'error', code: 'TRANSIT_CLEARANCE',
        message: `Take-off transit clears terrain by only ${t.minClearanceM.toFixed(0)} m (need ${clr} m). ${T('flyToMode') === 'pointToPoint' ? 'The sloping point-to-point leg cuts into terrain: switch to "Safely" or raise the take-off security height.' : 'Raise the take-off security height or move home.'}`,
      });
      if (t.rthWorstClearanceM < clr) r.issues.push({
        severity: 'error', code: 'RTH_CLEARANCE',
        message: `RTH at ${t.rthHeightM} m above home would clear terrain by only ${t.rthWorstClearanceM.toFixed(0)} m on the straight line home from WP ${t.rthWorstWp + 1}. Recommended RTH height ≥ ${t.rthRecommendedM} m.`, wps: [t.rthWorstWp],
      });
      if (t.rthHeightM > 1500) r.issues.push({ severity: 'warn', code: 'RTH_HIGH', message: `RTH height ${t.rthHeightM} m is above DJI's usual 1,500 m maximum. Move home closer to the block's high ground.` });
      if (T('transitSpeedMs') > M400_LIMITS.transitSpeedWarnMs) r.issues.push({ severity: 'warn', code: 'TRANSIT_SPEED', message: `Transit speed ${T('transitSpeedMs')} m/s is above the conservative ${M400_LIMITS.transitSpeedWarnMs} m/s.` });
    } catch (e) {
      r.issues.push({ severity: 'error', code: 'HOME', message: (e as Error).message + '. Load terrain that covers the home point.' });
    }
  } else {
    r.issues.push({ severity: 'info', code: 'NO_HOME', message: 'Home not set: take-off transit and RTH are not checked. Use "Set home on map".' });
  }
  const t = r.transit;
  r.totalMin = r.st.flightMin + (t ? t.timeS / 60 + t.rthDistanceM / T('transitSpeedMs') / 60 : 0);
  r.issues.push(...checkLimits(plan, r.flight, {
    sensor: state.survey.sensor, pulse: isPhoto() ? undefined : pulse(), scanFovH: isPhoto() ? undefined : plan.o.fovDeg,
    sortieMin: state.survey.sortieMin, totalMin: r.totalMin,
  }).filter(i => i.code !== 'SORTIES'));
  try {
    r.sp = planSorties(plan, state.dem.elev, state.home, state.takeoff, {
      usableMin: state.survey.sortieMin, firstLine: route.startIdx, speedMs: route.speedMs, fig8: !isPhoto() && state.survey.fig8,
      overlapLines: state.survey.sortieOverlap, maxWaypoints: state.survey.maxWp,
      climbMs: M400_LIMITS.climbWarnMs, descentMs: M400_LIMITS.descentWarnMs, full: r.flight,
    });
    r.issues.push(...r.sp.issues);
    for (const so of r.sp.sorties) if (so.transit) {
      if (so.transit.minClearanceM < clr) r.issues.push({ severity: 'error', code: 'SORTIE_TRANSIT', message: `Sortie ${so.index + 1}: take-off transit clears terrain by only ${so.transit.minClearanceM.toFixed(0)} m (need ${clr} m).` });
      if (so.transit.rthWorstClearanceM < clr) r.issues.push({ severity: 'error', code: 'SORTIE_RTH', message: `Sortie ${so.index + 1}: RTH at ${so.transit.rthHeightM} m clears terrain by only ${so.transit.rthWorstClearanceM.toFixed(0)} m. Recommended ≥ ${so.transit.rthRecommendedM} m.` });
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
  if (!so) return { plan: r.plan, route: r.route, flight: r.flight, transit: r.transit, cov: r.cov, issues: r.issues, lastLine: r.plan.lines.length - 1 };
  const cov = r.cov && { ...r.cov, swaths: r.cov.swaths.filter(s => s.line >= so.fromLine && s.line <= so.toLine) };
  return { plan: r.plan, route: so.route, flight: so.route, transit: so.transit, cov, issues: r.issues, lastLine: so.toLine };
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

function run() {
  const r = (state.result = compute());
  $<HTMLButtonElement>('exportJson').disabled = !r?.flight || r.issues.some(i => i.severity === 'error');
  $<HTMLButtonElement>('publishBtn').disabled = $<HTMLButtonElement>('exportJson').disabled;
  $<HTMLButtonElement>('optCourse').disabled = !state.aoi;
  $<HTMLButtonElement>('fetchDem').disabled = !state.aoi;
  $('blockName').textContent = state.aoi ? state.aoi.name : 'No block loaded';
  $('homeInfo').textContent = !state.home ? 'Not set'
    : `${state.home[1].toFixed(5)}, ${state.home[0].toFixed(5)}${r?.transit ? ` · ground ${r.transit.homeElev.toFixed(0)} m` : ''}`;
  renderView();
  renderStats(r);
  renderIssues(r);
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
  if (!r) { for (const id of ['route', 'wps', 'rec', 'swath', 'transit', 'rth']) src(id)!.setData(empty); return; }
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
  src('transit')!.setData(lineFc(r.transit?.path));
  src('rth')!.setData(lineFc(r.transit?.rthFromLast));
}

// ── Render: 3D (deck.gl, true heights) ───────────────────────────
function renderDeck(r: View | null) {
  if (!state.view3d || !r?.flight) { deck.setProps({ layers: [] }); return; }
  const col: Record<string, [number, number, number, number]> = {
    line: rgba('--c-line'), fig8: rgba('--c-fig8'), approach: rgba('--c-appr'), runin: rgba('--c-run'), runout: rgba('--c-run'),
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
  if (!r) { el.innerHTML = '<div class="empty">Load a block to plan.</div>'; return; }
  const { plan, route, st, cov, transit: t, photo } = r;
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
  );
  if (t) rows.push(
    ['Take-off transit', `${fmt(t.distanceM / 1000, 1)} km · ${fmt(t.timeS / 60, 1)} min`],
    ['Transit clearance', `${fmt(t.minClearanceM)} m min`],
    ['RTH height', `${fmt(t.rthHeightM)} m${state.takeoff.rthHeightM == null ? ' auto' : ''} (rec. ≥ ${fmt(t.rthRecommendedM)})`],
  );
  if (r.totalMin != null) rows.push(['Flight time*', `${fmt(r.sp ? r.sp.totalMin : r.totalMin)} min`, 'Sum over all sorties: each has its own transit, climb, figure-8, lines, RTH and descent.']);
  const sp = r.sp;
  const sortieHtml = sp ? `<h3>Sorties · ${sp.sorties.length} × ≤ ${state.survey.sortieMin} min</h3>` + sp.sorties.map(so =>
    `<div class="sortie${state.selSortie === so.index ? ' sel' : ''}${so.overBudget ? ' over' : ''}" data-sortie="${so.index}" data-tip="Transit ${fmt(so.time.transit, 1)} + climb/descent ${fmt(so.time.vertical, 1)} + route ${fmt(so.time.route, 1)} + RTH ${fmt(so.time.rth, 1)} min. Click to show it on the map."><span>S${so.index + 1} · L${so.fromLine + 1}–${so.toLine + 1}</span><b>${so.route.wps.length} WP · ${fmt(so.time.total, 1)} min</b></div>`).join('') : '';
  el.innerHTML = sortieHtml + rows.map(row => typeof row === 'string' ? `<h3>${row}</h3>`
    : `<div${row[2] ? ` data-tip="${esc(row[2])}"` : ''}><span>${row[0]}</span><b>${row[1]}</b></div>`).join('') +
    (st ? `<p class="foot">*route ÷ speed${t ? ' + transit + RTH from last WP' : ''}; excludes acceleration and figure-8 slow-down.</p>` : '');
}

function renderIssues(r: Result | null) {
  const el = $('issues');
  if (!r) { el.innerHTML = ''; return; }
  const list: Issue[] = r.demError ? [{ severity: 'error', code: 'DEM', message: r.demError }]
    : !state.dem ? [{ severity: 'warn', code: 'NO_DEM', message: 'No terrain loaded: showing plan geometry only. Heights, AGL, profile, overlap and checks need a DEM.' }]
    : r.issues;
  const order = { error: 0, warn: 1, info: 2 };
  el.innerHTML = '<h2>Checks</h2>' + (list.length ? [...list].sort((a, b) => order[a.severity] - order[b.severity])
    .map(i => `<div class="issue ${i.severity}"><b>${i.severity}</b><span>${i.message}</span></div>`).join('')
    : '<div class="issue ok"><b>ok</b><span>No problems found.</span></div>');
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
  state.aoi = aoi; state.demo = demo; state.selLine = 0;
  if (demo) { state.dem = { name: 'Synthetic ridge (demo)', elev: demoElev }; state.demBuf = null; state.params = { ...state.params, courseDeg: 20 }; }
  persist(); syncFields(); fitAoi(); updateDemInfo(); schedule();
  $('blockInfo').textContent = `${aoi.poly.length} vertices${aoi.polygonsFound > 1 ? `, largest of ${aoi.polygonsFound} polygons` : ''}.`;
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

function updateDemInfo(msg?: string) {
  $('demInfo').textContent = msg ?? (state.dem
    ? `Loaded: ${state.dem.name}${state.demo ? '. Demo terrain is synthetic, so the 3D ground (real) will not match. Fetch GLO-30 for the real ridge.' : ''}`
    : 'No DEM. Heights, profile and checks need terrain.');
}
async function useDemBuffer(buf: ArrayBuffer, name: string, save = true) {
  const raster = await readGeoTiff(buf);
  state.dem = { name: `${name} (${raster.width}×${raster.height})`, elev: rasterElev(raster), bbox: rasterBbox(raster) };
  state.demBuf = buf;
  state.demo = false; persist(); updateDemInfo(); schedule();
  if (save) await putBlob('dem', { name, buf });
}
$('fetchDem').onclick = async () => {
  if (!state.aoi) return;
  const key = otKey.value.trim();
  if (!key) { updateDemInfo('Enter your OpenTopography API key first.'); return; }
  const pts = [...state.aoi.poly, ...(state.home ? [state.home] : [])];
  const b = bufferedBbox(pts, 2000);
  updateDemInfo('Requesting Copernicus GLO-30 from OpenTopography…');
  try {
    const res = await fetch(openTopoUrl(b, key, 'COP30'));
    const buf = await res.arrayBuffer();
    const head = new TextDecoder().decode(buf.slice(0, 200));
    if (!res.ok || !/^(II\*|MM\0\*)/.test(head.slice(0, 4))) throw new Error(`OpenTopography: ${head.replace(/<[^>]+>/g, ' ').trim().slice(0, 160) || res.status}`);
    await useDemBuffer(buf, 'Copernicus GLO-30');
  } catch (e) { updateDemInfo((e as Error).message); }
};
async function onDemFile(f: File) {
  updateDemInfo('Reading ' + f.name + '…');
  try { await useDemBuffer(await f.arrayBuffer(), f.name); } catch (e) { updateDemInfo((e as Error).message); }
}
$<HTMLInputElement>('fileDem').onchange = e => { const f = (e.target as HTMLInputElement).files?.[0]; if (f) onDemFile(f); };

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
function buildMission() {
  const r = state.result;
  if (!r?.flight || !state.aoi) return null;
  return {
    format: '3dm-mission', version: 1, created: new Date().toISOString(),
    block: state.aoi, params: { ...DEFAULTS, ...planOpts() }, dem: state.dem?.name,
    sensor: isPhoto() ? { kind: 'photo', camera: camera(), frontlapPct: state.survey.frontlap, exposureS: 1 / state.survey.shutterInv, plan: r.photo }
      : { kind: 'lidar', payload: 'Zenmuse L3', pulse: pulse(), scanMode: scan(), fig8: state.survey.fig8 },
    home: state.home ? { lon: state.home[0], lat: state.home[1], groundH: r.transit?.homeElev ?? null } : null,
    takeoff: { ...TAKEOFF_DEFAULTS, ...state.takeoff, rthHeightM: r.transit?.rthHeightM ?? state.takeoff.rthHeightM ?? null },
    resume: state.resume.on ? state.resume : null, stats: r.st, coverage: r.cov && { ...r.cov, swaths: undefined }, issues: r.issues,
    sortieBudgetMin: state.survey.sortieMin,
    sorties: (r.sp?.sorties ?? []).map(so => ({
      index: so.index, fromLine: so.fromLine, toLine: so.toLine, minutes: +so.time.total.toFixed(2), time: so.time,
      transit: so.transit && { distanceM: so.transit.distanceM, minClearanceM: so.transit.minClearanceM, rthHeightM: so.transit.rthHeightM, path: so.transit.path },
      waypoints: so.route.wps.map((w, i) => ({
        i, role: w.role, line: w.line ?? null, lat: +w.lat.toFixed(8), lon: +w.lon.toFixed(8),
        h: +w.h.toFixed(2), hWrite: +w.hWrite.toFixed(2), speed: w.speed, dampingM: +w.dampingM.toFixed(2),
        turnMode: w.turnMode, actions: w.actions,
      })),
    })),
    heightNote: 'h = orthometric (DEM datum). hWrite = h + geoidN; geoid handling pending the RC sample export.',
    waypoints: r.flight.wps.map((w, i) => ({
      i, role: w.role, line: w.line ?? null, lat: +w.lat.toFixed(8), lon: +w.lon.toFixed(8),
      h: +w.h.toFixed(2), hWrite: +w.hWrite.toFixed(2), speed: w.speed, dampingM: +w.dampingM.toFixed(2),
      turnMode: w.turnMode, actions: w.actions,
    })),
  };
}
$('exportJson').onclick = () => {
  const mission = buildMission();
  if (!mission || !state.aoi) return;
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([JSON.stringify(mission, null, 1)], { type: 'application/json' }));
  a.download = `${state.aoi.name.replace(/[^\w.-]+/g, '_')}${state.resume.on ? `_resume_L${state.resume.fromLine}` : ''}.mission.json`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
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
}
for (const [el, key] of [[apiUrl, '3dm.apiUrl'], [apiToken, '3dm.apiToken']] as const) {
  el.addEventListener('change', () => { try { localStorage.setItem(key, el.value.trim()); } catch { /* ignore */ } });
}
const apiCfg = (): ApiCfg | null => (apiUrl.value.trim() && apiToken.value.trim() ? { url: apiUrl.value.trim(), token: apiToken.value.trim() } : null);
const syncInfo = (msg: string, err = false) => { const el = $('syncInfo'); el.textContent = msg; el.classList.toggle('err', err); };

$('publishBtn').onclick = async () => {
  const cfg = apiCfg(), mission = buildMission();
  if (!cfg) { syncInfo('Enter the sync server URL and office token.', true); return; }
  if (!mission || !state.aoi) return;
  const btn = $<HTMLButtonElement>('publishBtn');
  btn.disabled = true; syncInfo('Publishing…');
  try {
    const note = `${isPhoto() ? 'Photo' : 'LiDAR'} · ${state.result?.sp?.sorties.length ?? 1} sortie(s) · ${new Date().toLocaleString('en-ZA')}`;
    const { project, version } = await publish(cfg, state.aoi.name, mission, state.demBuf, note);
    syncInfo(`Published "${project.name}" v${version.n}: ${version.manifest?.sortieCount ?? 0} sortie(s), mission ${(version.mission_size / 1024).toFixed(0)} kB${version.dem_size ? `, DEM ${(version.dem_size / 1048576).toFixed(1)} MB` : ', no DEM (demo terrain)'}. Paired RCs pick it up on their next sync.`);
  } catch (e) {
    syncInfo('Publish failed: ' + (e as Error).message, true);
  } finally { btn.disabled = false; }
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
if (!state.demo) {
  getBlob<{ name: string; buf: ArrayBuffer }>('dem').then(d => {
    if (d && !state.dem) useDemBuffer(d.buf, d.name + ' · cached', false).catch(() => updateDemInfo());
  });
}
schedule();
