import 'maplibre-gl/dist/maplibre-gl.css';
import './style.css';
import maplibregl, { type GeoJSONSource, type LngLatBoundsLike } from 'maplibre-gl';
import { MapboxOverlay } from '@deck.gl/mapbox';
import { PathLayer, LineLayer } from '@deck.gl/layers';
import {
  DEFAULTS, TAKEOFF_DEFAULTS, M400_LIMITS, M400_SPEC, L3_PULSE, L3_SCAN, CAMERAS, SAFETY,
  planLines, buildRoute, flightRoute, stats, validate, readGeoTiff, rasterElev, rasterBbox, rasterGaps,
  bufferedBbox, openTopoUrl, planTransit, coverage, checkLimits, planSorties, writeWpml, writeKmz,
  cameraFovDeg, gsdCm, aglForGsd, photoPlan, lidarDensity,
  safetyChecks, clearanceBudget, verifyRouteFiles, toUgcsCsv,
  readMissionFiles, checkClearance, previewDsm, embeddedDsm, dsmFromRaster, writeGeoTiff,
  type MissionFile, type CheckReport, type Raster,
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
interface Dem { name: string; elev: ElevFn; bbox?: Bbox; synthetic: boolean; source: 'demo' | 'glo30' | 'file'; datum: VerticalDatum; gaps: number; raster?: Raster }
// A mission file (Pilot 2, FlightHub 2 or ours) being checked against the terrain (core/check.ts).
interface CheckState { name: string; mission: MissionFile; report: CheckReport | null }
// An issue that belongs to one sortie's route (its waypoint numbers refer to that route).
type PlanIssue = Issue & { sortie?: number };
// One route file as it will be flown: built once, checked by the independent verifier, exported byte for byte.
interface RouteFile { name: string; sortie: number | null; fromLine: number; toLine: number; transit: Transit | null; files: WpmlFiles; report: VerifyReport }
interface Verified { ok: boolean; routes: RouteFile[]; ms: number }
interface Result {
  plan: Plan; route: Route; flight: Route<FlightWp> | null; dense: Route<FlightWp> | null; st: RouteStats | null; issues: PlanIssue[];
  demError: string | null; transit: Transit | null; cov: Coverage | null; photo: PhotoPlan | null; totalMin: number | null;
  sp: SortiePlan | null; budget: ClearanceBudget | null; verify: Verified | null;
}
// What the map/profile show: the whole job, or one sortie.
interface View { plan: Plan; route: Route; flight: Route<FlightWp> | null; dense?: Route<FlightWp> | null; transit: Transit | null; cov: Coverage | null; issues: PlanIssue[]; lastLine: number }
type SensorKind = 'lidar' | 'photo';
interface SurveyCfg { sensor: SensorKind; pulse: string; scan: string; fig8: boolean; camera: string; frontlap: number; shutterInv: number; sortieMin: number; sortieOverlap: number; maxWp: number | null; djiCal: boolean; rgbPhotos: boolean; rgbFrontlap: number; fig8End: boolean; uncertaintyM: number; demDatum: VerticalDatum; transitInRoute: boolean; profile: string }
const SURVEY_DEFAULTS: SurveyCfg = { sensor: 'lidar', pulse: '100', scan: 'linear', fig8: false, camera: 'p1-35', frontlap: 80, shutterInv: 1000, sortieMin: M400_LIMITS.sortieMinDefault, sortieOverlap: 0, maxWp: null, djiCal: true, rgbPhotos: true, rgbFrontlap: 75, fig8End: false, uncertaintyM: SAFETY.uncertaintyDefaultM, demDatum: 'orthometric', transitInRoute: false, profile: 'l3' };
// LiDAR sensor profiles: a name, the across-track field of view used for line spacing, and the pulse rate used
// for the density estimate. Route files are only written for the Zenmuse L3 (the one whose values were copied
// from Pilot 2 exports); the others are for planning. Your own profiles are kept in this browser.
interface LidarProfile { id: string; name: string; fovDeg: number; khz: number | null; custom?: boolean }
const BUILTIN_PROFILES: LidarProfile[] = [
  { id: 'l3', name: 'DJI Zenmuse L3', fovDeg: DEFAULTS.fovDeg, khz: null },     // pulse rate from the L3 selector below
  { id: 'l2', name: 'DJI Zenmuse L2', fovDeg: 70, khz: 240 },
  { id: 'l1', name: 'DJI Zenmuse L1', fovDeg: 70, khz: 240 },
];
const PROFILE_STORE = '3dm.lidarProfiles';
let customProfiles: LidarProfile[] = [];
try {
  const raw = JSON.parse(localStorage.getItem(PROFILE_STORE) || '[]');
  if (Array.isArray(raw)) customProfiles = raw.filter(p => p && typeof p.name === 'string' && p.fovDeg >= 10 && p.fovDeg <= 120 && p.khz > 0)
    .map(p => ({ id: String(p.id), name: String(p.name).slice(0, 60), fovDeg: +p.fovDeg, khz: +p.khz, custom: true }));
} catch { /* ignore */ }
const profiles = () => [...BUILTIN_PROFILES, ...customProfiles];
const DEMO_DEM: Dem = { name: 'Synthetic ridge (demo)', elev: demoElev, synthetic: true, source: 'demo', datum: 'orthometric', gaps: 0 };
const LAYERS = [
  ['aoi', 'Block outline'], ['lines', 'Data lines'], ['turns', 'Run-in/out & turns'], ['fig8', 'Approach & exit'],
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
  drawing: false, drawPts: [] as LonLat[], editing: false,
  result: null as Result | null,
  planError: null as string | null,
  check: null as CheckState | null,
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
const profile = () => profiles().find(p => p.id === state.survey.profile) ?? BUILTIN_PROFILES[0];
const isL3 = () => profile().id === 'l3';
const lidarKhz = () => profile().khz ?? pulse().khz;
const isPhoto = () => state.survey.sensor === 'photo';
const wpCap = () => state.survey.maxWp ?? M400_LIMITS.maxWaypoints;   // empty box = the cap found on the RC

// ── Layout ────────────────────────────────────────────────────────
type Field<K> = { key: K; label: string; unit: string; step: number; min: number; max: number; only?: SensorKind; vmode?: 'slow' | 'raise'; adv?: boolean };
// The everyday settings stay in view; `adv` ones live under "Advanced" and keep their defaults unless changed.
const FIELDS: Field<keyof PlanOptions>[] = [
  { key: 'aglM', label: 'AGL', unit: 'm', step: 10, min: SAFETY.aglMinM, max: 1000 },
  { key: 'speedMs', label: 'Line speed', unit: 'm/s', step: 0.5, min: 1, max: M400_LIMITS.lineSpeedMaxMs },
  { key: 'sidelapPct', label: 'Sidelap', unit: '%', step: 5, min: 0, max: 90 },
  { key: 'courseDeg', label: 'Line course', unit: '°', step: 1, min: 0, max: 359 },
  { key: 'thinToleranceM', label: 'Thinning tolerance', unit: 'm', step: 5, min: 0, max: 200, adv: true },
  { key: 'fovDeg', label: 'FOV used', unit: '°', step: 1, min: 10, max: 120, only: 'lidar', adv: true },
  { key: 'runInM', label: 'Run-in', unit: 'm', step: 10, min: 20, max: 1000, adv: true },
  { key: 'runOutM', label: 'Run-out', unit: 'm', step: 10, min: 20, max: 1000, adv: true },
  { key: 'wpSpacingM', label: 'Max WP spacing', unit: 'm', step: 10, min: 20, max: 1000, adv: true },
  { key: 'corridorM', label: 'Terrain corridor ±', unit: 'm', step: 5, min: SAFETY.corridorMinM, max: 500, adv: true },
  { key: 'climbMs', label: 'Max climb rate', unit: 'm/s', step: 0.5, min: 0.5, max: M400_LIMITS.climbMaxMs, vmode: 'slow', adv: true },
  { key: 'descentMs', label: 'Max descent rate', unit: 'm/s', step: 0.5, min: 0.5, max: M400_LIMITS.descentMaxMs, vmode: 'slow', adv: true },
  { key: 'maxGradient', label: 'Max gradient', unit: 'rise/run', step: 0.01, min: 0.01, max: 0.5, vmode: 'raise', adv: true },
  { key: 'turnBankDeg', label: 'Line-end turn bank', unit: '°', step: 1, min: 5, max: M400_LIMITS.bankMaxDeg, adv: true },
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
  if (state.params.startSide != null && state.params.startSide !== 'left' && state.params.startSide !== 'right') delete state.params.startSide;
  if (state.params.startEnd != null && state.params.startEnd !== 'near' && state.params.startEnd !== 'far') delete state.params.startEnd;
  if (state.params.edgeLines != null && state.params.edgeLines !== 'inside' && state.params.edgeLines !== 'over') delete state.params.edgeLines;
  for (const k of Object.keys(state.params) as (keyof PlanOptions)[]) if (!FIELDS.some(f => f.key === k) && k !== 'verticalMode') delete state.params[k];
  if (state.takeoff.rthHeightM != null && !inRange(state.takeoff.rthHeightM, RTH_MIN, RTH_MAX)) state.takeoff.rthHeightM = null;
  if (state.takeoff.flyToMode != null && state.takeoff.flyToMode !== 'safely' && state.takeoff.flyToMode !== 'pointToPoint') delete state.takeoff.flyToMode;
  if (state.takeoff.rcLost != null && state.takeoff.rcLost !== 'goBack' && state.takeoff.rcLost !== 'continue') delete state.takeoff.rcLost;
  const s = state.survey;
  if (!inRange(s.uncertaintyM, SAFETY.uncertaintyMinM, UNC_MAX)) s.uncertaintyM = SAFETY.uncertaintyDefaultM;
  if (s.demDatum !== 'orthometric' && s.demDatum !== 'ellipsoidal') s.demDatum = 'orthometric';
  s.transitInRoute = s.transitInRoute === true;
  if (!profiles().some(p => p.id === s.profile)) s.profile = 'l3';
  // The figure-8 left the planner on 14 Oct 2026: a browser that saved it switched on must not keep flying it.
  s.fig8 = false; s.fig8End = false;
  if (!inRange(s.sortieMin, 5, 50)) s.sortieMin = M400_LIMITS.sortieMinDefault;
  if (!inRange(s.sortieOverlap, 0, 2)) s.sortieOverlap = 0;
  if (!inRange(s.rgbFrontlap, 50, 95)) s.rgbFrontlap = 75;
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

// Terrain-following presets: how closely the route hugs the ground. They set waypoint spacing (how finely the
// terrain is sampled along a line), the thinning tolerance (how much extra height is accepted to drop waypoints)
// and the corridor width. Anything else is "Custom".
const TF_PRESETS = [
  { id: 'precise', label: 'Precise: hug the terrain (slow, many waypoints)', wpSpacingM: 50, thinToleranceM: 5, corridorM: 40 },
  { id: 'standard', label: 'Standard', wpSpacingM: DEFAULTS.wpSpacingM, thinToleranceM: DEFAULTS.thinToleranceM, corridorM: DEFAULTS.corridorM },
  { id: 'fast', label: 'Fast: open ground, quick collection (few waypoints)', wpSpacingM: 250, thinToleranceM: 40, corridorM: 100 },
] as const;

// Base maps: free raster tile services that need no key. The choice is remembered in this browser.
const BASEMAPS = [
  { id: 'sat', name: 'Satellite (Esri)', tiles: ['https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'], maxzoom: 19, attribution: 'Imagery © Esri, Maxar, Earthstar Geographics', dim: true },
  { id: 'osm', name: 'Streets (OpenStreetMap)', tiles: ['https://tile.openstreetmap.org/{z}/{x}/{y}.png'], maxzoom: 19, attribution: '© OpenStreetMap contributors', dim: false },
  { id: 'otm', name: 'Topographic (OpenTopoMap)', tiles: ['https://a.tile.opentopomap.org/{z}/{x}/{y}.png', 'https://b.tile.opentopomap.org/{z}/{x}/{y}.png', 'https://c.tile.opentopomap.org/{z}/{x}/{y}.png'], maxzoom: 17, attribution: '© OpenStreetMap contributors, SRTM · © OpenTopoMap (CC-BY-SA)', dim: false },
  { id: 'esritopo', name: 'Topographic (Esri)', tiles: ['https://server.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/{z}/{y}/{x}'], maxzoom: 19, attribution: '© Esri, HERE, Garmin, FAO, NOAA, USGS', dim: false },
] as const;
const BASEMAP_STORE = '3dm.basemap';
let basemap: string = (() => { try { const v = localStorage.getItem(BASEMAP_STORE); return BASEMAPS.some(b => b.id === v) ? v! : 'sat'; } catch { return 'sat'; } })();

const app = document.getElementById('app')!;
app.innerHTML = `
<header>
  <div class="brand"><span class="mark">3DM</span> Mission Checker <span class="sub">M400 · Zenmuse L3 · Pilot 2 / FlightHub 2 routes against the terrain</span></div>
  <div class="blockname" id="blockName">No block loaded</div>
  <div class="head-actions">
    <button id="exportJson" class="ghost" disabled${btnTip('exportJson')}>Export mission.json</button>
    <button id="exportCsv" class="ghost" disabled${btnTip('exportCsv')}>UgCS CSV</button>
    <button id="exportKmz" class="ghost" disabled>Export KMZ</button>
  </div>
</header>
<aside>
  <section>
    <h2>Check a mission file ${ti('check')}</h2>
    <label class="drop small"><input type="file" id="fileCheck" accept=".kmz" hidden />
      <span>Drop a Pilot 2 / FlightHub 2 / planner KMZ to measure it against the terrain</span></label>
    <div class="fields" style="margin-top:8px">
      <label><span class="lt">Planned AGL <em>m</em>${ti('chkAgl')}</span><input type="number" id="chkAgl" step="10" min="30" max="1500" /></label>
      <label><span class="lt">Corridor <em>± m</em>${ti('chkCorr')}</span><input type="number" id="chkCorr" step="5" min="0" max="300" /></label>
    </div>
    <div class="row"><button id="chkFetch" class="ghost" disabled${btnTip('chkFetch')}>Fetch GLO-30 around this mission</button></div>
    <div class="note" id="chkInfo">No mission file loaded. Drop a KMZ exported from Pilot 2 or FlightHub 2 (route library → export), or one made here.</div>
    <div class="issues" id="chkLegs"></div>
    <div class="row"><button id="chkClear" class="link" hidden>Clear</button></div>
    <h2 style="margin-top:12px">A better DSM for Pilot 2 ${ti('dsmBtn')}</h2>
    <div class="row"><select id="dsmDilate" title="Each DSM cell becomes the highest ground within this radius: the corridor rule, which DJI's under-the-line follow lacks">
      <option value="0">Highest ground within 0 m</option><option value="30">within 30 m</option><option value="50">within 50 m</option><option value="75" selected>within 75 m</option></select>
      <select id="dsmMargin" title="Added to every cell. Pilot 2 smooths its route (straight legs between sparse waypoints) and came down to 35 m below surface + AGL over peaks"><option value="0">+ 0 m</option><option value="10">+ 10 m</option><option value="20">+ 20 m</option><option value="30">+ 30 m</option><option value="40" selected>+ 40 m</option><option value="50">+ 50 m</option></select></div>
    <div class="note" id="dsmPreview">Load a mission and terrain to see what a DSM would do to that route.</div>
    <div class="row"><button id="dsmSuggest" class="ghost" disabled${btnTip('dsmSuggest')}>Suggest</button>
      <button id="dsmBtn" class="ghost" disabled${btnTip('dsmBtn')}>Export DSM for Pilot 2</button></div>
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
  <details class="adv planner" id="plannerAdv"><summary>Plan a route here (steep sites; the Pilot 2 route with a DSM is the normal way)</summary>
  <section>
    <h2>Block</h2>
    <label class="drop"><input type="file" id="fileBlock" accept=".kmz,.kml" hidden />
      <strong>Drop KMZ / KML</strong><span>or click to choose. Largest polygon is used.</span></label>
    <div class="row"><button id="drawBlock" class="ghost" title="Draw the block on the map: click each corner, double-click or Enter to finish, Esc to cancel">Draw block</button><button id="editBlock" class="ghost" disabled title="Drag the corners of the block">Edit corners</button><button id="zoomBlock" class="ghost" disabled>Zoom to block</button></div>
    <div class="row"><button id="clearMission" class="ghost" disabled>Clear mission</button><button id="loadDemo" class="link">Demo block</button></div>
    <div class="note" id="blockInfo"></div>
  </section>
  <section>
    <h2>Sensor ${ti('sensor')}</h2>
    <div class="seg" id="sensorSeg"><button data-sensor="lidar">LiDAR</button><button data-sensor="photo">Photogrammetry</button></div>
    <div class="fields" data-only="lidar">
      <label class="span2"><span class="lt">Sensor profile ${ti('profile')}</span><select id="profileSel"></select></label>
      <details class="span2" id="profileNew"><summary>Add your own sensor</summary>
        <div class="fields">
          <label class="span2"><span class="lt">Name</span><input type="text" id="pfName" maxlength="60" placeholder="e.g. my fixed-wing LiDAR" /></label>
          <label><span class="lt">FOV used <em>°</em></span><input type="number" id="pfFov" min="10" max="120" step="1" /></label>
          <label><span class="lt">Pulse rate <em>kHz</em></span><input type="number" id="pfKhz" min="1" max="5000" step="1" /></label>
        </div>
        <div class="row"><button id="pfSave" class="ghost" type="button">Save profile</button><button id="pfDel" class="link" type="button">Delete selected</button></div>
      </details>
      <label data-l3><span class="lt">Pulse rate ${ti('lidarMode')}</span><select id="pulseSel">${L3_PULSE.map(p => opt(p.id, `${p.khz} kHz · AGL < ${p.maxAglM} m`)).join('')}</select></label>
      <label data-l3><span class="lt">Scan mode</span><select id="scanSel">${L3_SCAN.map(s => opt(s.id, `${s.name} ${s.fovH}°×${s.fovV}°`)).join('')}</select></label>
      <label class="check span2" data-l3><input type="checkbox" id="djiCalOn" /> DJI IMU calibration at start and end ${ti('djiCal')}</label>
      <label class="check span2" data-l3><input type="checkbox" id="rgbOn" /> L3 RGB photos on data lines ${ti('rgbPhotos')}</label>
      <label data-l3><span class="lt">RGB frontlap <em>%</em>${ti('rgbFrontlap')}</span><input type="number" id="rgbFrontlapIn" step="5" min="50" max="95" /></label>
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
    <div class="fields" id="fields">
      <label class="span2"><span class="lt">Terrain following ${ti('tfPreset')}</span>
        <select id="tfPreset">${TF_PRESETS.map(p => opt(p.id, p.label)).join('')}<option value="custom">Custom (see Advanced)</option></select></label>
      ${FIELDS.filter(f => !f.adv).map(f => fieldHtml(f, 'plan')).join('')}
      <label><span class="lt">Start corner ${ti('startCorner')}</span><select id="startCorner"><option value="left-near"></option><option value="right-near"></option><option value="left-far"></option><option value="right-far"></option></select></label>
      <label class="span2"><span class="lt">Vertical profile ${ti('verticalMode')}</span>
        <select id="vmodeSel"><option value="slow">Follow terrain; slow down to the climb/descent limits</option><option value="raise">Keep line speed; raise waypoints (gradient limit)</option></select></label>
      <details class="span2 adv"><summary>Advanced</summary><div class="fields">
        ${FIELDS.filter(f => f.adv).map(f => fieldHtml(f, 'plan')).join('')}
        <label><span class="lt">Terrain + position uncertainty <em>m</em>${ti('uncertaintyM')}</span><input type="number" id="uncIn" step="5" min="${SAFETY.uncertaintyMinM}" max="${UNC_MAX}" /></label>
        <label class="span2"><span class="lt">Outer lines ${ti('edgeLines')}</span>
          <select id="edgeLines"><option value="inside">Swath edge on the block edge (as FlightHub 2)</option><option value="over">Lines over the edges (half a strip outside, more margin)</option></select></label>
      </div></details>
    </div>
    <div class="row"><button id="optCourse" class="ghost" disabled${btnTip('optCourse')}>Follow contours (auto course)</button><button id="resetParams" class="link">Reset defaults</button></div>
  </section>
  <section>
    <h2>Home &amp; aircraft</h2>
    <div class="row"><button id="setHome" class="ghost"${btnTip('setHome')}>Set home on map</button><span class="note inline" id="homeInfo">Not set</span></div>
    <div class="fields" style="margin-top:10px">
      <label><span class="lt">RTH height <em>m</em>${ti('rthHeightM')}</span><input type="number" id="rthHeight" step="10" min="${RTH_MIN}" max="${RTH_MAX}" placeholder="auto" /></label>
      <label class="span2"><span class="lt">If the RC link is lost ${ti('rcLost')}</span>
        <select id="rcLost"><option value="goBack">Return home (DJI's RC-lost action)</option><option value="continue">Carry on with the route (expected signal loss)</option></select></label>
      <label><span class="lt">Usable time per battery <em>min</em><i class="ti" tabindex="0" data-tip="Planning endurance per battery set, landing reserve already removed, used only to estimate how many batteries a job takes. DJI quotes 59 min flight / 53 min hover for the M400 with the light H30T only; there is no official figure with the 1.75 kg L3." data-rec="${M400_LIMITS.sortieMinDefault} min with L3 until you have your own logs.">i</i></span><input type="number" id="sortieIn" step="1" min="5" max="50" /></label>
      <details class="span2 adv"><summary>Advanced</summary><div class="fields">
        ${TK_FIELDS.map(f => fieldHtml(f, 'takeoff')).join('')}
        <label class="span2"><span class="lt">Fly to route ${ti('flyToMode')}</span>
          <select id="flyToMode"><option value="safely">Safely: climb, then fly level</option><option value="pointToPoint">Point to point: climb, then slope</option><option value="route">Follow terrain: transit and return are waypoints of the route</option></select></label>
        <label><span class="lt">Lines re-flown <em>per split</em>${ti('sortieOverlap')}</span><input type="number" id="overlapIn" step="1" min="0" max="2" /></label>
        <label><span class="lt">Max WPs per route${ti('maxWp')}</span><input type="number" id="maxWpIn" step="50" min="50" placeholder="${M400_LIMITS.maxWaypoints}" /></label>
        <div class="sensor-note span2">M400 limits used (conservative): line speed ≤ ${M400_LIMITS.lineSpeedWarnMs} m/s (max ${M400_SPEC.maxHorizontalMs}), climb ≤ ${M400_LIMITS.climbWarnMs} m/s (max ${M400_SPEC.maxAscentMs}), descent ≤ ${M400_LIMITS.descentWarnMs} m/s (max ${M400_SPEC.maxDescentMs}), wind limit ${M400_SPEC.maxWindMs} m/s.</div>
      </div></details>
    </div>
  </section>
  <section>
    <h2>Continue from a line</h2>
    <label class="check"><input type="checkbox" id="resOn" /> Build a route that starts at a line ${ti('resOn')}</label>
    <div class="fields">
      <label><span class="lt">Data stopped on line ${ti('resLine')}</span><input type="number" id="resLine" min="1" value="1" /></label>
      <label><span class="lt">New speed <em>m/s</em>${ti('resSpeed')}</span><input type="number" id="resSpeed" step="0.5" min="1" max="20" value="14" /></label>
    </div>
    <div class="note">For a new landing zone, or when Pilot 2's own breakpoint cannot be used. Set the new home first.</div>
  </section>
  <section>
    <details class="adv"><summary>Sync to RC (not in use) ${ti('sync')}</summary>
    <label class="lbl" for="apiUrl">Sync server URL</label>
    <input id="apiUrl" type="url" placeholder="https://…" autocomplete="off" />
    <label class="lbl" for="apiToken" style="margin-top:8px">Office token ${ti('apiToken')}</label>
    <input id="apiToken" type="password" placeholder="Admin token" autocomplete="off" />
    <div class="row"><button id="publishBtn" disabled>Publish to RC</button><button id="pairBtn" class="ghost">Pair an RC</button></div>
    <div class="note" id="syncInfo"></div>
    <div class="pair-code" id="pairCode" hidden></div>
    </details>
  </section>
  </details>
</aside>
<main>
  <div id="map"></div>
  <div class="legend">
    <span><i style="background:var(--c-line)"></i>Data line</span><span><i style="background:var(--c-run)"></i>Run-in/out, turn</span>
    <span><i style="background:var(--c-appr)"></i>Approach / exit</span>
    <span><i style="background:var(--c-transit)"></i>Transit</span><span><i style="background:var(--c-rth)"></i>RTH</span>
    <span><i style="background:var(--c-swath)"></i>Swath</span><span><i style="background:var(--c-overlap)"></i>Strip overlap</span>
    <span><i class="dot wp"></i>Waypoint (zoom in)</span><span><i class="dot" style="background:var(--c-line)"></i>Record start</span><span><i class="dot" style="background:var(--c-appr)"></i>Record stop</span>
    <span><i class="dot flag"></i>Flagged WP (see Checks)</span>
  </div>
  <div class="map-toggles">
    <div class="place"><input id="placeSearch" type="search" placeholder="Find a place, or lat, lon" autocomplete="off" /><div class="place-results" id="placeResults" hidden></div></div>
    <select id="basemapSel" title="Base map">${BASEMAPS.map(b => opt(b.id, b.name)).join('')}</select>
    <button id="view3d" class="ghost"${btnTip('view3d')}>3D</button>
    <select id="sortieSel" title="Show the whole job or one sortie"></select>
    <button id="layersBtn" class="ghost"${btnTip('layers')}>Layers ▾</button>
    <div class="layers-panel" id="layersPanel" hidden>${LAYERS.map(([id, label]) => `<label class="check"><input type="checkbox" data-layer="${id}" /> ${label}</label>`).join('')}</div>
  </div>
  <div class="pick-hint" id="pickHint" hidden>Click the map to set home · Esc to cancel</div>
  <div class="pick-hint" id="drawHint" hidden>Click each corner of the block · double-click or Enter to finish · Esc to cancel</div>
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
  refreshStartCorner();
  $<HTMLSelectElement>('edgeLines').value = P('edgeLines');
  const match = TF_PRESETS.find(p => p.wpSpacingM === P('wpSpacingM') && p.thinToleranceM === P('thinToleranceM') && p.corridorM === P('corridorM'));
  $<HTMLSelectElement>('tfPreset').value = match ? match.id : 'custom';
  $<HTMLInputElement>('rthHeight').value = state.takeoff.rthHeightM != null ? String(state.takeoff.rthHeightM) : '';
  $<HTMLSelectElement>('flyToMode').value = state.survey.transitInRoute ? 'route' : T('flyToMode');
  $<HTMLSelectElement>('rcLost').value = T('rcLost');
  const ps = $<HTMLSelectElement>('profileSel');
  ps.innerHTML = profiles().map(p => opt(p.id, `${esc(p.name)} · ${p.fovDeg}°${p.khz ? ` · ${p.khz} kHz` : ''}${p.custom ? ' (yours)' : ''}`)).join('');
  ps.value = profile().id;
  $<HTMLButtonElement>('pfDel').hidden = !profile().custom;
  document.querySelectorAll<HTMLElement>('aside [data-l3]').forEach(el => { el.hidden = !isL3(); });
  const s = state.survey;
  $<HTMLSelectElement>('pulseSel').value = s.pulse;
  $<HTMLSelectElement>('scanSel').value = s.scan;
  $<HTMLSelectElement>('vmodeSel').value = P('verticalMode');
  document.querySelectorAll<HTMLElement>('aside [data-vmode]').forEach(el => { el.hidden = el.dataset.vmode !== P('verticalMode'); });
  $<HTMLInputElement>('djiCalOn').checked = s.djiCal;
  $<HTMLInputElement>('rgbOn').checked = s.rgbPhotos;
  $<HTMLSelectElement>('camSel').value = s.camera;
  $<HTMLInputElement>('frontlapIn').value = String(s.frontlap);
  $<HTMLInputElement>('shutterIn').value = String(s.shutterInv);
  $<HTMLInputElement>('sortieIn').value = String(s.sortieMin);
  $<HTMLInputElement>('rgbFrontlapIn').value = String(s.rgbFrontlap);
  $<HTMLInputElement>('rgbFrontlapIn').disabled = !s.rgbPhotos;
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
    if (!isL3()) { const q = profile(); el.textContent = `${q.name}: ${q.fovDeg}° used for the swath, ${q.khz} kHz for the density estimate. Planning only: route files are written for the Zenmuse L3.`; return; }
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
$('rcLost').addEventListener('change', e => { state.takeoff.rcLost = (e.target as HTMLSelectElement).value as TakeoffOptions['rcLost']; persist(); schedule(); });
$('flyToMode').addEventListener('change', e => {
  const v = (e.target as HTMLSelectElement).value;
  state.survey.transitInRoute = v === 'route';
  if (v !== 'route') state.takeoff.flyToMode = v as TakeoffOptions['flyToMode'];
  persist(); schedule();
});
$('profileSel').addEventListener('change', e => {
  state.survey.profile = (e.target as HTMLSelectElement).value;
  state.params.fovDeg = profile().fovDeg;                    // line spacing follows the sensor's field of view
  change();
});
$('pfSave').onclick = () => {
  const name = $<HTMLInputElement>('pfName').value.trim(), fov = numberIn($<HTMLInputElement>('pfFov')), khz = numberIn($<HTMLInputElement>('pfKhz'));
  $<HTMLInputElement>('pfName').classList.toggle('bad', !name);
  if (!name || fov == null || khz == null) return;
  const p: LidarProfile = { id: 'c' + Date.now().toString(36), name, fovDeg: fov, khz, custom: true };
  customProfiles.push(p);
  try { localStorage.setItem(PROFILE_STORE, JSON.stringify(customProfiles)); } catch { /* ignore */ }
  state.survey.profile = p.id; state.params.fovDeg = fov;
  $<HTMLInputElement>('pfName').value = ''; $<HTMLDetailsElement>('profileNew').open = false;
  change();
};
$('pfDel').onclick = () => {
  if (!profile().custom) return;
  customProfiles = customProfiles.filter(p => p.id !== profile().id);
  try { localStorage.setItem(PROFILE_STORE, JSON.stringify(customProfiles)); } catch { /* ignore */ }
  state.survey.profile = 'l3'; state.params.fovDeg = BUILTIN_PROFILES[0].fovDeg;
  change();
};
$('resetParams').onclick = () => { state.params = {}; change(); };
$('tfPreset').addEventListener('change', e => {
  const p = TF_PRESETS.find(x => x.id === (e.target as HTMLSelectElement).value);
  if (!p) return;   // "Custom" is only ever shown, never chosen
  state.params = { ...state.params, wpSpacingM: p.wpSpacingM, thinToleranceM: p.thinToleranceM, corridorM: p.corridorM }; change();
});
// Start corner: names follow the compass for the current course. Left/right are seen looking along the course;
// 'near' is the end behind you, so for course 90 (lines east–west) left-near is the north-west corner.
const COMPASS = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
const compass = (deg: number) => COMPASS[Math.round((((deg % 360) + 360) % 360) / 45) % 8];
function refreshStartCorner() {
  const c = P('courseDeg');
  const side = { left: compass(c - 90), right: compass(c + 90) }, end = { near: compass(c + 180), far: compass(c) };
  const sel = $<HTMLSelectElement>('startCorner');
  for (const o of Array.from(sel.options)) {
    const [s, e] = o.value.split('-') as ['left' | 'right', 'near' | 'far'];
    o.textContent = `${side[s]} side, ${end[e]} end`;
  }
  sel.value = `${P('startSide')}-${P('startEnd')}`;
}
$('startCorner').addEventListener('change', e => {
  const [s, en] = (e.target as HTMLSelectElement).value.split('-');
  state.params = { ...state.params, startSide: s as 'left' | 'right', startEnd: en as 'near' | 'far' }; change();
});
$('edgeLines').addEventListener('change', e => { state.params = { ...state.params, edgeLines: (e.target as HTMLSelectElement).value as 'inside' | 'over' }; change(); });
$('zoomBlock').onclick = () => fitAoi();
$('clearMission').onclick = () => {
  if (!state.aoi) return;
  stopEditing();
  state.aoi = null; state.demo = false; state.home = null; homeMarker.remove();
  state.resume.on = false; state.selLine = 0; state.selSortie = 'all';
  if (state.dem?.synthetic) { state.dem = null; state.demBuf = null; }
  $('blockInfo').textContent = 'Mission cleared. Drop a block to start again.';
  change(); updateDemInfo();
};
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
$('vmodeSel').addEventListener('change', e => { state.params.verticalMode = (e.target as HTMLSelectElement).value as PlanOptions['verticalMode']; change(); });
$('djiCalOn').addEventListener('change', e => { state.survey.djiCal = (e.target as HTMLInputElement).checked; change(); });
$('rgbOn').addEventListener('change', e => { state.survey.rgbPhotos = (e.target as HTMLInputElement).checked; change(); });
$('rgbFrontlapIn').addEventListener('input', e => { const v = numberIn(e.target as HTMLInputElement); if (v != null) { state.survey.rgbFrontlap = v; change(); } });
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
    glyphs: 'https://demotiles.maplibre.org/font/{fontstack}/{range}.pbf',   // for the START / END labels
    sources: {
      ...Object.fromEntries(BASEMAPS.map(b => ['bm-' + b.id, { type: 'raster', tileSize: 256, maxzoom: b.maxzoom, tiles: [...b.tiles], attribution: b.attribution }])),
      terrain3d: {
        type: 'raster-dem', tileSize: 256, maxzoom: 15, encoding: 'terrarium',
        tiles: ['https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png'],
        attribution: '3D ground: AWS Terrain Tiles (Mapzen)',
      },
    },
    // every base map is a layer; the chosen one is visible (applyBasemap), so our overlays always sit above all of them
    layers: BASEMAPS.map(b => ({
      id: 'bm-' + b.id, type: 'raster' as const, source: 'bm-' + b.id, layout: { visibility: (b.id === basemap ? 'visible' : 'none') as 'visible' | 'none' },
      paint: b.dim ? { 'raster-saturation': -0.4, 'raster-brightness-max': 0.78 } : {},
    })),
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
  'line', css('--c-line'), 'fig8', css('--c-fig8'), 'approach', css('--c-appr'), 'exit', css('--c-appr'), 'transit', css('--c-transit'), css('--c-run')] as unknown as string;
const visibleRoles = () => [
  ...(state.layers.lines ? ['line'] : []), ...(state.layers.turns ? ['runin', 'runout'] : []), ...(state.layers.fig8 ? ['fig8', 'approach', 'exit'] : []),
  ...(state.layers.transit ? ['transit'] : []),
];

let mapReady = false;
function onStyleReady() {   // style ready: add our layers without waiting for every tile
  applyBasemap();
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
  // Direction of flight along every leg (flat view), and START / END labels on the first and last waypoint.
  {
    const c = document.createElement('canvas'); c.width = c.height = 16;
    const g = c.getContext('2d')!;
    g.fillStyle = '#ffffff'; g.strokeStyle = '#000000'; g.lineWidth = 1.5;
    g.beginPath(); g.moveTo(2, 2); g.lineTo(14, 8); g.lineTo(2, 14); g.closePath(); g.fill(); g.stroke();
    if (!map.hasImage('arrow')) map.addImage('arrow', g.getImageData(0, 0, 16, 16));
  }
  map.addLayer({ id: 'route-arrows', type: 'symbol', source: 'route', minzoom: 11,
    layout: { 'symbol-placement': 'line', 'symbol-spacing': 90, 'icon-image': 'arrow', 'icon-size': 0.8, 'icon-rotation-alignment': 'map', 'icon-pitch-alignment': 'map', 'icon-keep-upright': false, 'icon-allow-overlap': true, 'icon-ignore-placement': true },
    paint: { 'icon-opacity': 0.9 } });
  map.addSource('ends', { type: 'geojson', data: empty });
  map.addLayer({ id: 'ends-dot', type: 'circle', source: 'ends', paint: { 'circle-radius': 9, 'circle-color': ['case', ['get', 'start'], '#2ecc71', '#e74c3c'], 'circle-stroke-color': '#fff', 'circle-stroke-width': 2 } });
  map.addLayer({ id: 'ends-label', type: 'symbol', source: 'ends',
    layout: { 'text-field': ['get', 'label'], 'text-size': 12, 'text-font': ['Open Sans Bold', 'Arial Unicode MS Bold'], 'text-offset': [0, -1.6], 'text-anchor': 'bottom', 'text-allow-overlap': true },
    paint: { 'text-color': '#fff', 'text-halo-color': '#000', 'text-halo-width': 1.5 } });
  map.addSource('draw', { type: 'geojson', data: empty });
  map.addLayer({ id: 'draw-fill', type: 'fill', source: 'draw', filter: ['==', '$type', 'Polygon'], paint: { 'fill-color': css('--c-aoi'), 'fill-opacity': 0.12 } });
  map.addLayer({ id: 'draw-line', type: 'line', source: 'draw', filter: ['==', '$type', 'LineString'], paint: { 'line-color': css('--c-aoi'), 'line-width': 2, 'line-dasharray': [2, 1.5] } });
  map.addLayer({ id: 'draw-pts', type: 'circle', source: 'draw', filter: ['==', '$type', 'Point'], paint: { 'circle-radius': 5, 'circle-color': css('--c-aoi'), 'circle-stroke-color': '#fff', 'circle-stroke-width': 1.5 } });
  // A checked mission file: legs green where they hold the planned clearance, red where short; the corrected route in blue.
  map.addSource('check', { type: 'geojson', data: empty });
  map.addLayer({ id: 'check-line', type: 'line', source: 'check', filter: ['==', '$type', 'LineString'], layout: { 'line-join': 'round', 'line-cap': 'round' },
    paint: { 'line-color': ['case', ['get', 'lifted'], css('--c-transit'), ['>', ['get', 'short'], 0], css('--c-err'), css('--c-ok')], 'line-width': ['case', ['get', 'lifted'], 2.5, 3.5] } });
  map.addLayer({ id: 'check-wps', type: 'circle', source: 'check', filter: ['==', '$type', 'Point'], minzoom: 12,
    paint: { 'circle-radius': 3, 'circle-color': ['case', ['get', 'lifted'], css('--c-transit'), '#fff'], 'circle-stroke-color': '#1c1c1c', 'circle-stroke-width': 1 } });
  map.on('click', 'route-hit', e => {
    if (state.pickingHome || state.drawing) return;
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

// ── Drawing a block on the map, and dragging its corners ──────────
function renderDraw(cursor?: LonLat) {
  const pts = state.drawPts, s = src('draw'); if (!s) return;
  const f: GeoJSON.Feature[] = pts.map(p => ({ type: 'Feature', properties: {}, geometry: { type: 'Point', coordinates: p } }));
  const ring = cursor ? [...pts, cursor] : pts;
  if (ring.length >= 2) f.push({ type: 'Feature', properties: {}, geometry: { type: 'LineString', coordinates: ring } });
  if (ring.length >= 3) f.push({ type: 'Feature', properties: {}, geometry: { type: 'Polygon', coordinates: [[...ring, ring[0]]] } });
  s.setData({ type: 'FeatureCollection', features: f });
}
function setDrawing(on: boolean) {
  if (on && state.pickingHome) setPicking(false);
  if (on) stopEditing();
  state.drawing = on; state.drawPts = [];
  $('drawHint').hidden = !on;
  $('drawBlock').textContent = on ? 'Cancel drawing' : 'Draw block';
  map.getCanvas().style.cursor = on ? 'crosshair' : '';
  if (on) map.doubleClickZoom.disable(); else map.doubleClickZoom.enable();
  renderDraw();
}
function finishDrawing() {
  const pts = state.drawPts.slice();
  setDrawing(false);
  if (pts.length < 3) { $('blockInfo').textContent = 'A block needs at least 3 corners.'; return; }
  setAoi({ name: `Drawn block (${pts.length} corners)`, poly: pts, polygonsFound: 1 });
}
$('drawBlock').onclick = () => setDrawing(!state.drawing);
map.on('click', e => {
  if (!state.drawing) return;
  state.drawPts.push([e.lngLat.lng, e.lngLat.lat]); renderDraw();
});
map.on('dblclick', e => { if (state.drawing) { e.preventDefault(); finishDrawing(); } });
map.on('mousemove', e => { if (state.drawing && state.drawPts.length) renderDraw([e.lngLat.lng, e.lngLat.lat]); });
document.addEventListener('keydown', e => {
  if (!state.drawing) return;
  if (e.key === 'Enter') { e.preventDefault(); finishDrawing(); }
  if (e.key === 'Escape') setDrawing(false);
  if (e.key === 'Backspace' && !(e.target instanceof HTMLInputElement)) { state.drawPts.pop(); renderDraw(); }
});

let cornerMarkers: maplibregl.Marker[] = [];
function stopEditing() {
  for (const m of cornerMarkers) m.remove();
  cornerMarkers = []; state.editing = false;
  $('editBlock').textContent = 'Edit corners';
}
function startEditing() {
  if (!state.aoi) return;
  if (state.drawing) setDrawing(false);
  stopEditing(); state.editing = true;
  $('editBlock').textContent = 'Done editing';
  state.aoi.poly.forEach((p, i) => {
    const el = document.createElement('div'); el.className = 'corner';
    const m = new maplibregl.Marker({ element: el, draggable: true }).setLngLat(p).addTo(map);
    m.on('dragend', () => {
      if (!state.aoi) return;
      const ll = m.getLngLat();
      const poly = state.aoi.poly.map((q, j) => (j === i ? [ll.lng, ll.lat] as LonLat : q));
      state.aoi = { ...state.aoi, poly, name: state.aoi.name.replace(/ \(edited\)$/, '') + ' (edited)' };
      state.demo = false;
      persist(); schedule();
    });
    cornerMarkers.push(m);
  });
}
$('editBlock').onclick = () => (state.editing ? stopEditing() : startEditing());
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
  map.setLayoutProperty('route-arrows', 'visibility', vis(flat));
  map.setFilter('route-arrows', ['in', ['get', 'role'], ['literal', visibleRoles()]]);
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
// Tilting the map by hand (right-drag / ctrl-drag) is the same as pressing 3D: terrain on and the route drawn at
// its real heights. Flattening it again goes back to the 2D drawing with the direction arrows.
map.on('pitchend', () => {
  const tilted = map.getPitch() > 5;
  if (tilted !== state.view3d) { state.view3d = tilted; persist(); applyView(false); }
});
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
const routeOpts = (plan: Plan, fromLine?: number) => ({
  // Terrain-following transit: home → approach and exit → home become waypoints of the route.
  transit: state.survey.transitInRoute && state.home
    ? { home: plan.proj.fwd(state.home[0], state.home[1]), speedMs: T('transitSpeedMs'), clearanceM: T('minClearanceM') } : null,
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
  const route = buildRoute(plan, routeOpts(plan, fromLine));
  const r: Result = { plan, route, flight: null, dense: null, st: null, issues: [], demError: null, transit: null, cov: null, photo: null, totalMin: null, sp: null, budget: null, verify: null };
  if (isPhoto()) r.photo = photoPlan(camera(), plan.o.aglM, route.speedMs, state.survey.frontlap, 1 / state.survey.shutterInv);
  if (!state.dem) return r;
  // Demo terrain belongs to the demo block only. Belt and braces: setAoi already drops it for any other block.
  if (state.dem.synthetic && !state.demo) { state.dem = null; state.demBuf = null; updateDemInfo(); return r; }
  try {
    const fr = flightRoute(plan, route, state.dem.elev);
    r.flight = fr.flight; r.dense = fr.dense;
  } catch (e) {
    r.demError = (e as Error).message + '. The DEM must cover the whole route with no gaps: block, run-ins, figure-8s and the terrain corridor around them. Fetch GLO-30 again for this route, or load a larger DEM.';
    return r;
  }
  r.st = stats(plan, r.flight);
  // The whole job is only a planning view: the cap applies to each sortie's route, which is what is flown.
  r.issues = validate(plan, r.flight).filter(i => !(isPhoto() && i.code === 'FIG8') && i.code !== 'WP_CAP_UNKNOWN');
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
  if (t?.inRoute) r.issues.push({ severity: 'info', code: 'TRANSIT_IN_ROUTE', message: 'Transit is part of the route: it starts and ends directly above home and follows the terrain in between, and the aircraft stops at the approach and exit points to start and stop recording. Take off within a few metres of the planned home point. How the aircraft ends a route above home has not been tried in the simulator yet.' });
  const limitCtx = { sensor: state.survey.sensor, pulse: isPhoto() || !isL3() ? undefined : pulse(), scanFovH: isPhoto() ? undefined : plan.o.fovDeg, sortieMin: state.survey.sortieMin };
  r.issues.push(...checkLimits(plan, r.flight, { ...limitCtx, totalMin: r.totalMin, topAboveHomeM: t?.topAboveHomeM }).filter(i => i.code !== 'SORTIES'));
  try {
    r.sp = planSorties(plan, state.dem.elev, state.home, state.takeoff, {
      usableMin: state.survey.sortieMin, firstLine: route.startIdx, speedMs: route.speedMs, fig8: !isPhoto() && state.survey.fig8,
      fig8End: !isPhoto() && state.survey.fig8 && state.survey.fig8End,
      overlapLines: state.survey.sortieOverlap, maxWaypoints: wpCap(),
      climbMs: P('climbMs'), descentMs: P('descentMs'), full: r.flight, transitInRoute: state.survey.transitInRoute,
    });
    r.issues.push(...r.sp.issues);
    // Each sortie is flown as its own route, with its own heights, figure-8s, transit and RTH: check each one.
    // (With a single sortie the route is the whole job, already checked above.)
    if (r.sp.sorties.length > 1) for (const so of r.sp.sorties) {
      const tag = `Sortie ${so.index + 1}: `;
      const own = [
        ...validate(plan, so.route, { maxWaypoints: wpCap() }),
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
  if (!so) return { plan: r.plan, route: r.route, flight: r.flight, dense: r.dense, transit: r.transit, cov: r.cov, issues: r.issues.filter(i => i.sortie == null), lastLine: r.plan.lines.length - 1 };
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
      maxSpeedMs: M400_LIMITS.lineSpeedMaxMs, vertAccelMs2: SAFETY.vertAccelMs2, brakeMs2: SAFETY.brakeMs2,
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
  $<HTMLButtonElement>('exportCsv').disabled = blocked || !r?.sp;
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
  $<HTMLButtonElement>('zoomBlock').disabled = !state.aoi;
  $<HTMLButtonElement>('editBlock').disabled = !state.aoi;
  $<HTMLButtonElement>('clearMission').disabled = !state.aoi;
  $<HTMLButtonElement>('fetchDem').disabled = !state.aoi;
  $('blockName').textContent = state.aoi ? state.aoi.name : 'No block loaded';
  // a checked mission file whose heights are relative to the take-off point follows the home position
  { const hk = state.home ? state.home.join(',') : ''; if (checkFile && hk !== lastHomeKey) { lastHomeKey = hk; runCheck(); } else lastHomeKey = hk; }
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
  if (!r) { for (const id of ['route', 'wps', 'rec', 'swath', 'overlap', 'transit', 'rth', 'ends']) src(id)!.setData(empty); return; }
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
  {
    // START = the first waypoint of the route (approach, where recording starts); END = the last (exit).
    const w = r.flight?.wps ?? [], a = w[0], b = w[w.length - 1];
    const pt = (x: { lon: number; lat: number } | undefined, start: boolean) => x ? [{ type: 'Feature' as const, properties: { start, label: start ? 'START' : 'END' }, geometry: { type: 'Point' as const, coordinates: [x.lon, x.lat] } }] : [];
    src('ends')!.setData({ type: 'FeatureCollection', features: [...pt(a, true), ...pt(b, false)] });
  }
}

// ── Render: 3D (deck.gl, true heights) ───────────────────────────
function renderDeck(r: View | null) {
  if (!state.view3d || !r?.flight) { deck.setProps({ layers: [] }); return; }
  const col: Record<string, [number, number, number, number]> = {
    line: rgba('--c-line'), fig8: rgba('--c-fig8'), approach: rgba('--c-appr'), exit: rgba('--c-appr'), runin: rgba('--c-run'), runout: rgba('--c-run'), transit: rgba('--c-transit'),
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
  if (!r) { el.innerHTML = `<div class="empty">${state.planError ? 'No plan: see Checks.' : state.check ? 'Mission file loaded: see the checker, the map and the profile.' : 'Drop a mission file to check it against the terrain, or plan a route under "Plan a route here".'}</div>`; return; }
  const { plan, route, st, cov, photo } = r;
  type Row = [string, string, string?] | string;
  const rows: Row[] = ['Block',
    ['Area', `${fmt(plan.areaHa)} ha`], ['Lines', `${plan.lines.length}`],
    [photo ? 'Footprint / spacing' : 'Swath / spacing', `${fmt(plan.swath)} / ${fmt(plan.spacing)} m`, 'At nominal AGL. Spacing = swath × (1 − sidelap).'],
    ['Waypoints', r.flight ? (r.flight.thinned && r.flight.thinned.before > r.flight.wps.length ? `${fmt(r.flight.wps.length)} (thinned from ${fmt(r.flight.thinned.before)})` : `${fmt(r.flight.wps.length)}`) : `${fmt(route.wps.length)}`, 'Thinning drops line waypoints wherever a straight leg stays within the tolerance above the terrain-following height; every leg is still held above the highest ground in its corridor.'],
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
    const p = { khz: lidarKhz() }, d = lidarDensity(p.khz * 1000, route.speedMs, plan.swath, plan.spacing);
    rows.push(`LiDAR · ${profile().name.replace(/^DJI Zenmuse /, '')}`,
      ['Pulse rate', `${p.khz} kHz`],
      ['Density per strip', `${fmt(d.perStrip)} pts/m²`, 'PRR ÷ (speed × swath) at nominal AGL, first return only, uniform spread assumed.'],
      ['Density with sidelap', `${fmt(d.total)} pts/m²`, 'PRR ÷ (speed × line spacing): all overlapping strips combined.'],
      ...(cov ? [['Min strip density', `${fmt(p.khz * 1000 / (route.speedMs * cov.swathMaxM))} pts/m²`, 'At the widest swath (highest AGL) on the lines.'] as Row] : []),
      ...(cov ? [['Sidelap achieved', `${fmt(cov.achievedMinPct)}–${fmt(cov.achievedMaxPct)} %`, 'Using each line waypoint\'s real AGL. Assumes level ground across-track.'] as Row] : []),
      ...(cov ? [['Swath on lines', `${fmt(cov.swathMinM)}–${fmt(cov.swathMaxM)} m`] as Row] : []),
    );
    const rgb = CAMERAS.find(c => c.id === 'l3-100')!;
    if (isL3()) {
      const everyM = rgbSpacingM(plan.o.aglM), minGapM = route.speedMs * rgb.minIntervalS;
      rows.push(['L3 RGB GSD / photo spacing', state.survey.rgbPhotos
        ? `${fmt(gsdCm(rgb, plan.o.aglM), 1)} cm · every ${fmt(everyM)} m (${state.survey.rgbFrontlap} % frontlap)${everyM < minGapM ? ` · ⚠ needs ${fmt(minGapM)} m at ${fmt(route.speedMs, 1)} m/s` : ''}`
        : `${fmt(gsdCm(rgb, plan.o.aglM), 1)} cm · photos off`,
        `L3 100 MP mapping cameras at nominal AGL, distance trigger. Their fastest rate is one photo per ${rgb.minIntervalS} s, so at line speed the spacing cannot be below ${fmt(minGapM)} m; the planner warns when the chosen frontlap asks for less.`]);
    }
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
    ...(t.inRoute ? [
      ['Transit', 'in the route, follows terrain', 'The way out and back are waypoints of the route, every "Max WP spacing", with heights from the DEM like every other leg. They are in the route length and time above.'] as Row,
      ['Transit clearance', `≥ ${fmt(Math.max(plan.o.aglM, T('minClearanceM')))} m`, 'The higher of the AGL and the minimum terrain clearance, over the highest DEM cell in the corridor of each transit leg.'] as Row,
    ] : [
      ['Take-off transit', `${fmt(t.distanceM / 1000, 1)} km · ${fmt(t.timeS / 60, 1)} min`] as Row,
      ['Transit clearance', `${fmt(t.minClearanceM)} m min`] as Row,
    ]),
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
  if (state.check?.report) { renderCheckProfile(svg, info); return; }
  const r = state.view;
  if (!r?.flight || !state.dem) {
    svg.replaceChildren();
    info.textContent = !state.dem ? 'Load terrain (Fetch GLO-30 or drop a GeoTIFF) to see the profile.' : '';
    return;
  }
  let pts: ProfilePt[], floor: number;
  if (state.selLine === 'transit' && r.transit?.inRoute) {
    const firstRunIn = r.flight.wps.findIndex(w => w.role === 'runin');
    pts = r.flight.wps.slice(0, firstRunIn + 1);
    floor = r.plan.o.aglM;
    $('floorLabel').textContent = 'Terrain + AGL';
    const tr = pts.filter(w => w.role === 'transit');
    info.textContent = `transit follows the terrain: ${tr.length} waypoints, height ${fmt(Math.min(...tr.map(w => w.h)))}–${fmt(Math.max(...tr.map(w => w.h)))} m`;
  } else if (state.selLine === 'transit' && r.transit) {
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
    const denseLine = r.dense?.wps.filter(w => w.line === state.selLine) ?? [];
    const thinNote = denseLine.length > wps.length ? ` · thinned ${denseLine.length} → ${wps.length} waypoints` : '';
    info.textContent = agl.length ? `AGL ${fmt(Math.min(...agl))}–${fmt(Math.max(...agl))} m (nominal ${floor}) · height ${fmt(Math.min(...wps.map(w => w.h)))}–${fmt(Math.max(...wps.map(w => w.h)))} m${thinNote}` : '';
    renderProfile(svg, r.plan, pts, state.dem.elev, floor, denseLine.length > wps.length ? denseLine : undefined);
    return;
  }
  renderProfile(svg, r.plan, pts, state.dem.elev, floor);
}
new ResizeObserver(() => renderProf()).observe($('profile'));

// ── Base map and place search ──────────────────────────────────────
$<HTMLSelectElement>('basemapSel').value = basemap;
function applyBasemap() {
  for (const b of BASEMAPS) if (map.getLayer('bm-' + b.id)) map.setLayoutProperty('bm-' + b.id, 'visibility', b.id === basemap ? 'visible' : 'none');
}
$('basemapSel').addEventListener('change', e => {
  basemap = (e.target as HTMLSelectElement).value;
  try { localStorage.setItem(BASEMAP_STORE, basemap); } catch { /* private window */ }
  applyBasemap();
});
const placeIn = $<HTMLInputElement>('placeSearch'), placeRes = $('placeResults');
interface Place { name: string; lat: number; lon: number; bbox?: [number, number, number, number] }   // bbox: west, south, east, north
function goTo(p: Place) {
  placeRes.hidden = true;
  if (p.bbox) map.fitBounds([[p.bbox[0], p.bbox[1]], [p.bbox[2], p.bbox[3]]], { padding: 40, maxZoom: 15, duration: 900 });
  else map.flyTo({ center: [p.lon, p.lat], zoom: 14, duration: 900 });
}
function showPlaces(items: Place[]) {
  placeRes.replaceChildren();
  if (!items.length) { const d = document.createElement('div'); d.className = 'none'; d.textContent = 'Nothing found. Try a town name, or "lat, lon".'; placeRes.append(d); }
  for (const p of items) { const b = document.createElement('button'); b.type = 'button'; b.textContent = p.name; b.onclick = () => goTo(p); placeRes.append(b); }
  placeRes.hidden = false;
}
async function searchPlace(q: string) {
  const m = q.match(/^\s*(-?\d+(?:\.\d+)?)\s*[, ]\s*(-?\d+(?:\.\d+)?)\s*$/);
  if (m) {
    const lat = +m[1], lon = +m[2];
    if (Math.abs(lat) <= 90 && Math.abs(lon) <= 180) { goTo({ name: q, lat, lon }); return; }
  }
  try {
    const res = await fetch(`https://nominatim.openstreetmap.org/search?format=jsonv2&limit=6&q=${encodeURIComponent(q)}`, { headers: { Accept: 'application/json' } });
    const js = await res.json() as { display_name: string; lat: string; lon: string; boundingbox?: string[] }[];
    showPlaces(js.map(r => ({
      name: r.display_name, lat: +r.lat, lon: +r.lon,
      bbox: r.boundingbox && r.boundingbox.length === 4 ? [+r.boundingbox[2], +r.boundingbox[0], +r.boundingbox[3], +r.boundingbox[1]] as [number, number, number, number] : undefined,
    })));
  } catch { showPlaces([]); }
}
placeIn.addEventListener('keydown', e => {
  if (e.key === 'Enter') { e.preventDefault(); const q = placeIn.value.trim(); if (q) void searchPlace(q); }
  if (e.key === 'Escape') placeRes.hidden = true;
});
document.addEventListener('click', e => { if (!(e.target as HTMLElement).closest('.place')) placeRes.hidden = true; });
$('stats').addEventListener('click', e => {
  const row = (e.target as HTMLElement).closest<HTMLElement>('[data-sortie]');
  if (!row) return;
  const i = Number(row.dataset.sortie);
  selectSortie(state.selSortie === i ? 'all' : i);
});

// ── Block input ──────────────────────────────────────────────────
function setAoi(aoi: Aoi, demo = false) {
  stopEditing();
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
  state.dem = { name: `${name} (${raster.width}×${raster.height})`, elev: rasterElev(raster), bbox: rasterBbox(raster), synthetic: false, raster, source, datum, gaps: rasterGaps(raster) };
  state.demBuf = buf;
  state.demo = false; persist(); updateDemInfo(); schedule();
  runCheck();
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
async function fetchGlo30(b: Bbox) {
  const key = otKey.value.trim();
  if (!key) { updateDemInfo('Enter your OpenTopography API key first (Terrain section).', true); return; }
  updateDemInfo('Requesting Copernicus GLO-30 from OpenTopography…');
  try {
    const res = await fetch(openTopoUrl(b, key, 'COP30'));
    const buf = await res.arrayBuffer();
    const head = new TextDecoder().decode(buf.slice(0, 200));
    if (!res.ok || !/^(II\*|MM\0\*)/.test(head.slice(0, 4))) throw new Error(`OpenTopography: ${head.replace(/<[^>]+>/g, ' ').trim().slice(0, 160) || res.status}`);
    await useDemBuffer(buf, 'Copernicus GLO-30', 'glo30', 'orthometric');
  } catch (e) { updateDemInfo((e as Error).message, true); }
}
$('fetchDem').onclick = () => {
  if (!state.aoi) return;
  const pts = [...state.aoi.poly, ...(state.home ? [state.home] : [])];
  fetchGlo30(bufferedBbox(pts, Math.max(2000, routeReachM())));
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
    if (/\.(kmz|kml)$/i.test(f.name)) routeOrBlock(f);
    else if (/\.tiff?$/i.test(f.name)) onDemFile(f);
  }
});

// ── Check a mission file (any DJI KMZ against the terrain; core/check.ts) ─────
// A dropped KMZ with a wpmz/waylines.wpml inside is a mission to check; anything else is a block polygon.
async function routeOrBlock(f: File) {
  try {
    const zip = await JSZip.loadAsync(await f.arrayBuffer());
    if (Object.keys(zip.files).some(n => /waylines\.wpml$/.test(n))) return onCheckFile(f);
  } catch { /* not a zip: fall through to the block reader */ }
  onBlockFile(f);
}
const chkAgl = $<HTMLInputElement>('chkAgl'), chkCorr = $<HTMLInputElement>('chkCorr');
chkAgl.value = String(state.aoi ? P('aglM') : 300); chkCorr.value = String(P('corridorM'));
const chkA = () => Math.max(30, Number(chkAgl.value) || P('aglM')), chkC = () => Math.max(0, Number(chkCorr.value) || 0);
let checkFile: { name: string; waylines: string; template: string | null; dsm: ElevFn | null; dsmTiles: number } | null = null;
let lastHomeKey = '';
async function onCheckFile(f: File) {
  try {
    const zip = await JSZip.loadAsync(await f.arrayBuffer());
    const get = async (re: RegExp) => { const n = Object.keys(zip.files).find(x => re.test(x)); return n ? zip.files[n].async('string') : null; };
    const waylines = await get(/waylines\.wpml$/);
    if (!waylines) throw new Error('No wpmz/waylines.wpml in this KMZ.');
    // the terrain model(s) the mission was planned on travel inside the KMZ (wpmz/res/dsm): read them, to see what a change does
    const tiles = Object.keys(zip.files).filter(n => /res\/dsm\/.*\.tiff?$/i.test(n));
    const dsm = await embeddedDsm(await Promise.all(tiles.map(n => zip.files[n].async('arraybuffer'))));
    checkFile = { name: f.name.replace(/\.kmz$/i, ''), waylines, template: await get(/template\.kml$/), dsm, dsmTiles: tiles.length };
    $<HTMLButtonElement>('chkFetch').disabled = false;
    // a mapping mission's template carries the AGL it was planned with; a waypoint template's "height" is absolute
    try {
      const probe = readMissionFiles(waylines, checkFile.template, 0);
      if (probe.templateType && probe.templateType !== 'waypoint' && probe.flightHeightM != null && probe.flightHeightM >= 30 && probe.flightHeightM <= 1500) chkAgl.value = String(probe.flightHeightM);
    } catch { /* needs the take-off point; handled in runCheck */ }
    const hint = /AGL(\d{2,4})/i.exec(f.name);   // this planner's own files carry the AGL in their name
    if (hint) chkAgl.value = hint[1];
    $('chkClear').hidden = false;
    runCheck();
  } catch (e) { state.check = null; $('chkInfo').textContent = (e as Error).message; $('chkLegs').innerHTML = ''; renderCheckMap(); }
}
function runCheck() {
  $<HTMLButtonElement>('dsmBtn').disabled = !state.dem?.raster;
  if (!checkFile) { renderDsmPreview(); return; }
  const info = $('chkInfo'), legs = $('chkLegs');
  $<HTMLButtonElement>('dsmSuggest').disabled = true;
  try {
    const dem = state.dem;
    const home = state.home && dem ? dem.elev(state.home[0], state.home[1]) : null;
    const mission = readMissionFiles(checkFile.waylines, checkFile.template, home ?? null);
    state.check = { name: checkFile.name, mission, report: null };
    if (!dem || dem.synthetic) { info.textContent = `${checkFile.name}: ${mission.wps.length} waypoints read. Load terrain (Fetch GLO-30 or drop a GeoTIFF) to measure it.`; legs.innerHTML = ''; renderCheckMap(); renderProf(); return; }
    if (mission.wps.some(w => dem.elev(w.lon, w.lat) == null)) { info.textContent = `${checkFile.name}: the loaded DEM does not cover this route. Load a block around it and fetch GLO-30, or drop a wider GeoTIFF.`; legs.innerHTML = ''; renderCheckMap(); renderProf(); return; }
    const report = checkClearance(mission.wps, dem.elev, { aglM: chkA(), corridorM: chkC(), marginM: 10 });
    state.check.report = report;
    const n = mission.notes.length ? ' ' + mission.notes.join(' ') : '';
    info.innerHTML = `<b>${esc(checkFile.name)}</b>: ${mission.wps.length} waypoints, ${(report.lengthM / 1000).toFixed(2)} km, heights ${esc(mission.heightMode)}${mission.templateType ? `, ${esc(mission.templateType)} template` : ''}.${esc(n)}<br>`
      + `Planned ${report.aglM} m: lowest clearance <b>${fmt(report.minUnderM)} m</b> over the ground under the line, <b>${fmt(report.minClearM)} m</b> over the ±${report.corridorM} m corridor. `
      + (report.nShort ? `<span style="color:var(--c-err)">${report.nShort} of ${report.legs.length} legs are more than ${report.marginM} m short (${(report.shortLengthM / 1000).toFixed(2)} km).</span>` : `<span style="color:var(--c-ok)">Every leg holds the planned clearance within ${report.marginM} m.</span>`)
      + ` Too high: ${fmt(report.aboveM.p50)} m above corridor + AGL in the middle, ${fmt(report.aboveM.max)} m at most → about ${Math.round(100 * report.densityFactor)} % of the planned point density.`;
    const bad = report.legs.filter(l => l.shortM > 0).sort((a, b) => b.shortM - a.shortM);
    legs.innerHTML = bad.slice(0, 12).map(l => `<div class="issue error"><b>short</b><span>Leg ${l.leg + 1} (WP ${l.leg + 1}→${l.leg + 2}, ${fmt(l.lengthM)} m): ${fmt(l.minUnderM)} m over the ground under the line, ${fmt(l.minClearM)} m over the corridor → ${fmt(l.shortM)} m short</span></div>`).join('')
      + (bad.length > 12 ? `<div class="issue info"><b>…</b><span>${bad.length - 12} more short legs</span></div>` : '');
    renderDsmPreview();
  } catch (e) {
    const msg = (e as Error).message;
    state.check = null;
    $('dsmPreview').textContent = 'Load a mission and terrain to see what a DSM would do to that route.';
    info.textContent = /take-off point/.test(msg) ? msg + ' (Set home on the map at the take-off position: the DEM ground there is then used.)' : msg;
    legs.innerHTML = '';
  }
  renderCheckMap(); renderProf();
}
function renderCheckMap() {
  const c = state.check, s = src('check');
  if (!s) return;
  if (!c) { s.setData(empty); return; }
  const feats: GeoJSON.Feature[] = [];
  const legShort = new Map<number, number>((c.report?.legs ?? []).map(l => [l.leg, l.shortM]));
  for (let i = 0; i < c.mission.wps.length - 1; i++) {
    const a = c.mission.wps[i], b = c.mission.wps[i + 1];
    feats.push({ type: 'Feature', properties: { short: legShort.get(i) ?? 0, lifted: false }, geometry: { type: 'LineString', coordinates: [[a.lon, a.lat], [b.lon, b.lat]] } });
  }
  for (const w of c.mission.wps) feats.push({ type: 'Feature', properties: { lifted: false }, geometry: { type: 'Point', coordinates: [w.lon, w.lat] } });
  s.setData({ type: 'FeatureCollection', features: feats });
  if (!c.report && c.mission.wps.length) {   // first look: bring the file into view
    const lons = c.mission.wps.map(w => w.lon), lats = c.mission.wps.map(w => w.lat);
    map.fitBounds([[Math.min(...lons), Math.min(...lats)], [Math.max(...lons), Math.max(...lats)]] as LngLatBoundsLike, { padding: 60, duration: 600 });
  }
}
// Profile of the checked file: ground under the line, highest ground in the corridor, corridor + AGL (the floor), the
// file's heights and, after a lift, the corrected heights.
function renderCheckProfile(svg: SVGSVGElement, info: HTMLElement) {
  const c = state.check!, r = c.report!;
  const Wd = svg.clientWidth || 600, Hd = svg.clientHeight || 160, L = 44, R = 10, T = 8, B = 18;
  const ys = [...r.stations.map(s => s.h), ...r.stations.map(s => s.under), ...r.stations.map(s => s.corridor + r.aglM)];
  const y0 = Math.min(...ys) - 10, y1 = Math.max(...ys) + 10, sMax = r.lengthM || 1;
  const X = (s: number) => L + ((Wd - L - R) * s) / sMax, Y = (h: number) => T + ((Hd - T - B) * (y1 - h)) / (y1 - y0);
  const path = (pts: { s: number; v: number }[]) => pts.map((p, i) => `${i ? 'L' : 'M'}${X(p.s).toFixed(1)},${Y(p.v).toFixed(1)}`).join(' ');
  const st = r.stations;
  const terrain = path(st.map(s => ({ s: s.s, v: s.under }))) + ` L${X(sMax).toFixed(1)},${(Hd - B).toFixed(1)} L${L},${(Hd - B).toFixed(1)} Z`;
  const ticks = [y0 + 10, (y0 + y1) / 2, y1 - 10].map(v => `<text class="axis" x="2" y="${(Y(v) + 4).toFixed(1)}">${fmt(v)}</text><line class="grid" x1="${L}" x2="${Wd - R}" y1="${Y(v).toFixed(1)}" y2="${Y(v).toFixed(1)}"/>`).join('');
  const shortSeg = st.filter(s => s.h - s.corridor < r.aglM - r.marginM).map(s => `<circle cx="${X(s.s).toFixed(1)}" cy="${Y(s.h).toFixed(1)}" r="2" fill="${css('--c-err')}"/>`).join('');
  svg.setAttribute('viewBox', `0 0 ${Wd} ${Hd}`);
  svg.innerHTML = ticks + `<path class="terrain" d="${terrain}"/>` + `<path d="${path(st.map(s => ({ s: s.s, v: s.corridor })))}" fill="none" stroke="#b08a5a" stroke-width="1"/>`
    + `<path class="agl-floor" d="${path(st.map(s => ({ s: s.s, v: s.corridor + r.aglM })))}"/>` + `<path class="flight" d="${path(st.map(s => ({ s: s.s, v: s.h })))}"/>` + shortSeg
    + `<text class="axis" x="${Wd - R}" y="${Hd - 4}" text-anchor="end">${(sMax / 1000).toFixed(1)} km along the route</text>`;
  $('floorLabel').textContent = 'Corridor + planned AGL';
  info.textContent = `${c.name}: file heights ${fmt(Math.min(...st.map(s => s.h)))}–${fmt(Math.max(...st.map(s => s.h)))} m; lowest clearance ${fmt(r.minClearM)} m over the corridor`;
}
$<HTMLInputElement>('fileCheck').onchange = e => { const f = (e.target as HTMLInputElement).files?.[0]; if (f) onCheckFile(f); };
$('chkFetch').onclick = () => {
  const c = state.check; if (!c) return;
  const pts = [...c.mission.wps.map(w => [w.lon, w.lat] as LonLat), ...(state.home ? [state.home] : [])];
  fetchGlo30(bufferedBbox(pts, Math.max(1500, chkC() + 1000)));
};
if (state.aoi) $<HTMLDetailsElement>('plannerAdv').open = true;
chkAgl.onchange = chkCorr.onchange = () => runCheck();
$('chkClear').onclick = () => { checkFile = null; state.check = null; renderDsmPreview(); $<HTMLButtonElement>('chkFetch').disabled = true; $('chkInfo').textContent = 'No mission file loaded.'; $('chkLegs').innerHTML = ''; $<HTMLButtonElement>('chkLift').disabled = true; $('chkClear').hidden = true; renderCheckMap(); renderProf(); };
// ── A better DSM for Pilot 2: preview against the loaded mission, suggest, export (never edits the route) ─────
// The DEM turned into the surface Pilot 2 follows: each cell the highest ground within `dilateM`, + `marginM`.
const surfaceCache = new Map<string, ElevFn>();
function surface(dem: Dem, dilateM: number, marginM: number): ElevFn {
  const key = `${dem.name}|${dilateM}|${marginM}`;
  let e = surfaceCache.get(key);
  if (!e) {
    const sp = dsmFromRaster(dem.raster!, { dilateM, marginM, datum: 'orthometric' });
    e = rasterElev({ width: sp.width, height: sp.height, west: sp.tie[0], north: sp.tie[1], dLon: sp.pixel[0], dLat: sp.pixel[1], data: sp.values, nodata: sp.nodata ?? null, pixelIsPoint: false });
    surfaceCache.set(key, e);
    if (surfaceCache.size > 40) surfaceCache.delete(surfaceCache.keys().next().value!);
  }
  return e;
}
const dsmChoice = () => ({ dilateM: Number($<HTMLSelectElement>('dsmDilate').value) || 0, marginM: Number($<HTMLSelectElement>('dsmMargin').value) || 0 });
function renderDsmPreview() {
  const c = state.check, dem = state.dem, el = $('dsmPreview');
  $<HTMLButtonElement>('dsmBtn').disabled = !dem?.raster;
  $<HTMLButtonElement>('dsmSuggest').disabled = true;
  if (!c?.report || !dem?.raster || dem.synthetic || !checkFile) { el.textContent = dem?.raster ? 'Drop a mission file to see what this DSM would do to that route.' : 'Load terrain (Fetch GLO-30 or drop a GeoTIFF) to make a DSM.'; return; }
  const cur = checkFile.dsm ?? dem.elev, { dilateM, marginM } = dsmChoice();
  const now = previewDsm(c.report, cur, cur), next = previewDsm(c.report, cur, surface(dem, dilateM, marginM));
  if (!now || !next) { el.textContent = 'The DSM does not cover the whole route.'; return; }
  const line = (p: NonNullable<typeof now>) => `${Math.round(100 * p.shortShare)} % of the route short by more than ${c.report!.marginM} m, lowest ${fmt(p.lowestClearM)} m, ${fmt(p.aboveP50M)} m above plan (median), about ${Math.round(100 * p.densityFactor)} % of the planned point density`;
  el.innerHTML = `<b>As planned now</b> (${checkFile.dsm ? `on the ${checkFile.dsmTiles} DSM tile(s) carried in the file` : 'no DSM in the file: the loaded terrain is assumed'}): ${line(now)}.<br>`
    + `<b>With this DSM</b> (preview): ${line(next)}.<br><span style="color:var(--muted)">First-order: assumes Pilot 2 passes the change in the surface straight through to its route, as it follows the surface now. Plan on it, export the mission, and check that file before flying.</span>`;
  $<HTMLButtonElement>('dsmSuggest').disabled = false;
}
$('dsmDilate').addEventListener('change', renderDsmPreview); $('dsmMargin').addEventListener('change', renderDsmPreview);
// The least height that leaves almost none of the route short: scan the options, keep the lowest median height above plan
// among those with at most 3 % of the route short (else the one with the least short).
$('dsmSuggest').onclick = () => {
  const c = state.check, dem = state.dem;
  if (!c?.report || !dem?.raster || !checkFile) return;
  const cur = checkFile.dsm ?? dem.elev;
  let best: { D: number; M: number; p: NonNullable<ReturnType<typeof previewDsm>> } | null = null;
  for (const D of [0, 30, 50, 75]) for (const M of [0, 10, 20, 30, 40, 50]) {
    const p = previewDsm(c.report, cur, surface(dem, D, M)); if (!p) continue;
    const ok = (x: typeof p) => x.shortShare <= 0.03;
    if (!best || (ok(p) && (!ok(best.p) || p.aboveP50M < best.p.aboveP50M)) || (!ok(best.p) && !ok(p) && p.shortShare < best.p.shortShare)) best = { D, M, p };
  }
  if (!best) return;
  $<HTMLSelectElement>('dsmDilate').value = String(best.D); $<HTMLSelectElement>('dsmMargin').value = String(best.M);
  renderDsmPreview();
};
// Export: EPSG:4326 GeoTIFF of the loaded DEM, WGS84 ellipsoidal heights (Pilot 2 reads them as ellipsoidal whatever the tags).
$('dsmBtn').onclick = () => {
  const dem = state.dem;
  if (!dem?.raster) return;
  const { dilateM, marginM } = dsmChoice();
  const pts: LonLat[] = state.check ? state.check.mission.wps.map(w => [w.lon, w.lat] as LonLat) : state.aoi ? state.aoi.poly : [];
  const bbox = pts.length ? bufferedBbox(pts, Math.max(1000, dilateM + 500)) : undefined;
  try {
    const spec = dsmFromRaster(dem.raster, { dilateM, marginM, datum: 'ellipsoidal', bbox });
    const name = `dsm-${(state.check?.name ?? state.aoi?.name ?? 'dem').replace(/[^\w-]+/g, '_').slice(0, 40)}-max${dilateM}m-plus${marginM}m-ellipsoidal.tif`;
    download(name, new Uint8Array(writeGeoTiff(spec)), 'image/tiff');
    $('demInfo').textContent = `DSM written: ${spec.width} × ${spec.height} cells, highest ground within ${dilateM} m + ${marginM} m, WGS84 ellipsoidal heights, EPSG:4326 → ${name}. Import in Pilot 2: Mapping mission → Terrain Follow → DSM file, plan the mission again on it, export the KMZ and drop it here to check it before flying.`;
  } catch (e) { $('demInfo').textContent = (e as Error).message; }
};

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
    const f = flightRoute(p, buildRoute(p, routeOpts(p)), state.dem.elev).flight;
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
  if (!isL3()) return 'Route files are only written for the Zenmuse L3 on the M400. Other sensor profiles are for planning.';
  if (state.survey.scan !== 'linear') return 'Only the Linear (repetitive) scan mode is verified from your samples so far.';
  return null;
}
function rgbSpacingM(aglM: number): number {
  const c = CAMERAS.find(c => c.id === 'l3-100')!;
  // photo every (footprint along track × (1 − frontlap)) metres: a distance trigger, never a timer
  return 2 * aglM * Math.tan((c.vfovDeg / 2) * Math.PI / 180) * (1 - state.survey.rgbFrontlap / 100);
}
function kmzFor(route: Route<FlightWp>, aglM: number): WpmlFiles {
  return writeWpml(route, {
    takeoff: { ...TAKEOFF_DEFAULTS, ...state.takeoff },
    // Model colouring stays OFF: with it on the L3 shoots on a 2 s timer and ignores the distance trigger; with it
    // off the photos follow the distance trigger and DJI Terra still colours the cloud from them (15 Oct 2026).
    lidar: { samplingRate: pulse().khz * 1000, returnMode: 'sedecupleReturn', scanningMode: 'repetitive', modelColoring: false },
    djiImuCalibration: state.survey.djiCal && !state.survey.transitInRoute,   // belongs to the first and last waypoint of a route
    rgbPhotoSpacingM: state.survey.rgbPhotos ? +rgbSpacingM(aglM).toFixed(2) : null,
    gimbalStartGroup: false,
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
      : { kind: 'lidar', payload: profile().name, fovDeg: r.plan.o.fovDeg, pulseKhz: lidarKhz(), ...(isL3() ? { pulse: pulse(), scanMode: scan() } : {}), fig8: state.survey.fig8, transitInRoute: state.survey.transitInRoute },
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

// The same routes as a CSV for UgCS: import it there to look at terrain clearance in software that shares
// nothing with this planner. Only routes that passed the checks here are exported.
$('exportCsv').onclick = async () => {
  const rel = releasable();
  if (!rel?.r.sp) return;
  const sorties = rel.r.sp.sorties;
  const name = (so: typeof sorties[number]) => `${missionBase()}${sorties.length > 1 ? `_S${String(so.index + 1).padStart(2, '0')}_L${so.fromLine + 1}-${so.toLine + 1}` : ''}_UgCS-check.csv`;
  const sel = state.selSortie !== 'all' ? sorties[state.selSortie] : undefined;
  const list = sel ? [sel] : sorties;
  if (list.length === 1) { download(name(list[0]), new Blob([toUgcsCsv(list[0].route)], { type: 'text/csv' })); return; }
  const zip = new JSZip();
  for (const so of list) zip.file(name(so), toUgcsCsv(so.route));
  download(`${missionBase()}_${list.length}-sorties_UgCS-check.zip`, await zip.generateAsync({ type: 'blob' }));
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
