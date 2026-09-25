import 'maplibre-gl/dist/maplibre-gl.css';
import './style.css';
import maplibregl, { type GeoJSONSource, type LngLatBoundsLike } from 'maplibre-gl';
import { MapboxOverlay } from '@deck.gl/mapbox';
import { PathLayer, LineLayer } from '@deck.gl/layers';
import {
  DEFAULTS, TAKEOFF_DEFAULTS, planLines, buildRoute, applyHeights, stats, validate, readGeoTiff, rasterElev, rasterBbox,
  bufferedBbox, openTopoUrl, planTransit, coverage,
  type PlanOptions, type Plan, type Route, type FlightWp, type RouteStats, type Issue, type ElevFn, type Bbox,
  type LonLat, type TakeoffOptions, type Transit, type Coverage, type Pt3,
} from '@3dm/core';
import { aoiFromFile, type Aoi } from './aoi.ts';
import { demoPoly, demoElev } from './demo.ts';
import { renderProfile, type ProfilePt } from './profile.ts';
import { TIPS } from './tips.ts';

// ── State ─────────────────────────────────────────────────────────
interface Dem { name: string; elev: ElevFn; bbox?: Bbox }
interface Result {
  plan: Plan; route: Route; flight: Route<FlightWp> | null; st: RouteStats | null; issues: Issue[];
  demError: string | null; transit: Transit | null; cov: Coverage | null;
}

const STORE = '3dm.planner.v1';
type Saved = { aoi: Aoi | null; params: Partial<PlanOptions>; demo: boolean; home: LonLat | null; takeoff: Partial<TakeoffOptions>; view3d: boolean; swaths: boolean };
const blank: Saved = { aoi: null, params: {}, demo: false, home: null, takeoff: {}, view3d: false, swaths: true };
const load = (): Saved => { try { return { ...blank, ...JSON.parse(localStorage.getItem(STORE) || '{}') }; } catch { return blank; } };
const saved = load();
const state = {
  ...saved,
  dem: (saved.demo ? { name: 'Synthetic ridge (demo)', elev: demoElev } : null) as Dem | null,
  resume: { on: false, fromLine: 1, speed: 14 },
  selLine: 0 as number | 'transit',
  pickingHome: false,
  result: null as Result | null,
};
const persist = () => {
  const { aoi, params, demo, home, takeoff, view3d, swaths } = state;
  try { localStorage.setItem(STORE, JSON.stringify({ aoi, params, demo, home, takeoff, view3d, swaths })); } catch { /* private mode */ }
};
const P = <K extends keyof PlanOptions>(k: K): PlanOptions[K] => (state.params[k] ?? DEFAULTS[k]) as PlanOptions[K];
const T = <K extends keyof TakeoffOptions>(k: K): TakeoffOptions[K] => (state.takeoff[k] ?? TAKEOFF_DEFAULTS[k]) as TakeoffOptions[K];

// ── Layout ────────────────────────────────────────────────────────
type Field<K> = { key: K; label: string; unit: string; step: number; min: number; max: number };
const FIELDS: Field<keyof PlanOptions>[] = [
  { key: 'aglM', label: 'AGL', unit: 'm', step: 10, min: 30, max: 1500 },
  { key: 'speedMs', label: 'Line speed', unit: 'm/s', step: 0.5, min: 1, max: 25 },
  { key: 'fovDeg', label: 'L3 across-track FOV', unit: '°', step: 1, min: 10, max: 120 },
  { key: 'sidelapPct', label: 'Sidelap (overlap)', unit: '%', step: 5, min: 0, max: 90 },
  { key: 'courseDeg', label: 'Line course', unit: '°', step: 1, min: 0, max: 359 },
  { key: 'runInM', label: 'Run-in', unit: 'm', step: 10, min: 0, max: 1000 },
  { key: 'runOutM', label: 'Run-out', unit: 'm', step: 10, min: 0, max: 1000 },
  { key: 'wpSpacingM', label: 'Max WP spacing', unit: 'm', step: 10, min: 20, max: 1000 },
  { key: 'corridorM', label: 'Terrain corridor ±', unit: 'm', step: 5, min: 0, max: 500 },
  { key: 'maxGradient', label: 'Max climb/descent', unit: 'rise/run', step: 0.01, min: 0.01, max: 1 },
  { key: 'fig8BankDeg', label: 'Figure-8 bank', unit: '°', step: 1, min: 5, max: 35 },
];
const TK_FIELDS: Field<'takeoffSecurityM' | 'transitSpeedMs' | 'minClearanceM'>[] = [
  { key: 'takeoffSecurityM', label: 'Take-off security height', unit: 'm', step: 5, min: 2, max: 1500 },
  { key: 'transitSpeedMs', label: 'Transit speed', unit: 'm/s', step: 0.5, min: 1, max: 25 },
  { key: 'minClearanceM', label: 'Min terrain clearance', unit: 'm', step: 5, min: 10, max: 500 },
];
const tip = (k: string) => (TIPS[k] ? ` data-tip="${TIPS[k].replace(/"/g, '&quot;')}"` : '');
const fieldHtml = (f: Field<string>, group: string) =>
  `<label${tip(f.key)}>${f.label} <em>${f.unit}</em><input type="number" data-group="${group}" data-key="${f.key}" step="${f.step}" min="${f.min}" max="${f.max}" /></label>`;

const app = document.getElementById('app')!;
app.innerHTML = `
<header>
  <div class="brand"><span class="mark">3DM</span> Planner <span class="sub">M400 · Zenmuse L3</span></div>
  <div class="blockname" id="blockName">No block loaded</div>
  <div class="head-actions">
    <button id="exportJson" class="ghost" disabled${tip('exportJson')}>Export mission.json</button>
    <button id="exportKmz" class="ghost" disabled data-tip="Waits for the RC Waypoint Route sample export (see samples/README.md). We don't guess the WPML header.">Export KMZ</button>
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
    <div class="row"><input id="otKey" type="password" placeholder="OpenTopography API key" autocomplete="off" />
      <button id="fetchDem" disabled${tip('fetchDem')}>Fetch GLO-30</button></div>
    <label class="drop small"><input type="file" id="fileDem" accept=".tif,.tiff" hidden />
      <span>…or drop a GeoTIFF DEM (EPSG:4326)</span></label>
    <div class="note" id="demInfo"></div>
  </section>
  <section>
    <h2>Survey</h2>
    <div class="fields" id="fields">${FIELDS.map(f => fieldHtml(f, 'plan')).join('')}</div>
    <div class="row"><button id="optCourse" class="ghost" disabled${tip('optCourse')}>Optimise course</button><button id="resetParams" class="link">Reset defaults</button></div>
  </section>
  <section>
    <h2>Home &amp; take-off</h2>
    <div class="row"><button id="setHome" class="ghost"${tip('setHome')}>Set home on map</button><span class="note inline" id="homeInfo">Not set</span></div>
    <div class="fields" id="tkFields" style="margin-top:10px">
      ${TK_FIELDS.map(f => fieldHtml(f, 'takeoff')).join('')}
      <label${tip('rthHeightM')}>RTH height <em>m above home</em><input type="number" id="rthHeight" step="10" min="20" max="1500" placeholder="auto" /></label>
      <label class="span2"${tip('flyToMode')}>Fly to route <em>DJI flyToWaylineMode</em>
        <select id="flyToMode"><option value="safely">Safely: climb, then fly level</option><option value="pointToPoint">Point to point: climb to security height, then slope</option></select></label>
    </div>
  </section>
  <section>
    <h2>Resume</h2>
    <label class="check"${tip('resOn')}><input type="checkbox" id="resOn" /> Build a resume route</label>
    <div class="fields">
      <label${tip('resLine')}>Data stopped on line<input type="number" id="resLine" min="1" value="1" /></label>
      <label${tip('resSpeed')}>New speed <em>m/s</em><input type="number" id="resSpeed" step="0.5" min="1" max="25" value="14" /></label>
    </div>
  </section>
</aside>
<main>
  <div id="map"></div>
  <div class="legend">
    <span><i style="background:var(--c-line)"></i>Data line</span><span><i style="background:var(--c-run)"></i>Run-in/out, turn</span>
    <span><i style="background:var(--c-fig8)"></i>Figure-8</span><span><i style="background:var(--c-appr)"></i>Approach</span>
    <span><i style="background:var(--c-transit)"></i>Take-off transit</span><span><i style="background:var(--c-rth)"></i>RTH from last WP</span>
    <span><i style="background:var(--c-swath)"></i>LiDAR swath</span>
  </div>
  <div class="map-toggles">
    <button id="view3d" class="ghost"${tip('view3d')}>3D</button>
    <button id="swaths" class="ghost"${tip('swaths')}>Swaths</button>
  </div>
  <div class="pick-hint" id="pickHint" hidden>Click the map to set home · Esc to cancel</div>
  <section class="bottom">
    <div class="stats" id="stats"></div>
    <div class="profile-wrap">
      <div class="profile-head"><h2>Profile</h2><select id="lineSel"></select>
        <span class="key"><i class="k-flight"></i>Flight height <i class="k-floor"></i><span id="floorLabel">Terrain + AGL</span> <i class="k-terr"></i>Terrain</span></div>
      <svg id="profile"></svg>
    </div>
    <div class="issues" id="issues"></div>
  </section>
</main>
<div id="tooltip" role="tooltip" hidden></div>`;

const $ = <T extends HTMLElement = HTMLElement>(id: string) => document.getElementById(id) as T;

// ── Tooltips (one floating element, so the sidebar's scroll box doesn't clip them) ──
const tipEl = $('tooltip');
document.addEventListener('mouseover', e => {
  const t = (e.target as HTMLElement).closest<HTMLElement>('[data-tip]');
  if (!t) { tipEl.hidden = true; return; }
  tipEl.textContent = t.dataset.tip!;
  tipEl.hidden = false;
  const r = t.getBoundingClientRect(), w = tipEl.offsetWidth, h = tipEl.offsetHeight;
  let x = r.right + 10, y = r.top;
  if (x + w > innerWidth - 8) x = Math.max(8, r.left - w - 10);
  if (y + h > innerHeight - 8) y = innerHeight - h - 8;
  tipEl.style.left = x + 'px'; tipEl.style.top = y + 'px';
});

// ── Inputs ────────────────────────────────────────────────────────
const syncFields = () => {
  document.querySelectorAll<HTMLInputElement>('input[data-group]').forEach(i => {
    i.value = String(i.dataset.group === 'plan' ? P(i.dataset.key as keyof PlanOptions) : T(i.dataset.key as keyof TakeoffOptions));
  });
  $<HTMLInputElement>('rthHeight').value = state.takeoff.rthHeightM != null ? String(state.takeoff.rthHeightM) : '';
  $<HTMLSelectElement>('flyToMode').value = T('flyToMode');
};
syncFields();
document.querySelector('aside')!.addEventListener('input', e => {
  const i = e.target as HTMLInputElement;
  if (!i.dataset.group) return;
  const v = Number(i.value);
  if (i.value === '' || !Number.isFinite(v)) return;
  if (i.dataset.group === 'plan') state.params[i.dataset.key as keyof PlanOptions] = v as never;
  else state.takeoff[i.dataset.key as keyof TakeoffOptions] = v as never;
  persist(); schedule();
});
$('rthHeight').addEventListener('input', e => {
  const v = (e.target as HTMLInputElement).value;
  state.takeoff.rthHeightM = v === '' ? null : Number(v);
  persist(); schedule();
});
$('flyToMode').addEventListener('change', e => { state.takeoff.flyToMode = (e.target as HTMLSelectElement).value as TakeoffOptions['flyToMode']; persist(); schedule(); });
$('resetParams').onclick = () => { state.params = {}; persist(); syncFields(); schedule(); };

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
    layers: [{ id: 'sat', type: 'raster', source: 'sat', paint: { 'raster-saturation': -0.35, 'raster-brightness-max': 0.8 } }],
  },
  center: [22.03, -33.21], zoom: 5, maxPitch: 80, attributionControl: { compact: true },
});
map.addControl(new maplibregl.NavigationControl({ visualizePitch: true }), 'top-right');
map.addControl(new maplibregl.ScaleControl({ unit: 'metric' }), 'bottom-right');
const deck = new MapboxOverlay({ interleaved: true, layers: [] });
map.addControl(deck);

const empty: GeoJSON.FeatureCollection = { type: 'FeatureCollection', features: [] };
const roleColor = () => ['match', ['get', 'role'],
  'line', css('--c-line'), 'fig8', css('--c-fig8'), 'approach', css('--c-appr'), css('--c-run')] as unknown as string;
const ROUTE_2D = ['route-line', 'transit-line', 'rth-line', 'wps'];

let mapReady = false;
map.on('load', () => {
  for (const id of ['aoi', 'route', 'wps', 'rec', 'swath', 'transit', 'rth']) map.addSource(id, { type: 'geojson', data: empty });
  map.addLayer({ id: 'swath-fill', type: 'fill', source: 'swath', paint: { 'fill-color': css('--c-swath'), 'fill-opacity': 0.16 } });
  map.addLayer({ id: 'aoi-fill', type: 'fill', source: 'aoi', paint: { 'fill-color': css('--c-aoi'), 'fill-opacity': 0.08 } });
  map.addLayer({ id: 'aoi-line', type: 'line', source: 'aoi', paint: { 'line-color': css('--c-aoi'), 'line-width': 2 } });
  map.addLayer({ id: 'route-line', type: 'line', source: 'route', layout: { 'line-join': 'round', 'line-cap': 'round' },
    paint: { 'line-color': roleColor(), 'line-width': ['case', ['get', 'sel'], 4.5, 2], 'line-opacity': ['case', ['get', 'sel'], 1, 0.85] } });
  map.addLayer({ id: 'transit-line', type: 'line', source: 'transit', paint: { 'line-color': css('--c-transit'), 'line-width': 2.5, 'line-dasharray': [2, 1.5] } });
  map.addLayer({ id: 'rth-line', type: 'line', source: 'rth', paint: { 'line-color': css('--c-rth'), 'line-width': 2, 'line-dasharray': [1, 1.5] } });
  map.addLayer({ id: 'route-hit', type: 'line', source: 'route', paint: { 'line-color': '#000', 'line-opacity': 0, 'line-width': 14 } });
  map.addLayer({ id: 'wps', type: 'circle', source: 'wps', minzoom: 12,
    paint: { 'circle-radius': ['case', ['get', 'flag'], 6, 3], 'circle-color': roleColor(),
      'circle-stroke-color': ['case', ['get', 'flag'], css('--c-err'), '#0b0f14'], 'circle-stroke-width': ['case', ['get', 'flag'], 2.5, 1] } });
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
  apply3d(false);
  if (state.aoi) fitAoi();
  schedule();
});
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
window.addEventListener('keydown', e => { if (e.key === 'Escape') setPicking(false); });

// ── 2D / 3D ──────────────────────────────────────────────────────
function apply3d(animate = true) {
  $('view3d').classList.toggle('active', state.view3d);
  $('swaths').classList.toggle('active', state.swaths);
  if (!mapReady) return;
  map.setTerrain(state.view3d ? { source: 'terrain3d', exaggeration: 1 } : null);
  for (const id of ROUTE_2D) map.setLayoutProperty(id, 'visibility', state.view3d ? 'none' : 'visible');
  map.setLayoutProperty('swath-fill', 'visibility', state.swaths ? 'visible' : 'none');
  if (animate) map.easeTo({ pitch: state.view3d ? 60 : 0, bearing: state.view3d ? map.getBearing() || -20 : 0, duration: 800 });
  renderDeck(state.result);
}
$('view3d').onclick = () => { state.view3d = !state.view3d; persist(); apply3d(); };
$('swaths').onclick = () => { state.swaths = !state.swaths; persist(); apply3d(false); };

// ── Compute ──────────────────────────────────────────────────────
let timer = 0;
function schedule() { clearTimeout(timer); timer = window.setTimeout(run, 60); }

function compute(): Result | null {
  if (!state.aoi) return null;
  const plan = planLines(state.aoi.poly, state.params);
  if (!plan.lines.length) return null;
  const fromLine = state.resume.on ? Math.min(plan.lines.length, Math.max(1, state.resume.fromLine)) - 1 : 0;
  const route = buildRoute(plan, state.resume.on ? { fromLine, speedMs: state.resume.speed } : {});
  const r: Result = { plan, route, flight: null, st: null, issues: [], demError: null, transit: null, cov: null };
  if (!state.dem) return r;
  try {
    r.flight = applyHeights(plan, route, state.dem.elev);
  } catch (e) {
    r.demError = (e as Error).message + '. The DEM does not cover the whole route (block + run-ins + figure-8).';
    return r;
  }
  r.st = stats(plan, r.flight);
  r.issues = validate(plan, r.flight);
  r.cov = coverage(plan, r.flight);
  if (state.home) {
    try {
      const t = (r.transit = planTransit(plan, r.flight, state.dem.elev, state.home, state.takeoff));
      const clr = T('minClearanceM');
      if (t.minClearanceM < clr) r.issues.push({
        severity: 'error', code: 'TRANSIT_CLEARANCE',
        message: `Take-off transit clears terrain by only ${t.minClearanceM.toFixed(0)} m (need ${clr} m). ${T('flyToMode') === 'pointToPoint' ? 'The sloping point-to-point leg cuts into terrain: switch to "Safely" or raise the take-off security height.' : 'Raise the take-off security height or move home.'}`,
      });
      if (t.rthWorstClearanceM < clr) r.issues.push({
        severity: 'error', code: 'RTH_CLEARANCE',
        message: `RTH at ${t.rthHeightM} m above home would clear terrain by only ${t.rthWorstClearanceM.toFixed(0)} m on the straight line home from WP ${t.rthWorstWp + 1}. Recommended RTH height: ≥ ${t.rthRecommendedM} m.`, wps: [t.rthWorstWp],
      });
      if (t.rthHeightM > 1500) r.issues.push({ severity: 'warn', code: 'RTH_HIGH', message: `RTH height ${t.rthHeightM} m is above DJI's usual 1,500 m maximum. Move home closer to the block's high ground.` });
    } catch (e) {
      r.issues.push({ severity: 'error', code: 'HOME', message: (e as Error).message + '. Load terrain that covers the home point.' });
    }
  } else {
    r.issues.push({ severity: 'info', code: 'NO_HOME', message: 'Home not set: take-off transit and RTH are not checked. Use "Set home on map".' });
  }
  return r;
}

function run() {
  const r = (state.result = compute());
  $<HTMLButtonElement>('exportJson').disabled = !r?.flight || r.issues.some(i => i.severity === 'error');
  $<HTMLButtonElement>('optCourse').disabled = !state.aoi;
  $<HTMLButtonElement>('fetchDem').disabled = !state.aoi;
  $('blockName').textContent = state.aoi ? state.aoi.name : 'No block loaded';
  $('homeInfo').textContent = !state.home ? 'Not set'
    : `${state.home[1].toFixed(5)}, ${state.home[0].toFixed(5)}${r?.transit ? ` · ground ${r.transit.homeElev.toFixed(0)} m` : ''}`;
  renderMap(r);
  renderDeck(r);
  renderStats(r);
  renderIssues(r);
  renderLineSel(r);
  renderProf();
}

// ── Render: 2D map ───────────────────────────────────────────────
const lonlat = (plan: Plan, w: { xy: [number, number] }) => plan.proj.inv(w.xy[0], w.xy[1]);
interface Seg { role: string; line: number | null; coords: number[][] }
function segments(r: Result, withZ: boolean): Seg[] {
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

function renderMap(r: Result | null) {
  if (!mapReady) return;
  const aoi = state.aoi;
  src('aoi')!.setData(aoi ? { type: 'Feature', properties: {}, geometry: { type: 'Polygon', coordinates: [[...aoi.poly, aoi.poly[0]]] } } : empty);
  if (!r) { for (const id of ['route', 'wps', 'rec', 'swath', 'transit', 'rth']) src(id)!.setData(empty); return; }
  const flagged = new Set(r.issues.filter(i => i.severity !== 'info').flatMap(i => i.wps ?? []));
  src('route')!.setData({ type: 'FeatureCollection', features: segments(r, false).map(c => ({
    type: 'Feature', properties: { role: c.role, line: c.line ?? '', sel: c.line === state.selLine }, geometry: { type: 'LineString', coordinates: c.coords } })) });
  src('wps')!.setData({ type: 'FeatureCollection', features: r.route.wps.map((w, i) => ({
    type: 'Feature', properties: { role: w.role, flag: flagged.has(i) }, geometry: { type: 'Point', coordinates: lonlat(r.plan, w) } })) });
  src('rec')!.setData({ type: 'FeatureCollection', features: r.route.wps.flatMap((w, i) => w.actions.map(a => ({
    type: 'Feature' as const, properties: { start: a === 'START_RECORD', label: `WP ${i + 1}: ${a === 'START_RECORD' ? 'start' : 'stop'} point-cloud recording` },
    geometry: { type: 'Point' as const, coordinates: lonlat(r.plan, w) } }))) });
  src('swath')!.setData({ type: 'FeatureCollection', features: (r.cov?.swaths ?? []).map(s => ({
    type: 'Feature', properties: { line: s.line },
    geometry: { type: 'Polygon', coordinates: [[...s.left, ...[...s.right].reverse(), s.left[0]]] } })) });
  src('transit')!.setData(lineFc(r.transit?.path));
  src('rth')!.setData(lineFc(r.transit?.rthFromLast));
}

// ── Render: 3D (deck.gl, true heights) ───────────────────────────
function renderDeck(r: Result | null) {
  if (!state.view3d || !r?.flight) { deck.setProps({ layers: [] }); return; }
  const col: Record<string, [number, number, number, number]> = {
    line: rgba('--c-line'), fig8: rgba('--c-fig8'), approach: rgba('--c-appr'), runin: rgba('--c-run'), runout: rgba('--c-run'),
  };
  const segs = segments(r, true);
  const drops = r.flight.wps.filter(w => w.role === 'line').map(w => ({ from: [w.lon, w.lat, w.h], to: [w.lon, w.lat, w.terrainUnderWp] }));
  const t = r.transit;
  deck.setProps({
    layers: [
      new LineLayer({ id: 'drops', data: drops, getSourcePosition: d => d.from as [number, number, number], getTargetPosition: d => d.to as [number, number, number],
        getColor: [220, 230, 240, 70], getWidth: 1, widthUnits: 'pixels' }),
      new PathLayer<Seg>({ id: 'route3d', data: segs, getPath: d => d.coords as [number, number, number][],
        getColor: d => (d.line === state.selLine ? [255, 255, 255, 255] : col[d.role] ?? col.runin),
        getWidth: d => (d.line === state.selLine ? 5 : 3), widthUnits: 'pixels', jointRounded: true, capRounded: true,
        updateTriggers: { getColor: state.selLine, getWidth: state.selLine },
        pickable: true, onClick: ({ object }) => { if (object?.line != null) selectLine(object.line); } }),
      ...(t ? [
        new PathLayer<Pt3[]>({ id: 'transit3d', data: [t.path], getPath: d => d.map(p => [p.lon, p.lat, p.h] as [number, number, number]),
          getColor: rgba('--c-transit'), getWidth: 3, widthUnits: 'pixels' }),
        new PathLayer<Pt3[]>({ id: 'rth3d', data: [t.rthFromLast], getPath: d => d.map(p => [p.lon, p.lat, p.h] as [number, number, number]),
          getColor: rgba('--c-rth'), getWidth: 2, widthUnits: 'pixels' }),
      ] : []),
    ],
  });
}

// ── Render: stats, checks, profile ───────────────────────────────
const fmt = (v: number, d = 0) => v.toLocaleString('en-ZA', { minimumFractionDigits: d, maximumFractionDigits: d });
function renderStats(r: Result | null) {
  const el = $('stats');
  if (!r) { el.innerHTML = '<div class="empty">Load a block to plan.</div>'; return; }
  const { plan, route, st, cov, transit: t } = r;
  const rows: [string, string, string?][] = [
    ['Area', `${fmt(plan.areaHa)} ha`], ['Lines', `${plan.lines.length}`],
    ['Swath / spacing', `${fmt(plan.swath)} / ${fmt(plan.spacing)} m`, 'At nominal AGL. Spacing = swath × (1 − sidelap).'],
    ['Waypoints', `${fmt(route.wps.length)}`], ['Fig-8 radius', `${fmt(route.fig8RadiusM)} m`],
  ];
  if (st) rows.push(
    ['Route', `${fmt(st.routeKm, 1)} km`], ['On-line data', `${fmt(st.dataKm, 1)} km`],
    ['Height (orthometric)', `${fmt(st.hMin)}–${fmt(st.hMax)} m`],
    ['AGL on lines', `${fmt(st.aglOnLineMin)}–${fmt(st.aglOnLineMax)} m`, 'Height above the terrain directly under each line waypoint.'],
  );
  if (cov) rows.push(
    ['Sidelap planned', `${fmt(cov.plannedSidelapPct)} %`, 'Strip overlap at nominal AGL.'],
    ['Sidelap achieved', `${fmt(cov.achievedMinPct)}–${fmt(cov.achievedMaxPct)} % (mean ${fmt(cov.achievedMeanPct)})`, 'Using each line waypoint\'s real AGL: overlap = 1 − spacing / (2·AGL·tan(FOV/2)). Assumes level ground across-track.'],
    ['Swath on lines', `${fmt(cov.swathMinM)}–${fmt(cov.swathMaxM)} m`],
  );
  if (t) rows.push(
    ['Take-off transit', `${fmt(t.distanceM / 1000, 1)} km · ${fmt(t.timeS / 60, 1)} min`],
    ['Transit clearance', `${fmt(t.minClearanceM)} m min`, 'Lowest height above terrain (±corridor) on the path from home to the first waypoint, excluding the vertical climb at home.'],
    ['RTH height', `${fmt(t.rthHeightM)} m${state.takeoff.rthHeightM == null ? ' (auto)' : ''} · rec. ≥ ${fmt(t.rthRecommendedM)} m`, TIPS.rthHeightM],
  );
  if (st) {
    const extra = t ? t.timeS / 60 + t.rthDistanceM / T('transitSpeedMs') / 60 : 0;
    rows.push(['Flight time*', `${fmt(st.flightMin + extra)} min`]);
  }
  el.innerHTML = rows.map(([k, v, h]) => `<div${h ? ` data-tip="${h.replace(/"/g, '&quot;')}"` : ''}><span>${k}</span><b>${v}</b></div>`).join('') +
    (st ? `<p class="foot">*route ÷ speed${t ? ' + transit + RTH from last WP' : ''}; excludes acceleration and figure-8 slow-down.</p>` : '');
}

function renderIssues(r: Result | null) {
  const el = $('issues');
  if (!r) { el.innerHTML = ''; return; }
  const list: Issue[] = r.demError ? [{ severity: 'error', code: 'DEM', message: r.demError }]
    : !state.dem ? [{ severity: 'warn', code: 'NO_DEM', message: 'No terrain loaded: showing plan geometry only. Heights, AGL, overlap and checks need a DEM.' }]
    : r.issues;
  const order = { error: 0, warn: 1, info: 2 };
  el.innerHTML = '<h2>Checks</h2>' + (list.length ? [...list].sort((a, b) => order[a.severity] - order[b.severity])
    .map(i => `<div class="issue ${i.severity}"><b>${i.severity}</b><span>${i.message}</span></div>`).join('')
    : '<div class="issue ok"><b>ok</b><span>No problems found.</span></div>');
}

function renderLineSel(r: Result | null) {
  const sel = $<HTMLSelectElement>('lineSel');
  const n = r?.plan.lines.length ?? 0;
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
function selectLine(i: number | 'transit') { state.selLine = i; renderMap(state.result); renderDeck(state.result); renderLineSel(state.result); renderProf(); }

function renderProf() {
  const svg = $('profile') as unknown as SVGSVGElement;
  const r = state.result;
  if (!r?.flight || !state.dem) { svg.replaceChildren(); return; }
  let pts: ProfilePt[], floor: number;
  if (state.selLine === 'transit' && r.transit) {
    pts = [...r.transit.path.map(p => ({ xy: r.plan.proj.fwd(p.lon, p.lat), h: p.h, role: 'transit' })),
      ...r.flight.wps.filter(w => w.role === 'approach' || w.role === 'fig8')];
    floor = T('minClearanceM');
    $('floorLabel').textContent = 'Terrain + min clearance';
  } else {
    pts = r.flight.wps.filter(w => w.line === state.selLine);
    floor = r.plan.o.aglM;
    $('floorLabel').textContent = 'Terrain + AGL';
  }
  renderProfile(svg, r.plan, pts, state.dem.elev, floor);
}
new ResizeObserver(() => renderProf()).observe($('profile'));

// ── Block input ──────────────────────────────────────────────────
function setAoi(aoi: Aoi, demo = false) {
  state.aoi = aoi; state.demo = demo; state.selLine = 0;
  state.dem = demo ? { name: 'Synthetic ridge (demo)', elev: demoElev } : null;
  if (demo) state.params = { ...state.params, courseDeg: 20 };
  persist(); syncFields(); fitAoi(); updateDemInfo(); schedule();
  $('blockInfo').textContent = `${aoi.poly.length} vertices${aoi.polygonsFound > 1 ? `, largest of ${aoi.polygonsFound} polygons` : ''}.`;
}
async function onBlockFile(f: File) {
  try { setAoi(await aoiFromFile(f)); } catch (e) { $('blockInfo').textContent = (e as Error).message; }
}
$('loadDemo').onclick = () => setAoi({ name: 'Demo: synthetic Nimba block', poly: demoPoly, polygonsFound: 1 }, true);
$<HTMLInputElement>('fileBlock').onchange = e => { const f = (e.target as HTMLInputElement).files?.[0]; if (f) onBlockFile(f); };

// ── Terrain input ────────────────────────────────────────────────
const otKey = $<HTMLInputElement>('otKey');
try { otKey.value = localStorage.getItem('3dm.otKey') ?? ''; } catch { /* ignore */ }
if (!otKey.value && import.meta.env.DEV && import.meta.env.VITE_OPENTOPO_KEY) otKey.value = import.meta.env.VITE_OPENTOPO_KEY;
otKey.onchange = () => { try { localStorage.setItem('3dm.otKey', otKey.value.trim()); } catch { /* ignore */ } };

function updateDemInfo(msg?: string) {
  $('demInfo').textContent = msg ?? (state.dem
    ? `Loaded: ${state.dem.name}${state.demo ? '. Demo terrain is synthetic, so the 3D ground (real) will not match it. Fetch GLO-30 for the real ridge.' : ''}`
    : 'No DEM. Heights and checks need terrain.');
}
async function useDemBuffer(buf: ArrayBuffer, name: string) {
  const raster = await readGeoTiff(buf);
  state.dem = { name: `${name} (${raster.width}×${raster.height})`, elev: rasterElev(raster), bbox: rasterBbox(raster) };
  state.demo = false; persist(); updateDemInfo(); schedule();
}
$('fetchDem').onclick = async () => {
  if (!state.aoi) return;
  const key = otKey.value.trim();
  if (!key) { updateDemInfo('Enter your OpenTopography API key first (opentopography.org → My Account → API).'); return; }
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
// With terrain: minimise route length × (mean line AGL / nominal AGL). Swath grows with AGL, so point
// density falls ~1/AGL; lines crossing steep ridges get pushed high by the gradient limit and lose
// density, lines along the ridge keep AGL even. Coarse 5° sweep, then ±4° at 1°.
function courseCost(c: number): number {
  const p = planLines(state.aoi!.poly, { ...state.params, courseDeg: c });
  if (!state.dem) return p.lines.length * 1e9 + p.lines.reduce((s, L) => s + (L.umax - L.umin), 0);
  try {
    const f = applyHeights(p, buildRoute(p), state.dem.elev);
    const st = stats(p, f);
    const lineAgl = f.wps.filter(w => w.role === 'line').map(w => w.h - w.terrainUnderWp);
    const meanAgl = lineAgl.reduce((s, v) => s + v, 0) / lineAgl.length;
    return st.routeKm * (meanAgl / p.o.aglM);
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
  state.params.courseDeg = fine.c; persist(); syncFields(); schedule();
};

// ── Export ───────────────────────────────────────────────────────
$('exportJson').onclick = () => {
  const r = state.result;
  if (!r?.flight || !state.aoi) return;
  const mission = {
    format: '3dm-mission', version: 1, created: new Date().toISOString(),
    block: state.aoi, params: { ...DEFAULTS, ...state.params }, dem: state.dem?.name,
    home: state.home ? { lon: state.home[0], lat: state.home[1], groundH: r.transit?.homeElev ?? null } : null,
    takeoff: { ...TAKEOFF_DEFAULTS, ...state.takeoff, rthHeightM: r.transit?.rthHeightM ?? state.takeoff.rthHeightM ?? null },
    resume: state.resume.on ? state.resume : null, stats: r.st, coverage: r.cov && { ...r.cov, swaths: undefined }, issues: r.issues,
    heightNote: 'h = orthometric (DEM datum). hWrite = h + geoidN; geoid handling pending the RC sample export.',
    waypoints: r.flight.wps.map((w, i) => ({
      i, role: w.role, line: w.line ?? null, lat: +w.lat.toFixed(8), lon: +w.lon.toFixed(8),
      h: +w.h.toFixed(2), hWrite: +w.hWrite.toFixed(2), speed: w.speed, dampingM: +w.dampingM.toFixed(2),
      turnMode: w.turnMode, actions: w.actions,
    })),
  };
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([JSON.stringify(mission, null, 1)], { type: 'application/json' }));
  a.download = `${state.aoi.name.replace(/[^\w.-]+/g, '_')}${state.resume.on ? `_resume_L${state.resume.fromLine}` : ''}.mission.json`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
};

updateDemInfo();
if (state.aoi) $('blockInfo').textContent = `${state.aoi.poly.length} vertices.`;
schedule();
