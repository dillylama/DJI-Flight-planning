import 'maplibre-gl/dist/maplibre-gl.css';
import './style.css';
import maplibregl, { type GeoJSONSource, type LngLatBoundsLike } from 'maplibre-gl';
import {
  DEFAULTS, planLines, buildRoute, applyHeights, stats, validate, readGeoTiff, rasterElev, rasterBbox,
  bufferedBbox, openTopoUrl,
  type PlanOptions, type Plan, type Route, type FlightWp, type RouteStats, type Issue, type ElevFn, type Bbox,
} from '@3dm/core';
import { aoiFromFile, type Aoi } from './aoi.ts';
import { demoPoly, demoElev } from './demo.ts';
import { renderProfile } from './profile.ts';

// ── State ─────────────────────────────────────────────────────────
interface Dem { name: string; elev: ElevFn; bbox?: Bbox }
interface Result { plan: Plan; route: Route; flight: Route<FlightWp> | null; st: RouteStats | null; issues: Issue[]; demError: string | null }

const STORE = '3dm.planner.v1';
type Saved = { aoi: Aoi | null; params: Partial<PlanOptions>; demo: boolean };
const load = (): Saved => {
  try { return { aoi: null, params: {}, demo: false, ...JSON.parse(localStorage.getItem(STORE) || '{}') }; } catch { return { aoi: null, params: {}, demo: false }; }
};
const saved = load();
const state = {
  aoi: saved.aoi as Aoi | null,
  dem: (saved.demo ? { name: 'Synthetic ridge (demo)', elev: demoElev } : null) as Dem | null,
  demo: saved.demo,
  params: { ...saved.params } as Partial<PlanOptions>,
  resume: { on: false, fromLine: 1, speed: 14 },
  selLine: 0,
  result: null as Result | null,
};
const persist = () => {
  try { localStorage.setItem(STORE, JSON.stringify({ aoi: state.aoi, params: state.params, demo: state.demo })); } catch { /* private mode */ }
};
const P = <K extends keyof PlanOptions>(k: K): PlanOptions[K] => (state.params[k] ?? DEFAULTS[k]) as PlanOptions[K];

// ── Layout ────────────────────────────────────────────────────────
const FIELDS: { key: keyof PlanOptions; label: string; unit: string; step: number; min: number; max: number; hint?: string }[] = [
  { key: 'aglM', label: 'AGL', unit: 'm', step: 10, min: 30, max: 1500 },
  { key: 'speedMs', label: 'Line speed', unit: 'm/s', step: 0.5, min: 1, max: 25 },
  { key: 'fovDeg', label: 'L3 across-track FOV', unit: '°', step: 1, min: 10, max: 120, hint: 'Effective FOV for the chosen scan mode' },
  { key: 'sidelapPct', label: 'Sidelap', unit: '%', step: 5, min: 0, max: 90 },
  { key: 'courseDeg', label: 'Line course', unit: '°', step: 1, min: 0, max: 359 },
  { key: 'runInM', label: 'Run-in', unit: 'm', step: 10, min: 0, max: 1000 },
  { key: 'runOutM', label: 'Run-out', unit: 'm', step: 10, min: 0, max: 1000 },
  { key: 'wpSpacingM', label: 'Max WP spacing', unit: 'm', step: 10, min: 20, max: 1000 },
  { key: 'corridorM', label: 'Terrain corridor ±', unit: 'm', step: 5, min: 0, max: 500 },
  { key: 'maxGradient', label: 'Max climb/descent', unit: 'rise/run', step: 0.01, min: 0.01, max: 1 },
  { key: 'fig8BankDeg', label: 'Figure-8 bank', unit: '°', step: 1, min: 5, max: 35 },
];

const app = document.getElementById('app')!;
app.innerHTML = `
<header>
  <div class="brand"><span class="mark">3DM</span> Planner <span class="sub">M400 · Zenmuse L3</span></div>
  <div class="blockname" id="blockName">No block loaded</div>
  <div class="head-actions">
    <button id="exportJson" class="ghost" disabled>Export mission.json</button>
    <button id="exportKmz" class="ghost" disabled title="Needs the RC Waypoint Route sample export (samples/README.md)">Export KMZ</button>
  </div>
</header>
<aside>
  <section>
    <h2>Block</h2>
    <label class="drop" id="dropBlock"><input type="file" id="fileBlock" accept=".kmz,.kml" hidden />
      <strong>Drop KMZ / KML</strong><span>or click to choose. Largest polygon is used.</span></label>
    <button id="loadDemo" class="link">Load demo block (synthetic Nimba ridge)</button>
    <div class="note" id="blockInfo"></div>
  </section>
  <section>
    <h2>Terrain</h2>
    <div class="row"><input id="otKey" type="password" placeholder="OpenTopography API key" autocomplete="off" />
      <button id="fetchDem" disabled>Fetch GLO-30</button></div>
    <label class="drop small" id="dropDem"><input type="file" id="fileDem" accept=".tif,.tiff" hidden />
      <span>…or drop a GeoTIFF DEM (EPSG:4326)</span></label>
    <div class="note" id="demInfo">No DEM. Heights and checks need terrain.</div>
  </section>
  <section>
    <h2>Survey</h2>
    <div class="fields" id="fields"></div>
    <div class="row"><button id="optCourse" class="ghost" disabled>Optimise course</button><button id="resetParams" class="link">Reset defaults</button></div>
  </section>
  <section>
    <h2>Resume</h2>
    <label class="check"><input type="checkbox" id="resOn" /> Build a resume route</label>
    <div class="fields two">
      <label>Data stopped on line<input type="number" id="resLine" min="1" value="1" /></label>
      <label>New speed <em>m/s</em><input type="number" id="resSpeed" step="0.5" min="1" max="25" value="14" /></label>
    </div>
    <div class="note">Restarts one line earlier with a fresh figure-8, recording on from the approach.</div>
  </section>
</aside>
<main>
  <div id="map"></div>
  <div class="legend">
    <span><i style="background:var(--c-line)"></i>Data line</span><span><i style="background:var(--c-run)"></i>Run-in/out, turn</span>
    <span><i style="background:var(--c-fig8)"></i>Figure-8</span><span><i style="background:var(--c-appr)"></i>Approach</span>
  </div>
  <section class="bottom">
    <div class="stats" id="stats"></div>
    <div class="profile-wrap">
      <div class="profile-head"><h2>Profile</h2><select id="lineSel"></select>
        <span class="key"><i class="k-flight"></i>Flight height <i class="k-floor"></i>Terrain + AGL <i class="k-terr"></i>Terrain</span></div>
      <svg id="profile"></svg>
    </div>
    <div class="issues" id="issues"></div>
  </section>
</main>`;

const $ = <T extends HTMLElement = HTMLElement>(id: string) => document.getElementById(id) as T;

const fieldsEl = $('fields');
for (const f of FIELDS) {
  const l = document.createElement('label');
  l.innerHTML = `${f.label} <em>${f.unit}</em><input type="number" data-key="${f.key}" step="${f.step}" min="${f.min}" max="${f.max}" />`;
  if (f.hint) l.title = f.hint;
  fieldsEl.append(l);
}
const syncFields = () => fieldsEl.querySelectorAll<HTMLInputElement>('input').forEach(i => { i.value = String(P(i.dataset.key as keyof PlanOptions)); });
syncFields();
fieldsEl.addEventListener('input', e => {
  const i = e.target as HTMLInputElement;
  const v = Number(i.value);
  if (i.value === '' || !Number.isFinite(v)) return;
  state.params[i.dataset.key as keyof PlanOptions] = v as never;
  persist(); schedule();
});
$('resetParams').onclick = () => { state.params = {}; persist(); syncFields(); schedule(); };

// ── Map ───────────────────────────────────────────────────────────
const css = (v: string) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
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
    },
    layers: [{ id: 'sat', type: 'raster', source: 'sat', paint: { 'raster-saturation': -0.35, 'raster-brightness-max': 0.8 } }],
  },
  center: [22.03, -33.21], zoom: 5, attributionControl: { compact: true },
});
map.addControl(new maplibregl.NavigationControl({ visualizePitch: true }), 'top-right');
map.addControl(new maplibregl.ScaleControl({ unit: 'metric' }), 'bottom-right');

const empty: GeoJSON.FeatureCollection = { type: 'FeatureCollection', features: [] };
const roleColor = () => ['match', ['get', 'role'],
  'line', css('--c-line'), 'fig8', css('--c-fig8'), 'approach', css('--c-appr'), css('--c-run')] as unknown as string;

let mapReady = false;
map.on('load', () => {
  for (const id of ['aoi', 'route', 'wps', 'rec']) map.addSource(id, { type: 'geojson', data: empty });
  map.addLayer({ id: 'aoi-fill', type: 'fill', source: 'aoi', paint: { 'fill-color': css('--c-aoi'), 'fill-opacity': 0.12 } });
  map.addLayer({ id: 'aoi-line', type: 'line', source: 'aoi', paint: { 'line-color': css('--c-aoi'), 'line-width': 2 } });
  map.addLayer({ id: 'route-line', type: 'line', source: 'route', layout: { 'line-join': 'round', 'line-cap': 'round' },
    paint: { 'line-color': roleColor(), 'line-width': ['case', ['get', 'sel'], 4.5, 2], 'line-opacity': ['case', ['get', 'sel'], 1, 0.85] } });
  map.addLayer({ id: 'route-hit', type: 'line', source: 'route', paint: { 'line-color': '#000', 'line-opacity': 0, 'line-width': 14 } });
  map.addLayer({ id: 'wps', type: 'circle', source: 'wps', minzoom: 12,
    paint: { 'circle-radius': ['case', ['get', 'flag'], 6, 3], 'circle-color': roleColor(),
      'circle-stroke-color': ['case', ['get', 'flag'], css('--c-err'), '#0b0f14'], 'circle-stroke-width': ['case', ['get', 'flag'], 2.5, 1] } });
  map.addLayer({ id: 'rec', type: 'circle', source: 'rec',
    paint: { 'circle-radius': 7, 'circle-color': ['case', ['get', 'start'], css('--c-line'), css('--c-appr')], 'circle-stroke-color': '#fff', 'circle-stroke-width': 2 } });
  map.on('click', 'route-hit', e => {
    const line = e.features?.[0]?.properties?.line;
    if (line != null && line !== '') selectLine(Number(line));
  });
  map.on('mouseenter', 'route-hit', () => { map.getCanvas().style.cursor = 'pointer'; });
  map.on('mouseleave', 'route-hit', () => { map.getCanvas().style.cursor = ''; });
  const recPopup = new maplibregl.Popup({ closeButton: false, offset: 10 });
  map.on('mouseenter', 'rec', e => { const f = e.features?.[0]; if (f) recPopup.setLngLat((f.geometry as GeoJSON.Point).coordinates as [number, number]).setText(f.properties.label).addTo(map); });
  map.on('mouseleave', 'rec', () => recPopup.remove());
  mapReady = true;
  if (state.aoi) fitAoi();
  schedule();
});

const src = (id: string) => map.getSource(id) as GeoJSONSource | undefined;

function fitAoi() {
  if (!state.aoi || !mapReady) return;
  const lons = state.aoi.poly.map(p => p[0]), lats = state.aoi.poly.map(p => p[1]);
  map.fitBounds([[Math.min(...lons), Math.min(...lats)], [Math.max(...lons), Math.max(...lats)]] as LngLatBoundsLike, { padding: 60, duration: 600 });
}

// ── Compute + render ─────────────────────────────────────────────
let timer = 0;
function schedule() { clearTimeout(timer); timer = window.setTimeout(run, 60); }

function compute(): Result | null {
  if (!state.aoi) return null;
  const plan = planLines(state.aoi.poly, state.params);
  if (!plan.lines.length) return null;
  const fromLine = state.resume.on ? Math.min(plan.lines.length, Math.max(1, state.resume.fromLine)) - 1 : 0;
  const route = buildRoute(plan, state.resume.on ? { fromLine, speedMs: state.resume.speed } : {});
  let flight: Route<FlightWp> | null = null, st: RouteStats | null = null, issues: Issue[] = [], demError: string | null = null;
  if (state.dem) {
    try {
      flight = applyHeights(plan, route, state.dem.elev);
      st = stats(plan, flight);
      issues = validate(plan, flight);
    } catch (e) {
      demError = (e as Error).message + '. The DEM does not cover the whole route (block + run-ins + figure-8).';
    }
  }
  return { plan, route, flight, st, issues, demError };
}

function run() {
  const r = (state.result = compute());
  $<HTMLButtonElement>('exportJson').disabled = !r?.flight || r.issues.some(i => i.severity === 'error');
  $<HTMLButtonElement>('optCourse').disabled = !state.aoi;
  $<HTMLButtonElement>('fetchDem').disabled = !state.aoi;
  $('blockName').textContent = state.aoi ? state.aoi.name : 'No block loaded';
  renderMap(r);
  renderStats(r);
  renderIssues(r);
  renderLineSel(r);
  renderProf();
}

function renderMap(r: Result | null) {
  if (!mapReady) return;
  const aoi = state.aoi;
  src('aoi')!.setData(aoi ? { type: 'Feature', properties: {}, geometry: { type: 'Polygon', coordinates: [[...aoi.poly, aoi.poly[0]]] } } : empty);
  if (!r) { for (const id of ['route', 'wps', 'rec']) src(id)!.setData(empty); return; }
  const { plan } = r;
  const wps = r.route.wps.map(w => { const [lon, lat] = plan.proj.inv(w.xy[0], w.xy[1]); return { ...w, lon, lat }; });
  const flagged = new Set(r.issues.filter(i => i.severity !== 'info').flatMap(i => i.wps ?? []));
  const selLine = state.selLine;

  // Segment coloured by the role of the waypoint it flies TO; consecutive same-role/same-line merged.
  const feats: GeoJSON.Feature[] = [];
  let cur: { role: string; line: number | null; coords: number[][] } | null = null;
  for (let i = 1; i < wps.length; i++) {
    const w = wps[i], line = w.line ?? null;
    if (!cur || cur.role !== w.role || cur.line !== line) {
      if (cur) feats.push(seg(cur));
      cur = { role: w.role, line, coords: [[wps[i - 1].lon, wps[i - 1].lat]] };
    }
    cur.coords.push([w.lon, w.lat]);
  }
  if (cur) feats.push(seg(cur));
  function seg(c: { role: string; line: number | null; coords: number[][] }): GeoJSON.Feature {
    return { type: 'Feature', properties: { role: c.role, line: c.line ?? '', sel: c.line === selLine }, geometry: { type: 'LineString', coordinates: c.coords } };
  }
  src('route')!.setData({ type: 'FeatureCollection', features: feats });
  src('wps')!.setData({ type: 'FeatureCollection', features: wps.map((w, i) => ({
    type: 'Feature', properties: { role: w.role, flag: flagged.has(i) }, geometry: { type: 'Point', coordinates: [w.lon, w.lat] } })) });
  src('rec')!.setData({ type: 'FeatureCollection', features: wps.flatMap((w, i) => w.actions.map(a => ({
    type: 'Feature' as const, properties: { start: a === 'START_RECORD', label: `WP ${i + 1}: ${a === 'START_RECORD' ? 'start' : 'stop'} point-cloud recording` },
    geometry: { type: 'Point' as const, coordinates: [w.lon, w.lat] } }))) });
}

const fmt = (v: number, d = 0) => v.toLocaleString('en-ZA', { minimumFractionDigits: d, maximumFractionDigits: d });
function renderStats(r: Result | null) {
  const el = $('stats');
  if (!r) { el.innerHTML = '<div class="empty">Load a block to plan.</div>'; return; }
  const { plan, route, st } = r;
  const rows: [string, string][] = [
    ['Area', `${fmt(plan.areaHa)} ha`], ['Lines', `${plan.lines.length}`],
    ['Swath / spacing', `${fmt(plan.swath)} / ${fmt(plan.spacing)} m`], ['Waypoints', `${fmt(route.wps.length)}`],
    ['Fig-8 radius', `${fmt(route.fig8RadiusM)} m`],
  ];
  if (st) rows.push(
    ['Route', `${fmt(st.routeKm, 1)} km`], ['On-line data', `${fmt(st.dataKm, 1)} km`],
    ['Flight time*', `${fmt(st.flightMin)} min`], ['Height (orthometric)', `${fmt(st.hMin)}–${fmt(st.hMax)} m`],
    ['AGL on lines', `${fmt(st.aglOnLineMin)}–${fmt(st.aglOnLineMax)} m`],
  );
  el.innerHTML = rows.map(([k, v]) => `<div><span>${k}</span><b>${v}</b></div>`).join('') +
    (st ? '<p class="foot">*distance ÷ speed; excludes take-off, transit, acceleration and figure-8 slow-down.</p>' : '');
}

function renderIssues(r: Result | null) {
  const el = $('issues');
  if (!r) { el.innerHTML = ''; return; }
  const list: Issue[] = r.demError ? [{ severity: 'error', code: 'DEM', message: r.demError }]
    : !state.dem ? [{ severity: 'warn', code: 'NO_DEM', message: 'No terrain loaded: showing plan geometry only. Heights, AGL and checks need a DEM.' }]
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
  if (state.selLine >= n || state.selLine < start) state.selLine = start;
  sel.innerHTML = Array.from({ length: n }, (_, i) => `<option value="${i}" ${i < start ? 'disabled' : ''}>Line ${i + 1}</option>`).join('');
  sel.value = String(state.selLine);
}
$<HTMLSelectElement>('lineSel').onchange = e => selectLine(Number((e.target as HTMLSelectElement).value));

function selectLine(i: number) { state.selLine = i; renderMap(state.result); renderLineSel(state.result); renderProf(); }

function renderProf() {
  const svg = $('profile') as unknown as SVGSVGElement;
  const r = state.result;
  if (!r?.flight || !state.dem) { svg.replaceChildren(); return; }
  renderProfile(svg, r.plan, r.flight.wps.filter(w => w.line === state.selLine), state.dem.elev);
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
otKey.onchange = () => { try { localStorage.setItem('3dm.otKey', otKey.value.trim()); } catch { /* ignore */ } };

function updateDemInfo(msg?: string) {
  $('demInfo').textContent = msg ?? (state.dem ? `Loaded: ${state.dem.name}` : 'No DEM. Heights and checks need terrain.');
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
  const b = bufferedBbox(state.aoi.poly, 2000);
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

// Drag-and-drop anywhere: route by extension.
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
  } catch { return Infinity; }                     // DEM doesn't cover this orientation's run-ins
}
$('optCourse').onclick = async () => {
  if (!state.aoi) return;
  const btn = $<HTMLButtonElement>('optCourse');
  btn.disabled = true; btn.textContent = 'Optimising…';
  await new Promise(r => setTimeout(r, 20));
  const sweep = (cs: number[]) => cs.map(c => ({ c: ((c % 180) + 180) % 180, cost: courseCost(((c % 180) + 180) % 180) })).reduce((a, b) => (b.cost < a.cost ? b : a));
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
    resume: state.resume.on ? state.resume : null, stats: r.st, issues: r.issues,
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
