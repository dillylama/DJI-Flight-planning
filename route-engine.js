// ════════════════════════════════════════════════════════════════
//  route-engine.js — M400 / L3 LiDAR route GENERATOR (geometry core)
//  Pure JS, no deps. Works in the browser (window.RouteEngine) and Node.
//  Produces an abstract route; the WPML writer (built from the RC export)
//  turns it into template.kml + waylines.wpml.
// ════════════════════════════════════════════════════════════════
(function (root) {
  const R = 6378137, D2R = Math.PI / 180, G = 9.81;

  const DEFAULTS = {
    aglM: 500,            // nominal clearance above terrain (m)
    speedMs: 17,          // line speed
    fovDeg: 70,           // L3 effective across-track FOV used for swath (set from your L3 scan mode)
    sidelapPct: 50,
    courseDeg: 0,         // line heading, degrees clockwise from north
    runInM: 150,
    runOutM: 150,
    wpSpacingM: 150,      // max waypoint spacing along lines
    corridorM: 75,        // lateral buffer when finding max terrain under the path
    demSampleM: 30,       // DEM sampling step (GLO-30 native)
    maxGradient: 0.15,    // max climb/descent between waypoints (rise/run)
    geoidN: 0,            // add to DEM (orthometric) height when writing ellipsoidal heights
    fig8BankDeg: 25,
    fig8RadiusM: null,    // null = computed from speed + bank
    fig8PtsPerLoop: 12,
    dampingFrac: 0.4,     // damping distance as fraction of the shorter adjacent leg
    dampingMaxM: 60,
  };

  // Local tangent plane about the block centroid (fine for blocks up to a few tens of km)
  function makeProj(lat0, lon0) {
    const k = Math.cos(lat0 * D2R);
    return {
      fwd: (lon, lat) => [(lon - lon0) * D2R * R * k, (lat - lat0) * D2R * R],
      inv: (x, y) => [lon0 + x / (R * k) / D2R, lat0 + y / R / D2R],
    };
  }

  const add = (a, b) => [a[0] + b[0], a[1] + b[1]];
  const mul = (a, s) => [a[0] * s, a[1] * s];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1];
  const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);

  function polygonAreaM2(xy) {
    let s = 0;
    for (let i = 0, j = xy.length - 1; i < xy.length; j = i++) s += (xy[j][0] + xy[i][0]) * (xy[j][1] - xy[i][1]);
    return Math.abs(s / 2);
  }

  // Along-track extent of the polygon inside the band v∈[v0,v1] (handles concave blocks:
  // the line spans the full extent and simply keeps recording across any notch).
  function bandExtent(uv, v0, v1) {
    let umin = Infinity, umax = -Infinity;
    for (let i = 0, j = uv.length - 1; i < uv.length; j = i++) {
      const a = uv[j], b = uv[i];
      let t0 = 0, t1 = 1;
      const dv = b[1] - a[1];
      if (Math.abs(dv) < 1e-9) { if (a[1] < v0 || a[1] > v1) continue; }
      else {
        let ta = (v0 - a[1]) / dv, tb = (v1 - a[1]) / dv;
        if (ta > tb) [ta, tb] = [tb, ta];
        t0 = Math.max(0, ta); t1 = Math.min(1, tb);
        if (t0 > t1) continue;
      }
      for (const t of [t0, t1]) {
        const u = a[0] + (b[0] - a[0]) * t;
        umin = Math.min(umin, u); umax = Math.max(umax, u);
      }
    }
    return umin <= umax ? [umin, umax] : null;
  }

  // ── 1. Plan lines ──────────────────────────────────────────────
  function planLines(polyLonLat, opts) {
    const o = { ...DEFAULTS, ...opts };
    const lat0 = polyLonLat.reduce((s, p) => s + p[1], 0) / polyLonLat.length;
    const lon0 = polyLonLat.reduce((s, p) => s + p[0], 0) / polyLonLat.length;
    const proj = makeProj(lat0, lon0);
    const xy = polyLonLat.map(([lo, la]) => proj.fwd(lo, la));

    const th = o.courseDeg * D2R;
    const d = [Math.sin(th), Math.cos(th)];     // along-line (E,N)
    const c = [Math.cos(th), -Math.sin(th)];    // across-line, to the right of d
    const uv = xy.map(p => [dot(p, d), dot(p, c)]);

    const swath = 2 * o.aglM * Math.tan((o.fovDeg / 2) * D2R);
    const spacing = swath * (1 - o.sidelapPct / 100);
    const vs = uv.map(p => p[1]);
    const vmin = Math.min(...vs), vmax = Math.max(...vs);
    const n = Math.ceil((vmax - vmin) / spacing) + 1;
    const vStart = (vmin + vmax) / 2 - ((n - 1) * spacing) / 2;

    const lines = [];
    for (let i = 0; i < n; i++) {
      const v = vStart + i * spacing;
      const ext = bandExtent(uv, v - spacing / 2, v + spacing / 2);
      if (!ext) continue;
      lines.push({ v, umin: ext[0], umax: ext[1] });
    }
    // serpentine
    lines.forEach((L, k) => { L.index = k; L.dir = k % 2 === 0 ? 1 : -1; });

    return { o, proj, d, c, lines, swath, spacing, areaHa: polygonAreaM2(xy) / 1e4, polyXY: xy };
  }

  // ── 2. Build route (full job, or a resume from any line) ─────────
  //   fromLine: index of the line where data stopped. The route restarts one line earlier.
  function buildRoute(plan, { fromLine = 0, speedMs } = {}) {
    const { o, d, c, lines } = plan;
    const v = speedMs || o.speedMs;
    const startIdx = Math.max(0, fromLine - (fromLine > 0 ? 1 : 0));
    const toXY = (u, vv) => add(mul(d, u), mul(c, vv));
    const wps = [];
    const push = (xy, role, extra = {}) => wps.push({ xy, role, speed: v, actions: [], ...extra });

    const first = lines[startIdx];
    const hdg = mul(d, first.dir);                           // unit heading of first line
    const u0 = first.dir > 0 ? first.umin - o.runInM : first.umax + o.runInM;
    const runInStart = toXY(u0, first.v);

    // Figure-8: crossover X one radius behind the run-in start, lobes left and right of the line axis.
    const r = o.fig8RadiusM || (v * v) / (G * Math.tan(o.fig8BankDeg * D2R));
    const X = add(runInStart, mul(hdg, -r));
    const right = [hdg[1], -hdg[0]];
    const approach = add(X, mul(hdg, -2.5 * r));
    push(approach, 'approach', { actions: ['START_RECORD'] });   // recording running before the 8
    push(X, 'fig8');
    const loop = (centre, startVec) => {
      for (let k = 1; k < o.fig8PtsPerLoop; k++) {
        const t = (2 * Math.PI * k) / o.fig8PtsPerLoop;
        push(add(centre, add(mul(startVec, r * Math.cos(t)), mul(hdg, r * Math.sin(t)))), 'fig8');
      }
      push(X, 'fig8');
    };
    loop(add(X, mul(right, -r)), right);          // left lobe
    loop(add(X, mul(right, r)), mul(right, -1));  // right lobe

    // Lines
    for (let i = startIdx; i < lines.length; i++) {
      const L = lines[i];
      const a = L.dir > 0 ? L.umin : L.umax, b = L.dir > 0 ? L.umax : L.umin;
      const s = L.dir;
      push(toXY(a - s * o.runInM, L.v), 'runin', { line: i });
      const nSeg = Math.max(1, Math.ceil(Math.abs(b - a) / o.wpSpacingM));
      for (let k = 0; k <= nSeg; k++) push(toXY(a + ((b - a) * k) / nSeg, L.v), 'line', { line: i });
      push(toXY(b + s * o.runOutM, L.v), 'runout', { line: i });
    }
    wps[wps.length - 1].actions.push('STOP_RECORD');

    // Damping distance per waypoint: fraction of the shorter adjacent leg (DJI requires < leg length)
    wps.forEach((w, i) => {
      const legs = [];
      if (i > 0) legs.push(dist(w.xy, wps[i - 1].xy));
      if (i < wps.length - 1) legs.push(dist(w.xy, wps[i + 1].xy));
      w.dampingM = Math.min(o.dampingMaxM, o.dampingFrac * Math.min(...legs));
      w.turnMode = 'toPointAndPassWithContinuityCurvature';
    });

    return { wps, startIdx, fig8RadiusM: r, speedMs: v };
  }

  // ── 3. Heights (absolute) ────────────────────────────────────────
  //   elev(lon,lat) → orthometric terrain height from the DEM (null/NaN if nodata)
  //   Each waypoint gets ≥ max terrain (+ corridor) along BOTH adjacent legs + AGL, so every
  //   straight leg clears terrain by at least aglM; then climb/descent is gradient-limited
  //   by raising (never lowering) waypoints.
  function applyHeights(plan, route, elev) {
    const { o, proj } = plan;
    const { wps } = route;
    const terr = (xy) => { const [lo, la] = proj.inv(xy[0], xy[1]); const h = elev(lo, la); return Number.isFinite(h) ? h : -Infinity; };
    const segMax = (a, b) => {
      const L = dist(a, b), n = Math.max(1, Math.ceil(L / o.demSampleM));
      const dir = L > 0 ? mul([b[0] - a[0], b[1] - a[1]], 1 / L) : [1, 0];
      const nrm = [dir[1], -dir[0]];
      let m = -Infinity;
      for (let k = 0; k <= n; k++) {
        const p = add(a, mul([b[0] - a[0], b[1] - a[1]], k / n));
        for (const off of [-o.corridorM, 0, o.corridorM]) m = Math.max(m, terr(add(p, mul(nrm, off))));
      }
      return m;
    };
    const legMax = [];
    for (let i = 0; i < wps.length - 1; i++) legMax.push(segMax(wps[i].xy, wps[i + 1].xy));
    wps.forEach((w, i) => {
      const t = Math.max(i > 0 ? legMax[i - 1] : -Infinity, i < legMax.length ? legMax[i] : -Infinity);
      if (!Number.isFinite(t)) throw new Error('DEM has no data under waypoint ' + i);
      w.terrainMax = t;
      w.h = t + o.aglM;
    });
    // gradient limit: backward pass (climb early), forward pass (descend late)
    for (let i = wps.length - 2; i >= 0; i--) {
      const dd = dist(wps[i].xy, wps[i + 1].xy);
      wps[i].h = Math.max(wps[i].h, wps[i + 1].h - o.maxGradient * dd);
    }
    for (let i = 1; i < wps.length; i++) {
      const dd = dist(wps[i].xy, wps[i - 1].xy);
      wps[i].h = Math.max(wps[i].h, wps[i - 1].h - o.maxGradient * dd);
    }
    wps.forEach(w => {
      const [lon, lat] = proj.inv(w.xy[0], w.xy[1]);
      w.lon = lon; w.lat = lat;
      w.hWrite = w.h + o.geoidN;                // what goes in the file if the header is ellipsoidal
      w.terrainUnderWp = terr(w.xy);
    });
    return route;
  }

  function stats(plan, route) {
    const { wps } = route;
    let len = 0, dataLen = 0;
    for (let i = 1; i < wps.length; i++) {
      const dd = dist(wps[i].xy, wps[i - 1].xy);
      len += dd;
      if (wps[i].role === 'line' && wps[i - 1].role === 'line') dataLen += dd;
    }
    const aglUnder = wps.filter(w => w.role === 'line').map(w => w.h - w.terrainUnderWp);
    return {
      areaHa: plan.areaHa, lines: plan.lines.length, swathM: plan.swath, spacingM: plan.spacing,
      waypoints: wps.length, routeKm: len / 1000, dataKm: dataLen / 1000,
      flightMin: len / route.speedMs / 60, fig8RadiusM: route.fig8RadiusM,
      hMin: Math.min(...wps.map(w => w.h)), hMax: Math.max(...wps.map(w => w.h)),
      aglOnLineMin: Math.min(...aglUnder), aglOnLineMax: Math.max(...aglUnder),
    };
  }

  const api = { DEFAULTS, planLines, buildRoute, applyHeights, stats };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.RouteEngine = api;
})(typeof window !== 'undefined' ? window : globalThis);
