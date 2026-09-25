import { dist, type ElevFn, type FlightWp, type Plan } from '@3dm/core';

const NS = 'http://www.w3.org/2000/svg';
const el = (tag: string, attrs: Record<string, string | number>) => {
  const e = document.createElementNS(NS, tag);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, String(v));
  return e;
};

// Height profile of one line (run-in → line → run-out): flight height, terrain under the path sampled
// densely from the DEM, and terrain + nominal AGL (the floor the flight path must stay above).
export function renderProfile(svg: SVGSVGElement, plan: Plan, wps: FlightWp[], elev: ElevFn) {
  svg.replaceChildren();
  const W = svg.clientWidth || 800, H = svg.clientHeight || 200;
  const pad = { l: 52, r: 12, t: 12, b: 24 };
  if (wps.length < 2) return;

  const cum = [0];
  for (let i = 1; i < wps.length; i++) cum.push(cum[i - 1] + dist(wps[i].xy, wps[i - 1].xy));
  const total = cum[cum.length - 1];

  const terr: [number, number][] = [];
  for (let i = 1; i < wps.length; i++) {
    const a = wps[i - 1], b = wps[i], L = cum[i] - cum[i - 1];
    const n = Math.max(1, Math.ceil(L / 15));
    for (let k = 0; k <= n; k++) {
      const t = k / n;
      const [lon, lat] = plan.proj.inv(a.xy[0] + (b.xy[0] - a.xy[0]) * t, a.xy[1] + (b.xy[1] - a.xy[1]) * t);
      const h = elev(lon, lat);
      if (h != null && Number.isFinite(h)) terr.push([cum[i - 1] + L * t, h]);
    }
  }
  const hs = [...wps.map(w => w.h), ...terr.map(t => t[1])];
  const hMin = Math.min(...hs) - 40, hMax = Math.max(...hs) + 40;
  const X = (d: number) => pad.l + (d / total) * (W - pad.l - pad.r);
  const Y = (h: number) => H - pad.b - ((h - hMin) / (hMax - hMin)) * (H - pad.t - pad.b);

  // grid + labels
  const step = [25, 50, 100, 200, 250, 500].find(s => (hMax - hMin) / s <= 6) ?? 1000;
  for (let h = Math.ceil(hMin / step) * step; h <= hMax; h += step) {
    svg.append(el('line', { x1: pad.l, x2: W - pad.r, y1: Y(h), y2: Y(h), class: 'grid' }));
    const t = el('text', { x: pad.l - 6, y: Y(h) + 4, 'text-anchor': 'end', class: 'axis' });
    t.textContent = `${h}`;
    svg.append(t);
  }
  const kmStep = total > 8000 ? 2000 : total > 3000 ? 1000 : 500;
  for (let d = 0; d <= total; d += kmStep) {
    const t = el('text', { x: X(d), y: H - 6, 'text-anchor': 'middle', class: 'axis' });
    t.textContent = `${(d / 1000).toFixed(1)} km`;
    svg.append(t);
  }

  if (terr.length) {
    const area = `M${X(terr[0][0])},${Y(hMin)} ` + terr.map(([d, h]) => `L${X(d)},${Y(h)}`).join(' ') + ` L${X(terr[terr.length - 1][0])},${Y(hMin)} Z`;
    svg.append(el('path', { d: area, class: 'terrain' }));
    svg.append(el('polyline', { points: terr.map(([d, h]) => `${X(d)},${Y(h + plan.o.aglM)}`).join(' '), class: 'agl-floor' }));
  }
  svg.append(el('polyline', { points: wps.map((w, i) => `${X(cum[i])},${Y(w.h)}`).join(' '), class: 'flight' }));
  wps.forEach((w, i) => svg.append(el('circle', { cx: X(cum[i]), cy: Y(w.h), r: w.role === 'line' ? 2.5 : 3.5, class: 'wp ' + w.role })));
}
