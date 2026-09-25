// Planar geometry in a local tangent plane (metres, x = east, y = north).

export type LonLat = [lon: number, lat: number];
export type XY = [x: number, y: number];

export const R_EARTH = 6378137;
export const D2R = Math.PI / 180;
export const G = 9.81;

export interface Projection {
  fwd(lon: number, lat: number): XY;
  inv(x: number, y: number): LonLat;
}

// Local tangent plane about (lat0, lon0) — fine for blocks up to a few tens of km.
export function makeProj(lat0: number, lon0: number): Projection {
  const k = Math.cos(lat0 * D2R);
  return {
    fwd: (lon, lat) => [(lon - lon0) * D2R * R_EARTH * k, (lat - lat0) * D2R * R_EARTH],
    inv: (x, y) => [lon0 + x / (R_EARTH * k) / D2R, lat0 + y / R_EARTH / D2R],
  };
}

export const add = (a: XY, b: XY): XY => [a[0] + b[0], a[1] + b[1]];
export const sub = (a: XY, b: XY): XY => [a[0] - b[0], a[1] - b[1]];
export const mul = (a: XY, s: number): XY => [a[0] * s, a[1] * s];
export const dot = (a: XY, b: XY): number => a[0] * b[0] + a[1] * b[1];
export const dist = (a: XY, b: XY): number => Math.hypot(a[0] - b[0], a[1] - b[1]);

export function polygonAreaM2(xy: XY[]): number {
  let s = 0;
  for (let i = 0, j = xy.length - 1; i < xy.length; j = i++) s += (xy[j][0] + xy[i][0]) * (xy[j][1] - xy[i][1]);
  return Math.abs(s / 2);
}

// Along-track extent of the polygon inside the band v∈[v0,v1] (u = along, v = across).
// Concave blocks: the line spans the full extent and keeps recording across any notch.
export function bandExtent(uv: XY[], v0: number, v1: number): [number, number] | null {
  let umin = Infinity, umax = -Infinity;
  for (let i = 0, j = uv.length - 1; i < uv.length; j = i++) {
    const a = uv[j], b = uv[i];
    let t0 = 0, t1 = 1;
    const dv = b[1] - a[1];
    if (Math.abs(dv) < 1e-9) {
      if (a[1] < v0 || a[1] > v1) continue;
    } else {
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
