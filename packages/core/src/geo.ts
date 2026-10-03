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

// Metres per degree of longitude and latitude on the WGS84 ellipsoid at a latitude.
const WGS84_E2 = 6.69437999014e-3;
export function metresPerDeg(latDeg: number): [east: number, north: number] {
  const s = Math.sin(latDeg * D2R), w = Math.sqrt(1 - WGS84_E2 * s * s);
  return [(R_EARTH / w) * Math.cos(latDeg * D2R) * D2R, ((R_EARTH * (1 - WGS84_E2)) / (w * w * w)) * D2R];
}

// Local tangent plane about (lat0, lon0) with true metres on the WGS84 ellipsoid: distances are right to
// about 0.1 % over blocks of a few tens of km. 'sphere' is the plane of the original JS engine (north–south
// distances 0.4 % long at mid latitudes), kept only so the port can be compared with it point for point.
export type ProjModel = 'wgs84' | 'sphere';
export function makeProj(lat0: number, lon0: number, model: ProjModel = 'wgs84'): Projection {
  if (model === 'sphere') {
    const k = Math.cos(lat0 * D2R);
    return {
      fwd: (lon, lat) => [(lon - lon0) * D2R * R_EARTH * k, (lat - lat0) * D2R * R_EARTH],
      inv: (x, y) => [lon0 + x / (R_EARTH * k) / D2R, lat0 + y / R_EARTH / D2R],
    };
  }
  const [kx, ky] = metresPerDeg(lat0);
  return {
    fwd: (lon, lat) => [(lon - lon0) * kx, (lat - lat0) * ky],
    inv: (x, y) => [lon0 + x / kx, lat0 + y / ky],
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
