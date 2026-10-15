// Minimal GeoTIFF writer: one uncompressed 32-bit float strip in a geographic CRS (EPSG:4326), north-up.
// geotiff.js can only write 8-bit samples, which cannot hold heights. Used for the test fixtures and for the
// DSM export for DJI Pilot 2 / FlightHub 2 terrain following (they accept a geodetic WGS84 GeoTIFF, EPSG:4326).
import { egm96ToEllipsoid } from 'egm96-universal';
import type { Raster, Bbox } from './dem.ts';
import { metresPerDeg } from './geo.ts';

export interface TiffSpec {
  width: number;
  height: number;
  values: ArrayLike<number>;      // row 0 first (north edge)
  pixel: [number, number];        // degrees per column, degrees per row
  tie: [number, number];          // lon, lat of the tie point of pixel (0,0)
  rasterType?: 1 | 2;             // 1 PixelIsArea (tie = outer corner), 2 PixelIsPoint (tie = centre); omitted = no key (area by default)
  modelType?: number;             // 2 geographic (default), 1 projected
  projectedCs?: number;           // adds ProjectedCSTypeGeoKey
  nodata?: number;                // adds GDAL_NODATA
  verticalCs?: number;            // adds VerticalCSTypeGeoKey (4979 = WGS84 ellipsoidal height, 5773 = EGM96)
}

type Entry = { tag: number; type: 2 | 3 | 4 | 12; values: number[] | string };
const SIZE = { 2: 1, 3: 2, 4: 4, 12: 8 } as const;

export function writeGeoTiff(s: TiffSpec): ArrayBuffer {
  const model = s.modelType ?? 2;
  const keys: [number, number][] = [[1024, model]];
  if (s.rasterType) keys.push([1025, s.rasterType]);
  if (model === 2) keys.push([2048, 4326], [2054, 9102]);   // GeographicType WGS84, angular unit degree
  if (s.projectedCs != null) keys.push([3072, s.projectedCs]);
  if (s.verticalCs != null) keys.push([4096, s.verticalCs], [4099, 9001]);   // VerticalCSType, VerticalUnits metre
  const pixels = s.width * s.height;
  if (s.values.length !== pixels) throw new Error('values do not match width × height');

  const entries: Entry[] = [
    { tag: 256, type: 4, values: [s.width] }, { tag: 257, type: 4, values: [s.height] },
    { tag: 258, type: 3, values: [32] },                    // BitsPerSample
    { tag: 259, type: 3, values: [1] },                     // no compression
    { tag: 262, type: 3, values: [1] },                     // BlackIsZero
    { tag: 273, type: 4, values: [0] },                     // StripOffsets, patched below
    { tag: 277, type: 3, values: [1] },                     // SamplesPerPixel
    { tag: 278, type: 4, values: [s.height] },              // RowsPerStrip
    { tag: 279, type: 4, values: [pixels * 4] },            // StripByteCounts
    { tag: 284, type: 3, values: [1] },                     // PlanarConfiguration
    { tag: 339, type: 3, values: [3] },                     // SampleFormat: IEEE float
    { tag: 33550, type: 12, values: [s.pixel[0], s.pixel[1], 0] },
    { tag: 33922, type: 12, values: [0, 0, 0, s.tie[0], s.tie[1], 0] },
    { tag: 34735, type: 3, values: [1, 1, 0, keys.length, ...keys.flatMap(([k, v]) => [k, 0, 1, v])] },
  ];
  if (s.nodata != null) entries.push({ tag: 42113, type: 2, values: String(s.nodata) + '\0' });

  const ifdAt = 8;
  let extra = ifdAt + 2 + entries.length * 12 + 4;
  const where = entries.map(e => {
    const bytes = SIZE[e.type] * e.values.length;
    if (bytes <= 4) return -1;
    const at = extra;
    extra += bytes + (bytes & 1);
    return at;
  });
  const dataAt = extra;
  const buf = new ArrayBuffer(dataAt + pixels * 4);
  const dv = new DataView(buf);
  dv.setUint16(0, 0x4949, true); dv.setUint16(2, 42, true); dv.setUint32(4, ifdAt, true);
  dv.setUint16(ifdAt, entries.length, true);
  entries.forEach((e, i) => {
    const p = ifdAt + 2 + i * 12;
    const vals = e.tag === 273 ? [dataAt] : e.values;
    dv.setUint16(p, e.tag, true); dv.setUint16(p + 2, e.type, true); dv.setUint32(p + 4, vals.length, true);
    const at = where[i] < 0 ? p + 8 : where[i];
    if (where[i] >= 0) dv.setUint32(p + 8, where[i], true);
    for (let j = 0; j < vals.length; j++) {
      if (typeof vals === 'string') dv.setUint8(at + j, vals.charCodeAt(j));
      else if (e.type === 3) dv.setUint16(at + 2 * j, vals[j], true);
      else if (e.type === 4) dv.setUint32(at + 4 * j, vals[j], true);
      else dv.setFloat64(at + 8 * j, vals[j], true);
    }
  });
  for (let i = 0; i < pixels; i++) dv.setFloat32(dataAt + 4 * i, s.values[i], true);
  return buf;
}

// A terrain model for DJI's own terrain following, made from the DEM the planner flies on.
//   dilateM   each cell becomes the HIGHEST cell within this radius: DJI follows the ground directly under the
//             line, so this puts the planner's lateral corridor rule into the surface it follows (75 m = the
//             planner default; 0 = the bare DEM).
//   marginM   added to every cell, on top of the AGL the mission will ask for.
//   datum     'ellipsoidal' writes WGS84 ellipsoidal heights (what DJI Terra's own DSMs carry), 'orthometric' the
//             DEM's heights above mean sea level. Pilot 2 does not document which it expects: check the first
//             mission made from the file with the mission checker before flying it.
//   bbox      crop (the whole raster when omitted). Cells with no data stay nodata.
export interface DsmOptions { dilateM?: number; marginM?: number; datum?: 'ellipsoidal' | 'orthometric'; bbox?: Bbox }
export function dsmFromRaster(r: Raster, o: DsmOptions = {}): TiffSpec {
  const dilateM = o.dilateM ?? 75, marginM = o.marginM ?? 0, datum = o.datum ?? 'ellipsoidal';
  const bad = (v: number) => !Number.isFinite(v) || (r.nodata != null && v === r.nodata) || v < -1000;
  // crop window in whole cells
  let c0 = 0, c1 = r.width, r0 = 0, r1 = r.height;
  if (o.bbox) {
    c0 = Math.max(0, Math.floor((o.bbox.west - r.west) / r.dLon)); c1 = Math.min(r.width, Math.ceil((o.bbox.east - r.west) / r.dLon));
    r0 = Math.max(0, Math.floor((r.north - o.bbox.north) / r.dLat)); r1 = Math.min(r.height, Math.ceil((r.north - o.bbox.south) / r.dLat));
    if (c1 - c0 < 2 || r1 - r0 < 2) throw new Error('The DSM window does not cover the DEM.');
  }
  const W = c1 - c0, H = r1 - r0;
  const midLat = r.north - ((r0 + r1) / 2) * r.dLat;
  const [mLon, mLat] = metresPerDeg(midLat);
  const rx = dilateM > 0 ? Math.ceil(dilateM / (r.dLon * mLon)) : 0, ry = dilateM > 0 ? Math.ceil(dilateM / (r.dLat * mLat)) : 0;
  const NODATA = -9999;
  const out = new Float32Array(W * H);
  for (let y = 0; y < H; y++) {
    const lat = r.north - (r0 + y + 0.5) * r.dLat;
    for (let x = 0; x < W; x++) {
      const v0 = r.data[(r0 + y) * r.width + c0 + x];
      if (bad(v0)) { out[y * W + x] = NODATA; continue; }
      let m = v0;
      for (let dy = -ry; dy <= ry; dy++) {
        const yy = r0 + y + dy; if (yy < 0 || yy >= r.height) continue;
        for (let dx = -rx; dx <= rx; dx++) {
          const xx = c0 + x + dx; if (xx < 0 || xx >= r.width) continue;
          if (Math.hypot(dx * r.dLon * mLon, dy * r.dLat * mLat) > dilateM) continue;
          const v = r.data[yy * r.width + xx];
          if (!bad(v) && v > m) m = v;
        }
      }
      m += marginM;
      const lon = r.west + (c0 + x + 0.5) * r.dLon;
      out[y * W + x] = datum === 'ellipsoidal' ? egm96ToEllipsoid(lat, lon, m) : m;
    }
  }
  return {
    width: W, height: H, values: out, pixel: [r.dLon, r.dLat], tie: [r.west + c0 * r.dLon, r.north - r0 * r.dLat],
    rasterType: 1, modelType: 2, nodata: NODATA, verticalCs: datum === 'ellipsoidal' ? 4979 : 5773,
  };
}
