import { ellipsoidToEgm96 } from 'egm96-universal';
import { fromArrayBuffer } from 'geotiff';
import { metresPerDeg } from './geo.ts';
import type { ElevFn } from './terrain.ts';

// Geographic (EPSG:4326) raster, north-up, row 0 at the north edge. Copernicus GLO-30 from OpenTopography is
// delivered like this. Projected client DTMs (e.g. UTM) need reprojection first — rejected for now.
// `west` / `north` are always the OUTER EDGE of pixel (0,0), whatever the file's raster type.
export interface Raster {
  width: number;
  height: number;
  west: number;            // lon of the west edge of column 0
  north: number;           // lat of the north edge of row 0
  dLon: number;            // degrees per column (> 0)
  dLat: number;            // degrees per row (> 0, rows go south)
  data: ArrayLike<number>;
  nodata: number | null;
  pixelIsPoint: boolean;   // how the file was registered (GLO-30 from OpenTopography: true)
  sourceDatum?: 'orthometric' | 'ellipsoidal';   // what the file's heights were read as (`data` is always orthometric)
  verticalCs?: number | null;                     // vertical CS code declared in the file, if any
}

export interface Bbox { west: number; south: number; east: number; north: number }

// Heights in a DEM are orthometric (EGM96 / EGM2008 / local mean sea level) unless told otherwise. A DTM
// exported from a PPK or LiDAR workflow is often WGS84 ellipsoidal: it must be declared, because treating
// it as orthometric shifts every flight height by the geoid separation (about 32 m at Prince Albert).
export type VerticalDatum = 'orthometric' | 'ellipsoidal';
export interface ReadDemOptions { verticalDatum?: VerticalDatum }

export async function readGeoTiff(buf: ArrayBuffer, opts: ReadDemOptions = {}): Promise<Raster> {
  const tiff = await fromArrayBuffer(buf);
  const img = await tiff.getImage();
  const keys = img.getGeoKeys() as Record<string, number> | null;
  const model = keys?.GTModelTypeGeoKey;
  if ((model != null && model !== 2) || keys?.ProjectedCSTypeGeoKey != null) {
    throw new Error('DEM is in a projected CRS. Use a geographic (EPSG:4326) GeoTIFF such as GLO-30 from OpenTopography.');
  }
  // Vertical keys, when the file carries them: heights must be metres, and the declared vertical datum must
  // agree with what the caller says (GeoTIFF 5001–5033 and EPSG 4979 mean heights above the ellipsoid).
  const vUnits = keys?.VerticalUnitsGeoKey, vCs = keys?.VerticalCSTypeGeoKey;
  if (vUnits != null && vUnits !== 9001) throw new Error(`DEM heights are not in metres (vertical unit code ${vUnits}). Convert the DEM to metres first.`);
  const declaredEllipsoidal = vCs != null && ((vCs >= 5001 && vCs <= 5033) || vCs === 4979);
  const declaredOrthometric = vCs != null && [5773, 3855, 5714, 5715].includes(vCs);
  if (declaredEllipsoidal && opts.verticalDatum !== 'ellipsoidal') throw new Error(`DEM declares ellipsoidal heights (vertical CS ${vCs}). Load it with the height datum set to "ellipsoidal".`);
  if (declaredOrthometric && opts.verticalDatum === 'ellipsoidal') throw new Error(`DEM declares heights above mean sea level (vertical CS ${vCs}), not ellipsoidal. Load it with the height datum set to "mean sea level".`);
  const fd = img.getFileDirectory() as { ModelTransformation?: ArrayLike<number> };
  const mt = fd.ModelTransformation;
  if (mt && (Math.abs(mt[1]) > 1e-12 || Math.abs(mt[4]) > 1e-12)) throw new Error('DEM is rotated or sheared; only north-up GeoTIFFs are supported.');
  const [ox, oy] = img.getOrigin();
  const [rx, ry] = img.getResolution();
  if (!(rx > 0) || !(ry < 0)) throw new Error('DEM is not north-up (unexpected pixel size signs).');
  const width = img.getWidth(), height = img.getHeight();
  const dLon = rx, dLat = -ry;
  if (dLon < 1e-7 || dLon > 0.1 || dLat < 1e-7 || dLat > 0.1) throw new Error(`DEM pixel size ${dLon}° × ${dLat}° is not plausible for a geographic DEM.`);

  // GeoTIFF: with RasterPixelIsPoint the tie point is the CENTRE of pixel (0,0); with PixelIsArea (or no key)
  // it is the outer corner. GLO-30 from OpenTopography is PixelIsPoint with centres on whole arc-seconds.
  const pixelIsPoint = keys?.GTRasterTypeGeoKey === 2;
  const west = pixelIsPoint ? ox - dLon / 2 : ox;
  const north = pixelIsPoint ? oy + dLat / 2 : oy;
  if (west < -180 || west + width * dLon > 180.0001 || north > 90.0001 || north - height * dLat < -90.0001) {
    throw new Error('DEM extent is outside ±180° / ±90° (longitudes must be −180…180).');
  }

  const [band] = (await img.readRasters({ samples: [0] })) as unknown as ArrayLike<number>[];
  if (band.length !== width * height) throw new Error('DEM band size does not match its dimensions.');
  const nd = img.getGDALNoData();
  const nodata = nd == null || Number.isNaN(nd) ? null : nd;
  const bad = (v: number) => !Number.isFinite(v) || (nodata != null && v === nodata) || v < -1000;

  let data: ArrayLike<number> = band;
  if (opts.verticalDatum === 'ellipsoidal') {
    // h_orthometric = h_ellipsoidal − N. N changes slowly, so take it on a coarse lattice and interpolate.
    const K = 33;
    const Ng: number[] = [];
    for (let j = 0; j < K; j++) for (let i = 0; i < K; i++) {
      const lat = north - (height * dLat * j) / (K - 1), lon = west + (width * dLon * i) / (K - 1);
      Ng.push(-ellipsoidToEgm96(lat, lon, 0));
    }
    const out = new Float32Array(width * height);
    for (let y = 0; y < height; y++) {
      const gy = ((y + 0.5) / height) * (K - 1), y0 = Math.min(K - 2, Math.floor(gy)), ty = gy - y0;
      for (let x = 0; x < width; x++) {
        const v = band[y * width + x];
        if (bad(v)) { out[y * width + x] = NaN; continue; }
        const gx = ((x + 0.5) / width) * (K - 1), x0 = Math.min(K - 2, Math.floor(gx)), tx = gx - x0;
        const N = (Ng[y0 * K + x0] * (1 - tx) + Ng[y0 * K + x0 + 1] * tx) * (1 - ty) + (Ng[(y0 + 1) * K + x0] * (1 - tx) + Ng[(y0 + 1) * K + x0 + 1] * tx) * ty;
        out[y * width + x] = v - N;
      }
    }
    data = out;
  }

  // Wrong units (feet, centimetres) or a non-height band show up as implausible values.
  let lo = Infinity, hi = -Infinity, valid = 0;
  for (let i = 0; i < data.length; i++) { const v = data[i]; if (bad(v)) continue; valid++; if (v < lo) lo = v; if (v > hi) hi = v; }
  if (!valid) throw new Error('DEM contains no valid heights.');
  if (lo < -500 || hi > 9000) throw new Error(`DEM heights range ${lo.toFixed(0)}…${hi.toFixed(0)} m, which is not plausible for metres above sea level. Check the units.`);

  return { width, height, west, north, dLon, dLat, data, nodata, pixelIsPoint, sourceDatum: opts.verticalDatum ?? 'orthometric', verticalCs: vCs ?? null };
}

export function rasterBbox(r: Raster): Bbox {
  return { west: r.west, north: r.north, east: r.west + r.width * r.dLon, south: r.north - r.height * r.dLat };
}

// Count of cells with no data (a user DTM with holes is refused wherever a route touches one).
export function rasterGaps(r: Raster): number {
  let n = 0;
  for (let i = 0; i < r.data.length; i++) { const v = r.data[i]; if (!Number.isFinite(v) || (r.nodata != null && v === r.nodata) || v < -1000) n++; }
  return n;
}

// Bilinear sampling on pixel centres, plus `upper` (highest of the cells around the point) for clearance
// maths. Both return null outside the raster or next to a gap, so nothing is ever planned over a hole.
export function rasterElev(r: Raster): ElevFn {
  const { width: W, height: H, data, nodata } = r;
  if (W < 2 || H < 2) throw new Error('DEM is too small.');
  const bad = (v: number) => !Number.isFinite(v) || (nodata != null && v === nodata) || v < -1000;
  // Fractional pixel-centre coordinates of a point, or null when it is outside the raster.
  const locate = (lon: number, lat: number) => {
    const fx = (lon - r.west) / r.dLon - 0.5;
    const fy = (r.north - lat) / r.dLat - 0.5;
    if (!(fx >= -0.5) || !(fy >= -0.5) || !(fx <= W - 0.5) || !(fy <= H - 0.5)) return null;   // also rejects NaN
    const x0 = Math.max(0, Math.min(W - 2, Math.floor(fx))), y0 = Math.max(0, Math.min(H - 2, Math.floor(fy)));
    return { i: y0 * W + x0, tx: Math.max(0, Math.min(1, fx - x0)), ty: Math.max(0, Math.min(1, fy - y0)) };
  };
  const elev: ElevFn = (lon, lat) => {
    const c = locate(lon, lat);
    if (!c) return null;
    const { i, tx, ty } = c;
    const cells = [[data[i], (1 - tx) * (1 - ty)], [data[i + 1], tx * (1 - ty)], [data[i + W], (1 - tx) * ty], [data[i + W + 1], tx * ty]];
    let h = 0, wSum = 0;
    for (const [v, w] of cells) {
      if (w < 1e-6) continue;              // a gap neighbour with (numerically) zero weight doesn't matter
      if (bad(v)) return null;
      h += v * w; wSum += w;
    }
    return h / wSum;
  };
  elev.upper = (lon, lat) => {
    const c = locate(lon, lat);
    if (!c) return null;
    const a = data[c.i], b = data[c.i + 1], d = data[c.i + W], e = data[c.i + W + 1];
    if (bad(a) || bad(b) || bad(d) || bad(e)) return null;
    return Math.max(a, b, d, e);
  };
  const midLat = r.north - (H * r.dLat) / 2;
  const [mLon, mLat] = metresPerDeg(midLat);
  elev.cell = [r.dLon * mLon, r.dLat * mLat];
  return elev;
}

// Bbox of a set of points grown by bufferM on every side (for the DEM request).
export function bufferedBbox(poly: [number, number][], bufferM: number): Bbox {
  const lats = poly.map(p => p[1]), lons = poly.map(p => p[0]);
  const lat0 = (Math.min(...lats) + Math.max(...lats)) / 2;
  const dLat = bufferM / 111320, dLon = bufferM / (111320 * Math.cos(lat0 * Math.PI / 180));
  return { west: Math.min(...lons) - dLon, east: Math.max(...lons) + dLon, south: Math.min(...lats) - dLat, north: Math.max(...lats) + dLat };
}

export type OpenTopoDem = 'COP30' | 'COP90' | 'SRTMGL1' | 'NASADEM' | 'AW3D30';

export function openTopoUrl(b: Bbox, apiKey: string, demtype: OpenTopoDem = 'COP30'): string {
  const q = new URLSearchParams({
    demtype, south: String(b.south), north: String(b.north), west: String(b.west), east: String(b.east),
    outputFormat: 'GTiff', API_Key: apiKey,
  });
  return 'https://portal.opentopography.org/API/globaldem?' + q;
}
