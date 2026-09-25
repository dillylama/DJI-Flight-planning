import { fromArrayBuffer } from 'geotiff';
import type { ElevFn } from './heights.ts';

// Geographic (EPSG:4326) raster, row 0 at the north edge. Copernicus GLO-30 from OpenTopography is
// delivered like this. Projected client DTMs (e.g. UTM) need reprojection first — rejected for now.
export interface Raster {
  width: number;
  height: number;
  west: number;            // lon of the left edge of column 0
  north: number;           // lat of the top edge of row 0
  dLon: number;            // degrees per column (> 0)
  dLat: number;            // degrees per row (> 0, rows go south)
  data: ArrayLike<number>;
  nodata: number | null;
}

export interface Bbox { west: number; south: number; east: number; north: number }

export async function readGeoTiff(buf: ArrayBuffer): Promise<Raster> {
  const tiff = await fromArrayBuffer(buf);
  const img = await tiff.getImage();
  const keys = img.getGeoKeys() as Record<string, number> | null;
  const model = keys?.GTModelTypeGeoKey;
  if (model != null && model !== 2) {
    throw new Error('DEM is in a projected CRS (GTModelType ' + model + '). Use a geographic (EPSG:4326) GeoTIFF such as GLO-30 from OpenTopography.');
  }
  const [west, north] = img.getOrigin();
  const [rx, ry] = img.getResolution();
  const [band] = (await img.readRasters({ samples: [0] })) as unknown as ArrayLike<number>[];
  const nd = img.getGDALNoData();
  return {
    width: img.getWidth(), height: img.getHeight(),
    west, north, dLon: Math.abs(rx), dLat: Math.abs(ry),
    data: band, nodata: nd == null ? null : nd,
  };
}

export function rasterBbox(r: Raster): Bbox {
  return { west: r.west, north: r.north, east: r.west + r.width * r.dLon, south: r.north - r.height * r.dLat };
}

// Bilinear sampling on pixel centres. Returns null outside the raster or next to nodata, so
// applyHeights refuses to plan over holes instead of silently treating them as sea level.
export function rasterElev(r: Raster): ElevFn {
  const { width: W, height: H, data, nodata } = r;
  const bad = (v: number) => !Number.isFinite(v) || (nodata != null && v === nodata) || v < -1000;
  return (lon, lat) => {
    const fx = (lon - r.west) / r.dLon - 0.5;
    const fy = (r.north - lat) / r.dLat - 0.5;
    if (fx < -0.5 || fy < -0.5 || fx > W - 0.5 || fy > H - 0.5) return null;
    const x0 = Math.max(0, Math.min(W - 2, Math.floor(fx))), y0 = Math.max(0, Math.min(H - 2, Math.floor(fy)));
    const tx = Math.max(0, Math.min(1, fx - x0)), ty = Math.max(0, Math.min(1, fy - y0));
    const i = y0 * W + x0;
    const cells = [[data[i], (1 - tx) * (1 - ty)], [data[i + 1], tx * (1 - ty)], [data[i + W], (1 - tx) * ty], [data[i + W + 1], tx * ty]];
    let h = 0, wSum = 0;
    for (const [v, w] of cells) {
      if (w < 1e-6) continue;              // a nodata neighbour with (numerically) zero weight doesn't matter
      if (bad(v)) return null;
      h += v * w; wSum += w;
    }
    return h / wSum;
  };
}

// Bbox of a polygon grown by bufferM on every side (for the DEM request: covers run-in/out,
// figure-8 and the terrain corridor).
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
