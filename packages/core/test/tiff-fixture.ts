// Minimal GeoTIFF writer for tests: one uncompressed 32-bit float strip in a geographic CRS.
// (geotiff.js can only write 8-bit samples, which cannot hold real heights.)
export interface TiffSpec {
  width: number;
  height: number;
  values: ArrayLike<number>;      // row 0 first (north edge)
  pixel: [number, number];        // degrees per column, degrees per row
  tie: [number, number];          // lon, lat of the tie point of pixel (0,0)
  rasterType?: 1 | 2;             // 1 PixelIsArea, 2 PixelIsPoint; omitted = no key (area by default)
  modelType?: number;             // 2 geographic (default), 1 projected
  projectedCs?: number;           // adds ProjectedCSTypeGeoKey
  nodata?: number;                // adds GDAL_NODATA
}

type Entry = { tag: number; type: 2 | 3 | 4 | 12; values: number[] | string };
const SIZE = { 2: 1, 3: 2, 4: 4, 12: 8 } as const;

export function makeGeoTiff(s: TiffSpec): ArrayBuffer {
  const model = s.modelType ?? 2;
  const keys: [number, number][] = [[1024, model]];
  if (s.rasterType) keys.push([1025, s.rasterType]);
  if (model === 2) keys.push([2048, 4326]);
  if (s.projectedCs != null) keys.push([3072, s.projectedCs]);
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
