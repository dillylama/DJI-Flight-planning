import { test } from 'node:test';
import assert from 'node:assert/strict';
import { existsSync, readFileSync } from 'node:fs';
import { egm96ToEllipsoid } from 'egm96-universal';
import {
  rasterElev, rasterGaps, readGeoTiff, bufferedBbox, openTopoUrl, planLines, buildRoute, applyHeights, validate, planTransit,
  TerrainGapError, type Raster, type LonLat,
} from '../src/index.ts';
import { makeGeoTiff, type TiffSpec } from './tiff-fixture.ts';

// 3×3 raster, 0.001° pixels, value = 100·col + 10·row, with one nodata cell at (2,2). west/north are pixel EDGES.
const r: Raster = {
  width: 3, height: 3, west: 10, north: 5, dLon: 0.001, dLat: 0.001,
  data: [0, 100, 200, 10, 110, 210, 20, 120, -9999], nodata: -9999, pixelIsPoint: false,
};
const elev = rasterElev(r);

test('bilinear sampling on pixel centres', () => {
  const near = (a: number | null, b: number) => assert.ok(a != null && Math.abs(a - b) < 1e-6, `${a} ≉ ${b}`);
  near(elev(10.0005, 4.9995), 0);                         // centre of (0,0)
  near(elev(10.0015, 4.9985), 110);                       // centre of (1,1)
  near(elev(10.001, 4.999), 55);                          // midway between the four top-left centres
});

test('outside the raster or touching nodata returns null', () => {
  assert.equal(elev(9.99, 4.9995), null);
  assert.equal(elev(10.0024, 4.9976), null);
  assert.equal(elev(NaN, 4.9995), null);
});

test('upper = highest of the four cells around the point, never below the bilinear value', () => {
  assert.equal(elev.upper!(10.001, 4.999), 110);          // cells 0, 100, 10, 110
  assert.equal(elev.upper!(10.0006, 4.9994), 110);        // same cell, near its low corner: bilinear is ~12
  assert.ok(elev.upper!(10.0006, 4.9994)! >= elev(10.0006, 4.9994)!);
  assert.equal(elev.upper!(10.002, 4.998), null);         // the cell that includes the nodata corner
  assert.equal(elev.cell!.length, 2);
});

test('buffered bbox and OpenTopography URL', () => {
  const b = bufferedBbox([[10, 5], [10.01, 5.01]], 1113.2);
  assert.ok(Math.abs(b.south - 4.99) < 1e-9 && Math.abs(b.north - 5.02) < 1e-9);
  const u = new URL(openTopoUrl(b, 'KEY'));
  assert.equal(u.searchParams.get('demtype'), 'COP30');
  assert.equal(u.searchParams.get('outputFormat'), 'GTiff');
});

// ── GeoTIFF registration: PixelIsPoint (tie point = pixel centre) vs PixelIsArea (tie point = pixel corner)
const px = 1 / 3600;
const tiff = (rasterType: 1 | 2 | undefined, values: ArrayLike<number>, W: number, H: number, extra: Partial<TiffSpec> = {}) =>
  makeGeoTiff({ width: W, height: H, values, pixel: [px, px], tie: [22, -33], rasterType, ...extra });
// 4 × 3 of real heights (they do not fit in 8 bits): value = 1000.25 + 10·(4·row + col)
const ramp = Float32Array.from({ length: 12 }, (_, i) => 1000.25 + i * 10);

test('PixelIsPoint GeoTIFF (GLO-30 from OpenTopography): the tie point is the centre of pixel (0,0)', async () => {
  const ras = await readGeoTiff(tiff(2, ramp, 4, 3));
  assert.equal(ras.pixelIsPoint, true);
  assert.ok(Math.abs(ras.west - (22 - px / 2)) < 1e-12 && Math.abs(ras.north - (-33 + px / 2)) < 1e-12);
  const e = rasterElev(ras);
  assert.ok(Math.abs(e(22, -33)! - 1000.25) < 1e-6, 'pixel (0,0) is AT the tie point');
  assert.ok(Math.abs(e(22 + px, -33 - px)! - 1050.25) < 1e-6, 'pixel (1,1) is one pixel on');
  assert.ok(Math.abs(e(22 + 3 * px, -33 - 2 * px)! - 1110.25) < 1e-6, 'last pixel');
  assert.equal(e(22 - 0.51 * px, -33), null, 'half a pixel outside the first centre is outside the raster');
});

test('PixelIsArea GeoTIFF (and a file with no raster-type key): the tie point is the outer corner of pixel (0,0)', async () => {
  for (const type of [1, undefined] as const) {
    const ras = await readGeoTiff(tiff(type, ramp, 4, 3));
    assert.equal(ras.pixelIsPoint, false);
    assert.equal(ras.west, 22);
    const e = rasterElev(ras);
    assert.ok(Math.abs(e(22 + px / 2, -33 - px / 2)! - 1000.25) < 1e-6);
    assert.ok(Math.abs(e(22 + 1.5 * px, -33 - 1.5 * px)! - 1050.25) < 1e-6);
  }
});

test('the two registrations of the same file differ by half a pixel, which is why the key matters', async () => {
  const slope = Float32Array.from({ length: 64 * 64 }, (_, i) => 500 + 20 * (i % 64));   // 20 m per pixel eastwards
  const asPoint = rasterElev(await readGeoTiff(tiff(2, slope, 64, 64)));
  const asArea = rasterElev(await readGeoTiff(tiff(1, slope, 64, 64)));
  const lon = 22 + 20.3 * px, lat = -33 - 30 * px;
  assert.ok(Math.abs(asPoint(lon, lat)! - asArea(lon, lat)! - 10) < 1e-3, 'half a pixel of a 20 m/pixel slope is 10 m');
});

test('nodata cells are gaps: never interpolated, counted, and refused next to the point', async () => {
  const v = Float32Array.from(ramp); v[5] = -32767;                      // pixel (1,1)
  const ras = await readGeoTiff(tiff(2, v, 4, 3, { nodata: -32767 }));
  assert.equal(ras.nodata, -32767);
  assert.equal(rasterGaps(ras), 1);
  const e = rasterElev(ras);
  assert.equal(e(22 + 0.5 * px, -33 - 0.5 * px), null);
  assert.equal(e.upper!(22 + 0.5 * px, -33 - 0.5 * px), null);
  assert.ok(Math.abs(e(22 + 3 * px, -33)! - 1030.25) < 1e-6, 'far corner still fine');
});

test('a DEM in the wrong units, projected, or empty is refused', async () => {
  await assert.rejects(readGeoTiff(tiff(2, Float32Array.from(ramp, x => x * 100), 4, 3)), /not plausible/);       // centimetres
  await assert.rejects(readGeoTiff(tiff(2, Float32Array.from(ramp, x => x * 10), 4, 3)), /not plausible/);        // decimetres
  await assert.rejects(readGeoTiff(tiff(2, ramp, 4, 3, { modelType: 1, projectedCs: 32734 })), /projected/);
  await assert.rejects(readGeoTiff(tiff(2, new Float32Array(12).fill(NaN), 4, 3)), /no valid heights/);
  await assert.rejects(readGeoTiff(tiff(2, new Float32Array(12).fill(-9999), 4, 3, { nodata: -9999 })), /no valid heights/);
  await assert.rejects(readGeoTiff(makeGeoTiff({ width: 4, height: 3, values: ramp, pixel: [30, 30], tie: [22, -33] })), /pixel size/);   // metres, not degrees
});

test('an ellipsoidal DTM is converted to orthometric heights when declared', async () => {
  // At 33°S 22°E the EGM96 geoid is ~+32.2 m: 1032.2 m ellipsoidal is ~1000 m above sea level.
  const flat = new Float32Array(64 * 64).fill(1032.2);
  const asIs = rasterElev(await readGeoTiff(tiff(1, flat, 64, 64)));
  const conv = rasterElev(await readGeoTiff(tiff(1, flat, 64, 64), { verticalDatum: 'ellipsoidal' }));
  assert.ok(Math.abs(asIs(22.005, -33.005)! - 1032.2) < 1e-3);
  const N = egm96ToEllipsoid(-33.005, 22.005, 0);
  assert.ok(N > 25 && N < 40, `geoid separation ${N}`);
  assert.ok(Math.abs(conv(22.005, -33.005)! - (1032.2 - N)) < 0.05, `converted to ${conv(22.005, -33.005)}, expected ${1032.2 - N}`);
});

// A real tile, kept locally only (gitignored with the Pilot 2 samples): centres must sit on whole arc-seconds.
const realTif = new URL('../../../samples/glo30-check.tif', import.meta.url);
(existsSync(realTif) ? test : test.skip)('real GLO-30 tile from OpenTopography is PixelIsPoint with cell centres on whole arc-seconds', async () => {
  const b = readFileSync(realTif);
  const ras = await readGeoTiff(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) as ArrayBuffer);
  assert.equal(ras.pixelIsPoint, true);
  const centreLon = (ras.west + ras.dLon / 2) * 3600, centreLat = (ras.north - ras.dLat / 2) * 3600;
  assert.ok(Math.abs(centreLon - Math.round(centreLon)) < 1e-3 && Math.abs(centreLat - Math.round(centreLat)) < 1e-3);
  const e = rasterElev(ras);
  assert.ok(Math.abs(e(ras.west + ras.dLon / 2, ras.north - ras.dLat / 2)! - ras.data[0]) < 1e-6);
  assert.ok(Math.abs(e(ras.west + 10.5 * ras.dLon, ras.north - 7.5 * ras.dLat)! - ras.data[7 * ras.width + 10]) < 1e-6);
});

// ── planning on analytic terrain
const lat0 = 7.55, lon0 = -8.55, m = 1 / 111320;
const poly: LonLat[] = [[-2000, -2000], [2000, -2000], [2000, 2000], [-2000, 2000]].map(([x, y]) => [lon0 + x * m, lat0 + y * m]);

test('validate flags high AGL, unknown WP cap, and nothing blocking on a flat block', () => {
  const plan = planLines(poly, {});
  const flat = applyHeights(plan, buildRoute(plan), () => 500);
  const codes = validate(plan, flat).map(i => i.code);
  assert.ok(!validate(plan, flat).some(i => i.severity === 'error'), codes.join());
  assert.ok(codes.includes('WP_CAP_UNKNOWN'));
  assert.ok(validate(plan, flat, { maxWaypoints: 10 }).some(i => i.code === 'WP_CAP'));

  const ridge = applyHeights(plan, buildRoute(plan), (lon) => ((lon - lon0) / m > 20 && (lon - lon0) / m < 60 ? 900 : 500));
  assert.ok(validate(plan, ridge).some(i => i.code === 'AGL_HIGH'));
});

test('a gap in the DEM anywhere in a corridor is an error, even when the rest of the leg has data', () => {
  const plan = planLines(poly, {});
  // a 60 m hole beside one line, inside its corridor but not under any waypoint
  const holed = (lon: number, lat: number) => {
    const x = (lon - lon0) / m, y = (lat - lat0) / m;
    return Math.hypot(x - 40, y - 310) < 30 ? null : 500;
  };
  assert.throws(() => applyHeights(plan, buildRoute(plan), holed), TerrainGapError);
  // and a gap on the way home is an error for transit / RTH too
  const flight = applyHeights(plan, buildRoute(plan), () => 500);
  const home: LonLat = [lon0 - 4000 * m, lat0];
  const gapOnTheWay = (lon: number) => ((lon - lon0) / m < -2900 && (lon - lon0) / m > -3000 ? null : 500);
  assert.throws(() => planTransit(plan, flight, gapOnTheWay, home), TerrainGapError);
});
