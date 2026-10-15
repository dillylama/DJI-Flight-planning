// GeoTIFF fixture for tests: the real writer lives in src/geotiff-write.ts (it is also the DSM export).
export { writeGeoTiff as makeGeoTiff, type TiffSpec } from '../src/geotiff-write.ts';
