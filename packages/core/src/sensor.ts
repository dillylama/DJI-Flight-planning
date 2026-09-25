import { D2R } from './geo.ts';

// ── Photogrammetry ──────────────────────────────────────────────
// Defined by FOV + pixel count (DJI publishes FOVs, not always focal lengths). Image long side across-track.
export interface Camera {
  id: string;
  name: string;
  imgW: number;            // pixels across-track (long side)
  imgH: number;            // pixels along-track
  hfovDeg: number;         // across-track FOV of ONE image (sets GSD)
  vfovDeg: number;         // along-track FOV (sets footprint along)
  coverHfovDeg?: number;   // across-track FOV of the whole capture if wider (e.g. L3's two cameras = 107°)
  minIntervalS: number;    // fastest sustained capture interval
  note?: string;
}

// FOV of a lens on a sensor dimension (mm): 2·atan(d / 2f)
export const lensFovDeg = (sensorMm: number, focalMm: number) => 2 * Math.atan(sensorMm / (2 * focalMm)) / D2R;
export const cameraFovDeg = (c: Camera) => c.coverHfovDeg ?? c.hfovDeg;
// GSD (cm/px) = ground width of one image ÷ pixels
export const gsdCm = (c: Camera, aglM: number) => 2 * aglM * Math.tan((c.hfovDeg / 2) * D2R) / c.imgW * 100;
export const aglForGsd = (c: Camera, gsd: number) => (gsd / 100) * c.imgW / (2 * Math.tan((c.hfovDeg / 2) * D2R));

export interface PhotoPlan {
  gsdCm: number;
  footprintAcrossM: number;
  footprintAlongM: number;
  photoSpacingM: number;   // distance between exposures for the frontlap
  intervalS: number;       // at the line speed
  maxSpeedMs: number;      // fastest speed the camera's min interval allows at this frontlap
  blurPx: number;          // forward motion during the exposure, in pixels
}

export function photoPlan(c: Camera, aglM: number, speedMs: number, frontlapPct: number, shutterS: number): PhotoPlan {
  const gsd = gsdCm(c, aglM) / 100;
  const across = 2 * aglM * Math.tan((cameraFovDeg(c) / 2) * D2R);
  const along = 2 * aglM * Math.tan((c.vfovDeg / 2) * D2R);
  const spacing = along * (1 - frontlapPct / 100);
  return {
    gsdCm: gsd * 100, footprintAcrossM: across, footprintAlongM: along, photoSpacingM: spacing,
    intervalS: spacing / speedMs, maxSpeedMs: spacing / c.minIntervalS, blurPx: (speedMs * shutterS) / gsd,
  };
}

// Frontlap achieved when shooting at a fixed interval (e.g. L3 RGB during a LiDAR flight).
export const frontlapAtInterval = (c: Camera, aglM: number, speedMs: number, intervalS: number) =>
  100 * (1 - (speedMs * intervalS) / (2 * aglM * Math.tan((c.vfovDeg / 2) * D2R)));

// ── LiDAR ───────────────────────────────────────────────────────
// Average density (pts/m²), one return per pulse, points spread uniformly over the swath. Non-repetitive
// and star patterns concentrate points toward the centre, so swath edges get less.
//   per strip: PRR / (v · W)          total with sidelap: PRR / (v · spacing)
export function lidarDensity(pulseRateHz: number, speedMs: number, swathM: number, spacingM: number) {
  return { perStrip: pulseRateHz / (speedMs * swathM), total: pulseRateHz / (speedMs * spacingM) };
}
