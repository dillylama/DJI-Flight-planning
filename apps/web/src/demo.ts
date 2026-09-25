import type { ElevFn, LonLat } from '@3dm/core';

// The synthetic Nimba block used by the core regression test: ~3,900 ha, ridge terrain 440–1,225 m.
const lat0 = 7.55, lon0 = -8.55, m2d = 1 / 111320;
const cosLat = Math.cos(lat0 * Math.PI / 180);

export const demoPoly: LonLat[] = [[-3300, -3000], [3100, -3300], [3400, 800], [1500, 3200], [-2800, 3000], [-3500, 200]]
  .map(([x, y]) => [lon0 + x * m2d / cosLat, lat0 + y * m2d]);

export const demoElev: ElevFn = (lon, lat) => {
  const x = (lon - lon0) / m2d * cosLat, y = (lat - lat0) / m2d;
  return 440 + 785 * Math.exp(-((x - 800) ** 2 / 1.2e6 + (y - 500) ** 2 / 1.2e7)) + 60 * Math.sin(x / 400) * Math.cos(y / 550);
};
