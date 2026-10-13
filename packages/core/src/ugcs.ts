import type { FlightWp } from './heights.ts';
import { isStop, type Route } from './route.ts';

// Route as a CSV that UgCS imports (Route → Import, columns as in the UgCS manual, "Importing routes").
// The point of it is a second opinion: UgCS converts AltitudeAMSL to height above ITS OWN elevation data and
// draws the route in 3D, so terrain clearance can be looked at in software that shares nothing with this one.
//   TurnType 1 = Stop & Turn, 2 = Adaptive Bank Turn (fly-through). Speed is m/s from that waypoint on.
// It carries no L3 actions: UgCS has no CSV column for point-cloud recording.
export function toUgcsCsv(route: Route<FlightWp>): string {
  const n = route.wps.length;
  const rows = route.wps.map((w, i) =>
    [i + 1, w.lat.toFixed(8), w.lon.toFixed(8), w.h.toFixed(2), (Math.floor(w.speed * 100 + 1e-9) / 100).toString(), i === 0 || i === n - 1 || isStop(w) ? 1 : 2].join(','));
  return ['WP,Latitude,Longitude,AltitudeAMSL,Speed,TurnType', ...rows].join('\r\n') + '\r\n';
}
