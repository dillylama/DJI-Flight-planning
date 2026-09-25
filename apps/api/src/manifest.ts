// Small summary of a 3dm-mission JSON, stored with each version so lists never have to open R2.

export interface SortieSummary {
  index: number;
  fromLine: number | null;
  toLine: number | null;
  waypoints: number | null;
  minutes: number | null;
}

export interface Manifest {
  format: '3dm-mission';
  formatVersion: number | null;
  name: string | null;
  created: string | null;
  sensor: string | null;
  waypoints: number | null;
  routeKm: number | null;
  flightMin: number | null;
  sortieCount: number;
  sorties: SortieSummary[];
}

type Obj = Record<string, unknown>;
const isObj = (v: unknown): v is Obj => typeof v === 'object' && v !== null && !Array.isArray(v);
const num = (v: unknown): number | null => (typeof v === 'number' && Number.isFinite(v) ? v : null);
const str = (v: unknown, max = 200): string | null => (typeof v === 'string' ? v.slice(0, max) : null);
const count = (v: unknown): number | null => (Array.isArray(v) ? v.length : num(v));
const round = (v: number | null, dp: number) => (v === null ? null : Math.round(v * 10 ** dp) / 10 ** dp);

export function isMission(m: unknown): m is Obj {
  return isObj(m) && m.format === '3dm-mission';
}

export function buildManifest(m: Obj): Manifest {
  const block = isObj(m.block) ? m.block : {};
  const sensor = isObj(m.sensor) ? m.sensor : {};
  const stats = isObj(m.stats) ? m.stats : {};
  const sorties: SortieSummary[] = Array.isArray(m.sorties)
    ? m.sorties.map((s, i) => {
        const o = isObj(s) ? s : {};
        const st = isObj(o.stats) ? o.stats : {};
        return {
          index: num(o.index) ?? i,
          fromLine: num(o.fromLine),
          toLine: num(o.toLine),
          waypoints: count(o.waypoints) ?? num(st.waypoints),
          minutes: round(num(o.minutes) ?? num(o.flightMin) ?? num(st.flightMin), 1),
        };
      })
    : [];
  return {
    format: '3dm-mission',
    formatVersion: num(m.version),
    name: str(block.name) ?? str(m.name),
    created: str(m.created, 40),
    sensor: str(sensor.kind, 40),
    waypoints: count(m.waypoints) ?? num(stats.waypoints),
    routeKm: round(num(stats.routeKm), 3),
    flightMin: round(num(stats.flightMin), 1),
    sortieCount: sorties.length,
    sorties,
  };
}
