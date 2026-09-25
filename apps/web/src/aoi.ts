import JSZip from 'jszip';
import type { LonLat } from '@3dm/core';

export interface Aoi { name: string; poly: LonLat[]; polygonsFound: number }

// Area of a lon/lat ring, only for choosing the largest polygon (units don't matter).
const ringArea = (r: LonLat[]) => {
  let s = 0;
  for (let i = 0, j = r.length - 1; i < r.length; j = i++) s += (r[j][0] + r[i][0]) * (r[j][1] - r[i][1]);
  return Math.abs(s / 2);
};

function parseCoords(text: string): LonLat[] {
  const pts = text.trim().split(/\s+/).map(t => t.split(',').map(Number)).filter(p => p.length >= 2 && p.every(Number.isFinite));
  const ring = pts.map(p => [p[0], p[1]] as LonLat);
  if (ring.length > 1) {
    const [a, b] = [ring[0], ring[ring.length - 1]];
    if (a[0] === b[0] && a[1] === b[1]) ring.pop();          // drop the closing vertex
  }
  return ring;
}

// Outer rings of every Polygon in a KML document (also DJI Pilot 2 template.kml, which is KML + wpml).
export function polygonsFromKml(kmlText: string): { name: string; ring: LonLat[] }[] {
  const doc = new DOMParser().parseFromString(kmlText, 'application/xml');
  const out: { name: string; ring: LonLat[] }[] = [];
  for (const poly of Array.from(doc.getElementsByTagNameNS('*', 'Polygon'))) {
    const outer = poly.getElementsByTagNameNS('*', 'outerBoundaryIs')[0] ?? poly;
    const coords = outer.getElementsByTagNameNS('*', 'coordinates')[0];
    if (!coords?.textContent) continue;
    const ring = parseCoords(coords.textContent);
    if (ring.length < 3) continue;
    const pm = poly.closest('Placemark');
    const name = pm?.getElementsByTagNameNS('*', 'name')[0]?.textContent?.trim() ?? '';
    out.push({ name, ring });
  }
  return out;
}

export async function aoiFromFile(file: File): Promise<Aoi> {
  const lower = file.name.toLowerCase();
  let texts: string[];
  if (lower.endsWith('.kmz')) {
    const zip = await JSZip.loadAsync(await file.arrayBuffer());
    const kmls = Object.values(zip.files).filter(f => !f.dir && /\.(kml|wpml)$/i.test(f.name));
    texts = await Promise.all(kmls.map(f => f.async('string')));
  } else if (lower.endsWith('.kml')) {
    texts = [await file.text()];
  } else {
    throw new Error('Unsupported block file: ' + file.name + ' (use .kmz or .kml)');
  }
  const polys = texts.flatMap(polygonsFromKml);
  if (!polys.length) throw new Error('No polygon found in ' + file.name);
  const best = polys.reduce((a, b) => (ringArea(b.ring) > ringArea(a.ring) ? b : a));
  return { name: best.name || file.name.replace(/\.(kmz|kml)$/i, ''), poly: best.ring, polygonsFound: polys.length };
}
