# apps/web — office planner

Vite + TypeScript + MapLibre GL, deployed to Cloudflare Pages. Uses `@3dm/core` for all route maths.

Planned features: AOI import (KMZ/KML/SHP), DEM (Copernicus GLO-30 via OpenTopography or a client
DTM), line planning, figure-8, per-line height/terrain profiles, 3D preview, validation panel
(waypoint cap, damping vs leg, AGL band, gradient, DEM nodata), sortie split, projects, and
**Publish to RC**. UI fonts: IBM Plex Mono + Barlow Semi Condensed.

## Status: v0.1 (local)

```bash
npm run dev -w @3dm/web     # http://localhost:5178
```

Working now:
- KMZ/KML block import (drag-and-drop anywhere; Pilot 2 KMZs work too). The largest polygon is used.
- Terrain: Copernicus GLO-30 fetched from OpenTopography (your API key is kept in the browser), or
  drop a GeoTIFF in EPSG:4326.
- Survey parameters; lines, run-in/out, figure-8 and recording start/stop drawn over imagery.
- Per-line height profile: flight height against terrain and terrain + AGL.
- Stats and checks (damping, AGL low/high, gradient, waypoint cap, figure-8 note).
- Resume ("data stopped on line N" + new speed).
- **Optimise course**: with terrain loaded it minimises route length × (mean line AGL / nominal),
  which favours lines along slopes (density ∝ 1/AGL); without terrain it picks the fewest lines.
- **Home & take-off**: set home on the map (draggable). Take-off security height, fly-to-route mode
  (DJI *safely* / *pointToPoint*), transit speed, min terrain clearance. The transit path is checked for
  terrain clearance; the RTH height (auto = recommended) is checked on the straight line home from every waypoint.
- **3D view**: routes, take-off transit and RTH drawn at their true heights (deck.gl) over MapLibre 3D terrain
  (AWS Terrain Tiles, display only), with drop lines from each data waypoint to the ground.
- **LiDAR overlap**: planned vs achieved sidelap using each waypoint's real AGL, swath range, and swath
  footprints on the map (darker where strips overlap).
- **Tooltips** on every option explaining what it does and the physics behind it.
- Export `mission.json` (includes home and take-off settings). KMZ export waits for the RC sample.
- A demo block (synthetic Nimba ridge) to try it without files.

In dev, the OpenTopography key can be put in `apps/web/.env.development.local` as `VITE_OPENTOPO_KEY=` (gitignored,
not included in production builds).

Next: sortie split, IndexedDB project store and DEM cache, publish to `apps/api`.
