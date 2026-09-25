# Architecture

## Workflow

1. **Office – plan** (`apps/web`). Import the block AOI (KMZ/KML/SHP), load terrain (Copernicus
   GLO-30 via OpenTopography, or the client's own DTM), set AGL / speed / sidelap / course / L3 scan
   mode, and review lines, figure-8, per-line height profiles and validation. Split into sorties.
2. **Office – publish.** The planner builds a **mission package** and uploads it to `apps/api`.
3. **Sync to the RC.** *3DM Fly* on the RC Plus 2 pulls published packages while it has
   internet (office wifi or a hotspot before leaving). After that everything is offline.
4. **Field – fly.** Two paths:
   - **Own app (preferred):** the KMZ is uploaded to the aircraft with MSDK v5
     `WaypointMissionManager` and started, paused and stopped from our app, which shows live
     monitoring, the pre-flight checklist, auto-resume and coverage.
   - **Pilot 2:** our app writes the KMZ into Pilot 2's route library (or saves it for Pilot 2's
     "Import route"), then the pilot closes our app and flies in Pilot 2. Only one app can hold
     the aircraft link at a time.
5. **Back to the office.** As-flown logs (lines completed with recording on, resume points) sync
   back and attach to the project.

## Mission package

One package per project version, immutable once published:

| Item | Purpose |
|---|---|
| `mission.json` | Plan parameters, AOI, lines, computed waypoints per sortie, core version |
| `sorties/NN.kmz` | Pilot 2 / MSDK-ready KMZ per sortie (`template.kml` + `waylines.wpml`, written consistently) |
| `dem.tif` | DEM clip of AOI + buffer, so the RC can regenerate resume routes offline |
| `tiles/` | Offline basemap tiles for the AOI |

Resume routes built on the RC use the **same `packages/core` code** as the office planner (run in
a WebView), so a field resume is identical to what the office would produce.

## Route engine rules (`packages/core`)

- Local tangent-plane projection about the block centroid (blocks up to a few tens of km).
- Swath = 2 · AGL · tan(FOV/2); line spacing = swath · (1 − sidelap).
- Line extent per band covers the full polygon extent, so concave notches keep recording.
- Waypoint height = max terrain over both adjacent legs, sampled every `demSampleM` along the leg
  and ±`corridorM` across it, + AGL. Then a backward pass (climb early) and a forward pass (descend
  late) enforce `maxGradient`, only ever raising waypoints.
- Figure-8 radius = v² / (g · tan(bank)); crossover one radius behind the run-in start;
  approach waypoint carries `START_RECORD`.
- Damping = min(`dampingMaxM`, `dampingFrac` × shorter adjacent leg), so it is always below the
  leg length as DJI requires.

### Known issues to address
- **AGL spread on steep terrain:** the max-over-legs rule can put line waypoints well above nominal
  AGL (synthetic Nimba: 501–827 m for 500 m nominal). Coverage is safe (the swath only widens), but
  point density and L3 range margin drop. Planned: density / range checks and adaptive waypoint
  spacing on steep lines.
- **Figure-8 at line speed:** a 63 m radius at 17 m/s with ~33 m between points will make the aircraft
  slow down in waypoint mode, so the actual bank and speed will be lower than designed. To be
  measured in the simulator.

## Security

- The DJI app key lives only in `apps/rc/local.properties` (gitignored). The repo is public.
- The RC is paired to the API with a per-device token issued via QR code from the web planner.
