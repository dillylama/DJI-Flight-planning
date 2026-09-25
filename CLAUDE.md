# DJI-Flight-planning — M400 / L3 LiDAR mission system (UgCS-style)

## Architecture (decided 25 Sep 2026)
Plan in the office on the web → sync mission package to the RC → fly offline from our own Android app
(MSDK v5), or inject the KMZ into Pilot 2 and fly there. No internet needed in the field.

```
packages/core   TypeScript, shared by web + RC: route engine, WPML writer/reader, DEM sampling,
                validation, sortie split, resume, mission-package format. Tested in Node.
apps/web        Office planner (Vite + TS + MapLibre) → Cloudflare Pages.
apps/api        Cloudflare Worker + D1 (projects) + R2 (mission packages, as-flown logs), RC pairing.
apps/rc         Android "3DM Fly" (Kotlin, MSDK v5 + UX SDK, WebView running core).
index.html      LEGACY single-file KMZ patcher, still served by GitHub Pages. Don't break it.
route-engine.js LEGACY JS engine — being ported to packages/core; keep until the port passes its tests.
```

Mission package (per project): plan params, AOI, lines, computed waypoints, one KMZ per sortie,
DEM clip (AOI + buffer), offline basemap tiles.

## DJI app
- Mobile SDK app "3DM Fly", package `com.threedronemapping.fly` (must match applicationId exactly).
- App key lives ONLY in `apps/rc/local.properties` (`DJI_APP_KEY=`), gitignored. Never commit it.

## Route engine behaviour (carry over into core)
- Lines from block polygon + AGL, speed, sidelap, course; spacing from L3 swath. Serpentine.
- ~150 m run-in / run-out on every line.
- Waypoints every ~150 m, **absolute heights** from the DEM (each waypoint ≥ max terrain along both
  adjacent legs + 75 m corridor sampled every ≤30 m across-track + AGL).
- **Vertical profile** (docs/ugcs-notes.md): default `slow` = follow terrain and slow each steep leg so the
  climb/descent rate sits at the limit (4 / 3 m/s, both ends of the leg capped because DJI ramps speed to the
  next waypoint; legs needing <1 m/s = TOO_STEEP). `raise` = old gradient-limited waypoint raising.
  DJI `waypointSpeed` = speed from that waypoint to the NEXT (verified) → `legSpeed(wps, i) = wps[i].speed`.
- Fly-through: `toPointAndPassWithContinuityCurvature`, per-waypoint damping < adjacent leg length.
- **Figure-8** (IMU excitation) before the first line, before any resumed segment, AND after the last line
  (vendors + DJI L2/L3 manuals: align at both ends for forward/backward trajectory processing). Recording runs
  from the approach waypoint (START_RECORD) through the end figure-8 to the exit waypoint (STOP_RECORD).
  Straight level run ≥ alignStraightS (12 s) or 2.5 r before the first 8 and after the last.
- **Resume**: line where data stopped + new speed → route starts one line earlier, fresh figure-8.
  Never rely on Pilot 2 breakpoints.
- **Sorties**: split into battery-sized routes along the same path as resume, one-line overlap.
- Regression test: synthetic Nimba block (`test/synthetic-nimba.js`): 23 lines, 902 WP, 137 km, ~135 min.

## SDK facts (see docs/msdk-research.md for sources)
- MSDK **5.18.0**; M400 since 5.15, L3 since 5.17. Kotlin 2.1, AGP 8.7, Gradle 8.12, JDK 17,
  compile/target 35, minSdk 24, arm64-v8a. RC Plus 2 = Android 11.
- L3 recording: `LidarKey.KeyPointCloudRecord` / `KeyPointCloudRecordStatus` exist; untested on L3.
- WPML: M400 `droneEnumValue` = 103 (verified). waylines.wpml heights are WGS84 ellipsoidal;
  template.kml heightMode EGM96. `recordPointCloud` action documented for M300/M350 only.
- Pilot 2 file injection is unofficial; prefer flying from our app, or Import Route / Cloud API library.

## Equipment (docs/equipment.md, packages/core/src/equipment.ts)
- M400 official: 25 m/s, ascent 10 / descent 8 m/s, wind 12 m/s; flight time only published with H30T.
  Planner uses conservative limits (~40–60 % of max) and 30 min usable per battery set with L3.
- L3 pulse rate → max AGL: 100 kHz <500 m, 350 kHz <300 m, 1000 kHz <100 m, 2000 kHz <50 m (RTF).
- DJI documents NO figure-8 for L3 (it auto-calibrates inside Area Routes); ours is optional, default on.
- Planner supports LiDAR (L3) and photogrammetry (P1 24/35/50, L3 RGB) modes.

## WPML (from Luke's Pilot 2 samples, 25 Sep 2026; details in docs/wpml.md)
- Header VERIFIED: WPML 1.0.6, M400 droneEnumValue 103/0, L3 payloadEnumValue 117/0, payloadPositionIndex 0 (default port).
- Heights: template.kml heightMode EGM96 (height + ellipsoidHeight); waylines.wpml executeHeightMode WGS84 (ellipsoidal).
  egm96-universal reproduces Pilot 2's ellipsoidHeight to 0.1 mm.
- Recording: recordPointCloud start/pause/resume/stopRecord; DJI IMU calibration = aircraftCalibration action
  (3 passes, 30 m) before startRecord and after stopRecord; L3 RGB = startContinuousShooting on multipleDistance.
- Pilot 2 forces first/last waypoint to toPointAndStopWithDiscontinuityCurvature, damping 0.
- Only verified L3 values are written: samplingRate (Hz), returnMode sedecupleReturn, scanningMode repetitive.
  Other return/scan modes need another sample. payloadSubEnumValue was 0 (waypoint) vs 1 (area): we write 0.
- Samples + generated tests hold real site coordinates → gitignored (public repo) until Luke decides.
- Still open: Pilot 2 import of our KMZ, waypoint cap (samples/generated/test-cap-*.kmz), actions in Pilot 2 waypoint UI.

## Phase plan
0. MSDK spike (go/no-go): M400 connect + telemetry; L3 detect + record start/stop; tiny KMZ via
   WaypointMissionManager in simulator; KMZ injection into Pilot 2's route library.
1. core: TS port of engine + DEM + validation + package format; WPML writer scaffold.
2. web planner + api (projects, profiles, validation, publish, RC pairing via QR).
3. RC app: sync/offline, pre-flight checklist, fly/monitor, auto-resume, sorties, as-flown logs.
4. Pilot 2 injection path. 5. Field validation (WP cap, simulator, short live block).

## Sync backend (apps/api) + production
- Local dev on port 8788 (8787 is SurveyDeliver); local admin token in apps/api/.dev.vars.
- PRODUCTION (25 Sep 2026): Cloudflare Pages project m400-planner (classic Pages, created with --force) serves the
  planner + API (_worker.js) at https://m400-planner.pages.dev; D1 m400-api, R2 m400-packages; ADMIN_TOKEN secret
  (value in deploy/planner/admin-token.local). Deploy: node deploy/planner/deploy.mjs. See deploy/planner/README.md.
- 3dronemapping.com DNS is at Wix → custom domain planner.3dronemapping.com needs a Wix CNAME (Luke).
- Planner dev prefill: apps/web/.env.development.local (VITE_OPENTOPO_KEY, VITE_API_URL, VITE_API_ADMIN_TOKEN).

## Sorties (packages/core/src/sorties.ts)
- Split at whole lines by usable minutes/battery set; each sortie = transit + climb + fig-8 + lines + RTH + descent,
  timed with conservative vertical rates (4 up / 3 down m/s). Home placement dominates sortie count on high blocks.

## Conventions / constraints
- RC Plus 2 browser file picker is broken — anything on the RC loads via our app, not the browser.
- FlightHub 2 is not part of this workflow.
- DJI's default ASTER/SRTM DEM causes terrain-follow pitch spikes; use Copernicus GLO-30 (OpenTopography)
  or the client's own DTM.
- UI fonts: IBM Plex Mono + Barlow Semi Condensed.
- Operator is experienced (BVLOS-rated, LiDAR specialist): explain physics precisely, flag
  corrections clearly, don't oversimplify.
- Local tooling: Node 24, git, gh. No Android SDK / JDK 17 yet (only Java 8) — needed for apps/rc.

## Progress tracker
- docs/progress.html is the project timeline/gates page, published as an artifact (URL in memory). Update it and
  republish at every milestone; commit docs-only changes straight to main.
