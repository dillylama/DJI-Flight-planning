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
  adjacent legs + 75 m corridor + AGL; gradient-limited by raising only).
- Fly-through: `toPointAndPassWithContinuityCurvature`, per-waypoint damping < adjacent leg length.
- **Figure-8** (IMU excitation) before the first line and before any resumed segment. Point-cloud
  recording must already be running during it (START_RECORD on the approach waypoint).
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

## Blocked on (do NOT guess — wrong values = Pilot 2 rejects file or L3 doesn't record)
1. L3 `payloadEnumValue` etc. for the WPML header (M400 drone enum 103 is verified — still confirm from the sample).
2. Exact WPML action XML for start/stop point-cloud recording.
3. Pilot 2 / aircraft max waypoints per route.
4. Height mode (WGS84 ellipsoidal vs other) → whether `geoidN` is applied.
Source: KMZ exported from the RC — **Waypoint Route** (not Area Route), L3 selected, 4 WPs: start
point-cloud recording on WP1, different speed at WP2, stop recording on WP4 → `samples/`.
Luke will supply later; scaffold the WPML writer around it, don't invent values.

## Phase plan
0. MSDK spike (go/no-go): M400 connect + telemetry; L3 detect + record start/stop; tiny KMZ via
   WaypointMissionManager in simulator; KMZ injection into Pilot 2's route library.
1. core: TS port of engine + DEM + validation + package format; WPML writer scaffold.
2. web planner + api (projects, profiles, validation, publish, RC pairing via QR).
3. RC app: sync/offline, pre-flight checklist, fly/monitor, auto-resume, sorties, as-flown logs.
4. Pilot 2 injection path. 5. Field validation (WP cap, simulator, short live block).

## Sync backend (apps/api)
- Worker + D1 + R2, local only so far (dev on port 8788; 8787 is SurveyDeliver). Admin token in apps/api/.dev.vars.
- Deploy needs Luke: Cloudflare account, D1/R2 create, ADMIN_TOKEN secret, domain, ALLOWED_ORIGINS, Access.
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
